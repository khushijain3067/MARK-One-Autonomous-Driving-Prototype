/**
 * SIH 2026 - Problem Statement: SIH26037
 * Team MARK-One: Adaptive Path Planning on Unstructured Indian Roads
 * Control: Pure Pursuit Lateral Controller & Adaptive Longitudinal Speed Governor
 */

import { SIM_CONFIG } from '../config.js';

export class VehicleController {
    constructor() {
        this.wheelbase = SIM_CONFIG.VEHICLE.WHEELBASE;
        this.maxSteer = SIM_CONFIG.VEHICLE.MAX_STEER;
        this.nominalSpeed = SIM_CONFIG.VEHICLE.TARGET_SPEED;
        
        // Controller diagnostic stats
        this.crossTrackError = 0;
        this.headingError = 0;
        this.currentLookaheadDist = 4.2;
        this.targetPoint = { x: 0, y: 0 };
    }

    /**
     * Computes steering and acceleration commands for the vehicle.
     * Ensures the vehicle smoothly drives around obstacles and NEVER stops if a safe path exists!
     * @param {Vehicle} ego Ego vehicle
     * @param {Object} selectedTrajectory Trajectory chosen by LatticePlanner
     * @param {Object} riskState Evaluated risk state
     * @param {string} planningStatus Status from planner
     */
    computeCommands(ego, selectedTrajectory, riskState, planningStatus) {
        if (!selectedTrajectory || !selectedTrajectory.points || selectedTrajectory.points.length === 0) {
            return { steering: 0, acceleration: -3.0, targetSpeed: 0 };
        }

        // 1. Dynamic Pure Pursuit Lookahead Distance
        // Scaled with velocity for smooth, jitter-free tracking up to 60 km/h
        const kv = 0.35;
        const L0 = 3.8;
        const lookaheadDist = Math.max(3.8, Math.min(8.8, kv * Math.max(2.0, ego.speed) + L0));
        this.currentLookaheadDist = lookaheadDist;

        // 2. Find target lookahead point on selected trajectory
        const pts = selectedTrajectory.points;
        let targetPt = pts[pts.length - 1]; // Fallback to end
        let minDiff = Infinity;

        for (let i = 0; i < pts.length; i++) {
            const pt = pts[i];
            const dist = Math.hypot(pt.x - ego.x, pt.y - ego.y);
            const diff = Math.abs(dist - lookaheadDist);
            if (diff < minDiff && pt.y >= ego.y - 0.2) {
                minDiff = diff;
                targetPt = pt;
            }
        }
        this.targetPoint = targetPt;

        // 3. Compute Pure Pursuit Steering Angle
        const dx = targetPt.x - ego.x;
        const dy = targetPt.y - ego.y;
        
        // Angle to target point relative to vehicle heading
        const angleToTarget = Math.atan2(dx, dy);
        const alpha = angleToTarget - ego.heading;

        // Pure pursuit formula: delta = atan2(2 * L * sin(alpha), Ld)
        let steerCmd = Math.atan2(2 * this.wheelbase * Math.sin(alpha), lookaheadDist);
        steerCmd = Math.max(-this.maxSteer, Math.min(this.maxSteer, steerCmd));

        // Cross-track and heading error estimation
        this.crossTrackError = dx * Math.cos(ego.heading) - dy * Math.sin(ego.heading);
        this.headingError = targetPt.heading - ego.heading;

        // Actuate turn signals based on steering
        if (steerCmd > 0.06) {
            ego.turnSignal = 'right';
        } else if (steerCmd < -0.06) {
            ego.turnSignal = 'left';
        } else {
            ego.turnSignal = 'none';
        }

        // 4. Longitudinal Speed Control
        // CORE PRINCIPLE: Maintain brisk cruising momentum (~40-42 km/h), bypass smoothly, and ONLY stop if dead-end blocked!
        let targetSpeed = this.nominalSpeed;
        let accelCmd = 0;

        if (planningStatus === 'BLOCKED_STOP' || !selectedTrajectory.isFeasible) {
            // Road completely blocked with zero traversable path -> safe halt
            targetSpeed = 0.0;
            accelCmd = -SIM_CONFIG.VEHICLE.MAX_BRAKE;
            ego.mode = 'EMERGENCY_STOP';
            ego.statusMessage = 'ROAD_BLOCKED';
        } else if (planningStatus === 'REPLANNING') {
            // Object-specific bypass speeds:
            const threat = riskState.highestRiskTrack;
            const isPothole = threat && (threat.type === 'POTHOLE' || threat.isSurfaceDefect);
            const isBiological = threat && (threat.type === 'PEDESTRIAN' || threat.type === 'CATTLE' || threat.isBiological);
            const isOncoming = threat && (threat.type === 'ONCOMING_VEHICLE' || threat.isOncoming);

            if (isPothole) {
                // Potholes: maintain fast cruising momentum (~39 km/h / 10.8 m/s) - smoothly steer around!
                targetSpeed = Math.min(this.nominalSpeed, 10.8);
            } else if (isBiological) {
                // Pedestrian or Sacred Cattle: respectful, safe slowdown (~28 km/h / 7.8 m/s)
                targetSpeed = Math.min(this.nominalSpeed, 7.8);
            } else if (isOncoming) {
                // Oncoming traffic: defensive corridor speed (~32 km/h / 9.0 m/s)
                targetSpeed = Math.min(this.nominalSpeed, 9.0);
            } else {
                // Standard vehicles, barricades, parked cars: active bypass speed (~38 km/h / 10.5 m/s)
                targetSpeed = Math.min(this.nominalSpeed, 10.5);
            }

            // Mild curvature modulation only for very sharp swerves
            if (selectedTrajectory.maxCurvature > 0.12) {
                targetSpeed = Math.max(9.5, targetSpeed - (selectedTrajectory.maxCurvature - 0.12) * 8.0);
            }

            const speedError = targetSpeed - ego.speed;
            accelCmd = Math.max(-3.5, Math.min(SIM_CONFIG.VEHICLE.MAX_ACCEL, speedError * 1.5));
            ego.mode = 'ACTIVE_REPLANNING';
            ego.statusMessage = 'AVOIDING_OBSTACLE';
        } else {
            // Nominal cruising speed (~42 km/h / 11.67 m/s) - brisk and continuous
            targetSpeed = this.nominalSpeed;
            const speedError = targetSpeed - ego.speed;
            accelCmd = Math.max(-2.5, Math.min(SIM_CONFIG.VEHICLE.MAX_ACCEL, speedError * 1.4));
            ego.mode = 'AUTONOMOUS';
            ego.statusMessage = 'NOMINAL_CRUISING';
        }

        return {
            steering: steerCmd,
            acceleration: accelCmd,
            targetSpeed: targetSpeed
        };
    }
}
