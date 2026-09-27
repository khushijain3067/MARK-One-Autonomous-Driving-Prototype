/**
 * SIH 2026 - Problem Statement: SIH26037
 * Team MARK-One: Adaptive Path Planning on Unstructured Indian Roads
 * Model: Autonomous Ego Vehicle (Kinematic Bicycle Model)
 */

import { SIM_CONFIG } from '../config.js';

export class Vehicle {
    constructor(x = 0, y = 0, heading = 0) {
        // Geometric dimensions
        this.length = SIM_CONFIG.VEHICLE.LENGTH;
        this.width = SIM_CONFIG.VEHICLE.WIDTH;
        this.wheelbase = SIM_CONFIG.VEHICLE.WHEELBASE;
        this.frontOverhang = (this.length - this.wheelbase) / 2;
        this.rearOverhang = (this.length - this.wheelbase) / 2;

        // Kinematic state
        this.x = x;                           // Lateral position (m)
        this.y = y;                           // Longitudinal position (m)
        this.heading = heading;               // Heading angle (rad, 0 = forward along +y)
        this.speed = SIM_CONFIG.VEHICLE.TARGET_SPEED; // m/s
        this.targetSpeed = SIM_CONFIG.VEHICLE.TARGET_SPEED;
        this.acceleration = 0;                // m/s^2
        this.steeringAngle = 0;              // rad
        this.targetSteeringAngle = 0;
        this.yawRate = 0;                     // rad/s
        this.distanceTraveled = 0;            // meters

        // Autonomous Status & Modes
        this.mode = 'AUTONOMOUS';             // 'AUTONOMOUS', 'EMERGENCY_STOP', 'YIELDING'
        this.brakeLights = false;
        this.turnSignal = 'none';             // 'none', 'left', 'right', 'hazard'
        this.statusMessage = 'NOMINAL_CRUISING';

        // History trail for visualization
        this.history = [];
        this.maxHistory = 100;
    }

    /**
     * Resets the vehicle to initial starting position & speed.
     */
    reset(x = 0, y = 0, heading = 0, speed = SIM_CONFIG.VEHICLE.TARGET_SPEED) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.speed = speed;
        this.targetSpeed = speed;
        this.acceleration = 0;
        this.steeringAngle = 0;
        this.targetSteeringAngle = 0;
        this.yawRate = 0;
        this.distanceTraveled = 0;
        this.mode = 'AUTONOMOUS';
        this.brakeLights = false;
        this.turnSignal = 'none';
        this.statusMessage = 'NOMINAL_CRUISING';
        this.history = [];
    }

    /**
     * Updates the vehicle state using the Kinematic Bicycle Model.
     * @param {number} dt Time step in seconds
     * @param {number} targetSteering Steering command from lateral controller (rad)
     * @param {number} targetAccel Longitudinal acceleration command from speed controller (m/s^2)
     */
    update(dt, targetSteering, targetAccel) {
        // 1. Steering rate limiting (actuator dynamics)
        const maxSteerChange = SIM_CONFIG.VEHICLE.MAX_STEER_RATE * dt;
        const steerDiff = Math.max(
            -SIM_CONFIG.VEHICLE.MAX_STEER,
            Math.min(SIM_CONFIG.VEHICLE.MAX_STEER, targetSteering)
        ) - this.steeringAngle;
        
        this.steeringAngle += Math.max(-maxSteerChange, Math.min(maxSteerChange, steerDiff));

        // 2. Acceleration limits & speed integration
        let clampedAccel = targetAccel;
        if (clampedAccel < -SIM_CONFIG.VEHICLE.MAX_BRAKE) {
            clampedAccel = -SIM_CONFIG.VEHICLE.MAX_BRAKE;
        } else if (clampedAccel > SIM_CONFIG.VEHICLE.MAX_ACCEL) {
            clampedAccel = SIM_CONFIG.VEHICLE.MAX_ACCEL;
        }
        this.acceleration = clampedAccel;
        
        // Active brake light detection
        this.brakeLights = (this.acceleration < -1.2);

        // Update velocity
        this.speed = Math.max(0, Math.min(SIM_CONFIG.VEHICLE.MAX_SPEED, this.speed + this.acceleration * dt));

        // 3. Kinematic bicycle model equations of motion
        // Along heading: delta x = v * sin(theta) * dt, delta y = v * cos(theta) * dt
        const ds = this.speed * dt;
        this.yawRate = (this.speed / this.wheelbase) * Math.tan(this.steeringAngle);
        
        this.heading += this.yawRate * dt;
        this.x += ds * Math.sin(this.heading);
        this.y += ds * Math.cos(this.heading);
        this.distanceTraveled += ds;

        // 4. Update path trail
        if (this.history.length === 0 || Math.hypot(this.x - this.history[this.history.length - 1].x, this.y - this.history[this.history.length - 1].y) > 0.5) {
            this.history.push({ x: this.x, y: this.y, heading: this.heading });
            if (this.history.length > this.maxHistory) {
                this.history.shift();
            }
        }
    }

    /**
     * Returns the 4 corner points of the vehicle footprint in world coordinates (meters).
     * Useful for sensor raycasts and collision checks.
     */
    getFootprint() {
        const cosH = Math.cos(this.heading);
        const sinH = Math.sin(this.heading);
        const halfW = this.width / 2;
        const frontL = this.wheelbase / 2 + this.frontOverhang;
        const rearL = this.wheelbase / 2 + this.rearOverhang;

        // Rotated offset helper
        const rotate = (dx, dy) => ({
            x: this.x + dx * cosH + dy * sinH,
            y: this.y - dx * sinH + dy * cosH
        });

        return [
            rotate(-halfW, frontL),  // Front Left
            rotate(halfW, frontL),   // Front Right
            rotate(halfW, -rearL),   // Rear Right
            rotate(-halfW, -rearL)   // Rear Left
        ];
    }

    /**
     * Returns speed in kilometers per hour.
     */
    getSpeedKmh() {
        return (this.speed * 3.6).toFixed(1);
    }
}
