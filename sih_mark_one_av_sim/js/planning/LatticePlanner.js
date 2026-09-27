/**
 * SIH 2026 - Problem Statement: SIH26037
 * Team MARK-One: Adaptive Path Planning on Unstructured Indian Roads
 * Planning: Dynamic Adaptive Trajectory Lattice & Free-Space Re-Planner
 * 
 * Accurately detects obstacles in ego path, evaluates lateral free space on LEFT, RIGHT and CENTER,
 * generates feasible S-curve bypass trajectories using a 3-Zone Spatial Model (Ahead, Passing, Passed),
 * handles both static and dynamic actors with relative velocity forecasting,
 * scores candidates using a multi-objective cost function, and returns toward nominal road centerline after clearing.
 */

import { SIM_CONFIG } from '../config.js';

export class LatticePlanner {
    constructor() {
        this.config = SIM_CONFIG.PLANNER;
        
        // Planning state
        this.selectedTrajectory = null;
        this.candidateTrajectories = [];
        this.planningStatus = 'NOMINAL'; // 'NOMINAL', 'REPLANNING', 'BLOCKED_STOP'
        this.activeDecision = 'MAINTAINING_COURSE';
        this.selectedPathSide = 'CENTER'; // 'LEFT', 'RIGHT', 'CENTER', 'NONE'
        
        // Free-space metrics
        this.corridors = {
            left: { width: 0, center: 0, isFeasible: true, cost: 0 },
            right: { width: 0, center: 0, isFeasible: true, cost: 0 },
            center: { width: 0, center: 0, isFeasible: true, cost: 0 }
        };

        this.costBreakdown = { collision: 0, boundary: 0, smoothness: 0, centerline: 0, total: 0 };
        this.primaryThreat = null;
    }

    /**
     * Executes the adaptive planning cycle.
     * @param {Vehicle} ego Ego vehicle state
     * @param {Array<Object>} fusedTracks Fused tracked obstacles
     * @param {Road} road Unstructured road model
     * @param {RiskAssessmentEngine} riskEngine Risk assessment state
     */
    plan(ego, fusedTracks, road, riskEngine) {
        this.candidateTrajectories = [];
        this.primaryThreat = null;

        const carHalfW = ego.width / 2;
        const carHalfL = (ego.length || 4.4) / 2;
        const egoFront = ego.y + carHalfL;
        const egoRear = ego.y - carHalfL;
        const lookaheadHorizon = Math.max(38.0, Math.min(54.0, ego.speed * 3.4));

        // 1. Identify obstacles relevant to ego vehicle:
        // Any obstacle that is not fully cleared (front is >= egoRear - 2.5) up to lookahead horizon
        const relevantObstacles = fusedTracks.filter(t => {
            const obsHalfL = (t.length || 2.5) / 2;
            const obsFront = t.y + obsHalfL;
            const obsRear = t.y - obsHalfL;
            return obsFront >= (egoRear - 2.5) && obsRear <= (ego.y + lookaheadHorizon + 15.0);
        });

        // 2. Road boundaries
        const roadBoundsNow = road.getBoundariesAt(ego.y);
        const roadBoundsAhead = road.getBoundariesAt(ego.y + lookaheadHorizon);
        const nominalCenterline = roadBoundsAhead.centerlineOffset;

        // 3. 3-Zone Spatial Threat Model:
        // Zone 3: Passed (egoRear > obsFront + 1.2) -> cleared, no active threat
        // Zone 2: Passing / Alongside (egoRear <= obsFront + 1.2 && egoFront >= obsRear - 0.5) -> HOLD BYPASS!
        // Zone 1: Approaching / Ahead (egoFront < obsRear - 0.5) -> SWERVE TO BYPASS BEFORE REACHING!

        let zone2Threat = null;
        let zone1Threat = null;
        let minZone1Dist = Infinity;

        for (const obs of relevantObstacles) {
            const obsHalfL = (obs.length || 2.5) / 2;
            const obsHalfW = (obs.width || 1.6) / 2;
            const obsFront = obs.y + obsHalfL;
            const obsRear = obs.y - obsHalfL;
            const dx = Math.abs(obs.x - ego.x);
            const dxCenter = Math.abs(obs.x - nominalCenterline);
            const pathBuffer = carHalfW + obsHalfW + (obs.isBiological ? 0.75 : 0.40);

            // Is vehicle currently alongside or just finishing alongside this obstacle?
            if (egoRear <= obsFront + 1.2 && egoFront >= obsRear - 0.5) {
                // If vehicle is laterally close or was steering around it, maintain it as active threat
                if (dx < pathBuffer + 2.0 || dxCenter < pathBuffer + 1.5) {
                    zone2Threat = obs;
                    break; // Zone 2 threat takes highest immediate priority
                }
            } else if (egoFront < obsRear - 0.5) {
                // Ahead: check if it intersects ego forward path or nominal centerline corridor
                const dy = obsRear - egoFront;
                if ((dx < pathBuffer + 0.6 || dxCenter < pathBuffer + 0.6) && dy < minZone1Dist) {
                    minZone1Dist = dy;
                    zone1Threat = obs;
                }
            }
        }

        // Active primary threat selection
        const primaryThreat = zone2Threat || zone1Threat;
        this.primaryThreat = primaryThreat;

        // 4. Calculate Free Space Corridors & Gap Intervals
        let leftBypassX = ego.x;
        let rightBypassX = ego.x;
        let leftFeasible = true;
        let rightFeasible = true;

        if (primaryThreat) {
            const obsY = primaryThreat.y;
            const obsBounds = road.getBoundariesAt(obsY);
            const obsHalfW = (primaryThreat.width || 1.6) / 2;
            const obsLeft = primaryThreat.x - obsHalfW;
            const obsRight = primaryThreat.x + obsHalfW;

            // Space on LEFT: road left edge to obstacle left edge
            const leftSpaceStart = obsBounds.leftEdge + carHalfW + 0.25;
            const leftSpaceEnd = obsLeft - carHalfW - (primaryThreat.isBiological ? 0.55 : 0.25);
            const leftWidth = leftSpaceEnd - leftSpaceStart;

            // Space on RIGHT: obstacle right edge to road right edge
            const rightSpaceStart = obsRight + carHalfW + (primaryThreat.isBiological ? 0.55 : 0.25);
            const rightSpaceEnd = obsBounds.rightEdge - carHalfW - 0.25;
            const rightWidth = rightSpaceEnd - rightSpaceStart;

            this.corridors.left.width = Math.max(0, leftWidth);
            this.corridors.right.width = Math.max(0, rightWidth);

            leftFeasible = leftWidth >= 0.15;
            rightFeasible = rightWidth >= 0.15;

            leftBypassX = (leftSpaceStart + leftSpaceEnd) / 2;
            rightBypassX = (rightSpaceStart + rightSpaceEnd) / 2;

            // Clamp bypass targets within safe drivable asphalt
            leftBypassX = Math.max(obsBounds.leftEdge + carHalfW + 0.20, Math.min(obsLeft - carHalfW - 0.20, leftBypassX));
            rightBypassX = Math.min(obsBounds.rightEdge - carHalfW - 0.20, Math.max(obsRight + carHalfW + 0.20, rightBypassX));

            this.corridors.left.center = leftBypassX;
            this.corridors.right.center = rightBypassX;
            this.corridors.left.isFeasible = leftFeasible;
            this.corridors.right.isFeasible = rightFeasible;
        } else {
            this.corridors.left.width = 4.0;
            this.corridors.right.width = 4.0;
            this.corridors.left.isFeasible = true;
            this.corridors.right.isFeasible = true;
        }

        // 5. Generate Candidate Trajectories
        const candidateProfiles = [];

        if (primaryThreat) {
            const isPothole = primaryThreat.typeKey === 'POTHOLE' || primaryThreat.isSurfaceDefect;

            // Check for flanking obstacles pinching both sides (e.g. twin barricades)
            let centerGapX = null;
            const sameZoneObs = relevantObstacles.filter(o => Math.abs(o.y - primaryThreat.y) < 5.0 && o.id !== primaryThreat.id);
            if (sameZoneObs.length > 0) {
                for (const other of sameZoneObs) {
                    const o1 = (primaryThreat.x < other.x) ? primaryThreat : other;
                    const o2 = (primaryThreat.x < other.x) ? other : primaryThreat;
                    const o1Right = o1.x + ((o1.width || 1.6) / 2);
                    const o2Left = o2.x - ((o2.width || 1.6) / 2);
                    const gapWidth = o2Left - o1Right;
                    if (gapWidth >= ego.width + 0.18) {
                        centerGapX = (o1Right + o2Left) / 2;
                        break;
                    }
                }
            }

            // If center gap between flanking obstacles exists, prioritize center chokepoint candidate
            if (centerGapX !== null) {
                candidateProfiles.push({
                    type: 'CENTER_CHOKEPOINT',
                    label: 'Candidate: Center Chokepoint Gap',
                    bypassX: centerGapX,
                    returnX: nominalCenterline,
                    preferredSide: 'CENTER'
                });
            }

            // Left Bypass candidate
            if (leftFeasible) {
                candidateProfiles.push({
                    type: 'LEFT_BYPASS',
                    label: 'Candidate: Safe Path Left',
                    bypassX: leftBypassX,
                    returnX: nominalCenterline,
                    preferredSide: 'LEFT'
                });
            }

            // Right Bypass candidate
            if (rightFeasible) {
                candidateProfiles.push({
                    type: 'RIGHT_BYPASS',
                    label: 'Candidate: Safe Path Right',
                    bypassX: rightBypassX,
                    returnX: nominalCenterline,
                    preferredSide: 'RIGHT'
                });
            }

            // Pothole-specific subtle shifts (dodges tires with minimal steering)
            if (isPothole) {
                const subtleLeft = Math.max(roadBoundsNow.leftEdge + carHalfW + 0.3, primaryThreat.x - (primaryThreat.width / 2) - 0.85);
                const subtleRight = Math.min(roadBoundsNow.rightEdge - carHalfW - 0.3, primaryThreat.x + (primaryThreat.width / 2) + 0.85);
                candidateProfiles.push({
                    type: 'POTHOLE_SUBTLE_LEFT',
                    label: 'Candidate: Pothole Dodge Left',
                    bypassX: subtleLeft,
                    returnX: nominalCenterline,
                    preferredSide: 'LEFT'
                });
                candidateProfiles.push({
                    type: 'POTHOLE_SUBTLE_RIGHT',
                    label: 'Candidate: Pothole Dodge Right',
                    bypassX: subtleRight,
                    returnX: nominalCenterline,
                    preferredSide: 'RIGHT'
                });
            }

            // Nominal Center Path
            candidateProfiles.push({
                type: 'NOMINAL_CENTER',
                label: 'Candidate: Maintain Centerline',
                bypassX: nominalCenterline,
                returnX: nominalCenterline,
                preferredSide: 'CENTER'
            });

            // Moderate variations
            if (leftFeasible) {
                candidateProfiles.push({
                    type: 'LEFT_MODERATE',
                    label: 'Candidate: Left Tight Corridor',
                    bypassX: (leftBypassX + nominalCenterline) / 2,
                    returnX: nominalCenterline,
                    preferredSide: 'LEFT'
                });
            }
            if (rightFeasible) {
                candidateProfiles.push({
                    type: 'RIGHT_MODERATE',
                    label: 'Candidate: Right Tight Corridor',
                    bypassX: (rightBypassX + nominalCenterline) / 2,
                    returnX: nominalCenterline,
                    preferredSide: 'RIGHT'
                });
            }
        } else {
            // Free driving / clear path
            candidateProfiles.push({
                type: 'NOMINAL_CENTER',
                label: 'Candidate: Road Centerline',
                bypassX: nominalCenterline,
                returnX: nominalCenterline,
                preferredSide: 'CENTER'
            });
            candidateProfiles.push({
                type: 'LEFT_LANE',
                label: 'Candidate: Left Corridor',
                bypassX: nominalCenterline - 1.2,
                returnX: nominalCenterline,
                preferredSide: 'LEFT'
            });
            candidateProfiles.push({
                type: 'RIGHT_LANE',
                label: 'Candidate: Right Corridor',
                bypassX: nominalCenterline + 1.2,
                returnX: nominalCenterline,
                preferredSide: 'RIGHT'
            });
        }

        // 6. Build and Evaluate each Candidate Trajectory
        let bestCandidate = null;
        let lowestCost = Infinity;

        for (const profile of candidateProfiles) {
            const trajectory = this.generateSCurveTrajectory(
                ego,
                profile.bypassX,
                profile.returnX,
                primaryThreat,
                lookaheadHorizon
            );

            trajectory.profile = profile;
            trajectory.label = profile.label;
            trajectory.preferredSide = profile.preferredSide;

            // Multi-objective cost evaluation
            const costEval = this.evaluateTrajectoryCost(trajectory, ego, relevantObstacles, road);
            trajectory.cost = costEval.totalCost;
            trajectory.costComponents = costEval;
            trajectory.isFeasible = costEval.isFeasible;
            trajectory.collisionPoint = costEval.collisionPoint;

            this.candidateTrajectories.push(trajectory);

            if (costEval.isFeasible && costEval.totalCost < lowestCost) {
                lowestCost = costEval.totalCost;
                bestCandidate = trajectory;
            }
        }

        // 7. Decision Selection & Action Dispatch
        if (bestCandidate) {
            this.selectedTrajectory = bestCandidate;
            this.costBreakdown = bestCandidate.costComponents;
            this.selectedPathSide = bestCandidate.preferredSide;

            if (primaryThreat) {
                this.planningStatus = 'REPLANNING';
                const side = bestCandidate.preferredSide;

                if (primaryThreat.typeKey === 'POTHOLE' || primaryThreat.isSurfaceDefect) {
                    this.activeDecision = `STEERING AROUND POTHOLE (${side})`;
                } else if (primaryThreat.typeKey === 'BARRICADE') {
                    this.activeDecision = `STEERING AROUND BARRICADE (${side})`;
                } else if (primaryThreat.typeKey === 'PEDESTRIAN') {
                    this.activeDecision = `AVOIDING PEDESTRIAN (${side})`;
                } else if (primaryThreat.typeKey === 'CATTLE') {
                    this.activeDecision = `DETOURING AROUND CATTLE (${side})`;
                } else if (primaryThreat.typeKey === 'PARKED_VEHICLE') {
                    this.activeDecision = `BYPASSING PARKED TEMPO (${side})`;
                } else if (primaryThreat.typeKey === 'ONCOMING_VEHICLE') {
                    this.activeDecision = `YIELDING TO ONCOMING VEHICLE (${side})`;
                } else if (primaryThreat.typeKey === 'BUS_TRUCK') {
                    this.activeDecision = `OVERTAKING HEAVY TRUCK (${side})`;
                } else if (primaryThreat.typeKey === 'MOTORCYCLE') {
                    this.activeDecision = `MAINTAINING BUFFER WITH BIKE (${side})`;
                } else if (primaryThreat.typeKey === 'DEBRIS') {
                    this.activeDecision = `AVOIDING ROAD DEBRIS (${side})`;
                } else {
                    this.activeDecision = `SAFE PATH ${side} SELECTED`;
                }
            } else {
                this.planningStatus = 'NOMINAL';
                this.activeDecision = 'NOMINAL_TRACKING';
                this.selectedPathSide = 'CENTER';
            }
        } else {
            // All candidates truly blocked: zero traversable path
            this.planningStatus = 'BLOCKED_STOP';
            this.activeDecision = 'ROAD BLOCKED — WAITING FOR SAFE PATH';
            this.selectedPathSide = 'NONE';
            this.selectedTrajectory = this.generateEmergencyStoppingPath(ego, 14.0);
            this.costBreakdown = { collision: 9999, boundary: 0, smoothness: 0, centerline: 0, total: 9999 };
        }

        return {
            selectedTrajectory: this.selectedTrajectory,
            candidateTrajectories: this.candidateTrajectories,
            planningStatus: this.planningStatus,
            activeDecision: this.activeDecision,
            selectedPathSide: this.selectedPathSide,
            corridors: this.corridors,
            primaryThreat: this.primaryThreat,
            costBreakdown: this.costBreakdown
        };
    }

    /**
     * Generates a 3-zone S-curve trajectory:
     * Zone 1 (Ahead): Swerves smoothly from ego position to bypassX before the obstacle
     * Zone 2 (Passing): Clears alongside bypassX until vehicle rear bumper passes obstacle front bumper
     * Zone 3 (Passed): Smoothly steers back toward the road's natural centerline (returnX)
     * Dynamically accounts for moving actors via relative velocity forecasting.
     */
    generateSCurveTrajectory(ego, bypassX, returnX, obstacle, totalLookahead) {
        const points = [];
        const numSteps = 38;
        const x0 = ego.x;
        const y0 = ego.y;
        const carHalfL = (ego.length || 4.4) / 2;
        
        let yAvoidEnd = y0 + 12.0;
        let yClearEnd = y0 + 20.0;
        let yReturnEnd = y0 + totalLookahead;

        if (obstacle) {
            const obsLen = obstacle.length || 2.5;
            const obsHalfL = obsLen / 2;
            const obsRear0 = obstacle.y - obsHalfL;
            const obsFront0 = obstacle.y + obsHalfL;
            const egoFront = y0 + carHalfL;
            const egoRear = y0 - carHalfL;

            // Longitudinal relative speed for dynamic actors
            const obsVy = (obstacle.vy !== undefined) ? obstacle.vy : (obstacle.speed !== undefined ? obstacle.speed * Math.cos(obstacle.heading || 0) : 0);
            const vRel = Math.max(1.5, ego.speed - obsVy);

            if (egoFront < obsRear0 - 0.5) {
                // Zone 1: Approaching obstacle ahead
                // Finish evasive lateral transition before front bumper reaches obstacle rear
                const distToObs = Math.max(1.0, obsRear0 - egoFront);
                const tAvoid = Math.max(0.4, (distToObs - 1.5) / vRel);
                yAvoidEnd = Math.max(y0 + 2.5, y0 + Math.min(distToObs, ego.speed * tAvoid));
                
                // Clear obstacle front
                const tClear = Math.max(1.2, (obsFront0 - egoRear + 2.5) / vRel);
                yClearEnd = Math.max(yAvoidEnd + 3.5, y0 + ego.speed * tClear);
                yReturnEnd = Math.max(yClearEnd + 14.0, y0 + totalLookahead);
            } else if (egoRear <= obsFront0 + 1.2) {
                // Zone 2: Currently alongside obstacle
                yAvoidEnd = y0 + 1.0;
                const tClear = Math.max(0.6, (obsFront0 - egoRear + 2.5) / vRel);
                yClearEnd = Math.max(y0 + 3.0, y0 + ego.speed * tClear);
                yReturnEnd = Math.max(yClearEnd + 14.0, y0 + totalLookahead);
            } else {
                // Zone 3: Obstacle passed; smooth recovery to centerline
                yAvoidEnd = y0 + 6.0;
                yClearEnd = y0 + 10.0;
                yReturnEnd = y0 + totalLookahead;
            }
        }

        let maxCurvature = 0;
        let prevHeading = ego.heading;

        for (let i = 0; i <= numSteps; i++) {
            const u = i / numSteps;
            const ptY = y0 + (yReturnEnd - y0) * u;
            let ptX = x0;

            if (ptY <= yAvoidEnd) {
                // Phase 1: Smooth quintic transition from x0 to bypassX
                const prog = Math.max(0, Math.min(1, (ptY - y0) / Math.max(0.1, yAvoidEnd - y0)));
                const s = 10 * Math.pow(prog, 3) - 15 * Math.pow(prog, 4) + 6 * Math.pow(prog, 5);
                ptX = x0 + (bypassX - x0) * s;
            } else if (ptY <= yClearEnd) {
                // Phase 2: Steady bypass alongside obstacle (holds lateral safety buffer)
                ptX = bypassX;
            } else {
                // Phase 3: Smooth quintic return toward natural road centerline (returnX)
                const prog = Math.max(0, Math.min(1, (ptY - yClearEnd) / Math.max(0.1, yReturnEnd - yClearEnd)));
                const s = 10 * Math.pow(prog, 3) - 15 * Math.pow(prog, 4) + 6 * Math.pow(prog, 5);
                ptX = bypassX + (returnX - bypassX) * s;
            }

            // Calculate heading along trajectory
            let heading = 0;
            if (points.length > 0) {
                const prev = points[points.length - 1];
                heading = Math.atan2(ptX - prev.x, ptY - prev.y);
                const curvature = Math.abs(heading - prevHeading) / Math.max(0.1, ptY - prev.y);
                if (curvature > maxCurvature) maxCurvature = curvature;
                prevHeading = heading;
            } else {
                heading = ego.heading;
            }

            points.push({
                x: ptX,
                y: ptY,
                heading: heading,
                progress: u,
                distFromEgo: ptY - y0
            });
        }

        return {
            points: points,
            bypassX: bypassX,
            returnX: returnX,
            maxCurvature: maxCurvature,
            isFeasible: true
        };
    }

    /**
     * Emergency fallback straight stop trajectory.
     */
    generateEmergencyStoppingPath(ego, stopDist) {
        const points = [];
        for (let i = 0; i <= 20; i++) {
            const u = i / 20;
            points.push({
                x: ego.x,
                y: ego.y + stopDist * u,
                heading: ego.heading,
                progress: u,
                distFromEgo: stopDist * u
            });
        }
        return {
            points: points,
            bypassX: ego.x,
            returnX: ego.x,
            maxCurvature: 0,
            isFeasible: false,
            label: 'Emergency Safe Stop'
        };
    }

    /**
     * Evaluates multi-objective cost for a candidate trajectory:
     * - Hard Physical Collision check against solid obstacles
     * - Safety buffer clearance evaluation
     * - Pothole wheel collision check (road surface defect penalty only)
     * - Road boundary adherence
     * - Smoothness & lateral jerk
     * - Centerline return cost
     */
    evaluateTrajectoryCost(trajectory, ego, obstacles, road) {
        let collisionCost = 0;
        let boundaryCost = 0;
        let potholeCost = 0;
        let isFeasible = true;
        let collisionPoint = null;

        const egoHalfW = ego.width / 2;
        const egoHalfL = (ego.length || 4.4) / 2;
        const pts = trajectory.points;

        for (let i = 1; i < pts.length; i++) {
            const pt = pts[i];

            // 1. Road Boundary Constraint Check
            const bounds = road.getBoundariesAt(pt.y);
            const distToLeft = pt.x - bounds.leftEdge;
            const distToRight = bounds.rightEdge - pt.x;
            const minBoundDist = Math.min(distToLeft, distToRight);

            if (minBoundDist < egoHalfW) {
                // Hard boundary violation: car body crosses over edge into unpaved ditch
                boundaryCost += 6000;
                isFeasible = false;
                collisionPoint = { x: pt.x, y: pt.y, type: 'BOUNDARY' };
                break;
            } else if (minBoundDist < egoHalfW + 0.35) {
                // Soft proximity penalty near road edge
                boundaryCost += Math.pow((egoHalfW + 0.35) - minBoundDist, 2) * 40.0;
            }

            // 2. Obstacle Proximity & Collision Check
            for (const obs of obstacles) {
                // Spatiotemporal position: evaluate obstacle position at time t when vehicle reaches pt.y
                const t = Math.max(0, (pt.y - ego.y) / Math.max(2.5, ego.speed));
                let obsX = obs.x;
                let obsY = obs.y;

                if (!obs.isSurfaceDefect && obs.speed > 0) {
                    obsX = obs.x + (obs.vx || 0) * t;
                    const vy = (obs.vy !== undefined) ? obs.vy : (obs.speed * Math.cos(obs.heading || 0));
                    obsY = obs.y + vy * t;
                }

                const obsHalfW = (obs.width || 1.6) / 2;
                const obsHalfL = (obs.length || 2.5) / 2;

                if (obs.typeKey === 'POTHOLE' || obs.isSurfaceDefect) {
                    // Pothole check: check if left or right wheels roll through pothole crater
                    const leftWheelX = pt.x - 0.75;
                    const rightWheelX = pt.x + 0.75;
                    const distLeftWheel = Math.hypot(leftWheelX - obsX, pt.y - obsY);
                    const distRightWheel = Math.hypot(rightWheelX - obsX, pt.y - obsY);
                    const potholeRadius = (obs.width || 1.4) / 2;

                    if (distLeftWheel < (potholeRadius + 0.10) || distRightWheel < (potholeRadius + 0.10)) {
                        potholeCost += 60.0; // High penalty to pick bypass, but does NOT mark infeasible
                    }
                } else {
                    // Physical solid obstacle collision check
                    const carFront = pt.y + egoHalfL;
                    const carRear = pt.y - egoHalfL;
                    const obsFront = obsY + obsHalfL;
                    const obsRear = obsY - obsHalfL;

                    const isLongitudinalOverlap = (carFront >= obsRear - 0.15) && (carRear <= obsFront + 0.15);

                    const carLeft = pt.x - egoHalfW;
                    const carRight = pt.x + egoHalfW;
                    const obsLeft = obsX - obsHalfW;
                    const obsRight = obsX + obsHalfW;

                    // Physical body penetration
                    const isPhysicalOverlap = (carRight >= obsLeft) && (carLeft <= obsRight);

                    if (isLongitudinalOverlap && isPhysicalOverlap) {
                        // Direct physical collision!
                        collisionCost += 8000;
                        isFeasible = false;
                        collisionPoint = { x: pt.x, y: pt.y, type: 'OBSTACLE', obstacle: obs };
                        break;
                    } else if (isLongitudinalOverlap) {
                        // Calculate lateral clearance
                        let lateralClearance = 0;
                        if (carLeft > obsRight) {
                            lateralClearance = carLeft - obsRight;
                        } else if (carRight < obsLeft) {
                            lateralClearance = obsLeft - carRight;
                        }

                        // Required buffer margin:
                        const safetyMargin = obs.isBiological ? 0.50 : (obs.typeKey === 'BARRICADE' ? 0.15 : 0.25);
                        const vuln = obs.vulnerabilityWeight || 1.0;

                        if (lateralClearance < 0.04) {
                            // Dangerously close grazing (< 4cm)
                            collisionCost += 5000;
                            isFeasible = false;
                            collisionPoint = { x: pt.x, y: pt.y, type: 'OBSTACLE', obstacle: obs };
                            break;
                        } else if (lateralClearance < safetyMargin) {
                            // Proximity buffer penalty (feasible, but penalizes tight passes)
                            collisionCost += Math.pow(safetyMargin - lateralClearance, 2) * 80.0 * vuln;
                        } else if (lateralClearance < 1.2) {
                            // Gentle distance potential
                            collisionCost += Math.pow(1.2 - lateralClearance, 2) * 12.0 * vuln;
                        }
                    }
                }
            }

            if (!isFeasible) break;
        }

        // 3. Smoothness / Lateral Jerk Cost
        const lateralShift = Math.abs(trajectory.bypassX - ego.x);
        const smoothnessCost = Math.pow(lateralShift, 2) * 2.2 + trajectory.maxCurvature * 12.0;

        // 4. Centerline Cost (Soft preference to stay near road center when clear)
        const endPt = pts[pts.length - 1];
        const endBounds = road.getBoundariesAt(endPt.y);
        const centerlineCost = Math.abs(endPt.x - endBounds.centerlineOffset) * 1.8;

        const totalCost = collisionCost + boundaryCost + potholeCost + smoothnessCost + centerlineCost;

        return {
            totalCost: totalCost,
            collision: collisionCost,
            boundary: boundaryCost,
            pothole: potholeCost,
            smoothness: smoothnessCost,
            centerline: centerlineCost,
            isFeasible: isFeasible,
            collisionPoint: collisionPoint
        };
    }
}
