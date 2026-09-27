/**
 * SIH 2026 - Problem Statement: SIH26037
 * Team MARK-One: Adaptive Path Planning on Unstructured Indian Roads
 * UI: High-DPI Real-time Canvas Rendering Engine (Production Automotive Engineering Style)
 */

import { SIM_CONFIG } from '../config.js';

export class Renderer {
    constructor(canvas) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.scale = SIM_CONFIG.PIXELS_PER_METER; // pixels per meter
        this.zoom = 1.0;
        
        // Display toggles
        this.showLidar = true;
        this.showRadar = true;
        this.showCameraFov = true;
        this.showCandidates = true;
        this.showSafeCorridor = true;
        this.showBoundingBoxes = true;
        this.showRiskTags = true;
        this.showPredictedTraj = true;

        // Interactive spawn mode
        this.spawnMode = null;

        this.handleResize();
        window.addEventListener('resize', () => this.handleResize());
    }

    handleResize() {
        const rect = this.canvas.parentElement.getBoundingClientRect();
        const dpr = window.devicePixelRatio || 1;
        this.width = rect.width;
        this.height = rect.height;
        this.canvas.width = this.width * dpr;
        this.canvas.height = this.height * dpr;
        this.ctx.resetTransform();
        this.ctx.scale(dpr, dpr);
    }

    /**
     * Converts world coordinates (meters) to screen coordinates (pixels).
     * Forward motion is upwards on the screen.
     */
    worldToScreen(worldX, worldY, egoX, egoY) {
        const currentScale = this.scale * this.zoom;
        const screenX = (this.width / 2) + (worldX - egoX) * currentScale;
        const screenY = (this.height * 0.72) - (worldY - egoY) * currentScale;
        return { x: screenX, y: screenY };
    }

    /**
     * Converts screen coordinates (pixels) back to world coordinates (meters).
     */
    screenToWorld(screenX, screenY, egoX, egoY) {
        const currentScale = this.scale * this.zoom;
        const worldX = egoX + (screenX - (this.width / 2)) / currentScale;
        const worldY = egoY - (screenY - (this.height * 0.72)) / currentScale;
        return { x: worldX, y: worldY };
    }

    /**
     * Master Render Frame.
     */
    render(simState) {
        const { ego, road, obstacles, sensors, fusedTracks, plannerResult, riskState, demoNarration } = simState;
        const ctx = this.ctx;

        // Clear canvas with clean neutral terrain background
        ctx.fillStyle = '#D6D3D1'; // Neutral light soil/earth terrain
        ctx.fillRect(0, 0, this.width, this.height);

        // 1. Draw Road Shoulders & Asphalt Roadway
        this.drawUnstructuredRoad(ctx, road, ego);

        // 2. Draw Roadside Elements (Tea stalls, trees, milestone stones)
        this.drawRoadsideScenery(ctx, road, ego);

        // 3. Draw Sensor Fields (Camera frustum, Radar cones)
        if (this.showCameraFov) this.drawCameraFov(ctx, sensors, ego);
        if (this.showRadar) this.drawRadarCones(ctx, sensors, ego);

        // 4. Draw LiDAR Scan Rays & Point Cloud Hits
        if (this.showLidar) this.drawLidarScan(ctx, sensors, ego);

        // 5. Draw Candidate Trajectories (Lattice tree)
        if (this.showCandidates && plannerResult.candidateTrajectories) {
            this.drawCandidateTrajectories(ctx, plannerResult.candidateTrajectories, ego);
        }

        // 6. Draw Selected Optimal Trajectory & Safe Corridor
        if (plannerResult.selectedTrajectory) {
            this.drawSelectedTrajectory(ctx, plannerResult.selectedTrajectory, ego, plannerResult.planningStatus);
        }

        // 7. Draw Vehicle Historical Breadcrumb Trail
        this.drawVehicleHistory(ctx, ego);

        // 8. Draw Obstacles (Vehicles, Cattle, Potholes, Pedestrians)
        this.drawObstacles(ctx, obstacles, fusedTracks, ego);

        // 9. Draw Autonomous Ego Vehicle
        this.drawEgoVehicle(ctx, ego);

        // 10. Draw Pure Pursuit Lookahead Target Point
        if (simState.controller && simState.controller.targetPoint) {
            this.drawLookaheadTarget(ctx, simState.controller.targetPoint, ego);
        }

        // 11. Draw On-Screen Engineering Decision Badge & Demo Narration Ticker
        this.drawHudOverlay(ctx, simState);
    }

    /**
     * Draws the unstructured asphalt road with irregular edges and unpaved shoulders.
     */
    drawUnstructuredRoad(ctx, road, ego) {
        const currentScale = this.scale * this.zoom;
        const viewAhead = 75.0; // meters ahead to draw
        const viewBehind = 25.0; // meters behind to draw
        const yStart = Math.floor(ego.y - viewBehind);
        const yEnd = Math.ceil(ego.y + viewAhead);
        const step = 2.0;

        // Outer Dirt/Gravel Shoulders
        ctx.fillStyle = '#78716C'; // Natural compacted dirt/gravel shoulder
        ctx.beginPath();
        for (let y = yStart; y <= yEnd; y += step) {
            const b = road.getBoundariesAt(y);
            const pt = this.worldToScreen(b.leftShoulder, y, ego.x, ego.y);
            if (y === yStart) ctx.moveTo(pt.x, pt.y);
            else ctx.lineTo(pt.x, pt.y);
        }
        for (let y = yEnd; y >= yStart; y -= step) {
            const b = road.getBoundariesAt(y);
            const pt = this.worldToScreen(b.rightShoulder, y, ego.x, ego.y);
            ctx.lineTo(pt.x, pt.y);
        }
        ctx.closePath();
        ctx.fill();

        // Main Irregular Asphalt Roadway (Authentic high-contrast road)
        ctx.fillStyle = '#334155'; // Slate-700 weathered asphalt
        ctx.beginPath();
        for (let y = yStart; y <= yEnd; y += step) {
            const b = road.getBoundariesAt(y);
            const pt = this.worldToScreen(b.leftEdge, y, ego.x, ego.y);
            if (y === yStart) ctx.moveTo(pt.x, pt.y);
            else ctx.lineTo(pt.x, pt.y);
        }
        for (let y = yEnd; y >= yStart; y -= step) {
            const b = road.getBoundariesAt(y);
            const pt = this.worldToScreen(b.rightEdge, y, ego.x, ego.y);
            ctx.lineTo(pt.x, pt.y);
        }
        ctx.closePath();
        ctx.fill();

        // Frayed Asphalt Edges (Degraded Indian road edges)
        ctx.strokeStyle = '#475569';
        ctx.lineWidth = 2.0;
        ctx.stroke();

        // Mud encroaching onto asphalt edges
        ctx.fillStyle = 'rgba(87, 83, 78, 0.45)';
        for (const mud of road.mudPatches) {
            if (mud.y >= yStart && mud.y <= yEnd) {
                const b = road.getBoundariesAt(mud.y);
                const edgeX = (mud.side < 0) ? b.leftEdge : b.rightEdge;
                const pt = this.worldToScreen(edgeX + (mud.side < 0 ? mud.encroach / 2 : -mud.encroach / 2), mud.y, ego.x, ego.y);
                ctx.beginPath();
                ctx.ellipse(pt.x, pt.y, mud.encroach * currentScale, (mud.length / 2) * currentScale, 0, 0, Math.PI * 2);
                ctx.fill();
            }
        }

        // Sporadic Faded Lane Markings
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.35)';
        ctx.lineWidth = 1.8;
        for (const frag of road.laneMarkingFragments) {
            if (frag.yStart >= yStart - 10 && frag.yStart <= yEnd + 10) {
                const b = road.getBoundariesAt(frag.yStart);
                const p1 = this.worldToScreen(b.centerlineOffset + frag.offset, frag.yStart, ego.x, ego.y);
                const p2 = this.worldToScreen(b.centerlineOffset + frag.offset, frag.yStart + frag.length, ego.x, ego.y);
                ctx.save();
                ctx.globalAlpha = frag.opacity;
                ctx.beginPath();
                ctx.moveTo(p1.x, p1.y);
                ctx.lineTo(p2.x, p2.y);
                ctx.stroke();
                ctx.restore();
            }
        }
    }

    /**
     * Draws roadside Indian scenery: banyan trees, tea stalls, milestone markers.
     */
    drawRoadsideScenery(ctx, road, ego) {
        for (const obj of road.roadsideObjects) {
            if (obj.y >= ego.y - 25 && obj.y <= ego.y + 75) {
                const b = road.getBoundariesAt(obj.y);
                const edgeX = (obj.side < 0) ? b.leftShoulder - obj.distFromEdge : b.rightShoulder + obj.distFromEdge;
                const pt = this.worldToScreen(edgeX, obj.y, ego.x, ego.y);

                if (obj.type === 'banyan' || obj.type === 'tree') {
                    // Tree canopy
                    ctx.fillStyle = '#166534';
                    ctx.beginPath();
                    ctx.arc(pt.x, pt.y, 16 * obj.scale, 0, Math.PI * 2);
                    ctx.fill();
                    ctx.fillStyle = '#15803D';
                    ctx.beginPath();
                    ctx.arc(pt.x - 3, pt.y - 3, 11 * obj.scale, 0, Math.PI * 2);
                    ctx.fill();
                } else if (obj.type === 'tea_stall') {
                    // Thatched roof tea dhaba
                    ctx.fillStyle = '#57534E';
                    ctx.fillRect(pt.x - 14, pt.y - 10, 28, 20);
                    ctx.fillStyle = '#D97706';
                    ctx.fillRect(pt.x - 10, pt.y - 12, 20, 4);
                } else if (obj.type === 'milestone') {
                    // Indian milestone (Yellow top, white bottom)
                    ctx.fillStyle = '#EAB308';
                    ctx.beginPath();
                    ctx.arc(pt.x, pt.y - 4, 5, Math.PI, 0);
                    ctx.fill();
                    ctx.fillStyle = '#F8FAFC';
                    ctx.fillRect(pt.x - 5, pt.y - 4, 10, 8);
                } else {
                    // Shrub
                    ctx.fillStyle = '#3F6212';
                    ctx.beginPath();
                    ctx.arc(pt.x, pt.y, 8 * obj.scale, 0, Math.PI * 2);
                    ctx.fill();
                }
            }
        }
    }

    /**
     * Draws Camera vision detection frustum.
     */
    drawCameraFov(ctx, sensors, ego) {
        const egoPt = this.worldToScreen(ego.x, ego.y, ego.x, ego.y);
        const fov = sensors.cameraConfig.FOV;
        const range = sensors.cameraConfig.RANGE * this.scale * this.zoom;

        const leftAngle = -Math.PI / 2 - ego.heading - fov / 2;
        const rightAngle = -Math.PI / 2 - ego.heading + fov / 2;

        ctx.save();
        ctx.fillStyle = 'rgba(37, 99, 235, 0.05)';
        ctx.beginPath();
        ctx.moveTo(egoPt.x, egoPt.y);
        ctx.arc(egoPt.x, egoPt.y, range, leftAngle, rightAngle);
        ctx.closePath();
        ctx.fill();
        ctx.strokeStyle = 'rgba(37, 99, 235, 0.25)';
        ctx.lineWidth = 1;
        ctx.stroke();
        ctx.restore();
    }

    /**
     * Draws 77GHz Radar scan cones (Near & Far range).
     */
    drawRadarCones(ctx, sensors, ego) {
        const egoPt = this.worldToScreen(ego.x, ego.y, ego.x, ego.y);
        ctx.save();

        // Far Range Radar Cone
        const fovFar = sensors.radarConfig.FOV_FAR;
        const rangeFar = sensors.radarConfig.RANGE_FAR * this.scale * this.zoom;
        const leftFar = -Math.PI / 2 - ego.heading - fovFar / 2;
        const rightFar = -Math.PI / 2 - ego.heading + fovFar / 2;

        ctx.fillStyle = 'rgba(217, 119, 6, 0.04)';
        ctx.beginPath();
        ctx.moveTo(egoPt.x, egoPt.y);
        ctx.arc(egoPt.x, egoPt.y, rangeFar, leftFar, rightFar);
        ctx.closePath();
        ctx.fill();
        ctx.strokeStyle = 'rgba(217, 119, 6, 0.20)';
        ctx.lineWidth = 1;
        ctx.stroke();

        // Near Range Wide-Angle Cone
        const fovNear = sensors.radarConfig.FOV_NEAR;
        const rangeNear = sensors.radarConfig.RANGE_NEAR * this.scale * this.zoom;
        const leftNear = -Math.PI / 2 - ego.heading - fovNear / 2;
        const rightNear = -Math.PI / 2 - ego.heading + fovNear / 2;

        ctx.fillStyle = 'rgba(217, 119, 6, 0.06)';
        ctx.beginPath();
        ctx.moveTo(egoPt.x, egoPt.y);
        ctx.arc(egoPt.x, egoPt.y, rangeNear, leftNear, rightNear);
        ctx.closePath();
        ctx.fill();
        ctx.strokeStyle = 'rgba(217, 119, 6, 0.25)';
        ctx.lineWidth = 1;
        ctx.stroke();

        ctx.restore();
    }

    /**
     * Draws 64-beam LiDAR rays and point cloud hits.
     */
    drawLidarScan(ctx, sensors, ego) {
        if (!sensors.lidarPoints) return;
        const egoPt = this.worldToScreen(ego.x, ego.y, ego.x, ego.y);

        ctx.save();
        ctx.lineWidth = 0.8;

        for (const pt of sensors.lidarPoints) {
            const hitPt = this.worldToScreen(pt.x, pt.y, ego.x, ego.y);
            
            // Ray beam: subtle engineering line
            ctx.strokeStyle = pt.hit ? 'rgba(37, 99, 235, 0.15)' : 'rgba(37, 99, 235, 0.04)';
            ctx.beginPath();
            ctx.moveTo(egoPt.x, egoPt.y);
            ctx.lineTo(hitPt.x, hitPt.y);
            ctx.stroke();

            // Point cloud hit dot: clean discrete dots
            if (pt.hit) {
                ctx.fillStyle = (pt.obstacleId && pt.obstacleId > 0) ? '#DC2626' : '#2563EB';
                ctx.beginPath();
                ctx.arc(hitPt.x, hitPt.y, 2.0, 0, Math.PI * 2);
                ctx.fill();
            }
        }
        ctx.restore();
    }

    /**
     * Draws all evaluated candidate trajectories in the lattice search tree.
     */
    drawCandidateTrajectories(ctx, candidates, ego) {
        ctx.save();

        for (const traj of candidates) {
            if (!traj.points || traj.points.length === 0) continue;

            ctx.beginPath();
            const startPt = this.worldToScreen(traj.points[0].x, traj.points[0].y, ego.x, ego.y);
            ctx.moveTo(startPt.x, startPt.y);

            for (let i = 1; i < traj.points.length; i++) {
                const pt = this.worldToScreen(traj.points[i].x, traj.points[i].y, ego.x, ego.y);
                ctx.lineTo(pt.x, pt.y);
            }

            if (traj.isFeasible) {
                // Feasible alternative candidates in subtle dashed blue
                ctx.strokeStyle = 'rgba(37, 99, 235, 0.45)';
                ctx.lineWidth = 1.4;
                ctx.setLineDash([5, 4]);
                ctx.stroke();

                // Small indicator dot at bypass apex
                const midIdx = Math.floor(traj.points.length * 0.45);
                const midPt = this.worldToScreen(traj.points[midIdx].x, traj.points[midIdx].y, ego.x, ego.y);
                ctx.fillStyle = 'rgba(37, 99, 235, 0.7)';
                ctx.beginPath();
                ctx.arc(midPt.x, midPt.y, 2.2, 0, Math.PI * 2);
                ctx.fill();
            } else {
                // Infeasible / blocked candidates in muted red dashed stroke
                ctx.strokeStyle = 'rgba(220, 38, 38, 0.45)';
                ctx.lineWidth = 1.4;
                ctx.setLineDash([4, 4]);
                ctx.stroke();

                // Clean collision indicator if collision point exists
                if (traj.collisionPoint) {
                    const colPt = this.worldToScreen(traj.collisionPoint.x, traj.collisionPoint.y, ego.x, ego.y);
                    ctx.save();
                    ctx.fillStyle = '#DC2626';
                    ctx.strokeStyle = '#FFFFFF';
                    ctx.lineWidth = 1.2;
                    ctx.beginPath();
                    ctx.arc(colPt.x, colPt.y, 4.5, 0, Math.PI * 2);
                    ctx.fill();
                    ctx.stroke();

                    ctx.strokeStyle = '#FFFFFF';
                    ctx.beginPath();
                    ctx.moveTo(colPt.x - 2.5, colPt.y - 2.5);
                    ctx.lineTo(colPt.x + 2.5, colPt.y + 2.5);
                    ctx.moveTo(colPt.x + 2.5, colPt.y - 2.5);
                    ctx.lineTo(colPt.x - 2.5, colPt.y + 2.5);
                    ctx.stroke();
                    ctx.restore();
                }
            }
        }
        ctx.restore();
    }

    /**
     * Draws the selected optimal trajectory and safe driving envelope corridor.
     */
    drawSelectedTrajectory(ctx, trajectory, ego, status) {
        const pts = trajectory.points;
        if (!pts || pts.length === 0) return;

        ctx.save();

        // 1. Safe Driving Envelope Corridor
        if (this.showSafeCorridor) {
            const halfW = (ego.width / 2) * this.scale * this.zoom;
            ctx.fillStyle = (status === 'BLOCKED_STOP')
                ? 'rgba(220, 38, 38, 0.12)'
                : 'rgba(22, 163, 74, 0.12)';
            
            ctx.beginPath();
            // Left boundary of corridor
            for (let i = 0; i < pts.length; i++) {
                const pt = this.worldToScreen(pts[i].x, pts[i].y, ego.x, ego.y);
                const normalX = -Math.cos(pts[i].heading);
                const normalY = Math.sin(pts[i].heading);
                const p = { x: pt.x + normalX * halfW, y: pt.y + normalY * halfW };
                if (i === 0) ctx.moveTo(p.x, p.y);
                else ctx.lineTo(p.x, p.y);
            }
            // Right boundary of corridor
            for (let i = pts.length - 1; i >= 0; i--) {
                const pt = this.worldToScreen(pts[i].x, pts[i].y, ego.x, ego.y);
                const normalX = Math.cos(pts[i].heading);
                const normalY = -Math.sin(pts[i].heading);
                const p = { x: pt.x + normalX * halfW, y: pt.y + normalY * halfW };
                ctx.lineTo(p.x, p.y);
            }
            ctx.closePath();
            ctx.fill();
        }

        // 2. Selected Trajectory Centerline (Clean solid stroke, NO glowing bloom)
        ctx.lineWidth = 3.2;
        ctx.strokeStyle = (status === 'BLOCKED_STOP') ? '#DC2626' : '#16A34A';
        ctx.shadowBlur = 0; // Pure clean line
        ctx.setLineDash([]);

        ctx.beginPath();
        const startPt = this.worldToScreen(pts[0].x, pts[0].y, ego.x, ego.y);
        ctx.moveTo(startPt.x, startPt.y);
        for (let i = 1; i < pts.length; i++) {
            const pt = this.worldToScreen(pts[i].x, pts[i].y, ego.x, ego.y);
            ctx.lineTo(pt.x, pt.y);
        }
        ctx.stroke();

        // 3. Directional Flow Arrows along Path
        ctx.fillStyle = (status === 'BLOCKED_STOP') ? '#DC2626' : '#16A34A';
        for (let i = 4; i < pts.length; i += 6) {
            const pt = this.worldToScreen(pts[i].x, pts[i].y, ego.x, ego.y);
            ctx.save();
            ctx.translate(pt.x, pt.y);
            ctx.rotate(-pts[i].heading);
            ctx.beginPath();
            ctx.moveTo(0, -5);
            ctx.lineTo(4, 3);
            ctx.lineTo(-4, 3);
            ctx.closePath();
            ctx.fill();
            ctx.restore();
        }

        ctx.restore();
    }

    /**
     * Draws history breadcrumb trail behind vehicle.
     */
    drawVehicleHistory(ctx, ego) {
        if (!ego.history || ego.history.length < 2) return;
        ctx.save();
        ctx.strokeStyle = 'rgba(37, 99, 235, 0.35)';
        ctx.lineWidth = 1.6;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        const p0 = this.worldToScreen(ego.history[0].x, ego.history[0].y, ego.x, ego.y);
        ctx.moveTo(p0.x, p0.y);
        for (let i = 1; i < ego.history.length; i++) {
            const p = this.worldToScreen(ego.history[i].x, ego.history[i].y, ego.x, ego.y);
            ctx.lineTo(p.x, p.y);
        }
        ctx.stroke();
        ctx.restore();
    }

    /**
     * Draws all obstacles with authentic Indian graphics and clean detection boxes.
     */
    drawObstacles(ctx, obstacles, fusedTracks, ego) {
        const currentScale = this.scale * this.zoom;
        const trackMap = new Map();
        if (fusedTracks) {
            for (const t of fusedTracks) trackMap.set(t.id, t);
        }

        for (const obs of obstacles) {
            if (obs.y < ego.y - 20 || obs.y > ego.y + 80) continue;

            const pt = this.worldToScreen(obs.x, obs.y, ego.x, ego.y);
            const w = obs.width * currentScale;
            const l = obs.length * currentScale;
            const track = trackMap.get(obs.id);

            ctx.save();
            ctx.translate(pt.x, pt.y);
            ctx.rotate(-obs.heading);

            // Render specific obstacle models
            if (obs.typeKey === 'POTHOLE') {
                this.renderPothole(ctx, w, l);
            } else if (obs.typeKey === 'AUTO_RICKSHAW') {
                this.renderAutoRickshaw(ctx, w, l);
            } else if (obs.typeKey === 'CATTLE') {
                this.renderCattle(ctx, w, l);
            } else if (obs.typeKey === 'PEDESTRIAN') {
                this.renderPedestrian(ctx, w, l);
            } else if (obs.typeKey === 'MOTORCYCLE') {
                this.renderMotorcycle(ctx, w, l);
            } else if (obs.typeKey === 'BARRICADE') {
                this.renderBarricade(ctx, w, l);
            } else if (obs.typeKey === 'HAND_CART') {
                this.renderHandCart(ctx, w, l);
            } else {
                // Standard Car / Truck / Tempo
                this.renderVehicleBox(ctx, w, l, obs.config.color, obs.typeKey);
            }

            ctx.restore();

            // Render Perception HUD Bounding Box & Risk Tag
            if (track && this.showBoundingBoxes) {
                this.renderDetectionHud(ctx, pt, w, l, track);
            }
        }
    }

    renderPothole(ctx, w, l) {
        ctx.fillStyle = '#18181B'; // Deep asphalt depression
        ctx.strokeStyle = '#B45309'; // Fractured rim
        ctx.lineWidth = 1.8;
        ctx.beginPath();
        ctx.ellipse(0, 0, w / 2, l / 2, 0, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();

        // Inner crack lines
        ctx.strokeStyle = '#44403C';
        ctx.beginPath();
        ctx.moveTo(-w * 0.3, 0);
        ctx.lineTo(w * 0.2, l * 0.2);
        ctx.stroke();
    }

    renderAutoRickshaw(ctx, w, l) {
        // Iconic Green body + Yellow roof
        ctx.fillStyle = '#15803D'; // Green lower body
        ctx.fillRect(-w / 2, -l / 2, w, l);

        ctx.fillStyle = '#EAB308'; // Yellow roof
        ctx.fillRect(-w * 0.45, -l * 0.45, w * 0.9, l * 0.7);

        // Windshield
        ctx.fillStyle = '#94A3B8';
        ctx.fillRect(-w * 0.4, -l * 0.45, w * 0.8, 3);

        // Headlight
        ctx.fillStyle = '#FEF08A';
        ctx.fillRect(-3, -l / 2 - 2, 6, 2);
    }

    renderCattle(ctx, w, l) {
        // Sacred Indian Cow (Natural cream hide with horns)
        ctx.fillStyle = '#F8FAFC';
        ctx.beginPath();
        ctx.ellipse(0, 0, w * 0.45, l * 0.45, 0, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = '#78350F';
        ctx.lineWidth = 1.2;
        ctx.stroke();

        // Head
        ctx.fillStyle = '#F1F5F9';
        ctx.beginPath();
        ctx.ellipse(0, -l * 0.42, w * 0.3, l * 0.22, 0, 0, Math.PI * 2);
        ctx.fill();

        // Horns
        ctx.strokeStyle = '#292524';
        ctx.lineWidth = 1.8;
        ctx.beginPath();
        ctx.arc(-w * 0.3, -l * 0.48, 5, 0, Math.PI * 0.7);
        ctx.arc(w * 0.3, -l * 0.48, 5, Math.PI * 0.3, Math.PI);
        ctx.stroke();
    }

    renderPedestrian(ctx, w, l) {
        ctx.fillStyle = '#475569'; // Muted attire
        ctx.beginPath();
        ctx.arc(0, 0, w * 0.55, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = '#FED7AA'; // Head
        ctx.beginPath();
        ctx.arc(0, -2, w * 0.32, 0, Math.PI * 2);
        ctx.fill();
    }

    renderMotorcycle(ctx, w, l) {
        ctx.fillStyle = '#1E293B'; // Bike frame
        ctx.fillRect(-w / 2, -l / 2, w, l);
        // Rider helmet
        ctx.fillStyle = '#D97706';
        ctx.beginPath();
        ctx.arc(0, 0, w * 0.45, 0, Math.PI * 2);
        ctx.fill();
        // Headlight
        ctx.fillStyle = '#FEF08A';
        ctx.fillRect(-2, -l / 2 - 2, 4, 2);
    }

    renderBarricade(ctx, w, l) {
        ctx.fillStyle = '#F59E0B'; // Warning yellow
        ctx.fillRect(-w / 2, -l / 2, w, l);
        ctx.fillStyle = '#18181B'; // Dark hazard stripes
        for (let x = -w / 2; x < w / 2; x += 8) {
            ctx.fillRect(x, -l / 2, 4, l);
        }
    }

    renderHandCart(ctx, w, l) {
        ctx.fillStyle = '#78350F'; // Timber cart
        ctx.fillRect(-w / 2, -l / 2, w, l);
        ctx.fillStyle = '#16A34A'; // Produce
        ctx.fillRect(-w * 0.35, -l * 0.35, w * 0.7, l * 0.7);
    }

    renderVehicleBox(ctx, w, l, color, typeKey) {
        ctx.fillStyle = color;
        ctx.fillRect(-w / 2, -l / 2, w, l);
        // Roof
        ctx.fillStyle = 'rgba(15, 23, 42, 0.35)';
        ctx.fillRect(-w * 0.4, -l * 0.3, w * 0.8, l * 0.5);
        // Windshield
        ctx.fillStyle = '#94A3B8';
        ctx.fillRect(-w * 0.38, -l * 0.35, w * 0.76, 3);
        // Headlights
        ctx.fillStyle = '#FEF08A';
        ctx.fillRect(-w * 0.45, -l / 2, 4, 2);
        ctx.fillRect(w * 0.45 - 4, -l / 2, 4, 2);
    }

    /**
     * Renders engineering HUD target reticles and risk tags around tracked objects.
     */
    renderDetectionHud(ctx, pt, w, l, track) {
        const halfSize = Math.max(w, l) / 2 + 6;
        let riskColor = '#2563EB'; // Safe/Low
        if (track.riskLevel === 'CRITICAL') riskColor = '#DC2626';
        else if (track.riskLevel === 'HIGH') riskColor = '#D97706';
        else if (track.riskLevel === 'MEDIUM') riskColor = '#B45309';

        ctx.save();
        ctx.strokeStyle = riskColor;
        ctx.lineWidth = 1.2;

        // Clean Corner Brackets
        const len = 5;
        // Top-Left
        ctx.beginPath();
        ctx.moveTo(pt.x - halfSize, pt.y - halfSize + len);
        ctx.lineTo(pt.x - halfSize, pt.y - halfSize);
        ctx.lineTo(pt.x - halfSize + len, pt.y - halfSize);
        // Top-Right
        ctx.moveTo(pt.x + halfSize - len, pt.y - halfSize);
        ctx.lineTo(pt.x + halfSize, pt.y - halfSize);
        ctx.lineTo(pt.x + halfSize, pt.y - halfSize + len);
        // Bottom-Right
        ctx.moveTo(pt.x + halfSize, pt.y + halfSize - len);
        ctx.lineTo(pt.x + halfSize, pt.y + halfSize);
        ctx.lineTo(pt.x + halfSize - len, pt.y + halfSize);
        // Bottom-Left
        ctx.moveTo(pt.x - halfSize + len, pt.y + halfSize);
        ctx.lineTo(pt.x - halfSize, pt.y + halfSize);
        ctx.lineTo(pt.x - halfSize, pt.y + halfSize - len);
        ctx.stroke();

        // Metadata Tag Card
        if (this.showRiskTags) {
            ctx.fillStyle = 'rgba(255, 255, 255, 0.96)';
            ctx.fillRect(pt.x + halfSize + 4, pt.y - halfSize, 126, 36);
            ctx.strokeStyle = '#CBD5E1';
            ctx.lineWidth = 1.0;
            ctx.strokeRect(pt.x + halfSize + 4, pt.y - halfSize, 126, 36);

            ctx.font = 'bold 9px "JetBrains Mono", monospace';
            ctx.fillStyle = riskColor;
            ctx.fillText(`${track.name.toUpperCase()} [${track.riskLevel}]`, pt.x + halfSize + 8, pt.y - halfSize + 11);

            ctx.font = '8px "JetBrains Mono", monospace';
            ctx.fillStyle = '#475569';
            ctx.fillText(`Dist: ${track.distance.toFixed(1)}m | TTC: ${(track.ttc < 50 ? track.ttc.toFixed(1) + 's' : 'SAFE')}`, pt.x + halfSize + 8, pt.y - halfSize + 22);
            ctx.fillText(`Sensors: ${track.sensors.join('+')}`, pt.x + halfSize + 8, pt.y - halfSize + 31);
        }

        ctx.restore();
    }

    /**
     * Draws the autonomous ego vehicle (Production Research Vehicle).
     */
    drawEgoVehicle(ctx, ego) {
        const currentScale = this.scale * this.zoom;
        const pt = this.worldToScreen(ego.x, ego.y, ego.x, ego.y);
        const w = ego.width * currentScale;
        const l = ego.length * currentScale;

        ctx.save();
        ctx.translate(pt.x, pt.y);
        ctx.rotate(-ego.heading);

        // 1. Realistic Headlight Beams
        const headlightGrad = ctx.createLinearGradient(0, -l / 2, 0, -l / 2 - 70);
        headlightGrad.addColorStop(0, 'rgba(254, 240, 138, 0.25)');
        headlightGrad.addColorStop(1, 'rgba(254, 240, 138, 0.0)');
        ctx.fillStyle = headlightGrad;
        ctx.beginPath();
        ctx.moveTo(-w * 0.4, -l / 2);
        ctx.lineTo(-w * 1.3, -l / 2 - 70);
        ctx.lineTo(w * 1.3, -l / 2 - 70);
        ctx.lineTo(w * 0.4, -l / 2);
        ctx.closePath();
        ctx.fill();

        // 2. Physical Steered Front Wheels
        const wheelW = 3.5;
        const wheelL = 10;
        const steerAngle = -ego.steeringAngle;

        // Front-Left Wheel
        ctx.save();
        ctx.translate(-w / 2 - 1, -l * 0.32);
        ctx.rotate(steerAngle);
        ctx.fillStyle = '#18181B';
        ctx.fillRect(-wheelW / 2, -wheelL / 2, wheelW, wheelL);
        ctx.restore();

        // Front-Right Wheel
        ctx.save();
        ctx.translate(w / 2 + 1, -l * 0.32);
        ctx.rotate(steerAngle);
        ctx.fillStyle = '#18181B';
        ctx.fillRect(-wheelW / 2, -wheelL / 2, wheelW, wheelL);
        ctx.restore();

        // Rear Wheels (Fixed)
        ctx.fillStyle = '#18181B';
        ctx.fillRect(-w / 2 - 1 - wheelW / 2, l * 0.32 - wheelL / 2, wheelW, wheelL);
        ctx.fillRect(w / 2 + 1 - wheelW / 2, l * 0.32 - wheelL / 2, wheelW, wheelL);

        // 3. Autonomous EV Body (Clean Pearl White Autonomous Research Car)
        ctx.fillStyle = '#F8FAFC'; // Clean pearl white chassis
        ctx.strokeStyle = '#0F172A'; // Subtle dark precision outline
        ctx.lineWidth = 1.5;
        ctx.fillRect(-w / 2, -l / 2, w, l);
        ctx.strokeRect(-w / 2, -l / 2, w, l);

        // Panoramic Dark Glass Cabin
        ctx.fillStyle = '#1E293B';
        ctx.fillRect(-w * 0.4, -l * 0.32, w * 0.8, l * 0.55);

        // Subtle Engineering Accent Badge
        ctx.fillStyle = '#2563EB';
        ctx.fillRect(-w * 0.25, -l / 2 + 3, w * 0.5, 2);

        // 4. Roof-Mounted LiDAR Sensor Dome
        ctx.fillStyle = '#0F172A';
        ctx.beginPath();
        ctx.arc(0, 0, 6.5, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = '#CBD5E1';
        ctx.lineWidth = 1.0;
        ctx.stroke();

        // Sensor indicator dot
        ctx.fillStyle = '#2563EB';
        ctx.beginPath();
        ctx.arc(0, 0, 2.5, 0, Math.PI * 2);
        ctx.fill();

        // 5. Active Brake Lights
        if (ego.brakeLights) {
            ctx.fillStyle = '#DC2626';
            ctx.fillRect(-w * 0.45, l / 2 - 3, w * 0.3, 3);
            ctx.fillRect(w * 0.15, l / 2 - 3, w * 0.3, 3);
        }

        ctx.restore();
    }

    /**
     * Draws Pure Pursuit lookahead point reticle.
     */
    drawLookaheadTarget(ctx, targetPoint, ego) {
        const pt = this.worldToScreen(targetPoint.x, targetPoint.y, ego.x, ego.y);
        ctx.save();
        ctx.strokeStyle = '#2563EB';
        ctx.lineWidth = 1.4;
        ctx.beginPath();
        ctx.arc(pt.x, pt.y, 5, 0, Math.PI * 2);
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(pt.x, pt.y, 2, 0, Math.PI * 2);
        ctx.fillStyle = '#2563EB';
        ctx.fill();
        ctx.restore();
    }

    /**
     * Draws on-screen floating engineering decision badge & demo narration ticker.
     */
    drawHudOverlay(ctx, simState) {
        const { demoNarration, isDemoMode, plannerResult } = simState;

        // SIH Demo Mode Banner
        if (isDemoMode && demoNarration) {
            ctx.save();
            ctx.fillStyle = 'rgba(255, 255, 255, 0.96)';
            ctx.strokeStyle = '#2563EB';
            ctx.lineWidth = 1.2;
            const bannerW = Math.min(this.width - 40, 680);
            const bannerX = (this.width - bannerW) / 2;
            ctx.fillRect(bannerX, 16, bannerW, 44);
            ctx.strokeRect(bannerX, 16, bannerW, 44);

            ctx.font = 'bold 10.5px "JetBrains Mono", monospace';
            ctx.fillStyle = '#2563EB';
            ctx.fillText('SIH 2026 LIVE DEMONSTRATION | TEAM MARK-ONE', bannerX + 16, 32);

            ctx.font = '10px "Inter", sans-serif';
            ctx.fillStyle = '#0F172A';
            ctx.fillText(demoNarration, bannerX + 16, 48);
            ctx.restore();
        }

        // Real-Time Engineering Decision Floating Badge (Top Left of Canvas)
        const threat = plannerResult.primaryThreat;
        const corridors = plannerResult.corridors || { left: { width: 0 }, right: { width: 0 } };
        const side = plannerResult.selectedPathSide || 'CENTER';

        ctx.save();
        if (plannerResult.planningStatus === 'REPLANNING') {
            const cardX = 18;
            const cardY = isDemoMode ? 72 : 18;
            const cardW = 310;
            const cardH = 92;

            // Clean white engineering card
            ctx.fillStyle = 'rgba(255, 255, 255, 0.96)';
            ctx.fillRect(cardX, cardY, cardW, cardH);
            ctx.strokeStyle = '#CBD5E1';
            ctx.lineWidth = 1.0;
            ctx.strokeRect(cardX, cardY, cardW, cardH);

            // Left accent bar
            ctx.fillStyle = '#2563EB';
            ctx.fillRect(cardX, cardY, 4, cardH);

            // Header line
            ctx.font = 'bold 10.5px "JetBrains Mono", monospace';
            ctx.fillStyle = '#D97706';
            ctx.fillText('OBSTACLE DETECTED • REPLANNING TRAJECTORY', cardX + 12, cardY + 18);

            // Threat info
            ctx.font = '10px "JetBrains Mono", monospace';
            ctx.fillStyle = '#0F172A';
            const threatName = threat ? threat.name : 'ROAD HAZARD';
            const threatDist = threat ? `${threat.distance.toFixed(1)}m` : 'AHEAD';
            ctx.fillText(`TARGET: ${threatName.toUpperCase()} at ${threatDist}`, cardX + 12, cardY + 36);

            // Free space info
            ctx.font = '9.5px "JetBrains Mono", monospace';
            ctx.fillStyle = '#475569';
            ctx.fillText(`FREE SPACE: Left ${corridors.left.width.toFixed(1)}m | Right ${corridors.right.width.toFixed(1)}m`, cardX + 12, cardY + 53);

            // Selected Safe Path
            ctx.font = 'bold 10.5px "JetBrains Mono", monospace';
            ctx.fillStyle = '#15803D';
            ctx.fillText(`SAFE PATH: ${side} CORRIDOR SELECTED`, cardX + 12, cardY + 70);

            // Risk status
            ctx.font = '9px "JetBrains Mono", monospace';
            ctx.fillStyle = '#2563EB';
            ctx.fillText(`COLLISION RISK: MINIMAL (OPTIMAL LATTICE S-CURVE)`, cardX + 12, cardY + 84);

        } else if (plannerResult.planningStatus === 'BLOCKED_STOP') {
            const cardX = 18;
            const cardY = isDemoMode ? 72 : 18;
            const cardW = 320;
            const cardH = 70;

            ctx.fillStyle = 'rgba(254, 242, 242, 0.96)';
            ctx.fillRect(cardX, cardY, cardW, cardH);
            ctx.strokeStyle = '#FECACA';
            ctx.lineWidth = 1.0;
            ctx.strokeRect(cardX, cardY, cardW, cardH);
            ctx.fillStyle = '#DC2626';
            ctx.fillRect(cardX, cardY, 4, cardH);

            ctx.font = 'bold 10.5px "JetBrains Mono", monospace';
            ctx.fillStyle = '#DC2626';
            ctx.fillText('ROAD BLOCKED • WAITING FOR SAFE PATH', cardX + 12, cardY + 22);

            ctx.font = '9.5px "Inter", sans-serif';
            ctx.fillStyle = '#991B1B';
            ctx.fillText('All Traversable Corridors Obstructed • Safe Halt Engaged', cardX + 12, cardY + 40);
            ctx.fillText('Scanning for clearance to resume forward trajectory...', cardX + 12, cardY + 56);

        } else {
            // Nominal state badge
            const cardX = 18;
            const cardY = isDemoMode ? 72 : 18;
            ctx.fillStyle = 'rgba(255, 255, 255, 0.94)';
            ctx.fillRect(cardX, cardY, 270, 26);
            ctx.strokeStyle = '#E2E8F0';
            ctx.lineWidth = 1.0;
            ctx.strokeRect(cardX, cardY, 270, 26);
            ctx.fillStyle = '#16A34A';
            ctx.fillRect(cardX, cardY, 3, 26);

            ctx.font = 'bold 9.5px "JetBrains Mono", monospace';
            ctx.fillStyle = '#15803D';
            ctx.fillText('PATH CLEAR • NOMINAL CRUISING (42 km/h)', cardX + 10, cardY + 17);
        }
        ctx.restore();
    }
}
