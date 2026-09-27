/**
 * SIH 2026 - Problem Statement: SIH26037
 * Team MARK-One: Adaptive Path Planning on Unstructured Indian Roads
 * Perception: Simulated Multi-Sensor Suite (Camera, LiDAR, Radar)
 */

import { SIM_CONFIG } from '../config.js';

export class SensorSuite {
    constructor() {
        this.lidarConfig = SIM_CONFIG.SENSORS.LIDAR;
        this.radarConfig = SIM_CONFIG.SENSORS.RADAR;
        this.cameraConfig = SIM_CONFIG.SENSORS.CAMERA;

        // Sensor outputs
        this.lidarPoints = [];      // Array of { x, y, range, angle, hit: boolean, obstacleId }
        this.radarDetections = [];  // Array of { id, range, azimuth, relSpeed, snr }
        this.cameraDetections = []; // Array of { id, class, confidence, bbox: {x, y, w, l} }
    }

    /**
     * Executes a full multi-sensor scan cycle relative to ego vehicle.
     * @param {Vehicle} ego Vehicle instance
     * @param {Array<Obstacle>} obstacles Active obstacles in the environment
     * @param {Road} road Unstructured road model
     */
    scan(ego, obstacles, road) {
        this.scanLidar(ego, obstacles, road);
        this.scanRadar(ego, obstacles);
        this.scanCamera(ego, obstacles);
    }

    /**
     * Simulates 64-beam 2D LiDAR raycasting.
     */
    scanLidar(ego, obstacles, road) {
        this.lidarPoints = [];
        const numRays = this.lidarConfig.RAYS;
        const halfFov = this.lidarConfig.FOV / 2;
        const maxRange = this.lidarConfig.RANGE;
        
        // Ray emission angle relative to vehicle heading
        for (let i = 0; i < numRays; i++) {
            const relAngle = -halfFov + (i / (numRays - 1)) * (halfFov * 2);
            const globalAngle = ego.heading + relAngle;
            
            // Ray direction vector
            const rayDirX = Math.sin(globalAngle);
            const rayDirY = Math.cos(globalAngle);
            
            let closestDist = maxRange;
            let hitObstacleId = null;
            let hitType = null;

            // 1. Raycast against all obstacles in range
            for (const obs of obstacles) {
                // Quick bounding circle distance check
                const dx = obs.x - ego.x;
                const dy = obs.y - ego.y;
                const distToObs = Math.hypot(dx, dy);
                if (distToObs > maxRange + 5.0) continue;
                if (obs.isSurfaceDefect) continue; // Potholes/surface defects are ground-level features, not impenetrable vertical walls

                // Approximate obstacle as oriented box or circle
                const radius = Math.max(obs.width, obs.length) / 2;
                const distAlongRay = dx * rayDirX + dy * rayDirY;
                if (distAlongRay > 0 && distAlongRay < closestDist) {
                    const perpDist = Math.abs(dx * (-rayDirY) + dy * rayDirX);
                    if (perpDist < radius) {
                        const hitDist = distAlongRay - Math.sqrt(Math.max(0, radius * radius - perpDist * perpDist));
                        if (hitDist > 0 && hitDist < closestDist) {
                            closestDist = hitDist;
                            hitObstacleId = obs.id;
                            hitType = obs.typeKey;
                        }
                    }
                }
            }

            // 2. Raycast against unpaved road boundaries
            // Sample along ray to check road edge intersection
            for (let r = 5.0; r < closestDist; r += 2.0) {
                const sampleX = ego.x + rayDirX * r;
                const sampleY = ego.y + rayDirY * r;
                const bounds = road.getBoundariesAt(sampleY);
                if (sampleX < bounds.leftEdge || sampleX > bounds.rightEdge) {
                    closestDist = r;
                    hitObstacleId = -1; // Road edge
                    hitType = 'ROAD_BOUNDARY';
                    break;
                }
            }

            // Store point cloud hit point
            const hitX = ego.x + rayDirX * closestDist;
            const hitY = ego.y + rayDirY * closestDist;
            this.lidarPoints.push({
                x: hitX,
                y: hitY,
                range: closestDist,
                relAngle: relAngle,
                globalAngle: globalAngle,
                hit: closestDist < maxRange,
                obstacleId: hitObstacleId,
                hitType: hitType
            });
        }
    }

    /**
     * Simulates 77 GHz Automotive FMCW Radar:
     * High Doppler velocity resolution & distance measurement.
     */
    scanRadar(ego, obstacles) {
        this.radarDetections = [];
        const maxRange = this.radarConfig.RANGE_FAR;
        const fov = this.radarConfig.FOV_FAR;

        for (const obs of obstacles) {
            // Potholes do not produce strong FMCW radar returns (transparent/ground plane)
            if (obs.isSurfaceDefect) continue;

            const dx = obs.x - ego.x;
            const dy = obs.y - ego.y;
            const range = Math.hypot(dx, dy);
            
            if (range <= maxRange && dy > -1.0) {
                // Azimuth angle relative to vehicle heading
                const angleToObs = Math.atan2(dx, dy);
                const relAzimuth = angleToObs - ego.heading;

                if (Math.abs(relAzimuth) <= fov / 2) {
                    // Compute Doppler relative velocity along line-of-sight
                    const unitX = dx / (range || 1);
                    const unitY = dy / (range || 1);
                    
                    const egoVx = ego.speed * Math.sin(ego.heading);
                    const egoVy = ego.speed * Math.cos(ego.heading);
                    
                    const relVx = obs.vx - egoVx;
                    const relVy = obs.vy - egoVy;
                    const dopplerSpeed = relVx * unitX + relVy * unitY; // negative = closing in

                    // Signal-to-noise ratio simulation
                    const snr = Math.max(12, 45 - (range / maxRange) * 25);

                    this.radarDetections.push({
                        id: obs.id,
                        range: range,
                        azimuth: relAzimuth,
                        relSpeed: dopplerSpeed,
                        snr: snr,
                        type: obs.typeKey
                    });
                }
            }
        }
    }

    /**
     * Simulates 4K Vision Camera detection & semantic classification.
     */
    scanCamera(ego, obstacles) {
        this.cameraDetections = [];
        const maxRange = this.cameraConfig.RANGE;
        const fov = this.cameraConfig.FOV;

        for (const obs of obstacles) {
            const dx = obs.x - ego.x;
            const dy = obs.y - ego.y;
            const range = Math.hypot(dx, dy);

            if (range <= maxRange && dy > -0.5) {
                const angleToObs = Math.atan2(dx, dy);
                const relAzimuth = angleToObs - ego.heading;

                if (Math.abs(relAzimuth) <= fov / 2) {
                    // Realistic simulated neural network classification confidence
                    // Decreases slightly with range and environmental complexity
                    const baseConfidence = 0.96 - (range / maxRange) * 0.15;
                    const confidence = Math.max(0.75, Math.min(0.99, baseConfidence + (Math.sin(obs.id) * 0.03)));

                    this.cameraDetections.push({
                        id: obs.id,
                        class: obs.typeKey,
                        className: obs.name,
                        confidence: confidence,
                        range: range,
                        azimuth: relAzimuth,
                        isBiological: obs.isBiological,
                        isSurfaceDefect: obs.isSurfaceDefect,
                        x: obs.x,
                        y: obs.y,
                        width: obs.width,
                        length: obs.length
                    });
                }
            }
        }
    }
}
