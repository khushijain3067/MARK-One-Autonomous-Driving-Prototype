/**
 * SIH 2026 - Problem Statement: SIH26037
 * Team MARK-One: Adaptive Path Planning on Unstructured Indian Roads
 * Perception: Multi-Sensor Fusion Engine
 * 
 * Fuses Camera (classification), LiDAR (accurate geometry), and Radar (Doppler velocity)
 * into a single unified track list for Risk Assessment and Planning.
 */

export class SensorFusionEngine {
    constructor() {
        this.trackedObjects = new Map(); // id -> TrackedObject
        this.fusionStats = {
            fusedTracksCount: 0,
            activeSensors: ['Camera', 'LiDAR', 'Radar'],
            averageConfidence: 0.94
        };
    }

    /**
     * Executes the Sensor Fusion pipeline on current sensor outputs.
     * @param {SensorSuite} sensors Sensor suite instance
     * @param {Vehicle} ego Ego vehicle state
     * @param {Array<Obstacle>} rawObstacles Ground truth references for position fidelity
     */
    fuse(sensors, ego, rawObstacles) {
        const currentFrameTracks = new Map();
        let totalConfidence = 0;

        // Map raw obstacles for state association
        const obsMap = new Map();
        for (const o of rawObstacles) {
            obsMap.set(o.id, o);
        }

        // 1. Process Vision Camera detections (Semantic baseline)
        for (const cam of sensors.cameraDetections) {
            const raw = obsMap.get(cam.id);
            if (!raw) continue;

            currentFrameTracks.set(cam.id, {
                id: cam.id,
                type: cam.class,
                name: cam.className,
                x: raw.x,
                y: raw.y,
                width: raw.width,
                length: raw.length,
                speed: raw.speed,
                vx: raw.vx,
                vy: raw.vy,
                heading: raw.heading,
                isBiological: raw.isBiological,
                isSurfaceDefect: raw.isSurfaceDefect,
                vulnerabilityWeight: raw.vulnerabilityWeight,
                distance: cam.range,
                confidence: cam.confidence,
                sensors: ['CAMERA'],
                riskLevel: 'LOW',
                ttc: Infinity,
                clearance: 99.0
            });
        }

        // 2. Fuse Radar detections (High-precision velocity & closing speed)
        for (const rad of sensors.radarDetections) {
            let track = currentFrameTracks.get(rad.id);
            if (track) {
                track.sensors.push('RADAR');
                track.relSpeed = rad.relSpeed;
                // Boost confidence with dual-modality confirmation
                track.confidence = Math.min(0.99, track.confidence + 0.05);
            } else {
                const raw = obsMap.get(rad.id);
                if (raw) {
                    currentFrameTracks.set(rad.id, {
                        id: rad.id,
                        type: raw.typeKey,
                        name: raw.name,
                        x: raw.x,
                        y: raw.y,
                        width: raw.width,
                        length: raw.length,
                        speed: raw.speed,
                        vx: raw.vx,
                        vy: raw.vy,
                        heading: raw.heading,
                        isBiological: raw.isBiological,
                        isSurfaceDefect: raw.isSurfaceDefect,
                        vulnerabilityWeight: raw.vulnerabilityWeight,
                        distance: rad.range,
                        confidence: 0.86,
                        sensors: ['RADAR'],
                        riskLevel: 'LOW',
                        ttc: Infinity,
                        clearance: 99.0,
                        relSpeed: rad.relSpeed
                    });
                }
            }
        }

        // 3. Fuse LiDAR points (Precise boundary detection & geometric confirmation)
        const lidarHitObstacleIds = new Set();
        for (const pt of sensors.lidarPoints) {
            if (pt.hit && pt.obstacleId && pt.obstacleId > 0) {
                lidarHitObstacleIds.add(pt.obstacleId);
            }
        }

        for (const obsId of lidarHitObstacleIds) {
            let track = currentFrameTracks.get(obsId);
            if (track) {
                if (!track.sensors.includes('LIDAR')) {
                    track.sensors.push('LIDAR');
                    track.confidence = Math.min(0.99, track.confidence + 0.04);
                }
            } else {
                const raw = obsMap.get(obsId);
                if (raw) {
                    currentFrameTracks.set(obsId, {
                        id: obsId,
                        type: raw.typeKey,
                        name: raw.name,
                        x: raw.x,
                        y: raw.y,
                        width: raw.width,
                        length: raw.length,
                        speed: raw.speed,
                        vx: raw.vx,
                        vy: raw.vy,
                        heading: raw.heading,
                        isBiological: raw.isBiological,
                        isSurfaceDefect: raw.isSurfaceDefect,
                        vulnerabilityWeight: raw.vulnerabilityWeight,
                        distance: Math.hypot(raw.x - ego.x, raw.y - ego.y),
                        confidence: 0.88,
                        sensors: ['LIDAR'],
                        riskLevel: 'LOW',
                        ttc: Infinity,
                        clearance: 99.0
                    });
                }
            }
        }

        // 4. Calculate relative spatial metrics relative to Ego
        const fusedList = [];
        for (const [id, track] of currentFrameTracks.entries()) {
            const dx = track.x - ego.x;
            const dy = track.y - ego.y;
            track.distance = Math.hypot(dx, dy);
            track.lateralOffset = dx;
            track.longitudinalDist = dy;

            // Relative speed closing rate
            const relVy = ego.speed - track.vy;
            track.relVy = relVy;

            // Time To Collision (TTC)
            if (relVy > 0.3 && dy > 0) {
                track.ttc = dy / relVy;
            } else {
                track.ttc = 999.0;
            }

            fusedList.push(track);
            totalConfidence += track.confidence;
        }

        // Sort by distance (closest first)
        fusedList.sort((a, b) => a.distance - b.distance);

        this.fusionStats.fusedTracksCount = fusedList.length;
        this.fusionStats.averageConfidence = fusedList.length > 0
            ? (totalConfidence / fusedList.length)
            : 0.95;

        return fusedList;
    }
}
