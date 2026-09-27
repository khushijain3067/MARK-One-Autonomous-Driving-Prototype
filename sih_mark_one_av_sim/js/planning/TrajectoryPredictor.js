/**
 * SIH 2026 - Problem Statement: SIH26037
 * Team MARK-One: Adaptive Path Planning on Unstructured Indian Roads
 * Planning: Trajectory Prediction Engine
 * 
 * Short-term spatiotemporal motion forecasting for irregular and unpredictable
 * Indian road users (pedestrians, weaving bikes, wandering cattle, slow autos).
 */

export class TrajectoryPredictor {
    constructor() {
        this.horizon = 3.0; // 3.0 second forecasting horizon
        this.timeStep = 0.5; // Evaluate at 0.5s intervals
    }

    /**
     * Forecasts future positions for all tracked objects across the horizon.
     * @param {Array<Object>} trackedObjects Fused obstacle tracks
     * @returns {Map<number, Array<{time: number, x: number, y: number}>>}
     */
    predict(trackedObjects) {
        const predictions = new Map();

        for (const track of trackedObjects) {
            const trajectory = [];
            const steps = Math.round(this.horizon / this.timeStep);

            for (let i = 1; i <= steps; i++) {
                const t = i * this.timeStep;
                let predX = track.x;
                let predY = track.y;

                if (track.isSurfaceDefect || track.speed === 0) {
                    // Static obstacles & potholes do not move
                    predX = track.x;
                    predY = track.y;
                } else if (track.type === 'PEDESTRIAN') {
                    // Pedestrian informal crossing progression
                    predX = track.x + (track.vx || (track.heading ? Math.sin(track.heading) * track.speed : 0)) * t;
                    predY = track.y + (track.vy || 0) * t;
                } else if (track.type === 'MOTORCYCLE') {
                    // Two-wheeler with forward progress and slight lateral variance
                    predX = track.x + Math.sin(t * 2.0) * 0.4;
                    predY = track.y + (track.vy || track.speed) * t;
                } else if (track.type === 'CATTLE') {
                    // Slow meander
                    predX = track.x + (track.vx || 0) * t * 0.7;
                    predY = track.y + (track.vy || 0) * t * 0.7;
                } else {
                    // Constant velocity projection
                    predX = track.x + (track.vx || 0) * t;
                    predY = track.y + (track.vy || track.speed) * t;
                }

                trajectory.push({ time: t, x: predX, y: predY });
            }

            track.predictedTrajectory = trajectory;
            predictions.set(track.id, trajectory);
        }

        return predictions;
    }
}
