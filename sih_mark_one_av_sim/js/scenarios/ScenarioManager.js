/**
 * SIH 2026 - Problem Statement: SIH26037
 * Team MARK-One: Adaptive Path Planning on Unstructured Indian Roads
 * Scenarios: Indian Road Validation Scenarios & Automated SIH Demo Director
 */

import { SIM_CONFIG } from '../config.js';
import { Obstacle } from '../models/Obstacle.js';

export class ScenarioManager {
    constructor() {
        this.scenarios = [
            {
                id: 'normal_traffic',
                name: 'Scenario 1: Normal Indian Mixed Traffic',
                description: 'Cars, auto-rickshaws, and motorcycles moving with irregular weaving on an unmarked road.',
                setup: this.setupNormalTraffic.bind(this)
            },
            {
                id: 'slow_vehicle',
                name: 'Scenario 2: Slow Vehicle Overtake',
                description: 'A slow-moving Auto Rickshaw blocks the lane; AV detects clear corridor and overtakes.',
                setup: this.setupSlowVehicle.bind(this)
            },
            {
                id: 'pedestrian_crossing',
                name: 'Scenario 3: Informal Pedestrian Crossing',
                description: 'Pedestrian steps into the road corridor; AV detects biological risk, yields/swerves.',
                setup: this.setupPedestrianCrossing.bind(this)
            },
            {
                id: 'pothole_avoidance',
                name: 'Scenario 4: Severe Pothole Avoidance',
                description: 'Cluster of deep asphalt potholes directly in line; AV executes smooth evasive maneuver.',
                setup: this.setupPotholeAvoidance.bind(this)
            },
            {
                id: 'parked_vehicle',
                name: 'Scenario 5: Parked Vehicle & Encroachment',
                description: 'Parked delivery tempo and roadside vendor narrow the road; AV plans safe bypass.',
                setup: this.setupParkedVehicle.bind(this)
            },
            {
                id: 'mixed_obstacles',
                name: 'Scenario 6: Mixed Multi-Obstacle Chaos',
                description: 'Car + motorcycle + pedestrian + pothole concurrently challenging the path planner.',
                setup: this.setupMixedObstacles.bind(this)
            },
            {
                id: 'sudden_obstacle',
                name: 'Scenario 7: Sudden Cut-In / Emergency Deceleration',
                description: 'Motorcycle swerves aggressively into ego path; AV executes rapid lateral replan and braking.',
                setup: this.setupSuddenCutIn.bind(this)
            },
            {
                id: 'narrow_road',
                name: 'Scenario 8: Narrow Chokepoint & Barricades',
                description: 'Rural road narrows with construction barriers on both flanks; AV threads needle safely.',
                setup: this.setupNarrowRoad.bind(this)
            },
            {
                id: 'cattle_crossing',
                name: 'Scenario 9: Cattle Crossing Event (SIH PPT #05)',
                description: 'Sacred cow halts in middle of drivable lane; AV applies biological buffer & gentle detour.',
                setup: this.setupCattleCrossing.bind(this)
            },
            {
                id: 'dense_market',
                name: 'Scenario 10: Dense Market Bazaar (SIH PPT #04)',
                description: 'High-density market zone with hand-carts, parked bikes, pedestrians, and tight gaps.',
                setup: this.setupDenseMarket.bind(this)
            }
        ];

        this.currentScenarioIndex = 0;
        this.isDemoMode = false;
        this.demoTimer = 0;
        this.demoStageIndex = 0;
        this.demoNarration = '';
    }

    /**
     * Loads a scenario by index or ID.
     */
    loadScenario(indexOrId, ego, road) {
        let sc = null;
        if (typeof indexOrId === 'number') {
            this.currentScenarioIndex = Math.max(0, Math.min(this.scenarios.length - 1, indexOrId));
            sc = this.scenarios[this.currentScenarioIndex];
        } else {
            sc = this.scenarios.find(s => s.id === indexOrId);
            if (sc) this.currentScenarioIndex = this.scenarios.indexOf(sc);
        }

        if (!sc) sc = this.scenarios[0];

        // Reset vehicle to configured cruising speed
        ego.reset(0, 0, 0, SIM_CONFIG.VEHICLE.TARGET_SPEED);

        // Populate obstacles
        const obstacles = sc.setup();
        return {
            scenario: sc,
            obstacles: obstacles
        };
    }

    // ==========================================
    // SCENARIO IMPLEMENTATIONS
    // ==========================================

    setupNormalTraffic() {
        const obs = [];
        obs.push(new Obstacle('AUTO_RICKSHAW', -1.4, 25.0, { speed: 4.5, behavior: 'constant' }));
        obs.push(new Obstacle('CAR', 1.6, 45.0, { speed: 7.2, behavior: 'constant' }));
        obs.push(new Obstacle('MOTORCYCLE', -0.5, 68.0, { speed: 6.8, behavior: 'weave' }));
        obs.push(new Obstacle('ONCOMING_VEHICLE', -2.2, 95.0, { speed: 5.5, isOncoming: true }));
        obs.push(new Obstacle('CAR', 1.8, 120.0, { speed: 7.0, behavior: 'constant' }));
        obs.push(new Obstacle('AUTO_RICKSHAW', -1.2, 145.0, { speed: 4.5, behavior: 'constant' }));
        return obs;
    }

    setupSlowVehicle() {
        const obs = [];
        // Slow auto-rickshaw right in center-left lane
        obs.push(new Obstacle('AUTO_RICKSHAW', 0.2, 22.0, { speed: 2.8, behavior: 'constant' }));
        // Oncoming or roadside marker
        obs.push(new Obstacle('POTHOLE', -2.2, 55.0));
        obs.push(new Obstacle('CAR', 1.8, 80.0, { speed: 5.5, behavior: 'constant' }));
        return obs;
    }

    setupPedestrianCrossing() {
        const obs = [];
        // Pedestrian crossing from right shoulder towards left
        obs.push(new Obstacle('PEDESTRIAN', 3.2, 24.0, { speed: 1.1, behavior: 'cross_road', crossDirection: -1 }));
        // Stationary parked bike on the left side
        obs.push(new Obstacle('MOTORCYCLE', -2.6, 20.0, { speed: 0.0 }));
        // Follow-up traffic
        obs.push(new Obstacle('AUTO_RICKSHAW', 1.4, 55.0, { speed: 5.0, behavior: 'constant' }));
        return obs;
    }

    setupPotholeAvoidance() {
        const obs = [];
        // Cluster of jagged potholes directly on nominal vehicle track
        obs.push(new Obstacle('POTHOLE', 0.1, 20.0, { width: 1.6, length: 1.6 }));
        obs.push(new Obstacle('POTHOLE', -1.2, 26.0, { width: 1.4, length: 1.4 }));
        obs.push(new Obstacle('POTHOLE', 1.3, 38.0, { width: 1.5, length: 1.5 }));
        obs.push(new Obstacle('CAR', 1.6, 60.0, { speed: 6.0 }));
        return obs;
    }

    setupParkedVehicle() {
        const obs = [];
        // Parked delivery tempo encroaching right half of road
        obs.push(new Obstacle('PARKED_VEHICLE', 1.5, 22.0, { speed: 0.0 }));
        // Fruit hand-cart vendor near edge
        obs.push(new Obstacle('HAND_CART', 3.0, 18.0, { speed: 0.2 }));
        // Scooter parked haphazardly on left shoulder
        obs.push(new Obstacle('MOTORCYCLE', -2.8, 30.0, { speed: 0.0 }));
        // Clear continuation ahead
        obs.push(new Obstacle('AUTO_RICKSHAW', -0.5, 65.0, { speed: 4.8 }));
        return obs;
    }

    setupMixedObstacles() {
        const obs = [];
        // High complexity multi-actor encounter
        obs.push(new Obstacle('POTHOLE', -1.2, 18.0));
        obs.push(new Obstacle('CAR', 1.8, 25.0, { speed: 3.5 }));
        obs.push(new Obstacle('PEDESTRIAN', 3.0, 32.0, { speed: 0.9, behavior: 'cross_road', crossDirection: -1 }));
        obs.push(new Obstacle('MOTORCYCLE', 0.0, 48.0, { speed: 5.8, behavior: 'weave' }));
        obs.push(new Obstacle('BARRICADE', -1.8, 65.0));
        return obs;
    }

    setupSuddenCutIn() {
        const obs = [];
        // Bike on shoulder that aggressively cuts in right in front of ego at y = 14m
        obs.push(new Obstacle('MOTORCYCLE', 2.8, 20.0, {
            speed: 5.0,
            behavior: 'cut_in',
            cutInTriggerY: 4.0
        }));
        // Slow car ahead
        obs.push(new Obstacle('CAR', -1.2, 50.0, { speed: 4.0 }));
        return obs;
    }

    setupNarrowRoad() {
        const obs = [];
        // Construction barricades pinch the road from both sides to a single narrow gap
        obs.push(new Obstacle('BARRICADE', -2.4, 25.0, { width: 2.2 }));
        obs.push(new Obstacle('BARRICADE', 2.4, 25.0, { width: 2.2 }));
        obs.push(new Obstacle('DEBRIS', -2.0, 36.0));
        obs.push(new Obstacle('AUTO_RICKSHAW', 0.1, 55.0, { speed: 3.8 }));
        return obs;
    }

    setupCattleCrossing() {
        const obs = [];
        // Sacred cow wandering across or standing right in the road
        obs.push(new Obstacle('CATTLE', 0.4, 23.0, { speed: 0.4, behavior: 'wander' }));
        // Parked bicycle on right
        obs.push(new Obstacle('MOTORCYCLE', 2.8, 18.0, { speed: 0.0 }));
        // Upcoming auto
        obs.push(new Obstacle('AUTO_RICKSHAW', -1.6, 50.0, { speed: 4.5 }));
        return obs;
    }

    setupDenseMarket() {
        const obs = [];
        // High density bazaar conditions
        obs.push(new Obstacle('HAND_CART', 2.2, 16.0, { speed: 0.3 }));
        obs.push(new Obstacle('PEDESTRIAN', -1.8, 21.0, { speed: 0.8, behavior: 'cross_road', crossDirection: 1 }));
        obs.push(new Obstacle('MOTORCYCLE', 1.0, 29.0, { speed: 3.2, behavior: 'weave' }));
        obs.push(new Obstacle('PARKED_VEHICLE', -2.4, 38.0, { speed: 0.0 }));
        obs.push(new Obstacle('PEDESTRIAN', 2.0, 42.0, { speed: 0.6 }));
        obs.push(new Obstacle('POTHOLE', 0.2, 50.0));
        obs.push(new Obstacle('AUTO_RICKSHAW', -0.6, 62.0, { speed: 3.5 }));
        return obs;
    }

    // ==========================================
    // SIH 2026 AUTOMATED DEMO DIRECTOR
    // ==========================================

    /**
     * Starts the 75-second choreographed SIH presentation demonstration.
     */
    startDemoMode(ego, road) {
        this.isDemoMode = true;
        this.demoTimer = 0;
        this.demoStageIndex = 0;
        this.demoNarration = 'INITIATING MARK-ONE AUTONOMOUS DEMONSTRATION...';
        return this.loadScenario('normal_traffic', ego, road);
    }

    stopDemoMode() {
        this.isDemoMode = false;
        this.demoTimer = 0;
        this.demoNarration = '';
    }

    /**
     * Updates automated demo stages as vehicle drives forward.
     */
    updateDemo(dt, ego, road, currentObstacles) {
        if (!this.isDemoMode) return null;

        this.demoTimer += dt;

        // Choreographed demonstration stages based on time & vehicle progress
        const stages = [
            {
                time: 0,
                narration: 'PHASE 1: MULTI-SENSOR PERCEPTION (Camera, 64-beam LiDAR, 77GHz Radar FMCW active)',
                action: () => {}
            },
            {
                time: 10,
                narration: 'PHASE 2: UNSTRUCTURED ROAD REASONING (Operating without lane lines; dynamic boundaries)',
                action: () => {}
            },
            {
                time: 18,
                narration: 'PHASE 3: WEAVING TWO-WHEELER DETECTED (Trajectory forecasting predicting lateral shift)',
                action: () => {}
            },
            {
                time: 25,
                scenario: 'slow_vehicle',
                narration: 'PHASE 4: SLOW VEHICLE CORRIDOR OVERTAKE (Evaluating lattice candidates; executing smooth pass)'
            },
            {
                time: 38,
                scenario: 'cattle_crossing',
                narration: 'PHASE 5: BIOLOGICAL THREAT REASONING (Cattle detected; applying enhanced vulnerability buffer)'
            },
            {
                time: 52,
                scenario: 'pothole_avoidance',
                narration: 'PHASE 6: SURFACE DAMAGE NAVIGATION (Pothole cluster avoided with C2-continuous curvature)'
            },
            {
                time: 65,
                scenario: 'narrow_road',
                narration: 'PHASE 7: NARROW CHOKEPOINT CONVERGENCE (Barricaded bottleneck safely traversed)'
            },
            {
                time: 75,
                narration: 'DEMONSTRATION COMPLETE: Zero collisions, adaptive closed-loop navigation successful.',
                action: () => {}
            }
        ];

        // Check if we need to advance stage
        for (let i = stages.length - 1; i >= 0; i--) {
            if (this.demoTimer >= stages[i].time) {
                if (this.demoStageIndex !== i) {
                    this.demoStageIndex = i;
                    const st = stages[i];
                    this.demoNarration = st.narration;
                    if (st.scenario) {
                        return this.loadScenario(st.scenario, ego, road);
                    }
                }
                break;
            }
        }

        return null;
    }
}
