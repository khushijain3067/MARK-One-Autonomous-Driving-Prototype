/**
 * SIH 2026 - Problem Statement: SIH26037
 * Team MARK-One: Adaptive Path Planning on Unstructured Indian Roads
 * Main Simulation Orchestrator (Closed-Loop Autonomous Driving Architecture)
 */

import { SIM_CONFIG } from './config.js';
import { Road } from './models/Road.js';
import { Vehicle } from './models/Vehicle.js';
import { Obstacle } from './models/Obstacle.js';
import { SensorSuite } from './perception/Sensors.js';
import { SensorFusionEngine } from './perception/SensorFusion.js';
import { TrajectoryPredictor } from './planning/TrajectoryPredictor.js';
import { RiskAssessmentEngine } from './risk/RiskAssessment.js';
import { LatticePlanner } from './planning/LatticePlanner.js';
import { VehicleController } from './control/VehicleController.js';
import { ScenarioManager } from './scenarios/ScenarioManager.js';
import { Renderer } from './ui/Renderer.js';
import { Dashboard } from './ui/Dashboard.js';

export class Simulation {
    constructor() {
        this.canvas = document.getElementById('sim-canvas');
        this.isRunning = true;
        this.simSpeedMultiplier = 1.0;
        this.lastTimestamp = performance.now();
        
        // Planning cycle rate limiter (replan at 20 Hz, animate physics at 60 Hz)
        this.timeSinceLastPlan = 0;
        this.planInterval = 1 / SIM_CONFIG.PLANNING_FREQUENCY;

        // Interactive obstacle placement mode
        this.spawnMode = null;

        // Initialize Core Subsystems
        this.road = new Road();
        this.ego = new Vehicle(0, 0, 0);
        this.sensors = new SensorSuite();
        this.sensorFusion = new SensorFusionEngine();
        this.trajectoryPredictor = new TrajectoryPredictor();
        this.riskEngine = new RiskAssessmentEngine();
        this.planner = new LatticePlanner();
        this.controller = new VehicleController();
        this.scenarioManager = new ScenarioManager();
        this.renderer = new Renderer(this.canvas);

        // Simulation State Cache
        this.obstacles = [];
        this.fusedTracks = [];
        this.plannerResult = {
            selectedTrajectory: null,
            candidateTrajectories: [],
            planningStatus: 'ACTIVE',
            activeDecision: 'INITIALIZING',
            costBreakdown: {}
        };
        this.riskState = {
            overallRiskLevel: 'SAFE',
            highestRiskTrack: null,
            activeThreatsCount: 0
        };

        // Initialize UI Dashboard
        this.dashboard = new Dashboard({
            onTogglePlayPause: () => this.togglePlayPause(),
            onStep: () => this.stepOnce(),
            onReset: () => this.reset(),
            onSelectScenario: (id) => this.selectScenario(id),
            onToggleDemoMode: () => this.toggleDemoMode(),
            onChangeSimSpeed: (speed) => { this.simSpeedMultiplier = speed; },
            onToggleLayer: (layer, enabled) => this.toggleLayer(layer, enabled),
            onSetSpawnMode: (type) => this.setSpawnMode(type)
        });

        // Bind Canvas Click for Obstacle Spawning
        this.canvas.addEventListener('click', (e) => this.handleCanvasClick(e));

        // Load Default Scenario or URL query parameter scenario
        const urlParams = new URLSearchParams(window.location.search);
        const initialScenario = urlParams.get('scenario') || 'normal_traffic';
        this.selectScenario(initialScenario);

        // Start Master Animation Loop
        requestAnimationFrame((t) => this.loop(t));
    }

    /**
     * Master Closed-Loop Simulation Frame
     */
    loop(timestamp) {
        let dt = (timestamp - this.lastTimestamp) / 1000;
        this.lastTimestamp = timestamp;

        // Clamp dt to avoid huge jumps on tab switch
        if (dt > 0.1) dt = 0.1;

        if (this.isRunning) {
            const effectiveDt = dt * this.simSpeedMultiplier;
            this.updateClosedLoop(effectiveDt);
        }

        // Render Frame
        this.renderer.render({
            ego: this.ego,
            road: this.road,
            obstacles: this.obstacles,
            sensors: this.sensors,
            sensorFusion: this.sensorFusion,
            fusedTracks: this.fusedTracks,
            plannerResult: this.plannerResult,
            riskState: this.riskState,
            controller: this.controller,
            isDemoMode: this.scenarioManager.isDemoMode,
            demoNarration: this.scenarioManager.demoNarration
        });

        // Update Telemetry UI
        this.dashboard.update({
            ego: this.ego,
            sensorFusion: this.sensorFusion,
            fusedTracks: this.fusedTracks,
            plannerResult: this.plannerResult,
            riskState: this.riskState,
            isDemoMode: this.scenarioManager.isDemoMode,
            demoNarration: this.scenarioManager.demoNarration
        });

        requestAnimationFrame((t) => this.loop(t));
    }

    /**
     * Complete Closed-Loop Autonomous Pipeline Execution:
     * PERCEPTION -> SENSOR FUSION -> TRAJECTORY PREDICTION -> RISK ASSESSMENT ->
     * ADAPTIVE LATTICE PLANNING -> PURE PURSUIT VEHICLE CONTROL -> KINEMATICS UPDATE
     */
    updateClosedLoop(dt) {
        // 0. Update automated demo director if active
        if (this.scenarioManager.isDemoMode) {
            const demoResult = this.scenarioManager.updateDemo(dt, this.ego, this.road, this.obstacles);
            if (demoResult) {
                this.obstacles = demoResult.obstacles;
            }
        }

        // 1. Update dynamic obstacle movements (weaving, cattle wander, pedestrian crossing)
        for (const obs of this.obstacles) {
            obs.update(dt, this.ego.y);
        }

        // 2. PERCEPTION LAYER: Multi-Sensor Scan (Camera, 64-beam LiDAR, 77GHz Radar)
        this.sensors.scan(this.ego, this.obstacles, this.road);

        // 3. SENSOR FUSION ENGINE: Fuse Radar Doppler, LiDAR point-cloud & Camera classification
        this.fusedTracks = this.sensorFusion.fuse(this.sensors, this.ego, this.obstacles);

        // 4. TRAJECTORY PREDICTION: Spatiotemporal motion forecasting
        this.trajectoryPredictor.predict(this.fusedTracks);

        // 5. RISK ASSESSMENT ENGINE: Threat evaluation, TTC, and biological vulnerability margins
        this.riskState = this.riskEngine.evaluate(this.fusedTracks, this.ego, this.road);

        // 6. ADAPTIVE PATH PLANNER: Multi-objective candidate lattice evaluation (at 20 Hz)
        this.timeSinceLastPlan += dt;
        if (this.timeSinceLastPlan >= this.planInterval || !this.plannerResult.selectedTrajectory) {
            this.timeSinceLastPlan = 0;
            this.plannerResult = this.planner.plan(this.ego, this.fusedTracks, this.road, this.riskEngine);
        }

        // 7. VEHICLE CONTROLLER: Pure Pursuit steering & speed governor
        const ctrlCmds = this.controller.computeCommands(
            this.ego,
            this.plannerResult.selectedTrajectory,
            this.riskState,
            this.plannerResult.planningStatus
        );

        // 8. VEHICLE KINEMATICS: Integrate Kinematic Bicycle Model
        this.ego.update(dt, ctrlCmds.steering, ctrlCmds.acceleration);

        // 9. Periodic road feature cleanup & forward procedural extension
        if (this.ego.y > 1000) {
            // Smooth wrap or continuous cruising
            this.ego.reset(this.ego.x, 0, this.ego.heading, this.ego.speed);
            for (const obs of this.obstacles) {
                obs.y -= 1000;
            }
        }
    }

    // ==========================================
    // USER INTERACTION HANDLERS
    // ==========================================

    togglePlayPause() {
        this.isRunning = !this.isRunning;
        return this.isRunning;
    }

    stepOnce() {
        this.isRunning = false;
        this.updateClosedLoop(SIM_CONFIG.TIME_STEP);
    }

    reset() {
        this.scenarioManager.stopDemoMode();
        this.selectScenario(this.scenarioManager.scenarios[this.scenarioManager.currentScenarioIndex].id);
    }

    selectScenario(scenarioId) {
        const res = this.scenarioManager.loadScenario(scenarioId, this.ego, this.road);
        this.obstacles = res.obstacles;
        this.plannerResult.selectedTrajectory = null; // Force replan
        if (this.dashboard && this.dashboard.selectScenario) {
            this.dashboard.selectScenario.value = scenarioId;
        }
        this.updateClosedLoop(0.001); // Prime pipeline
    }

    toggleDemoMode() {
        if (this.scenarioManager.isDemoMode) {
            this.scenarioManager.stopDemoMode();
            return false;
        } else {
            const res = this.scenarioManager.startDemoMode(this.ego, this.road);
            this.obstacles = res.obstacles;
            this.isRunning = true;
            return true;
        }
    }

    toggleLayer(layer, enabled) {
        if (layer === 'lidar') this.renderer.showLidar = enabled;
        else if (layer === 'radar') this.renderer.showRadar = enabled;
        else if (layer === 'camera') this.renderer.showCameraFov = enabled;
        else if (layer === 'candidates') this.renderer.showCandidates = enabled;
        else if (layer === 'corridor') this.renderer.showSafeCorridor = enabled;
    }

    setSpawnMode(typeKey) {
        this.spawnMode = typeKey;
        this.canvas.style.cursor = typeKey ? 'crosshair' : 'default';
    }

    handleCanvasClick(e) {
        if (!this.spawnMode) return;

        const rect = this.canvas.getBoundingClientRect();
        const screenX = e.clientX - rect.left;
        const screenY = e.clientY - rect.top;

        // Convert click screen position to world coordinates
        const worldPt = this.renderer.screenToWorld(screenX, screenY, this.ego.x, this.ego.y);

        // Spawn obstacle in world
        const newObs = new Obstacle(this.spawnMode, worldPt.x, worldPt.y, {
            speed: (this.spawnMode === 'POTHOLE' || this.spawnMode === 'BARRICADE') ? 0 : 3.0
        });
        this.obstacles.push(newObs);

        // Force instant replanning cycle
        this.plannerResult.selectedTrajectory = null;
    }
}

// Instantiate Simulation on DOM load
window.addEventListener('DOMContentLoaded', () => {
    window.sim = new Simulation();
});
