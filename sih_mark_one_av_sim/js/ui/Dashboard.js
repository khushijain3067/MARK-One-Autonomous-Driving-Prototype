/**
 * SIH 2026 - Problem Statement: SIH26037
 * Team MARK-One: Adaptive Path Planning on Unstructured Indian Roads
 * UI: Telemetry Dashboard & Interactive Controls
 */

export class Dashboard {
    constructor(callbacks) {
        this.callbacks = callbacks;
        this.initDomElements();
        this.bindEvents();
    }

    initDomElements() {
        // Telemetry Elements
        this.elSpeed = document.getElementById('val-speed');
        this.elSpeedGauge = document.getElementById('gauge-speed-fill');
        this.elSteer = document.getElementById('val-steer');
        this.elMode = document.getElementById('val-mode');
        this.elDistance = document.getElementById('val-dist');
        this.elAccel = document.getElementById('val-accel');

        // Perception & Sensor Fusion Elements
        this.elFusionCount = document.getElementById('val-fusion-count');
        this.elFusionConf = document.getElementById('val-fusion-conf');
        this.elObjectList = document.getElementById('detected-objects-list');

        // Risk Assessment Elements
        this.elRiskLevel = document.getElementById('val-risk-level');
        this.elRiskBadge = document.getElementById('risk-badge');
        this.elHighestRiskCard = document.getElementById('highest-risk-card');

        // Path Planning Elements
        this.elPlanStatus = document.getElementById('val-plan-status');
        this.elPlanDecision = document.getElementById('val-plan-decision');
        this.elFreeLeft = document.getElementById('val-free-left');
        this.elFreeRight = document.getElementById('val-free-right');
        this.elCandidatesCount = document.getElementById('val-candidates-count');
        this.elTotalCost = document.getElementById('val-total-cost');
        this.elCostCol = document.getElementById('cost-col');
        this.elCostBound = document.getElementById('cost-bound');
        this.elCostSmooth = document.getElementById('cost-smooth');

        // 5-Stage Closed-Loop Pipeline Elements
        this.pipeSteps = [1, 2, 3, 4, 5].map(i => ({
            el: document.getElementById(`pipe-step-${i}`),
            detail: document.getElementById(`pipe-detail-${i}`)
        }));

        // Controls
        this.btnPlayPause = document.getElementById('btn-play-pause');
        this.btnReset = document.getElementById('btn-reset');
        this.btnStep = document.getElementById('btn-step');
        this.selectScenario = document.getElementById('select-scenario');
        this.btnDemoMode = document.getElementById('btn-demo-mode');
        this.demoStatusBanner = document.getElementById('demo-status-banner');

        // Speed buttons
        this.speedButtons = document.querySelectorAll('.btn-sim-speed');

        // Toggle switches
        this.toggleLidar = document.getElementById('toggle-lidar');
        this.toggleRadar = document.getElementById('toggle-radar');
        this.toggleCamera = document.getElementById('toggle-camera');
        this.toggleCandidates = document.getElementById('toggle-candidates');
        this.toggleCorridor = document.getElementById('toggle-corridor');

        // Interactive Spawn buttons
        this.spawnButtons = document.querySelectorAll('.btn-spawn-obs');
    }

    bindEvents() {
        // Play / Pause
        if (this.btnPlayPause) {
            this.btnPlayPause.addEventListener('click', () => {
                const isRunning = this.callbacks.onTogglePlayPause();
                this.btnPlayPause.innerHTML = isRunning
                    ? '<span class="icon">⏸</span> Pause'
                    : '<span class="icon">▶</span> Run';
                this.btnPlayPause.classList.toggle('active', isRunning);
            });
        }

        // Step
        if (this.btnStep) {
            this.btnStep.addEventListener('click', () => this.callbacks.onStep());
        }

        // Reset
        if (this.btnReset) {
            this.btnReset.addEventListener('click', () => this.callbacks.onReset());
        }

        // Scenario Selector
        if (this.selectScenario) {
            this.selectScenario.addEventListener('change', (e) => {
                this.callbacks.onSelectScenario(e.target.value);
            });
        }

        // Demo Mode Button
        if (this.btnDemoMode) {
            this.btnDemoMode.addEventListener('click', () => {
                const isDemo = this.callbacks.onToggleDemoMode();
                this.btnDemoMode.classList.toggle('active', isDemo);
                this.btnDemoMode.innerText = isDemo ? '⏹ STOP DEMO MODE' : '✨ SIH DEMO MODE';
            });
        }

        // Sim Speed buttons
        this.speedButtons.forEach(btn => {
            btn.addEventListener('click', (e) => {
                this.speedButtons.forEach(b => b.classList.remove('active'));
                btn.classList.add('active');
                const speed = parseFloat(btn.dataset.speed || '1.0');
                this.callbacks.onChangeSimSpeed(speed);
            });
        });

        // Layer Toggles
        if (this.toggleLidar) {
            this.toggleLidar.addEventListener('change', (e) => this.callbacks.onToggleLayer('lidar', e.target.checked));
        }
        if (this.toggleRadar) {
            this.toggleRadar.addEventListener('change', (e) => this.callbacks.onToggleLayer('radar', e.target.checked));
        }
        if (this.toggleCamera) {
            this.toggleCamera.addEventListener('change', (e) => this.callbacks.onToggleLayer('camera', e.target.checked));
        }
        if (this.toggleCandidates) {
            this.toggleCandidates.addEventListener('change', (e) => this.callbacks.onToggleLayer('candidates', e.target.checked));
        }
        if (this.toggleCorridor) {
            this.toggleCorridor.addEventListener('change', (e) => this.callbacks.onToggleLayer('corridor', e.target.checked));
        }

        // Interactive Spawn Buttons
        this.spawnButtons.forEach(btn => {
            btn.addEventListener('click', () => {
                const obsType = btn.dataset.type;
                if (btn.classList.contains('active')) {
                    btn.classList.remove('active');
                    this.callbacks.onSetSpawnMode(null);
                } else {
                    this.spawnButtons.forEach(b => b.classList.remove('active'));
                    btn.classList.add('active');
                    this.callbacks.onSetSpawnMode(obsType);
                }
            });
        });
    }

    /**
     * Updates telemetry values and UI elements in real-time.
     */
    update(simState) {
        const { ego, fusedTracks, plannerResult, riskState, isDemoMode, demoNarration } = simState;

        // 1. Vehicle Telemetry
        const speedKmh = (ego.speed * 3.6).toFixed(1);
        if (this.elSpeed) this.elSpeed.innerText = speedKmh;
        if (this.elSpeedGauge) {
            const pct = Math.min(100, (ego.speed / 16.67) * 100);
            this.elSpeedGauge.style.width = `${pct}%`;
        }

        const steerDeg = (ego.steeringAngle * (180 / Math.PI)).toFixed(1);
        if (this.elSteer) {
            const dir = (ego.steeringAngle < -0.02) ? '◀ LEFT' : (ego.steeringAngle > 0.02 ? 'RIGHT ▶' : 'CENTER');
            this.elSteer.innerText = `${steerDeg}° (${dir})`;
        }

        if (this.elMode) {
            this.elMode.innerText = ego.mode;
            this.elMode.className = `status-pill mode-${ego.mode.toLowerCase()}`;
        }

        if (this.elDistance) {
            this.elDistance.innerText = `${ego.distanceTraveled.toFixed(1)} m`;
        }

        if (this.elAccel) {
            this.elAccel.innerText = `${ego.acceleration >= 0 ? '+' : ''}${ego.acceleration.toFixed(2)} m/s²`;
        }

        // 2. Multi-Sensor Perception & Fusion
        if (this.elFusionCount) this.elFusionCount.innerText = `${fusedTracks.length} Tracked`;
        if (this.elFusionConf) {
            const conf = (simState.sensorFusion.fusionStats.averageConfidence * 100).toFixed(0);
            this.elFusionConf.innerText = `${conf}% Confidence`;
        }

        this.renderObjectTable(fusedTracks);

        // 3. Risk Assessment
        if (this.elRiskLevel) {
            this.elRiskLevel.innerText = riskState.overallRiskLevel;
            this.elRiskBadge.className = `risk-badge risk-${riskState.overallRiskLevel.toLowerCase()}`;
        }

        this.renderHighestRiskCard(riskState.highestRiskTrack);

        // 4. Adaptive Path Planner Status
        if (this.elPlanStatus) {
            this.elPlanStatus.innerText = plannerResult.planningStatus;
            this.elPlanStatus.className = `plan-status status-${plannerResult.planningStatus.toLowerCase()}`;
        }

        if (this.elPlanDecision) {
            this.elPlanDecision.innerText = plannerResult.activeDecision;
        }

        // Update Free Space Corridor Display
        const corridors = plannerResult.corridors || { left: { width: 3.5 }, right: { width: 3.5 } };
        if (this.elFreeLeft) {
            const wL = (corridors.left ? corridors.left.width.toFixed(1) : '3.5');
            this.elFreeLeft.innerText = `${wL} m`;
            this.elFreeLeft.className = `cell-value ${corridors.left && corridors.left.width >= 0.2 ? 'green' : 'red'}`;
        }
        if (this.elFreeRight) {
            const wR = (corridors.right ? corridors.right.width.toFixed(1) : '3.5');
            this.elFreeRight.innerText = `${wR} m`;
            this.elFreeRight.className = `cell-value ${corridors.right && corridors.right.width >= 0.2 ? 'green' : 'red'}`;
        }

        if (this.elCandidatesCount) {
            this.elCandidatesCount.innerText = `${plannerResult.candidateTrajectories ? plannerResult.candidateTrajectories.length : 0} Paths Evaluated`;
        }

        if (plannerResult.costBreakdown) {
            const c = plannerResult.costBreakdown;
            if (this.elTotalCost) this.elTotalCost.innerText = (c.total < 900 ? c.total.toFixed(1) : 'INF (BLOCKED)');
            if (this.elCostCol) this.elCostCol.innerText = c.collision ? c.collision.toFixed(0) : '0';
            if (this.elCostBound) this.elCostBound.innerText = c.boundary ? c.boundary.toFixed(0) : '0';
            if (this.elCostSmooth) this.elCostSmooth.innerText = c.smoothness ? c.smoothness.toFixed(1) : '0';
        }

        // 5. Update 5-Stage Closed-Loop Pipeline Bar
        if (this.pipeSteps && this.pipeSteps.length === 5) {
            const threat = plannerResult.primaryThreat;
            const side = plannerResult.selectedPathSide || 'CENTER';
            const isReplanning = plannerResult.planningStatus === 'REPLANNING';

            // Stage 1: Detection
            if (this.pipeSteps[0].detail) {
                this.pipeSteps[0].detail.innerText = threat ? `${threat.name.toUpperCase()} (${threat.distance.toFixed(1)}m)` : 'Scanning (3 Sensors)';
            }

            // Stage 2: Risk Assessment
            if (this.pipeSteps[1].detail) {
                this.pipeSteps[1].detail.innerText = `Threat: ${riskState.overallRiskLevel}`;
                this.pipeSteps[1].el.classList.toggle('alert', riskState.overallRiskLevel === 'HIGH' || riskState.overallRiskLevel === 'CRITICAL');
            }

            // Stage 3: Path Replanning
            if (this.pipeSteps[2].detail) {
                this.pipeSteps[2].detail.innerText = isReplanning ? `L: ${corridors.left.width.toFixed(1)}m | R: ${corridors.right.width.toFixed(1)}m` : 'Free Space Clear';
                this.pipeSteps[2].el.classList.toggle('active', isReplanning);
            }

            // Stage 4: Path Selected
            if (this.pipeSteps[3].detail) {
                this.pipeSteps[3].detail.innerText = `Corridor: ${side}`;
                this.pipeSteps[3].el.classList.toggle('active', side === 'LEFT' || side === 'RIGHT');
            }

            // Stage 5: Vehicle Control
            if (this.pipeSteps[4].detail) {
                this.pipeSteps[4].detail.innerText = `Pure Pursuit (δ: ${steerDeg}°)`;
            }
        }

        // 6. Demo Mode Banner
        if (this.demoStatusBanner) {
            if (isDemoMode) {
                this.demoStatusBanner.style.display = 'block';
                this.demoStatusBanner.innerHTML = `<strong>✨ DEMO MODE:</strong> ${demoNarration}`;
            } else {
                this.demoStatusBanner.style.display = 'none';
            }
        }
    }

    renderObjectTable(tracks) {
        if (!this.elObjectList) return;

        if (tracks.length === 0) {
            this.elObjectList.innerHTML = '<div class="empty-state">No obstacles detected in sensor range (Path Clear)</div>';
            return;
        }

        let html = '';
        for (const t of tracks) {
            const riskClass = `risk-${t.riskLevel.toLowerCase()}`;
            const ttcStr = (t.ttc < 50) ? `${t.ttc.toFixed(1)}s` : 'CLEAR';
            const sensorBadges = t.sensors.map(s => `<span class="badge-sensor">${s[0]}</span>`).join('');

            html += `
                <div class="object-row ${riskClass}">
                    <div class="obj-col-main">
                        <span class="obj-name">${t.name}</span>
                        <div class="obj-meta">
                            ${sensorBadges}
                            <span>${(t.confidence * 100).toFixed(0)}%</span>
                        </div>
                    </div>
                    <div class="obj-col-dist">
                        <span class="val-num">${t.distance.toFixed(1)}m</span>
                        <span class="val-sub">TTC: ${ttcStr}</span>
                    </div>
                    <div class="obj-col-risk">
                        <span class="pill-risk ${riskClass}">${t.riskLevel}</span>
                    </div>
                </div>
            `;
        }
        this.elObjectList.innerHTML = html;
    }

    renderHighestRiskCard(track) {
        if (!this.elHighestRiskCard) return;

        if (!track || track.riskLevel === 'SAFE' || track.riskLevel === 'LOW') {
            this.elHighestRiskCard.innerHTML = `
                <div class="risk-card safe">
                    <div class="risk-icon">🛡️</div>
                    <div class="risk-details">
                        <div class="risk-title">PATH CLEAR</div>
                        <div class="risk-sub">No high-risk obstacles ahead. Cruising safely.</div>
                    </div>
                </div>
            `;
            return;
        }

        const isCrit = track.riskLevel === 'CRITICAL';
        this.elHighestRiskCard.innerHTML = `
            <div class="risk-card ${isCrit ? 'critical' : 'warning'}">
                <div class="risk-icon">${isCrit ? '⚠️' : '⚡'}</div>
                <div class="risk-details">
                    <div class="risk-title">${track.name.toUpperCase()} (${track.riskLevel})</div>
                    <div class="risk-sub">
                        Distance: <strong>${track.distance.toFixed(1)} m</strong> | 
                        TTC: <strong>${track.ttc < 50 ? track.ttc.toFixed(1) + 's' : 'SLOW'}</strong> | 
                        Action: <strong>REPLANNING</strong>
                    </div>
                </div>
            </div>
        `;
    }
}
