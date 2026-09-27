/**
 * SIH 2026 - Problem Statement: SIH26037
 * Team MARK-One: Adaptive Path Planning on Unstructured Indian Roads
 * Risk Assessment & Collision Threat Analyzer
 */

import { SIM_CONFIG } from '../config.js';

export class RiskAssessmentEngine {
    constructor() {
        this.highestRiskTrack = null;
        this.overallRiskLevel = 'SAFE'; // 'SAFE', 'LOW', 'MEDIUM', 'HIGH', 'CRITICAL'
        this.activeThreatsCount = 0;
    }

    /**
     * Evaluates collision threat for all fused tracks.
     * @param {Array<Object>} fusedTracks Fused tracked obstacles
     * @param {Vehicle} ego Ego vehicle
     * @param {Road} road Road model
     */
    evaluate(fusedTracks, ego, road) {
        let maxThreatScore = 0;
        let mostDangerousTrack = null;
        this.activeThreatsCount = 0;

        for (const track of fusedTracks) {
            const dx = track.x - ego.x;
            const dy = track.y - ego.y;
            const dist = Math.hypot(dx, dy);

            // Check if obstacle is ahead of ego vehicle
            const isAhead = (dy > -1.0);
            
            // Check if obstacle is inside vehicle's current lane corridor
            const halfCorridorWidth = (ego.width / 2) + 0.9;
            const isInDirectPath = Math.abs(dx) <= halfCorridorWidth;

            let threatScore = 0;

            if (isAhead && dist < 42.0) {
                // Proximity component
                const proximityScore = Math.max(0, (40.0 - dist) / 40.0) * 45;

                // Time-to-collision component
                let ttcScore = 0;
                if (track.ttc < 5.0) {
                    ttcScore = Math.max(0, (5.0 - track.ttc) / 5.0) * 35;
                }

                // In-path factor
                const pathFactor = isInDirectPath ? 1.5 : (Math.abs(dx) < 2.5 ? 1.0 : 0.4);

                // Vulnerability factor (Cattle, Pedestrian)
                const vulnFactor = track.vulnerabilityWeight || 1.0;

                threatScore = (proximityScore + ttcScore) * pathFactor * vulnFactor;
            }

            // Assign Risk Level
            let level = 'LOW';
            if (track.isSurfaceDefect) {
                // Potholes/surface defects: participate in risk & avoidance planning, but never cause fatal emergency halts
                level = (isInDirectPath && dist < 14.0) ? 'MEDIUM' : 'LOW';
            } else if (threatScore > 85 || (isInDirectPath && dist < 5.5)) {
                level = 'CRITICAL';
            } else if (threatScore > 50 || (isInDirectPath && dist < 16.0)) {
                level = 'HIGH';
            } else if (threatScore > 25 || dist < 25.0) {
                level = 'MEDIUM';
            } else {
                level = 'LOW';
            }

            track.riskLevel = level;
            track.threatScore = threatScore;

            if (level === 'HIGH' || level === 'CRITICAL') {
                this.activeThreatsCount++;
            }

            if (threatScore > maxThreatScore) {
                maxThreatScore = threatScore;
                mostDangerousTrack = track;
            }
        }

        this.highestRiskTrack = mostDangerousTrack;
        if (mostDangerousTrack) {
            this.overallRiskLevel = mostDangerousTrack.riskLevel;
        } else {
            this.overallRiskLevel = 'SAFE';
        }

        return {
            overallRiskLevel: this.overallRiskLevel,
            highestRiskTrack: this.highestRiskTrack,
            activeThreatsCount: this.activeThreatsCount
        };
    }
}
