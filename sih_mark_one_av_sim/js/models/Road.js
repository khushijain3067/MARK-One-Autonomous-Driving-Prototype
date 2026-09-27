/**
 * SIH 2026 - Problem Statement: SIH26037
 * Team MARK-One: Adaptive Path Planning on Unstructured Indian Roads
 * Model: Unstructured Indian Road Environment
 * 
 * Features:
 * - Variable width with chokepoints and wide stretches
 * - Irregular broken asphalt edges transitioning to dirt/gravel shoulders
 * - Faded/broken lane marking fragments (or zero markings)
 * - Potholes, mud patches, and roadside Indian elements (tea stalls, trees, milestone stones)
 * - Dynamic boundary lookup for collision and road-staying cost functions
 */

import { SIM_CONFIG } from '../config.js';

export class Road {
    constructor() {
        this.baseWidth = SIM_CONFIG.ROAD.BASE_WIDTH;
        this.shoulderWidth = SIM_CONFIG.ROAD.SHOULDER_WIDTH;
        
        // Procedural road boundary generator seeds & noise parameters
        this.widthVariationPeriod = 120.0; // meters
        this.edgeNoisePeriod = 15.0;       // meters
        
        // Road surface defects & roadside landmarks
        this.potholes = [];
        this.mudPatches = [];
        this.roadsideObjects = [];
        this.laneMarkingFragments = [];
        
        this.initRoadFeatures();
    }

    /**
     * Initializes permanent Indian road features:
     * - sporadic faded paint fragments
     * - roadside tea stalls, banyan trees, milestone stones
     */
    initRoadFeatures() {
        // Generate sporadic fragmented lane markings (mostly faded or missing)
        for (let y = 0; y < 1500; y += 18) {
            // Random chance for marking to be present, faded, or absent
            const roll = Math.sin(y * 0.13);
            if (roll > 0.15) {
                this.laneMarkingFragments.push({
                    yStart: y,
                    length: 4.0 + Math.abs(Math.sin(y * 0.4)) * 3.0,
                    opacity: 0.15 + Math.abs(Math.cos(y * 0.2)) * 0.25, // Faded
                    offset: Math.sin(y * 0.05) * 0.3 // Not even straight
                });
            }
        }

        // Roadside vegetation & Indian scenery (trees, stalls, electric poles)
        for (let y = 5; y < 1500; y += 16 + Math.random() * 12) {
            const side = Math.random() > 0.5 ? 1 : -1;
            const distFromEdge = 1.8 + Math.random() * 2.5;
            
            const types = ['tree', 'banyan', 'tea_stall', 'electric_pole', 'milestone', 'bush'];
            const type = types[Math.floor(Math.random() * types.length)];
            
            this.roadsideObjects.push({
                y: y,
                side: side,
                distFromEdge: distFromEdge,
                type: type,
                scale: 0.8 + Math.random() * 0.6,
                rotation: Math.random() * Math.PI * 2
            });
        }

        // Mud and gravel patches on shoulders encroaching onto road
        for (let y = 10; y < 1500; y += 28 + Math.random() * 20) {
            const side = Math.random() > 0.5 ? 1 : -1;
            this.mudPatches.push({
                y: y,
                side: side,
                length: 6.0 + Math.random() * 8.0,
                encroach: 0.4 + Math.random() * 0.8, // Encroachment into asphalt
                roughness: 0.7
            });
        }
    }

    /**
     * Calculates the exact road geometry at a given longitudinal distance (y in meters).
     * Returns:
     * - width: current asphalt width
     * - leftEdge: x position of left asphalt edge
     * - rightEdge: x position of right asphalt edge
     * - leftShoulder: outer dirt boundary
     * - rightShoulder: outer dirt boundary
     */
    getBoundariesAt(y) {
        // Macro width variation (e.g. narrowing near culverts, widening near markets)
        const macroFactor = Math.sin(y / this.widthVariationPeriod * Math.PI * 2) * 0.8
                          + Math.cos(y / (this.widthVariationPeriod * 0.45) * Math.PI * 2) * 0.5;
        
        // Edge irregularity noise (broken asphalt borders)
        const leftNoise = Math.sin(y / this.edgeNoisePeriod) * 0.35 + Math.sin(y / 4.7) * 0.15;
        const rightNoise = Math.cos(y / (this.edgeNoisePeriod * 1.2)) * 0.38 + Math.sin(y / 5.3) * 0.18;
        
        // Current asphalt road width
        const width = Math.max(
            SIM_CONFIG.ROAD.MIN_WIDTH,
            Math.min(SIM_CONFIG.ROAD.MAX_WIDTH, this.baseWidth + macroFactor)
        );

        const halfWidth = width / 2;
        const leftEdge = -halfWidth + leftNoise;
        const rightEdge = halfWidth + rightNoise;

        return {
            width: width,
            leftEdge: leftEdge,
            rightEdge: rightEdge,
            leftShoulder: leftEdge - this.shoulderWidth,
            rightShoulder: rightEdge + this.shoulderWidth,
            centerlineOffset: (leftEdge + rightEdge) / 2
        };
    }

    /**
     * Checks if a point (x, y) is on drivable asphalt with safe road margins.
     * Returns clearance distance to nearest road boundary (positive = inside, negative = off-road).
     */
    getRoadClearance(x, y) {
        const bounds = this.getBoundariesAt(y);
        const distToLeft = x - bounds.leftEdge;
        const distToRight = bounds.rightEdge - x;
        return Math.min(distToLeft, distToRight);
    }

    /**
     * Checks if a point is within the unpaved shoulder buffer.
     */
    isOffRoad(x, y) {
        const bounds = this.getBoundariesAt(y);
        return x < bounds.leftEdge || x > bounds.rightEdge;
    }
}
