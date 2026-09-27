/**
 * SIH 2026 - Problem Statement: SIH26037
 * Team MARK-One: Adaptive Path Planning on Unstructured Indian Roads
 * Model: Multi-Class Indian Road Obstacles & Dynamic Agents
 */

export const OBSTACLE_TYPES = {
    CAR: {
        name: 'Car',
        length: 4.2,
        width: 1.8,
        color: '#ff5252',
        isBiological: false,
        isSurfaceDefect: false,
        vulnerabilityWeight: 1.0,
        defaultSpeed: 6.5,
    },
    AUTO_RICKSHAW: {
        name: 'Auto Rickshaw',
        length: 2.8,
        width: 1.4,
        color: '#ffd600', // Yellow & Green iconic auto
        secondaryColor: '#00c853',
        isBiological: false,
        isSurfaceDefect: false,
        vulnerabilityWeight: 1.2,
        defaultSpeed: 4.5,
    },
    MOTORCYCLE: {
        name: 'Motorcycle',
        length: 2.1,
        width: 0.8,
        color: '#ff9100',
        isBiological: false,
        isSurfaceDefect: false,
        vulnerabilityWeight: 1.4,
        defaultSpeed: 7.2,
    },
    BUS_TRUCK: {
        name: 'Bus/Truck',
        length: 8.5,
        width: 2.5,
        color: '#e040fb',
        isBiological: false,
        isSurfaceDefect: false,
        vulnerabilityWeight: 1.0,
        defaultSpeed: 5.0,
    },
    PEDESTRIAN: {
        name: 'Pedestrian',
        length: 0.7,
        width: 0.7,
        color: '#ff4081',
        isBiological: true,
        isSurfaceDefect: false,
        vulnerabilityWeight: 2.2, // Highest protection weight
        defaultSpeed: 1.2,
    },
    CATTLE: {
        name: 'Cattle (Cow)',
        length: 2.2,
        width: 1.1,
        color: '#ffffff', // White/Cream sacred cow
        accentColor: '#8d6e63',
        isBiological: true,
        isSurfaceDefect: false,
        vulnerabilityWeight: 2.0, // High cultural and biological safety priority
        defaultSpeed: 0.5,
    },
    POTHOLE: {
        name: 'Pothole',
        length: 1.4,
        width: 1.4,
        color: '#212121', // Dark depression with cracked rim
        rimColor: '#e65100',
        isBiological: false,
        isSurfaceDefect: true,
        vulnerabilityWeight: 1.1, // Must be avoided to protect suspension
        defaultSpeed: 0.0,
    },
    PARKED_VEHICLE: {
        name: 'Parked Tempo',
        length: 4.5,
        width: 1.9,
        color: '#78909c',
        isBiological: false,
        isSurfaceDefect: false,
        vulnerabilityWeight: 1.0,
        defaultSpeed: 0.0,
    },
    BARRICADE: {
        name: 'Barricade',
        length: 0.8,
        width: 2.4,
        color: '#ffab00', // Yellow-black road work stripes
        isBiological: false,
        isSurfaceDefect: false,
        vulnerabilityWeight: 1.1,
        defaultSpeed: 0.0,
    },
    HAND_CART: {
        name: 'Hand Cart (Thela)',
        length: 2.2,
        width: 1.3,
        color: '#8d6e63',
        isBiological: false,
        isSurfaceDefect: false,
        vulnerabilityWeight: 1.5,
        defaultSpeed: 1.0,
    },
    DEBRIS: {
        name: 'Road Debris',
        length: 1.0,
        width: 1.0,
        color: '#a1887f',
        isBiological: false,
        isSurfaceDefect: false,
        vulnerabilityWeight: 0.9,
        defaultSpeed: 0.0,
    },
    ONCOMING_VEHICLE: {
        name: 'Oncoming Vehicle',
        length: 4.6,
        width: 1.9,
        color: '#d32f2f',
        isBiological: false,
        isSurfaceDefect: false,
        vulnerabilityWeight: 1.6,
        defaultSpeed: 6.0,
        isOncoming: true
    }
};

let nextObstacleId = 1001;

export class Obstacle {
    constructor(typeKey, x, y, options = {}) {
        this.id = nextObstacleId++;
        this.typeKey = typeKey;
        this.config = OBSTACLE_TYPES[typeKey] || OBSTACLE_TYPES.CAR;
        
        this.name = this.config.name;
        this.x = x;                           // lateral position (m)
        this.y = y;                           // longitudinal position (m)
        this.length = options.length || this.config.length;
        this.width = options.width || this.config.width;
        this.isOncoming = options.isOncoming || this.config.isOncoming || false;
        this.heading = (options.heading !== undefined) ? options.heading : (this.isOncoming ? Math.PI : 0);
        
        // Kinematics
        const speed = (options.speed !== undefined) ? options.speed : this.config.defaultSpeed;
        this.speed = speed;
        this.vx = options.vx || (speed * Math.sin(this.heading));
        this.vy = (options.vy !== undefined) ? options.vy : (speed * Math.cos(this.heading));
        
        // Behavioral attributes
        this.behavior = options.behavior || 'constant'; // 'constant', 'weave', 'cross_road', 'wander', 'cut_in'
        this.crossDirection = options.crossDirection || (Math.random() > 0.5 ? 1 : -1);
        this.phase = Math.random() * Math.PI * 2;
        this.isBiological = this.config.isBiological;
        this.isSurfaceDefect = this.config.isSurfaceDefect;
        this.vulnerabilityWeight = this.config.vulnerabilityWeight;

        // Custom cut-in trigger
        this.hasCutIn = false;
        this.cutInTriggerY = options.cutInTriggerY || null;

        // Tracking metadata
        this.detectedBySensors = [];
        this.riskLevel = 'SAFE';
    }

    /**
     * Updates obstacle position according to Indian road user behaviors.
     */
    update(dt, egoY = 0) {
        if (this.speed === 0 && !this.behavior) return;

        // Special behavioral patterns
        switch (this.behavior) {
            case 'cross_road': // Pedestrian informal crossing
                this.x += this.crossDirection * this.speed * dt;
                // Reverse or pause at road edges
                if (Math.abs(this.x) > 4.2) {
                    this.crossDirection *= -1;
                }
                this.heading = (this.crossDirection > 0) ? Math.PI / 2 : -Math.PI / 2;
                break;

            case 'wander': // Cattle slow meandering
                this.phase += dt * 0.8;
                this.vx = Math.sin(this.phase) * 0.4;
                this.vy = Math.max(0, Math.cos(this.phase) * 0.3);
                this.x += this.vx * dt;
                this.y += this.vy * dt;
                this.heading = Math.atan2(this.vx, this.vy);
                break;

            case 'weave': // Two-wheeler weaving in traffic
                this.phase += dt * 2.2;
                this.x += Math.sin(this.phase) * 1.5 * dt;
                this.y += this.speed * dt;
                this.heading = Math.sin(this.phase) * 0.25;
                break;

            case 'cut_in': // Aggressive cut-in when ego vehicle approaches
                if (!this.hasCutIn && this.cutInTriggerY && egoY >= this.cutInTriggerY) {
                    this.hasCutIn = true;
                }
                if (this.hasCutIn) {
                    // Swift swerve directly towards center lane
                    const targetX = 0.0;
                    const dx = targetX - this.x;
                    this.x += Math.sign(dx) * Math.min(Math.abs(dx), 3.2 * dt);
                    this.y += this.speed * dt;
                    this.heading = Math.sign(dx) * 0.35;
                } else {
                    this.y += this.speed * dt;
                }
                break;

            case 'constant':
            default:
                this.x += this.vx * dt;
                this.y += this.vy * dt;
                break;
        }
    }

    /**
     * Returns oriented bounding box corners for sensor collision checks.
     */
    getPolygon() {
        const cosH = Math.cos(this.heading);
        const sinH = Math.sin(this.heading);
        const halfW = this.width / 2;
        const halfL = this.length / 2;

        const rotate = (dx, dy) => ({
            x: this.x + dx * cosH + dy * sinH,
            y: this.y - dx * sinH + dy * cosH
        });

        return [
            rotate(-halfW, halfL),
            rotate(halfW, halfL),
            rotate(halfW, -halfL),
            rotate(-halfW, -halfL)
        ];
    }
}
