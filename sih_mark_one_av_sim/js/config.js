/**
 * SIH 2026 - Problem Statement: SIH26037
 * Team MARK-One: Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles on Unstructured Indian Roads
 * Global Configuration & Physics Constants
 */

export const SIM_CONFIG = {
    // Canvas & World Scale
    PIXELS_PER_METER: 16,        // 1 meter = 16 pixels on screen
    TIME_STEP: 1 / 60,            // Simulation dt = 16.67ms (60 FPS)
    PLANNING_FREQUENCY: 20,       // Trajectory replanning runs at 20 Hz (every 50ms)
    
    // Ego Autonomous Vehicle Specifications (Mid-size EV prototype)
    VEHICLE: {
        LENGTH: 4.4,              // meters
        WIDTH: 1.85,              // meters
        WHEELBASE: 2.7,           // meters (L)
        MAX_STEER: 0.60,          // radians (~34.4 degrees)
        MAX_STEER_RATE: 1.35,     // rad/s (responsive smooth steering)
        MAX_SPEED: 16.67,         // m/s (~60 km/h limit)
        TARGET_SPEED: 11.67,      // m/s (~42 km/h normal cruising speed, noticeably faster for live SIH demo)
        MAX_ACCEL: 2.8,           // m/s^2 (comfortable brisk acceleration)
        MAX_BRAKE: 6.5,           // m/s^2 (emergency deceleration)
        COLOR: '#00e5ff',         // Electric Cyan
        ACCENT_COLOR: '#00b0ff',
    },

    // Multi-Sensor Suite (Camera, LiDAR, Radar - Mark-One PPT Pipeline)
    SENSORS: {
        LIDAR: {
            RAYS: 64,             // 64-beam simulated LiDAR
            FOV: Math.PI * 0.75,  // 135-degree forward scan
            RANGE: 45.0,          // 45 meters range
            UPDATE_RATE: 20,      // 20 Hz
            COLOR: 'rgba(0, 229, 255, 0.25)',
            HIT_COLOR: '#ff3d00',
        },
        RADAR: {
            RANGE_FAR: 60.0,      // 60 meters long-range
            FOV_FAR: Math.PI * 0.25, // 45 degrees
            RANGE_NEAR: 25.0,     // 25 meters short-range
            FOV_NEAR: Math.PI * 0.65, // 117 degrees
            UPDATE_RATE: 40,      // 40 Hz FMCW
            COLOR: 'rgba(255, 171, 0, 0.12)',
        },
        CAMERA: {
            FOV: Math.PI * 0.55,  // 100 degrees wide angle
            RANGE: 48.0,          // 48 meters detection horizon
            COLOR: 'rgba(76, 175, 80, 0.08)',
        }
    },

    // Adaptive Path Planner Parameters (Lattice / Quintic Polynomial)
    PLANNER: {
        LOOKAHEAD_TIME: 2.8,      // seconds ahead to plan trajectory
        LOOKAHEAD_DISTANCE_MIN: 16.0, // meters
        LOOKAHEAD_DISTANCE_MAX: 42.0, // meters
        NUM_LATERAL_SAMPLES: 17,  // Candidate lateral paths evaluated per cycle
        LATERAL_SAMPLE_STEP: 0.4, // meters step across road width
        SAFETY_MARGIN: 0.40,      // meters clearance around ego car
        
        // Multi-Objective Cost Function Weights
        WEIGHT_COLLISION: 150.0,  // Severe penalty for potential collision
        WEIGHT_BOUNDARY: 85.0,    // High penalty for unpaved shoulder / off-road
        WEIGHT_SMOOTHNESS: 4.5,   // Penalty for lateral acceleration / jerk
        WEIGHT_CENTERLINE: 2.5,   // Mild preference to stay near road center
        WEIGHT_POTHOLE: 50.0,     // Penalty for traversing road potholes
        WEIGHT_PROGRESS: 2.0,     // Progress toward target cruising speed
    },

    // Risk Classification Thresholds
    RISK: {
        TTC_CRITICAL: 1.5,        // seconds: Emergency Stop & rapid evasive action
        TTC_HIGH: 2.8,            // seconds: Active avoidance replanning
        TTC_MEDIUM: 4.5,          // seconds: Advisory slowdown / corridor shift
        MIN_CLEARANCE_CRITICAL: 0.8, // meters
        MIN_CLEARANCE_HIGH: 1.8,     // meters
        MIN_CLEARANCE_MEDIUM: 3.0,   // meters
    },

    // Unstructured Road Geometry
    ROAD: {
        BASE_WIDTH: 8.5,          // meters (nominal two-lane width)
        MIN_WIDTH: 5.5,           // narrowest chokepoint
        MAX_WIDTH: 11.0,          // widest bazaar stretch
        SHOULDER_WIDTH: 2.5,      // unpaved dirt shoulder on each side
        SEGMENT_LENGTH: 40.0,     // meters per procedural segment
    }
};
