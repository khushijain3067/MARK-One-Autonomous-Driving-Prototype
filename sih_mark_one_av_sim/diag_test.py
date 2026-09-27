"""
Simulate the JavaScript autonomous vehicle loop in Python to diagnose exactly where and why the vehicle stops or gets stuck.
"""
import math

class SimVehicle:
    def __init__(self, x=0, y=0, speed=11.67):
        self.x = x
        self.y = y
        self.heading = 0.0
        self.speed = speed
        self.steering = 0.0
        self.width = 1.85
        self.length = 4.4
        self.wheelbase = 2.7
        self.mode = 'AUTONOMOUS'

class RoadModel:
    def get_boundaries_at(self, y):
        macro = math.sin(y / 120.0 * math.pi * 2) * 0.8 + math.cos(y / 54.0 * math.pi * 2) * 0.5
        left_noise = math.sin(y / 15.0) * 0.35 + math.sin(y / 4.7) * 0.15
        right_noise = math.cos(y / 18.0) * 0.38 + math.sin(y / 5.3) * 0.18
        width = max(5.5, min(11.0, 8.5 + macro))
        half_w = width / 2
        return {
            'width': width,
            'leftEdge': -half_w + left_noise,
            'rightEdge': half_w + right_noise,
            'centerlineOffset': (-half_w + left_noise + half_w + right_noise) / 2
        }

def test_scenarios():
    scenarios = {
        'potholes': [
            {'typeKey': 'POTHOLE', 'x': 0.1, 'y': 20.0, 'width': 1.6, 'length': 1.6, 'isSurfaceDefect': True, 'speed': 0},
            {'typeKey': 'POTHOLE', 'x': -1.2, 'y': 26.0, 'width': 1.4, 'length': 1.4, 'isSurfaceDefect': True, 'speed': 0},
            {'typeKey': 'POTHOLE', 'x': 1.3, 'y': 38.0, 'width': 1.5, 'length': 1.5, 'isSurfaceDefect': True, 'speed': 0},
        ],
        'barricades': [
            {'typeKey': 'BARRICADE', 'x': -2.4, 'y': 25.0, 'width': 2.2, 'length': 0.8, 'isSurfaceDefect': False, 'speed': 0},
            {'typeKey': 'BARRICADE', 'x': 2.4, 'y': 25.0, 'width': 2.2, 'length': 0.8, 'isSurfaceDefect': False, 'speed': 0},
        ],
        'parked_vehicle': [
            {'typeKey': 'PARKED_VEHICLE', 'x': 1.5, 'y': 22.0, 'width': 1.9, 'length': 4.5, 'isSurfaceDefect': False, 'speed': 0},
        ],
        'slow_vehicle': [
            {'typeKey': 'AUTO_RICKSHAW', 'x': 0.2, 'y': 22.0, 'width': 1.4, 'length': 2.8, 'isSurfaceDefect': False, 'speed': 2.8, 'vy': 2.8},
        ]
    }

    road = RoadModel()

    for name, obstacles in scenarios.items():
        ego = SimVehicle(0, 0, speed=11.67)
        dt = 1/60.0
        stuck = False
        print(f"\n--- TESTING SCENARIO: {name} ---")

        for frame in range(600): # 10 seconds
            # 1. Update obstacles
            for o in obstacles:
                if o.get('vy', 0) > 0:
                    o['y'] += o['vy'] * dt

            # 2. Planning logic (same as LatticePlanner.js)
            bounds_now = road.get_boundaries_at(ego.y)
            bounds_ahead = road.get_boundaries_at(ego.y + 40.0)
            nominal_center = bounds_ahead['centerlineOffset']

            # Find primary threat
            primary_threat = None
            min_dy = 999.0
            car_half_w = ego.width / 2

            for obs in obstacles:
                dy = obs['y'] - ego.y
                dx = obs['x'] - ego.x
                obs_half_w = obs['width'] / 2
                path_buffer = car_half_w + obs_half_w + (0.75 if obs.get('isBiological') else 0.45)
                # Check if ahead
                if dy > 0 and abs(dx) < path_buffer and dy < min_dy:
                    min_dy = dy
                    primary_threat = obs

            # If no obstacle directly in forward path:
            if not primary_threat:
                for obs in obstacles:
                    dy = obs['y'] - ego.y
                    dx = obs['x'] - ego.x
                    if 1.5 < dy < 35.0 and abs(dx) < 3.2 and dy < min_dy:
                        min_dy = dy
                        primary_threat = obs

            # Check if planning status becomes BLOCKED_STOP
            # Let's check collision check logic from lines 537-550
            if primary_threat:
                dy_threat = primary_threat['y'] - ego.y
                # If vehicle is passing alongside obstacle:
                obs_half_w = primary_threat['width'] / 2
                obs_half_l = primary_threat['length'] / 2
                ego_half_w = ego.width / 2
                ego_half_l = ego.length / 2

                margin_x = 0.20 if primary_threat['typeKey'] == 'BARRICADE' else 0.35
                margin_y = 0.5

                dist_x = abs(ego.x - primary_threat['x'])
                dist_y = abs(ego.y - primary_threat['y'])

                # Check if current ego state collides
                if not primary_threat.get('isSurfaceDefect'):
                    if dist_x < (ego_half_w + obs_half_w + margin_x) and dist_y < (ego_half_l + obs_half_l + margin_y):
                        print(f"Frame {frame} (t={frame*dt:.2f}s, y={ego.y:.2f}m, speed={ego.speed:.2f}): COLLISION/BLOCKED with {primary_threat['typeKey']} at x={primary_threat['x']}, y={primary_threat['y']}! dist_x={dist_x:.2f}, dist_y={dist_y:.2f}, req_x={ego_half_w + obs_half_w + margin_x:.2f}")
                        stuck = True
                        break

            # Update ego vehicle simple progression
            ego.y += ego.speed * dt

        if not stuck:
            print(f"Scenario {name}: PASSED cleanly. Final y = {ego.y:.1f}m.")

if __name__ == '__main__':
    test_scenarios()
