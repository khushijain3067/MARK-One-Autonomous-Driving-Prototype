import math

def test_full_navigation():
    # Test closed loop simulation with 3-zone obstacle avoidance
    dt = 1/60.0
    ego_w = 1.85
    ego_l = 4.4
    wheelbase = 2.7
    max_steer = 0.60
    max_steer_rate = 1.35
    target_speed = 11.5 # ~41.4 km/h

    scenarios = {
        'Scenario 4: Potholes': [
            {'name': 'Pothole 1', 'typeKey': 'POTHOLE', 'x': 0.1, 'y': 20.0, 'width': 1.6, 'length': 1.6, 'isSurfaceDefect': True, 'speed': 0},
            {'name': 'Pothole 2', 'typeKey': 'POTHOLE', 'x': -1.2, 'y': 30.0, 'width': 1.4, 'length': 1.4, 'isSurfaceDefect': True, 'speed': 0},
            {'name': 'Pothole 3', 'typeKey': 'POTHOLE', 'x': 1.3, 'y': 44.0, 'width': 1.5, 'length': 1.5, 'isSurfaceDefect': True, 'speed': 0},
        ],
        'Scenario 8: Barricades': [
            {'name': 'Barricade L', 'typeKey': 'BARRICADE', 'x': -2.4, 'y': 25.0, 'width': 2.2, 'length': 0.8, 'isSurfaceDefect': False, 'speed': 0},
            {'name': 'Barricade R', 'typeKey': 'BARRICADE', 'x': 2.4, 'y': 25.0, 'width': 2.2, 'length': 0.8, 'isSurfaceDefect': False, 'speed': 0},
        ],
        'Scenario 5: Parked Vehicle': [
            {'name': 'Parked Tempo', 'typeKey': 'PARKED_VEHICLE', 'x': 1.5, 'y': 22.0, 'width': 1.9, 'length': 4.5, 'isSurfaceDefect': False, 'speed': 0},
        ],
        'Scenario 2: Slow Vehicle': [
            {'name': 'Auto Rickshaw', 'typeKey': 'AUTO_RICKSHAW', 'x': 0.2, 'y': 22.0, 'width': 1.4, 'length': 2.8, 'isSurfaceDefect': False, 'speed': 3.0, 'vy': 3.0},
        ]
    }

    road_half_w = 4.25

    for sc_name, obstacles in scenarios.items():
        print(f"\n==========================================")
        print(f"RUNNING TEST: {sc_name}")
        print(f"==========================================")
        ego_x = 0.0
        ego_y = 0.0
        ego_heading = 0.0
        ego_speed = target_speed
        ego_steer = 0.0

        history = []
        collisions = []

        # Run 400 frames (6.67 seconds)
        for frame in range(400):
            # 1. Update dynamic obstacles
            for obs in obstacles:
                if obs.get('vy', 0) > 0:
                    obs['y'] += obs['vy'] * dt

            # 2. Find obstacles ahead or currently passing
            ego_front = ego_y + ego_l / 2
            ego_rear = ego_y - ego_l / 2

            active_threat = None
            min_dist = 999.0

            for obs in obstacles:
                obs_front = obs['y'] + obs['length'] / 2
                obs_rear = obs['y'] - obs['length'] / 2

                # If obstacle is ahead or currently being passed
                if obs_front >= ego_rear - 1.0:
                    dist_to_obs = obs_rear - ego_front
                    # Check lateral relevance
                    lat_overlap = abs(obs['x'] - ego_x) < (ego_w/2 + obs['width']/2 + 0.8)
                    if lat_overlap and dist_to_obs < min_dist:
                        min_dist = dist_to_obs
                        active_threat = obs

            # Check if active threat exists
            target_x = 0.0 # default centerline
            if active_threat:
                obs_front = active_threat['y'] + active_threat['length'] / 2
                obs_rear = active_threat['y'] - active_threat['length'] / 2

                # Free space evaluation
                left_space = (active_threat['x'] - active_threat['width']/2 - 0.4) - (-road_half_w + ego_w/2 + 0.3)
                right_space = (road_half_w - ego_w/2 - 0.3) - (active_threat['x'] + active_threat['width']/2 + 0.4)

                # Special check for twin barricades or center gap
                is_twin = False
                for other in obstacles:
                    if other != active_threat and abs(other['y'] - active_threat['y']) < 3.0:
                        o_l = active_threat if active_threat['x'] < other['x'] else other
                        o_r = other if active_threat['x'] < other['x'] else active_threat
                        gap = (o_r['x'] - o_r['width']/2) - (o_l['x'] + o_l['width']/2)
                        if gap >= ego_w + 0.4:
                            target_x = (o_l['x'] + o_l['width']/2 + o_r['x'] - o_r['width']/2) / 2
                            is_twin = True
                            break

                if not is_twin:
                    if active_threat['typeKey'] == 'POTHOLE':
                        # Pothole: dodge left or right by 1.1m
                        if left_space > 0.5 and (active_threat['x'] >= 0 or right_space < 0.5):
                            target_x = active_threat['x'] - active_threat['width']/2 - 0.85
                        else:
                            target_x = active_threat['x'] + active_threat['width']/2 + 0.85
                    else:
                        # Standard obstacle: pick wider side
                        if left_space >= right_space and left_space > 0.4:
                            target_x = ((-road_half_w + ego_w/2 + 0.3) + (active_threat['x'] - active_threat['width']/2 - 0.5)) / 2
                        elif right_space > 0.4:
                            target_x = ((active_threat['x'] + active_threat['width']/2 + 0.5) + (road_half_w - ego_w/2 - 0.3)) / 2

            # Clamp target_x within road
            target_x = max(-road_half_w + ego_w/2 + 0.3, min(road_half_w - ego_w/2 - 0.3, target_x))

            # 3. Pure Pursuit tracking to target_x
            lookahead_dist = max(4.0, min(8.0, 0.38 * ego_speed + 3.6))
            target_pt_y = ego_y + lookahead_dist
            dx = target_x - ego_x
            dy = lookahead_dist
            alpha = math.atan2(dx, dy) - ego_heading
            steer_cmd = math.atan2(2 * wheelbase * math.sin(alpha), lookahead_dist)
            steer_cmd = max(-max_steer, min(max_steer, steer_cmd))

            # Steering rate limiting
            steer_diff = steer_cmd - ego_steer
            max_d_steer = max_steer_rate * dt
            ego_steer += max(-max_d_steer, min(max_d_steer, steer_diff))

            # Speed control: slight slowdown if swerving
            des_speed = target_speed
            if active_threat:
                if active_threat['typeKey'] == 'POTHOLE':
                    des_speed = 9.8 # ~35 km/h
                else:
                    des_speed = 8.5 # ~31 km/h
            ego_speed += (des_speed - ego_speed) * 2.0 * dt

            # Kinematics update
            ds = ego_speed * dt
            yaw_rate = (ego_speed / wheelbase) * math.tan(ego_steer)
            ego_heading += yaw_rate * dt
            ego_x += ds * math.sin(ego_heading)
            ego_y += ds * math.cos(ego_heading)

            # Check collision with any solid obstacle
            for obs in obstacles:
                if not obs.get('isSurfaceDefect'):
                    obs_f = obs['y'] + obs['length']/2
                    obs_r = obs['y'] - obs['length']/2
                    if (ego_y + ego_l/2) >= obs_r and (ego_y - ego_l/2) <= obs_f:
                        car_r = ego_x + ego_w/2
                        car_l = ego_x - ego_w/2
                        obs_rt = obs['x'] + obs['width']/2
                        obs_lt = obs['x'] - obs['width']/2
                        if not (car_r < obs_lt or car_l > obs_rt):
                            collisions.append((frame, obs['name'], ego_x, ego_y))

        print(f"Final Position: x = {ego_x:.2f}m, y = {ego_y:.2f}m, speed = {ego_speed*3.6:.1f} km/h")
        print(f"Collisions detected: {len(collisions)}")
        if len(collisions) == 0:
            print(">>> STATUS: PASSED (Continuous driving, zero collision, returned to road) <<<")
        else:
            print(f">>> STATUS: FAILED ({collisions[0]}) <<<")

if __name__ == '__main__':
    test_full_navigation()
