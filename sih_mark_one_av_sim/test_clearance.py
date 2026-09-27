import math

def test_clearance_math():
    y0 = 0.0
    ego_x = 0.0
    ego_half_w = 0.925
    ego_half_l = 2.2

    # Parked vehicle at x = 1.5, y = 22.0
    obs_x = 1.5
    obs_y = 22.0
    obs_half_w = 0.95
    obs_half_l = 2.25

    # Safe bypass on the left
    bypass_x = -1.8

    # The obstacle rear bumper is at:
    obs_rear_y = obs_y - obs_half_l # 19.75m

    # The front bumper of the ego vehicle is at pt_y + ego_half_l.
    # Therefore, when the front bumper reaches obs_rear_y - 0.5m buffer:
    # pt_y = obs_rear_y - 0.5 - ego_half_l = 19.75 - 0.5 - 2.2 = 17.05m.
    y_avoid_target = max(6.0, obs_rear_y - 1.0 - ego_half_l) # 16.55m

    print(f"Obstacle at y = {obs_y:.2f}, rear bumper at y = {obs_rear_y:.2f}")
    print(f"Vehicle must achieve full lateral bypass at y_avoid_target = {y_avoid_target:.2f}m")

    # Generate quintic from y0 to y_avoid_target
    num_steps = 36
    pts = []
    for i in range(num_steps + 1):
        u = i / num_steps
        pt_y = y0 + 38.0 * u
        if pt_y <= y_avoid_target:
            prog = pt_y / y_avoid_target
            s = 10 * math.pow(prog, 3) - 15 * math.pow(prog, 4) + 6 * math.pow(prog, 5)
            pt_x = ego_x + (bypass_x - ego_x) * s
        elif pt_y <= obs_y + obs_half_l + 3.0:
            pt_x = bypass_x
        else:
            prog = (pt_y - (obs_y + obs_half_l + 3.0)) / (38.0 - (obs_y + obs_half_l + 3.0))
            prog = min(1.0, max(0.0, prog))
            s = 10 * math.pow(prog, 3) - 15 * math.pow(prog, 4) + 6 * math.pow(prog, 5)
            pt_x = bypass_x + (0.0 - bypass_x) * s
        pts.append((pt_x, pt_y))

    # Now check for collision between vehicle footprint and obstacle bounding box along all points
    min_lateral_clearance_at_overlap = 999.0
    collision = False

    for pt_x, pt_y in pts:
        # Check longitudinal overlap: front bumper of car to rear bumper of car vs obstacle front to rear
        car_front = pt_y + ego_half_l
        car_rear = pt_y - ego_half_l
        obs_front = obs_y + obs_half_l
        obs_rear = obs_y - obs_half_l

        # Overlapping in Y?
        if car_front >= obs_rear and car_rear <= obs_front:
            # Check lateral clearance
            # Car right edge = pt_x + ego_half_w
            # Obs left edge = obs_x - obs_half_w
            car_right = pt_x + ego_half_w
            obs_left = obs_x - obs_half_w
            clearance = obs_left - car_right
            min_lateral_clearance_at_overlap = min(min_lateral_clearance_at_overlap, clearance)
            if clearance < 0.25: # minimum 25cm physical gap
                print(f"COLLISION at pt_y={pt_y:.2f} (car_front={car_front:.2f}): car_right={car_right:.2f}, obs_left={obs_left:.2f}, clearance={clearance:.2f}m")
                collision = True

    print(f"Result: Collision = {collision}, Min Lateral Clearance = {min_lateral_clearance_at_overlap:.2f}m")

if __name__ == '__main__':
    test_clearance_math()
