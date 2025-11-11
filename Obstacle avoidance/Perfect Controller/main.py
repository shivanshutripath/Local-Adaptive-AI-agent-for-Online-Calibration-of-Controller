import pygame
import math
from robot import Graphics, Robot, Ultrasonic, distance   # <- match the real filename/case

from rt_types import Ring, Sample
from rt_checks import (
    assert_no_collision, assert_min_clearance,
    assert_avoids_within_T, assert_recovers_within_N
)

MAP_DIMENSIONS = (600, 1200)  # (height, width)

# environment graphics
gfx = Graphics(MAP_DIMENSIONS, 'DDR.png', 'ObstacleMap.png')

# the robot
start = (200, 200)  # screen pixels
robot = Robot(start, 0.01 * 3779.52)  # axle length ≈ 1 cm in pixels

# the sensor
sensor_range = (250, math.radians(40))  # (range_px, half_fov_rad)
ultra_sonic = Ultrasonic(sensor_range, gfx.map)

clock = pygame.time.Clock()
last_time = pygame.time.get_ticks()
running = True

# Initialize ring buffer for real-time tests
ring = Ring(capacity=60*1)  # store ~5s at 60 FPS
start_time = pygame.time.get_ticks() / 1000.0

# simulation loop
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # time management
    now = pygame.time.get_ticks()
    dt = (now - last_time) / 1000.0
    last_time = now

    # graphics update
    gfx.map.blit(gfx.map_img, (0, 0))  # redraw the map

    # sense + avoid
    point_cloud = ultra_sonic.sense_obstacles(robot.x, robot.y, robot.heading)
    # Calculate closest obstacle distance using the distance function from robot.py
    closest = float('inf') if not point_cloud else min(distance((robot.x, robot.y), p) for p in point_cloud)
    robot.avoid_obstacles(point_cloud, dt)

    # kinematics and draw
    robot.kinematics(dt)
    gfx.draw_robot(robot.x, robot.y, robot.heading)
    gfx.draw_sensor_data(point_cloud)

    # Record sample for real-time testing
    now = pygame.time.get_ticks() / 1000.0
    sample = Sample(
        t=now - start_time,
        x=robot.x, y=robot.y, theta=robot.heading,
        vl=robot.vl, vr=robot.vr,
        closest_obs=closest, # closest obstacle distance
        avoid_active=(closest < robot.min_obs_dist and robot.count_down > 0),
        collision=(closest <= 30.0)  # collision detected if obstacle distance is 0 or negative
    )
    ring.push(sample)

    # # Run real-time tests
    # window = list(ring.last(60*5))  # ~3s window for 60 FPS
    # try:
    #     assert_no_collision(window)
    #     assert_min_clearance(window, threshold_px=30.0)
    #     assert_avoids_within_T(ring.all(), T=2.0)   # since start
    #     assert_recovers_within_N(ring.all(), N=3.0)  # since start
    # except AssertionError as e:
    #     print(f"[FAULT] {e}")
    #     # Uncomment the following line to stop on test failure:
    #     # pygame.quit(); raise SystemExit(e)

    # pygame.display.update()
    # clock.tick(60)  # cap to 60 FPS

    # --- at top, before the loop ---

    last_faults = set()   # remember what we've already reported

    # --- inside the loop, right after ring.push(sample) ---
    # Build a ~3s time-based window
    now_t = sample.t
    WINDOW_SEC = 3.0
    window = [s for s in ring.all() if now_t - s.t <= WINDOW_SEC]

    # Run real-time tests on the window only
    faults = []
    try:
        assert_no_collision(window)
    except AssertionError as e:
        faults.append(f"[no_collision] {e}")

    try:
        assert_min_clearance(window, threshold_px=45.0)
    except AssertionError as e:
        faults.append(f"[min_clearance≥30px] {e}")

    # Only run “avoid within T” if a threat appeared in the window
    if any((math.isfinite(s.closest_obs) and s.closest_obs < robot.min_obs_dist) for s in window):
        try:
            assert_avoids_within_T(window, T=10)
        except AssertionError as e:
            faults.append(f"[avoid_within_T=10s] {e}")

    # Only run “recover within N” if avoidance was ever active in the window
    if any(s.avoid_active for s in window):
        try:
            assert_recovers_within_N(window, N=3)
        except AssertionError as e:
            faults.append(f"[recover_within_N=7s] {e}")

    # Print only NEW faults (reduce spam)
    new_faults = set(faults) - last_faults
    for msg in sorted(new_faults):
        print(f"[FAULT] {msg}")
    last_faults = set(faults)


    # if 'next_debug_print' not in globals():
    #     next_debug_print = 0.0
    # now_s = pygame.time.get_ticks() / 1000.0

    # # print every 0.5s so console isn't flooded
    # if now_s >= next_debug_print:
    #     print(f"[debug] t={now_s - start_time:6.2f}s closest={closest!r}")
    #     next_debug_print = now_s + 0.5

    # # print immediately if within 10 px (tune as you like)
    # if math.isfinite(closest) and closest < 40.0:
    # print(f"[near]  t={now_s - start_time:6.2f}s closest={closest:.2f}px (<40)")


    pygame.display.update()
    clock.tick(60)  # cap to 60 FPS


