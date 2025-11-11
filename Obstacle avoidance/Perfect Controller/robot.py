# Differential drive kinematics (screen coords: +y is down)
# x_dot = ((Vl+Vr)/2) * cos(theta)
# y_dot = ((Vl+Vr)/2) * sin(theta)
# theta_dot = (Vr - Vl) / L

import pygame
import math
import numpy as np

def distance(point1, point2): # Euclidean distance between two points
    p1 = np.array(point1, dtype=float)
    p2 = np.array(point2, dtype=float)
    return np.linalg.norm(p1 - p2)

class Robot: # differential drive robot
    def __init__(self, startpos, axle_length_px):
        self.m2p = 3779.52  # meter->pixel (kept for reference)
        # Robot parameters (all in *pixels* per second / pixels)
        self.w = axle_length_px     # wheelbase L in pixels

        self.x = float(startpos[0]) # pixels
        self.y = float(startpos[1]) # pixels
        self.heading = 0.0          # radians

        # wheel velocities (px/s)
        self.vl = 0.01 * self.m2p
        self.vr = 0.01 * self.m2p

        self.maxspeed = 0.02 * self.m2p
        self.minspeed = 0.01 * self.m2p  # optional “cruise” speed magnitude

        self.min_obs_dist = 100.0  # pixels
        self.count_down = 5.0      # seconds for avoidance maneuver

    def avoid_obstacles(self, point_cloud, dt):
        if len(point_cloud) > 0:
            # find closest hit
            dists = [distance((self.x, self.y), p) for p in point_cloud]
            closest = min(dists)
        else:
            closest = np.inf

        if closest < self.min_obs_dist and self.count_down > 0:
            self.count_down -= dt
            self.move_backward()
        else:
            # reset countdown and drive forward
            self.count_down = 5.0
            self.move_forward()

    def move_forward(self):
        # both wheels forward at maxspeed
        self.vl =  self.maxspeed#-2*self.maxspeed#-self.maxspeed
        self.vr =  self.maxspeed#-2*self.maxspeed#-self.maxspeed

    def move_backward(self):
        # simple turning reverse (left slower backward than right)
        self.vl = -self.maxspeed * 0.6
        self.vr = -self.maxspeed

    def kinematics(self, dt):
        v = (self.vl + self.vr) * 0.5
        self.x += v * math.cos(self.heading) * dt
        self.y += v * math.sin(self.heading) * dt
        self.heading = (self.heading + ((self.vr - self.vl) / self.w) * dt) % (2 * math.pi)

        # clamp speeds symmetrically so reverse is allowed
        self.vr = max(-self.maxspeed, min(self.maxspeed, self.vr))
        self.vl = max(-self.maxspeed, min(self.maxspeed, self.vl))

        # optional: enforce a minimum magnitude when moving (skip if near zero)
        for side in ("vl", "vr"):
            val = getattr(self, side)
            if abs(val) > 1e-6 and abs(val) < self.minspeed:
                setattr(self, side, self.minspeed * (1 if val > 0 else -1))

class Graphics:
    def __init__(self, dimensions, robot_img_path, map_img_path):
        pygame.init()
        self.red = (255, 0, 0)

        # dimensions (height, width)
        self.height, self.width = dimensions

        # window settings - create display first so surfaces can be converted
        pygame.display.set_caption("Obstacle Avoidance")
        self.map = pygame.display.set_mode((self.width, self.height))

        # load images (convert after display is created)
        self.robot = pygame.image.load(robot_img_path).convert_alpha()
        self.map_img = pygame.image.load(map_img_path).convert()
        self.map.blit(self.map_img, (0, 0))

    def draw_robot(self, x, y, heading):
        rotated = pygame.transform.rotozoom(self.robot, -math.degrees(heading), 1)
        rect = rotated.get_rect(center=(int(x), int(y)))
        self.map.blit(rotated, rect)

    def draw_sensor_data(self, point_cloud):
        for px, py in point_cloud:
            pygame.draw.circle(self.map, self.red, (int(px), int(py)), 2)

class Ultrasonic:
    def __init__(self, sensor_range, surface):
        self.range_px, self.half_fov = sensor_range
        self.map_width, self.map_height = pygame.display.get_surface().get_size()
        self.surface = surface

    def sense_obstacles(self, x, y, heading):
        obstacles = []
        x1, y1 = x, y
        start_angle = heading - self.half_fov
        finish_angle = heading + self.half_fov

        # cast 10 rays in the FOV
        for angle in np.linspace(start_angle, finish_angle, 10, endpoint=False):
            x2 = x1 + self.range_px * math.cos(angle)
            y2 = y1 + self.range_px * math.sin(angle)  # + for screen coords

            # sample 100 points along the ray
            for i in range(1, 101):
                u = i / 100.0
                sx = int(x2 * u + x1 * (1 - u))
                sy = int(y2 * u + y1 * (1 - u))

                if 0 <= sx < self.map_width and 0 <= sy < self.map_height:
                    color = self.surface.get_at((sx, sy))
                    # visualize the beam (optional)
                    self.surface.set_at((sx, sy), (0, 208, 255))
                    # obstacle is black (RGB)
                    if color[:3] == (0, 0, 0):
                        obstacles.append([sx, sy])
                        break
                else:
                    break  # ray left the map early
        return obstacles
