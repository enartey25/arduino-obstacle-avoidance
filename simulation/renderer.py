"""
Simulation visualizer for autonomous RL car.
Supports Pygame interactive rendering with graceful fallback to Matplotlib/headless mode.
"""

import math
from typing import List, Tuple, Optional

try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False


class ArenaRenderer:
    def __init__(self, arena_w: float = 300.0, arena_h: float = 300.0,
                 window_width: int = 700, window_height: int = 700,
                 headless: bool = False):
        self.arena_w = arena_w
        self.arena_h = arena_h
        self.win_w = window_width
        self.win_h = window_height
        self.scale_x = self.win_w / self.arena_w
        self.scale_y = self.win_h / self.arena_h

        self.headless = headless or (not PYGAME_AVAILABLE)
        self.screen = None
        self.clock = None
        self.font = None

        if not self.headless:
            self._init_pygame()

    def _init_pygame(self):
        pygame.init()
        pygame.display.set_caption("Autonomous RL Car - Simulation & Telemetry")
        self.screen = pygame.display.set_mode((self.win_w, self.win_h))
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("Consolas", 14)

    def world_to_screen(self, x: float, y: float) -> Tuple[int, int]:
        """Convert arena world cm to screen pixels."""
        sx = int(x * self.scale_x)
        sy = int((self.arena_h - y) * self.scale_y)  # Invert Y for screen coords
        return sx, sy

    def render(self, env, action_name: str = "", reward: float = 0.0, fps: int = 30):
        if self.headless or self.screen is None:
            return

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.close()
                return

        # Background
        self.screen.fill((24, 26, 31))

        # Draw obstacles & walls
        for seg in env.segments:
            p1 = self.world_to_screen(seg[0][0], seg[0][1])
            p2 = self.world_to_screen(seg[1][0], seg[1][1])
            pygame.draw.line(self.screen, (100, 110, 130), p1, p2, 3)

        # Draw sensor rays
        cos_h = math.cos(env.vehicle.heading)
        sin_h = math.sin(env.vehicle.heading)
        readings = env.sensor_array.last_readings

        for i, sensor in enumerate(env.sensor_array.layout):
            off_x = sensor["offset_x"]
            off_y = sensor["offset_y"]
            ox = env.vehicle.x + (off_x * cos_h - off_y * sin_h)
            oy = env.vehicle.y + (off_x * sin_h + off_y * cos_h)

            ray_angle = env.vehicle.heading + math.radians(sensor["angle_deg"])
            dist = readings[i] if i < len(readings) else 50.0

            end_x = ox + dist * math.cos(ray_angle)
            end_y = oy + dist * math.sin(ray_angle)

            p_start = self.world_to_screen(ox, oy)
            p_end = self.world_to_screen(end_x, end_y)

            # Color by distance category
            if dist < 15.0:
                ray_color = (240, 60, 60)    # Red - Blocked
            elif dist <= 35.0:
                ray_color = (255, 190, 40)   # Yellow - Near
            else:
                ray_color = (60, 220, 100)   # Green - Clear

            pygame.draw.line(self.screen, ray_color, p_start, p_end, 2)
            pygame.draw.circle(self.screen, ray_color, p_end, 3)

        # Draw vehicle body (oriented rectangle)
        vw, vl = env.vehicle.width, env.vehicle.length
        corners = [
            (-vl / 2, -vw / 2),
            (vl / 2, -vw / 2),
            (vl / 2, vw / 2),
            (-vl / 2, vw / 2)
        ]
        screen_corners = []
        for cx, cy in corners:
            wx = env.vehicle.x + (cx * cos_h - cy * sin_h)
            wy = env.vehicle.y + (cx * sin_h + cy * cos_h)
            screen_corners.append(self.world_to_screen(wx, wy))

        pygame.draw.polygon(self.screen, (50, 130, 240), screen_corners)
        pygame.draw.polygon(self.screen, (200, 220, 255), screen_corners, 2)

        # Heading indicator (nose line)
        nose_x = env.vehicle.x + (vl / 2 + 5.0) * cos_h
        nose_y = env.vehicle.y + (vl / 2 + 5.0) * sin_h
        pygame.draw.line(self.screen, (255, 255, 255),
                         self.world_to_screen(env.vehicle.x, env.vehicle.y),
                         self.world_to_screen(nose_x, nose_y), 3)

        # Telemetry HUD
        hud_lines = [
            f"Step: {env.current_step}/{env.max_steps}",
            f"Action: {action_name}",
            f"Sensors: [{readings[0]:.1f}, {readings[1]:.1f}, {readings[2]:.1f}, {readings[3]:.1f}] cm",
            f"State: s0={int(readings[0]<15)}.. s3={int(readings[3]<15)} | R: {reward:+.2f}"
        ]
        for idx, line in enumerate(hud_lines):
            text_surf = self.font.render(line, True, (230, 230, 240))
            self.screen.blit(text_surf, (12, 12 + idx * 20))

        pygame.display.flip()
        self.clock.tick(fps)

    def close(self):
        if PYGAME_AVAILABLE and not self.headless:
            pygame.quit()
            self.screen = None

