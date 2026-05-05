"""
map_viewer.py — Real-time 2D mission-space map viewer.

Renders the arena boundary, named obstacles (semitransparent), AprilTag
landmarks with facing arrows, task waypoints, and the live robot pose in a
pygame window.

Stand-alone (shows initial_state pose):
    python3 -m Jetson.map_viewer
    python3 Jetson/map_viewer.py

Embedded in mission_controller or any thread:
    from Jetson.map_viewer import MapViewer
    viewer = MapViewer()
    viewer.start()                                  # opens window in bg thread
    viewer.update(x, y, heading)                    # call every control tick
    viewer.set_objects([{"x":1.2,"y":0.8,"label":"tray"}])
    viewer.stop()

All coordinates are in the arena frame (metres), origin = arena corner (0,0),
matching mission_config.yaml.  Heading is CCW-positive from world +X (radians).
"""

import math
import sys
import threading
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml

# ---------------------------------------------------------------------------
# Window layout
# ---------------------------------------------------------------------------
WIN_W   = 980
WIN_H   = 760
MARGIN  = 70     # pixels of padding around the arena drawing area
LEGEND_W = 185   # width reserved on the right for the legend
FPS     = 30

# ---------------------------------------------------------------------------
# Colour palette  (R, G, B) or (R, G, B, A)
# ---------------------------------------------------------------------------
C_BG        = ( 13,  13,  30)
C_ARENA     = ( 22,  22,  50)
C_GRID      = ( 42,  42,  72)
C_GRID_LBL  = ( 70,  70, 105)
C_BOUNDARY  = (185, 185, 210)

# Per-obstacle name → (R, G, B, A) — A controls semitransparency (0-255)
OBSTACLE_COLORS: Dict[str, Tuple[int, int, int, int]] = {
    "table":      ( 55, 105, 230, 110),
    "prep":       ( 55, 185,  75, 100),
    "dishwasher": (215, 135,  35, 110),
    "ramp":       (200, 195,  45, 100),
    "_default":   (155,  75, 195, 100),
}

C_TAG_FILL  = (  0,  45,  55)
C_TAG_RING  = (  0, 205, 225)
C_TAG_ARROW = ( 15, 250, 250)
C_TAG_TEXT  = (205, 250, 255)
C_TAG_NAME  = (130, 210, 215)

C_WAYPOINT  = (215,  65, 225)
C_INIT      = (105, 105, 125)
C_TRAIL_END = ( 25, 155,  60)   # bright end of trail gradient

C_ROBOT_DOT  = (  0, 255,  90)
C_ROBOT_BODY = (  0, 225,  80)
C_ROBOT_BOX  = (190, 255, 195)
C_ROBOT_RBOX = ( 70, 190, 115)
C_ROBOT_HEAD = (255, 205,   0)

C_OBJ_DETECT = (255, 135,  55)

TRAIL_MAX = 700

# ---------------------------------------------------------------------------
# Try importing robot geometry from config — fall back to sensible defaults
# so the viewer also works as a pure YAML inspector.
# ---------------------------------------------------------------------------
try:
    _repo_root = Path(__file__).resolve().parent.parent
    if str(_repo_root) not in sys.path:
        sys.path.insert(0, str(_repo_root))
    from Jetson.config import L_X, L_Y, MAX_Y
except Exception:
    L_X   = 0.1665   # half wheelbase front-to-back [m]
    L_Y   = 0.2075   # half wheelbase left-to-right [m]
    MAX_Y = 0.525    # most forward point (arm extended) [m]


# ===========================================================================
# MapViewer
# ===========================================================================

class MapViewer:
    """
    Thread-safe 2D top-down map viewer for the Butler Bot arena.

    Parameters
    ----------
    config_path : path to mission_config.yaml (defaults to
                  <this file's directory>/mission_config.yaml)
    """

    def __init__(self, config_path: Optional[Path] = None):
        if config_path is None:
            config_path = Path(__file__).parent / "mission_config.yaml"
        with open(config_path) as f:
            cfg = yaml.safe_load(f)

        arena_map        = cfg["map"]
        boundary         = arena_map["boundary"]
        self._landmarks  = arena_map.get("landmarks", [])
        self._obstacles  = arena_map.get("obstacles", [])
        self._tasks      = cfg.get("tasks", [])
        init             = cfg.get("initial_state", {})

        xs = [p["x"] for p in boundary]
        ys = [p["y"] for p in boundary]
        self._ax0 = min(xs);  self._ay0 = min(ys)
        self._aw  = max(xs) - self._ax0
        self._ah  = max(ys) - self._ay0
        self._boundary_pts = boundary

        # Robot state
        init_pos     = init.get("position", {})
        self._rx     = float(init_pos.get("x", self._aw / 2))
        self._ry     = float(init_pos.get("y", self._ah / 2))
        self._rtheta = float(init.get("heading", 0.0))
        self._init_x = self._rx
        self._init_y = self._ry
        self._trail: List[Tuple[float, float]] = []

        # Detected objects overlay
        self._objects: List[Dict] = []

        self._lock    = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # Set by _run() before first draw
        self._ppm = 1.0   # pixels per metre
        self._ox  = 0.0   # screen X origin of arena (px)
        self._oy  = 0.0   # screen Y origin of arena (px)
        self._apx_h = 1.0 # arena height in pixels

    # -----------------------------------------------------------------------
    # Public API
    # -----------------------------------------------------------------------

    def update(self, x: float, y: float, heading: float) -> None:
        """Update robot pose (arena frame metres, heading radians CCW from +X).

        The AprilTag localizer returns a pose that is 180° rotated relative to
        the map frame (the camera faces the tags from the opposite side of the
        arena coordinate origin), so x, y, and heading are all flipped here
        before rendering.
        """
        # 180° flip: negate position and wrap heading
        x = x
        y = y
        heading = heading
        with self._lock:
            self._trail.append((self._rx, self._ry))
            if len(self._trail) > TRAIL_MAX:
                self._trail.pop(0)
            self._rx, self._ry, self._rtheta = x, y, heading

    def set_objects(self, objects: List[Dict]) -> None:
        """
        Overwrite the list of detected objects drawn on the map.
        Each entry: {"x": float, "y": float, "label": str}
        """
        with self._lock:
            self._objects = list(objects)

    def start(self) -> None:
        """Open the window in a background daemon thread."""
        self._running = True
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="MapViewer"
        )
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def run_blocking(self) -> None:
        """Run in the calling thread — use this for stand-alone execution."""
        self._running = True
        self._run()

    # -----------------------------------------------------------------------
    # Coordinate helpers
    # -----------------------------------------------------------------------

    def _ws(self, wx: float, wy: float) -> Tuple[int, int]:
        """Arena world (m) → screen (px).  +Y world = upward on screen."""
        sx = self._ox + (wx - self._ax0) * self._ppm
        sy = self._oy + self._apx_h - (wy - self._ay0) * self._ppm
        return int(sx), int(sy)

    def _robot_pt(self, rx: float, ry: float, theta: float,
                  fwd: float, side: float) -> Tuple[int, int]:
        """Robot body frame (fwd, side) → screen coords."""
        c, s = math.cos(theta), math.sin(theta)
        wx = rx + fwd * c - side * s
        wy = ry + fwd * s + side * c
        return self._ws(wx, wy)

    # -----------------------------------------------------------------------
    # Main render loop
    # -----------------------------------------------------------------------

    def _run(self) -> None:
        import pygame
        pygame.init()

        draw_w = WIN_W - 2 * MARGIN - LEGEND_W
        draw_h = WIN_H - 2 * MARGIN

        # Uniform scale that fits the arena without distortion
        self._ppm    = min(draw_w / self._aw, draw_h / self._ah)
        self._apx_h  = self._ah * self._ppm
        apx_w        = self._aw * self._ppm
        # Centre the arena in the available draw area
        self._ox = MARGIN + (draw_w - apx_w) / 2
        self._oy = MARGIN + (draw_h - self._apx_h) / 2

        screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption("Butler Bot — Mission Space")
        clock = pygame.time.Clock()

        f9  = pygame.font.SysFont("monospace",  9)
        f11 = pygame.font.SysFont("monospace", 11)
        f13 = pygame.font.SysFont("monospace", 13, bold=True)
        f15 = pygame.font.SysFont("monospace", 15, bold=True)

        # Pre-render static elements onto a surface (avoids per-frame cost)
        static_surf = self._build_static_surface(pygame, draw_w + 2*MARGIN, WIN_H, f9, f11, f13)

        while self._running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self._running = False

            screen.fill(C_BG)
            screen.blit(static_surf, (0, 0))

            # Thread-safe snapshot
            with self._lock:
                trail   = list(self._trail)
                rx      = self._rx
                ry      = self._ry
                rtheta  = self._rtheta
                objects = list(self._objects)

            self._draw_trail(screen, trail)
            self._draw_objects(screen, objects, f11)
            self._draw_robot(screen, rx, ry, rtheta, f11)
            self._draw_legend(screen, f9, f15)

            pygame.display.flip()
            clock.tick(FPS)

        pygame.quit()

    # -----------------------------------------------------------------------
    # Static surface (arena + grid + obstacles + tags + waypoints)
    # -----------------------------------------------------------------------

    def _build_static_surface(self, pygame, w, h, f9, f11, f13):
        surf = pygame.Surface((w, h))
        surf.fill(C_BG)

        # Arena fill
        tl = self._ws(self._ax0, self._ay0 + self._ah)
        br = self._ws(self._ax0 + self._aw, self._ay0)
        pygame.draw.rect(surf, C_ARENA,
                         pygame.Rect(tl, (br[0]-tl[0], br[1]-tl[1])))

        self._draw_grid(surf, pygame, f9)
        self._draw_obstacles(surf, pygame, f11)
        self._draw_waypoints(surf, pygame, f11)
        self._draw_landmarks(surf, pygame, f13, f9)

        # Arena boundary outline (on top)
        bpts = [self._ws(p["x"], p["y"]) for p in self._boundary_pts]
        pygame.draw.polygon(surf, C_BOUNDARY, bpts, 2)

        # Start pose marker
        sx, sy = self._ws(self._init_x, self._init_y)
        pygame.draw.circle(surf, C_INIT, (sx, sy), 5, 1)
        lbl = f9.render("start", True, C_INIT)
        surf.blit(lbl, (sx + 7, sy - 5))

        return surf

    # -----------------------------------------------------------------------
    # Draw helpers (static elements)
    # -----------------------------------------------------------------------

    def _draw_grid(self, surf, pygame, font):
        step = 0.5
        x = self._ax0
        while x <= self._ax0 + self._aw + 1e-9:
            p0 = self._ws(x, self._ay0)
            p1 = self._ws(x, self._ay0 + self._ah)
            pygame.draw.line(surf, C_GRID, p0, p1, 1)
            lbl = font.render(f"{x:.1f}", True, C_GRID_LBL)
            surf.blit(lbl, (p0[0] - lbl.get_width() // 2, p0[1] + 3))
            x = round(x + step, 6)
        y = self._ay0
        while y <= self._ay0 + self._ah + 1e-9:
            p0 = self._ws(self._ax0, y)
            p1 = self._ws(self._ax0 + self._aw, y)
            pygame.draw.line(surf, C_GRID, p0, p1, 1)
            lbl = font.render(f"{y:.1f}", True, C_GRID_LBL)
            surf.blit(lbl, (p0[0] - lbl.get_width() - 5,
                            p0[1] - lbl.get_height() // 2))
            y = round(y + step, 6)

    @staticmethod
    def _semitrans_poly(pygame, pts, rgba):
        """Return a (Surface, (x, y)) with a semitransparent filled polygon."""
        xs = [p[0] for p in pts];  ys = [p[1] for p in pts]
        x0, y0 = min(xs), min(ys)
        w = max(max(xs) - x0, 1) + 6
        h = max(max(ys) - y0, 1) + 6
        s = pygame.Surface((w, h), pygame.SRCALPHA)
        shifted = [(p[0]-x0+3, p[1]-y0+3) for p in pts]
        pygame.draw.polygon(s, rgba, shifted)
        return s, (x0-3, y0-3)

    def _draw_obstacles(self, surf, pygame, font):
        for obs in self._obstacles:
            name = obs.get("name", "_default")
            bnd  = obs.get("boundary", [])
            if len(bnd) < 3:
                continue
            spts = [self._ws(p["x"], p["y"]) for p in bnd]
            rgba = OBSTACLE_COLORS.get(name, OBSTACLE_COLORS["_default"])
            r, g, b, a = rgba

            s, offset = self._semitrans_poly(pygame, spts, rgba)
            surf.blit(s, offset)
            pygame.draw.polygon(surf, (r, g, b), spts, 2)

            cx = sum(p[0] for p in spts) // len(spts)
            cy = sum(p[1] for p in spts) // len(spts)
            lbl = font.render(name, True, (r, g, b))
            surf.blit(lbl, (cx - lbl.get_width() // 2, cy - lbl.get_height() // 2))

    def _draw_waypoints(self, surf, pygame, font):
        for task in self._tasks:
            goal = task.get("goal", {}).get("position", {})
            heading = task.get("goal", {}).get("heading")
            if not goal:
                continue
            sx, sy = self._ws(goal["x"], goal["y"])
            d = 7
            pygame.draw.line(surf, C_WAYPOINT, (sx-d, sy-d), (sx+d, sy+d), 2)
            pygame.draw.line(surf, C_WAYPOINT, (sx+d, sy-d), (sx-d, sy+d), 2)
            pygame.draw.circle(surf, C_WAYPOINT, (sx, sy), d+2, 1)
            # Heading arrow for waypoint
            if heading is not None:
                al = int(0.15 * self._ppm)
                ax = sx + int(al * math.cos(heading))
                ay = sy - int(al * math.sin(heading))
                pygame.draw.line(surf, C_WAYPOINT, (sx, sy), (ax, ay), 2)
            lbl = font.render(task.get("name", ""), True, C_WAYPOINT)
            surf.blit(lbl, (sx + 10, sy - 7))

    def _draw_landmarks(self, surf, pygame, font_lbl, font_name):
        tag_r       = max(5, int(0.055 * self._ppm))
        arrow_px    = max(14, int(0.20 * self._ppm))
        for lm in self._landmarks:
            pos     = lm.get("position", {})
            heading = float(lm.get("heading", 0.0))
            tag_id  = lm.get("id", "?")
            name    = lm.get("name", "")
            sx, sy  = self._ws(pos["x"], pos["y"])

            # Tag body square
            rect = pygame.Rect(sx - tag_r, sy - tag_r, tag_r*2, tag_r*2)
            pygame.draw.rect(surf, C_TAG_FILL, rect)
            pygame.draw.rect(surf, C_TAG_RING, rect, 2)

            # Outward-normal arrow (+Y screen = -Y world after flip)
            ax = sx + int(arrow_px * math.cos(heading))
            ay = sy - int(arrow_px * math.sin(heading))
            pygame.draw.line(surf, C_TAG_ARROW, (sx, sy), (ax, ay), 2)
            for angle in (+0.45, -0.45):
                hd = heading + math.pi + angle
                bx = ax + int(7 * math.cos(hd))
                by = ay - int(7 * math.sin(hd))
                pygame.draw.line(surf, C_TAG_ARROW, (ax, ay), (bx, by), 2)

            # Labels
            id_lbl = font_lbl.render(f"#{tag_id}", True, C_TAG_TEXT)
            surf.blit(id_lbl, (sx - id_lbl.get_width() // 2, sy - tag_r - 16))
            nm_lbl = font_name.render(name, True, C_TAG_NAME)
            surf.blit(nm_lbl, (sx - nm_lbl.get_width() // 2, sy + tag_r + 2))

    # -----------------------------------------------------------------------
    # Draw helpers (dynamic elements — drawn every frame)
    # -----------------------------------------------------------------------

    def _draw_trail(self, screen, trail):
        import pygame
        if len(trail) < 2:
            return
        spts = [self._ws(tx, ty) for tx, ty in trail]
        n = len(spts)
        for i in range(1, n):
            t = i / n
            r = int(C_TRAIL_END[0] * t * 0.6)
            g = int(C_TRAIL_END[1] * t * 0.6)
            b = int(C_TRAIL_END[2] * t * 0.6)
            pygame.draw.line(screen, (r, g, b), spts[i-1], spts[i], 1)

    def _draw_objects(self, screen, objects, font):
        import pygame
        for obj in objects:
            sx, sy = self._ws(obj["x"], obj["y"])
            pygame.draw.circle(screen, C_OBJ_DETECT, (sx, sy), 8)
            pygame.draw.circle(screen, (255, 255, 255), (sx, sy), 8, 1)
            lbl = font.render(obj.get("label", "obj"), True, C_OBJ_DETECT)
            screen.blit(lbl, (sx + 10, sy - 6))

    def _draw_robot(self, screen, rx: float, ry: float, rtheta: float, font):
        import pygame

        sx, sy = self._ws(rx, ry)

        # ── Robot body footprint (2·L_X × 2·L_Y rectangle) ────────────────
        body_corners = [
            self._robot_pt(rx, ry, rtheta,  L_X,  L_Y),
            self._robot_pt(rx, ry, rtheta,  L_X, -L_Y),
            self._robot_pt(rx, ry, rtheta, -L_X, -L_Y),
            self._robot_pt(rx, ry, rtheta, -L_X,  L_Y),
        ]
        bs, boff = self._semitrans_poly(pygame, body_corners, (*C_ROBOT_BODY, 60))
        screen.blit(bs, boff)
        pygame.draw.polygon(screen, C_ROBOT_BOX, body_corners, 2)

        # ── Arm-extended range box (forward to MAX_Y) ──────────────────────
        range_corners = [
            self._robot_pt(rx, ry, rtheta,  MAX_Y,  L_Y),
            self._robot_pt(rx, ry, rtheta,  MAX_Y, -L_Y),
            self._robot_pt(rx, ry, rtheta, -L_X,   -L_Y),
            self._robot_pt(rx, ry, rtheta, -L_X,    L_Y),
        ]
        rs, roff = self._semitrans_poly(pygame, range_corners, (*C_ROBOT_RBOX, 30))
        screen.blit(rs, roff)
        pygame.draw.polygon(screen, C_ROBOT_RBOX, range_corners, 1)

        # ── Heading arrow ──────────────────────────────────────────────────
        al = int(0.28 * self._ppm)
        ax = sx + int(al * math.cos(rtheta))
        ay = sy - int(al * math.sin(rtheta))
        pygame.draw.line(screen, C_ROBOT_HEAD, (sx, sy), (ax, ay), 3)
        for angle in (+0.45, -0.45):
            hd = rtheta + math.pi + angle
            bx = ax + int(11 * math.cos(hd))
            by = ay - int(11 * math.sin(hd))
            pygame.draw.line(screen, C_ROBOT_HEAD, (ax, ay), (bx, by), 2)

        # ── Robot centre dot ───────────────────────────────────────────────
        pygame.draw.circle(screen, C_ROBOT_DOT, (sx, sy), 7)
        pygame.draw.circle(screen, (255, 255, 255), (sx, sy), 7, 1)

        # ── Pose readout at bottom ─────────────────────────────────────────
        txt = font.render(
            f"robot  x={rx:.3f} m   y={ry:.3f} m   θ={math.degrees(rtheta):.1f}°",
            True, C_ROBOT_DOT,
        )
        screen.blit(txt, (MARGIN, WIN_H - MARGIN + 12))

    def _draw_legend(self, screen, font, title_font):
        import pygame
        title = title_font.render("Butler Bot — Mission Map", True, (205, 205, 250))
        screen.blit(title, (MARGIN, 12))

        entries = [
            (C_ROBOT_DOT,   "● Robot centre"),
            (C_ROBOT_BOX,   "▭ Robot footprint"),
            (C_ROBOT_RBOX,  "▭ Arm range"),
            (C_TAG_RING,    "■ AprilTag"),
            (C_WAYPOINT,    "× Task waypoint"),
            (C_INIT,        "○ Start pose"),
            (C_OBJ_DETECT,  "● Detected object"),
            ((150, 150, 170), ""),   # spacer
            ((150, 150, 170), "Obstacles:"),
        ]
        for name, rgba in OBSTACLE_COLORS.items():
            if name != "_default":
                r, g, b = rgba[0], rgba[1], rgba[2]
                entries.append(((r, g, b), f"  ■ {name}"))

        x0 = WIN_W - LEGEND_W + 10
        y0 = MARGIN
        for color, label in entries:
            lbl = font.render(label, True, color)
            screen.blit(lbl, (x0, y0))
            y0 += 15

        # Scale bar
        bar_m  = 0.5
        bar_px = int(bar_m * self._ppm)
        bx     = WIN_W - LEGEND_W + 10
        by     = WIN_H - MARGIN - 30
        pygame.draw.line(screen, C_BOUNDARY, (bx, by), (bx + bar_px, by), 2)
        pygame.draw.line(screen, C_BOUNDARY, (bx, by-4), (bx, by+4), 2)
        pygame.draw.line(screen, C_BOUNDARY, (bx+bar_px, by-4), (bx+bar_px, by+4), 2)
        lbl = font.render(f"{bar_m:.1f} m", True, C_BOUNDARY)
        screen.blit(lbl, (bx + bar_px//2 - lbl.get_width()//2, by + 5))


# ---------------------------------------------------------------------------
# Stand-alone entry point
# ---------------------------------------------------------------------------

def main():
    viewer = MapViewer()
    viewer.run_blocking()


if __name__ == "__main__":
    main()
