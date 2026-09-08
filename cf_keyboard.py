#!/usr/bin/env python3
"""
cf_keyboard.py -- terminal / pygame keyboard shim, shared by both front ends.

Split out of crazyflie_vicon_teleop.py so the standalone script and the ROS 2
teleop node use one implementation. Pure Python: no ROS, no cflib.

Two backends. pygame gives true key-down/key-up, so "hold W to keep moving"
works properly, but it needs a window and therefore a display. termios reads
raw stdin and has no key-up event at all, so held keys are emulated with a
timeout; it works over SSH, which pygame does not.
"""
import os
import sys
import termios
import time
import tty

class KeyboardBase:
    def poll(self) -> None: ...
    def held(self, key: str) -> bool: ...
    def pressed(self, key: str) -> bool:
        """Edge-triggered: True once per press."""
        ...
    def close(self) -> None: ...
    name = "base"


class PygameKeyboard(KeyboardBase):
    """
    True key-down/key-up. Release a key and motion stops immediately.
    Requires a display; the small window must have focus to receive keys.
    """
    name = "pygame"

    def __init__(self):
        import pygame  # noqa
        self.pygame = pygame
        os.environ.setdefault("SDL_VIDEO_WINDOW_POS", "40,40")
        pygame.init()
        pygame.display.set_mode((520, 200))
        pygame.display.set_caption("Crazyflie teleop -- keep this window focused")
        pygame.key.set_repeat(0)
        self._map = {
            "w": pygame.K_w, "s": pygame.K_s, "a": pygame.K_a, "d": pygame.K_d,
            "q": pygame.K_q, "e": pygame.K_e, "r": pygame.K_r, "f": pygame.K_f,
            "t": pygame.K_t, "l": pygame.K_l, "m": pygame.K_m, "z": pygame.K_z,
            "h": pygame.K_h,
            "space": pygame.K_SPACE, "shift": pygame.K_LSHIFT, "esc": pygame.K_ESCAPE,
        }
        self._edges: set[str] = set()
        self._quit = False

    def poll(self):
        self._edges.clear()
        rev = {v: k for k, v in self._map.items()}
        for ev in self.pygame.event.get():
            if ev.type == self.pygame.QUIT:
                self._quit = True
            elif ev.type == self.pygame.KEYDOWN and ev.key in rev:
                self._edges.add(rev[ev.key])
        self._state = self.pygame.key.get_pressed()

    def held(self, key):
        st = getattr(self, "_state", None)
        return bool(st[self._map[key]]) if st is not None else False

    def pressed(self, key):
        return key in self._edges

    @property
    def window_closed(self):
        return self._quit

    def close(self):
        try:
            self.pygame.quit()
        except Exception:
            pass


class TermiosKeyboard(KeyboardBase):
    """
    Headless/SSH fallback. A terminal gives no key-release events, so a held key
    arrives as an autorepeat stream. We treat a key as "held" for HOLD_DECAY
    seconds after the last repeat. Consequence: motion coasts ~0.25 s after you
    let go. That is a real handling difference -- fly slower on this backend.
    """
    name = "termios"
    HOLD_DECAY = 0.25

    def __init__(self):
        import termios, tty, select  # noqa
        self.termios, self.tty, self.select = termios, tty, select
        self.fd = sys.stdin.fileno()
        if not os.isatty(self.fd):
            raise RuntimeError("stdin is not a TTY; no keyboard backend available.")
        self._old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        self._last: dict[str, float] = {}
        self._edges: set[str] = set()

    def poll(self):
        self._edges.clear()
        now = time.time()
        while self.select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            key = None
            if ch == " ":
                key = "space"
            elif ch == "\x1b":
                # Arrow/function keys are ESC-prefixed sequences. Drain them so
                # pressing an arrow key does not read as "quit".
                seq = ""
                while self.select.select([sys.stdin], [], [], 0.002)[0]:
                    seq += sys.stdin.read(1)
                key = "esc" if seq == "" else None
            elif ch == "\x03":
                raise KeyboardInterrupt
            elif ch.isalpha():
                key = ch.lower()
                if ch.isupper():
                    self._last["shift"] = now
            if key:
                if now - self._last.get(key, 0.0) > self.HOLD_DECAY * 2:
                    self._edges.add(key)
                self._last[key] = now

    def held(self, key):
        return (time.time() - self._last.get(key, 0.0)) < self.HOLD_DECAY

    def pressed(self, key):
        return key in self._edges

    window_closed = False

    def close(self):
        try:
            self.termios.tcsetattr(self.fd, self.termios.TCSADRAIN, self._old)
        except Exception:
            pass


def make_keyboard(force: str | None = None) -> KeyboardBase:
    if force == "termios":
        return TermiosKeyboard()
    if force == "pygame":
        return PygameKeyboard()
    has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    if has_display:
        try:
            kb = PygameKeyboard()
            print("[kb] pygame backend -- keep the teleop window focused.")
            return kb
        except Exception as exc:
            print(f"[kb] pygame unavailable ({exc}); falling back to terminal.")
    kb = TermiosKeyboard()
    print("[kb] terminal raw backend -- motion coasts ~0.25s after key release.")
    return kb


