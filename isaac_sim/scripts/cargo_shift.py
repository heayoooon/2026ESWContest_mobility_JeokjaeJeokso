"""Cargo-shift (화물 이동) judgement from the load-cell centre of gravity.

Why: the sensor Pi's algorithm (2026-10-08) compares the COG with a reference that slowly follows the
measured COG, so a shift that stays put (the cargo slid and stopped) becomes the new reference and is
never reported - the "no rear belt, cargo shift" run is level 0 throughout.

Rule used here:
  * reference = median COG and weight of the first quiet half second after a reset (cargo as loaded);
    rebias() re-learns the IMU bias after a road slope / bank change (twin sandbox);
  * "settled" = median of the last SETTLE_S seconds, only while the vehicle has been quiet (horizontal
    acceleration below QUIET_G) for that whole window, so inertial load transfer while accelerating,
    braking or cornering is never mistaken for a shift;
  * level from the settled offset to the reference: >= 12 mm caution, >= 25 mm warning, >= 50 mm danger;
    danger as well when the plate loses >= 20 % of the reference weight or one end of the plate reads
    below -0.5 kg after being 1 kg heavier at the reference (cargo tipping / hanging over the edge);
  * the level latches (moved cargo stays moved) until reset() - re-seating the cargo or a new recording.
"""
import math
from collections import deque

QUIET_G = .06        # horizontal acceleration that counts as "steady driving / parked"
SETTLE_S = .7        # quiet time before a COG offset is trusted
REF_S = .5           # quiet time used for the reference at the start
SHIFT_MM = (12., 25., 50.)   # recorded drives settle within 8 mm; the hand-pushed shift run settles at 28-30 mm
WEIGHT_LOSS = .20
END_NEGATIVE_KG = -.5
END_DROP_KG = 1.     # ... and lighter than at the reference by this much (a rear-heavy load is not a tip-over)


class CargoShift:
    def __init__(self):
        self.reset()

    def reset(self):
        self.history = deque()        # (t, cog_x, cog_y, kg, quiet)
        self.reference = None         # (cog_x, cog_y, kg)
        self.reference_ends = (0., 0.)  # (north kg, south kg)
        self.bias = None              # IMU horizontal bias (G), from the reference window
        self.level = 0
        self.shift_mm = None
        self.weight_change = None
        self.reason = ''
        self.reason_ko = ''           # same, in Korean, for the display Pi
        self.bias_pending = False      # True after rebias(): wait for a steady window, then take it as bias

    def rebias(self):
        """The road slope / bank changed (the twin knows; the real Pi assumes level ground): keep the loaded-
        position reference, re-learn the IMU bias (the new
        gravity component) from the next steady half second, and judge nothing until then. Without this a
        slope reads as permanent acceleration (never steady) and the switch itself looked like a shift."""
        self.bias_pending, self.rebias_t = True, None

    def update(self, t, cog_x, cog_y, kg, ax, ay, north_kg=None, south_kg=None):
        """Feed one sample (time s, COG mm, plate weight kg, IMU G). Returns (level, shift_mm, reason)."""
        if any(v is None or (isinstance(v, float) and math.isnan(v)) for v in (t, cog_x, cog_y, kg, ax, ay)):
            return self.level, self.shift_mm, self.reason
        if self.history and t < self.history[-1][0]:
            self.reset()                                   # time went backwards: new recording / loop
        bx, by = self.bias or (0., 0.)
        quiet = math.hypot(ax - bx, ay - by) < QUIET_G
        self.history.append((t, cog_x, cog_y, kg, quiet, ax, ay,
                             north_kg if north_kg is not None else 0., south_kg if south_kg is not None else 0.))
        while self.history and t - self.history[0][0] > max(SETTLE_S, REF_S) + .05:
            self.history.popleft()

        if self.bias_pending:
            if self.rebias_t is None:
                self.rebias_t = t                         # only samples after the change count
            window = [h for h in self.history if h[0] >= self.rebias_t and t - h[0] <= REF_S]
            mx, my = median(h[5] for h in window), median(h[6] for h in window)
            if t - window[0][0] >= REF_S - .12 and all(math.hypot(h[5] - mx, h[6] - my) < QUIET_G for h in window):
                self.bias, self.bias_pending = (mx, my), False
                # samples taken with the old bias must not count as steady
                self.history = deque((*h[:4], True, *h[5:]) for h in self.history if t - h[0] <= REF_S)
            return self.level, self.shift_mm, self.reason

        if self.reference is None:
            # Quiet = horizontal acceleration near the bias (0 G until a slope change is announced). A merely
            # *steady* window is not enough: a truck accelerating evenly from rest would then be the reference.
            window = [h for h in self.history if t - h[0] <= REF_S]
            if t - self.history[0][0] >= REF_S - .05 and all(h[4] for h in window):
                self.reference = (median(h[1] for h in window), median(h[2] for h in window),
                                  median(h[3] for h in window))
                self.reference_ends = (median(h[7] for h in window), median(h[8] for h in window))
            return self.level, self.shift_mm, self.reason

        window = [h for h in self.history if t - h[0] <= SETTLE_S]
        settled = t - self.history[0][0] >= SETTLE_S - .05 and all(h[4] for h in window)
        level, reason, reason_ko = 0, '', ''
        if settled:
            sx, sy, skg = (median(h[i] for h in window) for i in (1, 2, 3))
            rx, ry, rkg = self.reference
            self.shift_mm = math.hypot(sx - rx, sy - ry)
            self.weight_change = (skg - rkg) / rkg if rkg > .5 else 0.
            level = sum(self.shift_mm >= s for s in SHIFT_MM)
            if level:
                direction = ('front' if sy > ry else 'rear') if abs(sy - ry) >= abs(sx - rx) else ('left' if sx < rx else 'right')
                reason = f'shift {self.shift_mm:.0f} mm to the {direction}'
                reason_ko = f'실은 위치에서 {KO_DIR[direction]}으로 {self.shift_mm:.0f} mm 밀림 (가감속 없는 상태)'
            if self.weight_change <= -WEIGHT_LOSS:
                level, reason = 3, f'plate lost {-self.weight_change * 100:.0f} % of the load'
                reason_ko = f'판이 받는 무게 {-self.weight_change * 100:.0f}% 줄어듦 (화물이 판 밖으로)'
            if north_kg is not None:
                for now, ref in zip((median(h[7] for h in window), median(h[8] for h in window)), self.reference_ends):
                    if now <= END_NEGATIVE_KG and now <= ref - END_DROP_KG:
                        level, reason = 3, 'one end of the plate unloaded (tipping)'
                        reason_ko = '판 한쪽 끝 하중이 빠짐 (화물 쓰러짐 의심)'
        if level > self.level:
            self.level, self.reason, self.reason_ko = level, reason, reason_ko
        return self.level, self.shift_mm, self.reason


KO_DIR = {'front': '앞쪽', 'rear': '뒤쪽', 'left': '왼쪽', 'right': '오른쪽'}


def median(values):
    values = sorted(values)
    n = len(values)
    return values[n // 2] if n % 2 else .5 * (values[n // 2 - 1] + values[n // 2])
