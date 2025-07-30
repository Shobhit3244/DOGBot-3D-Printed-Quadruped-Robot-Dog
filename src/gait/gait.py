class RectBoxGait:
    """
    Rectangle in X-direction only.
    • X  : forward (+) / backward (–)
    • Y  : stays 0  (no side-to-side drift)
    • Z  : stays constant (no step-lift here – add if you want)
    """

    def __init__(self, home_coords, step_length=6.0):
        self.home = home_coords          # leg_num ➜ {'x','y','z'}
        self.L    = step_length          # how far to reach front/back
        self.groups = [[1, 2], [0, 3]]   # (FR+BL) then (FL+BR)

    # ────────────────────────────────────────────────────────────
    def _vertices(self, leg):
        h = self.home[leg]
        x0, y0, z0 = h['x'], h['y'], h['z']
        L = self.L
        # four corners: back-left, back-right, front-right, front-left
        # but Y never changes ⇒ y0 for every vertex
        return [
            (x0 - L/2, y0, z0),      # 0  “back”
            (x0 - L/2, y0, z0),      # 1  (no lateral, keep same)
            (x0 + L/2, y0, z0),      # 2  “front”
            (x0 + L/2, y0, z0)       # 3  (no lateral, keep same)
        ]

    # ────────────────────────────────────────────────────────────
    def get_step_sequence(self, mode='fwd'):
        """Return 4 sub-steps (=2 s) – only X is modified."""
        seq, idxA, idxB = [], 0, 2       # start groups on opposite ends
        if mode == 'back':
            idxA, idxB = 2, 0            # start reversed for backward walk

        for _ in range(4):               # four sub-steps
            phase = {}
            for leg in self.groups[0]:   # group-A moving
                phase[leg] = self._vertices(leg)[idxA]
            for leg in self.groups[1]:   # group-B stance
                phase[leg] = self._vertices(leg)[idxB]
            seq.append(phase)
            idxA, idxB = (idxA+1)%4, (idxB+1)%4   # advance around rectangle
        return seq
