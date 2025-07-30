class RectBoxGait:
    """
    Rectangular footprint gait. Foot paths are boxes in ZX-plane with fixed Y height.
    Two groups of legs move simultaneously:
      - Group 1: front_right, back_left
      - Group 2: front_left, back_right
    Each step consists of 4 substeps (vertices of rectangle),
    hence total step time is 2 seconds (0.5 s/substep).
    """

    def __init__(self, home_coords, step_length=6.0, step_depth=4.0):
        self.home = home_coords  # Dict: leg -> {'x', 'y', 'z'}
        self.L = step_length     # stride length (along X)
        self.D = step_depth      # stride depth (along Z)

        self.leg_groups = [
            ['front_right', 'back_left'],   # group 1 moves together
            ['front_left', 'back_right']    # group 2 moves together
        ]

    def _get_rectangle_vertices(self, leg, mode='fwd'):
        h = self.home[leg]
        x0, y0, z0 = h['x'], h['y'], h['z']
        L, D = self.L, self.D

        # Define direction based on mode
        if mode == 'back':
            D = -D
        elif mode == 'turn_left':
            L, D = D, L  # rotate rectangle 90 deg left
        elif mode == 'turn_right':
            L, D = -D, -L # rotate 90 deg right

        # Vertices of rectangle path in order of movement substeps
        return [
            (x0 - L/2, y0, z0 - D/2),  # corner 1
            (x0 - L/2, y0, z0 + D/2),  # corner 2
            (x0 + L/2, y0, z0 + D/2),  # corner 3
            (x0 + L/2, y0, z0 - D/2),  # corner 4
        ]

    def get_step_sequence(self, mode='fwd'):
        """
        Generates a 4-substep movement sequence for walking or turning.
        At each substep, one group is moving to the next vertex while the other group holds stance.
        The groups alternate moving from vertex to vertex.
        """
        sequence = []
        vertex_count = 4  # rectangle vertices

        # Initialize last vertex index for each group (start at 0)
        group_vertex_indices = [0, 2]  # start diagonally opposite

        for substep in range(vertex_count):
            phase = {}
            # Group 1 moves
            g1 = self.leg_groups[0]
            idx_g1 = group_vertex_indices[0]
            for leg in g1:
                verts = self._get_rectangle_vertices(leg, mode)
                phase[leg] = verts[idx_g1]

            # Group 2 moves
            g2 = self.leg_groups[1]
            idx_g2 = group_vertex_indices[1]
            for leg in g2:
                verts = self._get_rectangle_vertices(leg, mode)
                phase[leg] = verts[idx_g2]

            sequence.append(phase)
            # Advance vertex indices for next substep (wrap around)
            group_vertex_indices[0] = (idx_g1 + 1) % vertex_count
            group_vertex_indices[1] = (idx_g2 + 1) % vertex_count

        return sequence
