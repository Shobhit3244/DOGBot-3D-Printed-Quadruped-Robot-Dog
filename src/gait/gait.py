class TrotBoxGaitOverlap:
    """
    Trot gait with overlapping diagonal pairs and box step pattern.
    - Diagonal pairs: FL+BR (legs 0,3) and FR+BL (legs 1,2)
    - Box step: lift 2cm → move 2cm → lower 2cm → return 2cm
    - Second pair starts when first pair is on step 3 (overlapping pattern)
    - Total cycle: 6 phases instead of 4
    """

    def __init__(self, home_coords, step_length=2.0, lift_height=2.0):
        self.home = home_coords  # leg_num -> {'x', 'y', 'z'}
        self.L = step_length     # forward/backward movement (2cm)
        self.H = lift_height     # up/down movement (2cm)

        # Diagonal pairs for trot gait
        self.pair1 = [0, 3]  # FL + BR (front_left + back_right)
        self.pair2 = [1, 2]  # FR + BL (front_right + back_left)

        print(f"TrotBoxGaitOverlap initialized:")
        print(f"  Step length: {self.L}cm")
        print(f"  Lift height: {self.H}cm")
        print(f"  Pair 1 (FL+BR): legs {self.pair1}")
        print(f"  Pair 2 (FR+BL): legs {self.pair2}")
        print(f"  Pattern: Second pair starts on step 3 of first pair")

    def _get_box_step_position(self, leg_num, step_num, direction='fwd'):
        """
        Get the (x, y) position for a leg at a specific step in the box pattern.

        Args:
            leg_num: 0-3 (leg identifier)
            step_num: 1-4 (box step number)
            direction: 'fwd' or 'back'

        Returns:
            tuple: (x, y) coordinates relative to home position
        """
        home = self.home[leg_num]
        x0, y0 = home['x'], home['y']

        # Direction multiplier
        dir_mult = 1 if direction == 'fwd' else -1

        # Box step positions
        if step_num == 1:
            # Lift up
            return (x0 + self.L, y0)
        elif step_num == 2:
            # Move forward/backward while lifted
            return (x0 + self.L, y0 + dir_mult * self.H)
        elif step_num == 3:
            # Lower down at new position
            return (x0, y0 + dir_mult * self.H)
        elif step_num == 4:
            # Return to home position
            return (x0, y0)
        else:
            # Default to home position
            return (x0, y0)

    def _get_stance_position(self, leg_num):
        """Get stance position (home position) for a leg."""
        home = self.home[leg_num]
        return (home['x'], home['y'])

    def get_step_sequence(self, mode='fwd'):
        """
        Generate the 6-phase overlapping trot gait sequence.

        Phase 1: Pair1 step1, Pair2 stance
        Phase 2: Pair1 step2, Pair2 stance  
        Phase 3: Pair1 step3, Pair2 step1  <-- Second pair starts here
        Phase 4: Pair1 step4, Pair2 step2
        Phase 5: Pair1 stance, Pair2 step3
        Phase 6: Pair1 stance, Pair2 step4

        Args:
            mode: 'fwd' for forward, 'back' for backward

        Returns:
            list: 6-phase sequence, each phase is dict {leg_num: (x, y)}
        """
        sequence = []

        print(f"Generating {mode} trot sequence with overlapping pairs...")

        # Phase 1: Pair1 step1, Pair2 stance
        phase1 = {}
        for leg in self.pair1:
            phase1[leg] = self._get_box_step_position(leg, 1, mode)
        for leg in self.pair2:
            phase1[leg] = self._get_stance_position(leg)
        sequence.append(phase1)

        # Phase 2: Pair1 step2, Pair2 stance
        phase2 = {}
        for leg in self.pair1:
            phase2[leg] = self._get_box_step_position(leg, 2, mode)
        for leg in self.pair2:
            phase2[leg] = self._get_stance_position(leg)
        sequence.append(phase2)

        # Phase 3: Pair1 step3, Pair2 step1 (Second pair starts)
        phase3 = {}
        for leg in self.pair1:
            phase3[leg] = self._get_box_step_position(leg, 3, mode)
        for leg in self.pair2:
            phase3[leg] = self._get_box_step_position(leg, 1, mode)
        sequence.append(phase3)

        # Phase 4: Pair1 step4, Pair2 step2
        phase4 = {}
        for leg in self.pair1:
            phase4[leg] = self._get_box_step_position(leg, 4, mode)
        for leg in self.pair2:
            phase4[leg] = self._get_box_step_position(leg, 2, mode)
        sequence.append(phase4)

        # Phase 5: Pair1 stance, Pair2 step3
        phase5 = {}
        for leg in self.pair1:
            phase5[leg] = self._get_stance_position(leg)
        for leg in self.pair2:
            phase5[leg] = self._get_box_step_position(leg, 3, mode)
        sequence.append(phase5)

        # Phase 6: Pair1 stance, Pair2 step4
        phase6 = {}
        for leg in self.pair1:
            phase6[leg] = self._get_stance_position(leg)
        for leg in self.pair2:
            phase6[leg] = self._get_box_step_position(leg, 4, mode)
        sequence.append(phase6)

        print(f"Generated 6-phase sequence for {mode} movement")
        return sequence
