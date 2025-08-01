class TrotBoxGait:
    """
    Trot gait implementation with box step pattern.
    - Only diagonal pairs lift at any time (FL+BR, then FR+BL)
    - Each leg follows box step: lift 2cm → move 2cm → lower 2cm → return 2cm  
    - Only forward/backward movement (no turning)
    - Returns 2D coordinates (x, y) for 2-DOF system
    """

    def __init__(self, home_coords, step_length=4.0, lift_height=4.0):
        self.home = home_coords  # {leg_num: {'x': x, 'y': y}}
        self.step_length = step_length  # 2cm forward/backward movement
        self.lift_height = lift_height  # 2cm vertical lift
        
        # Diagonal pairs for trot gait
        # Pair 1: Front Left (0) + Back Right (3)  
        # Pair 2: Front Right (1) + Back Left (2)
        self.diagonal_pairs = [
            [0, 3],  # FL + BR swing first
            [1, 2]   # FR + BL swing second
        ]
        
        print(f"TrotBoxGait initialized:")
        print(f"  Step length: {step_length}cm")
        print(f"  Lift height: {lift_height}cm") 
        print(f"  Diagonal pairs: {self.diagonal_pairs}")

    def get_box_step_positions(self, leg_num, phase, direction='forward'):
        """
        Generate box step positions for a single leg.
        Box step: lift → move → lower → return (4 phases per leg)
        
        Args:
            leg_num: 0-3 (leg identifier)
            phase: 0-3 (which phase of the box step)  
            direction: 'forward' or 'backward'
            
        Returns:
            (x, y): 2D coordinates relative to home position
        """
        home = self.home[leg_num]
        home_x, home_y = home['x'], home['y']
        
        # Direction multiplier
        dir_mult = 1 if direction == 'forward' else -1
        
        if phase == 0:
            # Phase 0: Lift up (vertical movement only)
            return (home_x, home_y + self.lift_height)
            
        elif phase == 1:
            # Phase 1: Move forward/backward while lifted
            return (home_x + dir_mult * self.step_length, home_y + self.lift_height)
            
        elif phase == 2:
            # Phase 2: Lower down to ground
            return (home_x + dir_mult * self.step_length, home_y)
            
        elif phase == 3:
            # Phase 3: Return to home position on ground
            return (home_x, home_y)
            
        else:
            # Default: home position
            return (home_x, home_y)

    def get_step_sequence(self, direction='forward'):
        """
        Generate complete trot gait sequence with box steps.
        
        Trot gait cycle:
        - 8 total phases per complete cycle
        - Phases 0-3: First diagonal pair (FL+BR) swings with box step
        - Phases 4-7: Second diagonal pair (FR+BL) swings with box step
        
        Args:
            direction: 'forward' or 'backward'
            
        Returns:  
            List of phase positions: [{leg_num: (x, y), ...}, ...]
        """
        sequence = []
        
        print(f"Generating trot sequence: {direction}")
        
        # 8 phases total for complete trot cycle
        for global_phase in range(8):
            phase_positions = {}
            
            if global_phase < 4:
                # Phases 0-3: First diagonal pair (FL+BR) swings
                swing_legs = self.diagonal_pairs[0]  # [0, 3] = FL, BR
                stance_legs = self.diagonal_pairs[1]  # [1, 2] = FR, BL
                box_phase = global_phase  # 0, 1, 2, 3
                
            else:
                # Phases 4-7: Second diagonal pair (FR+BL) swings  
                swing_legs = self.diagonal_pairs[1]  # [1, 2] = FR, BL
                stance_legs = self.diagonal_pairs[0]  # [0, 3] = FL, BR
                box_phase = global_phase - 4  # 0, 1, 2, 3
            
            # Set positions for swinging legs (box step)
            for leg_num in swing_legs:
                pos = self.get_box_step_positions(leg_num, box_phase, direction)
                phase_positions[leg_num] = pos
                
            # Set positions for stance legs (stay at home)
            for leg_num in stance_legs:
                home = self.home[leg_num]
                phase_positions[leg_num] = (home['x'], home['y'])
            
            sequence.append(phase_positions)
            
            # Debug output for first few phases
            if global_phase < 2:
                swing_names = [f"Leg{leg}" for leg in swing_legs]
                stance_names = [f"Leg{leg}" for leg in stance_legs] 
                print(f"  Phase {global_phase}: {swing_names} swing (box phase {box_phase}), {stance_names} stance")
        
        print(f"Generated {len(sequence)} phases for trot {direction}")
        return sequence

    def get_diagonal_pair_info(self):
        """Return information about diagonal pairs"""
        return {
            'pair_1': {
                'legs': self.diagonal_pairs[0],
                'names': ['front_left', 'back_right']
            },
            'pair_2': {
                'legs': self.diagonal_pairs[1], 
                'names': ['front_right', 'back_left']
            }
        }