import numpy as np

class GaitPlanner:
    def __init__(self, step_height=4.0, step_length=8.0, ground_y=0.0):
        self.step_height = step_height
        self.step_length = step_length
        self.ground_y = ground_y

    def walk_cycle(self, t, T):
        """
        Simple walk cycle producing foot x, y, z positions.
        t: current time in step cycle
        T: total step duration
        """
        phase = (t % T) / T
        if phase < 0.5:
            # Swing phase: move foot forward and lifted
            x = -self.step_length/2 + self.step_length * 2 * phase
            y = self.ground_y + self.step_height * np.sin(np.pi * 2 * phase)
        else:
            # Stance phase: foot on ground, moving backward
            x = self.step_length/2 - self.step_length * 2 * (phase - 0.5)
            y = self.ground_y
        z = 0  # No lateral shift in basic walk
        return x, y, z

    def turn_cycle(self, t, T, radius, side):
        sign = 1 if side == 'left' else -1
        x, y, z = self.walk_cycle(t, T)
        z += sign * (radius / 4)  # shift laterally for turning
        return x, y, z

    def hop_pattern(self, t, T):
        x = 0
        y = self.ground_y + self.step_height * abs(np.sin(np.pi * (t % T) / T))
        z = 0
        return x, y, z
