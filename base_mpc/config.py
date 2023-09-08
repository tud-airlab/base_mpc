import numpy as np



import copy

# ===== HIGH-LEVEL SETTINGS ============ #
# --- Main MPC Parameters --- #
class AlbertConfig:

    def __str__(self):
        return self.name


    def __init__(self):
        '''
        Vehicle bounds (need to be checked)
        '''

        self.name = 'Albert'

        self.length = 0.65
        self.width = 0.65
        self.com_to_back = 0.325

        self.lower_bound = dict()
        self.upper_bound = dict()

        self.lower_bound['x'] = -50.0
        self.upper_bound['x'] = 50.0

        self.lower_bound['y'] = -50.0
        self.upper_bound['y'] = 50.0


        self.lower_bound['psi'] = -10.5 * np.pi
        self.upper_bound['psi'] = +10.5 * np.pi

        self.lower_bound['v'] = 0
        self.upper_bound['v'] = 1.2

        self.lower_bound['w'] = -2.0
        self.upper_bound['w'] = 2.0

        self.lower_bound['alpha'] = -2.0  # Not correct!
        self.upper_bound['alpha'] = 2.0

        self.lower_bound['a'] = -1.0
        self.upper_bound['a'] = 1.0

        self.lower_bound['slack'] = 0
        self.upper_bound['slack'] = 10000

class MPCConfig:
        M = 4
        FORCES_N = 15
        FORCES_N_bar = FORCES_N + 2
        FORCES_NU = 3
        FORCES_NX = 40
        FORCES_TOTAL_V = FORCES_NX + FORCES_NU
        FORCES_NPAR = 58

        NEAR_GOAL_THRESHOLD = 0.2


