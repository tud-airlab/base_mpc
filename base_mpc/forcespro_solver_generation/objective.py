"""
Objective.py defines the objective for the solver. Currently it is system specific, it should be general.
@Todo: Generalize the objectives used.
"""

import numpy as np
import casadi





class CostTerm:

    def __init__(self, weight, variable):
        self.weight = weight
        self.variable = variable

    def cost(self):
        raise IOError('Costterm with undefined cost!')

class QuadraticCost(CostTerm):

    def __init__(self, weight, variable):
        super().__init__(weight, variable)

    def cost(self):
        return self.weight * self.variable ** 2


class BASE_MPCObjective:

    def __init__(self, params, system, config):
        self.define_parameters(params)
        self.system = system
        self.config = config


    def define_parameters(self, params):
        params.add_parameter("x_goal")
        params.add_parameter("y_goal")
        params.add_parameter("psi_goal")
        params.add_parameter("Wrepulsive")
        params.add_parameter("Wx")
        params.add_parameter("Wy")
        params.add_parameter("Wpsi")
        params.add_parameter("Walpha")
        params.add_parameter("Wtheta")
        params.add_parameter("Wa")
        params.add_parameter("Ws")
        params.add_parameter("Wv")
        params.add_parameter("Wforward")
        params.add_parameter("Ww")


    def get_value(self, x, u, settings, stage_idx):

        # print(stage_idx)
        cost = 0
        # if stage_idx == settings.N_bar - 1:  # settings.N - 1:
        # if stage_idx == settings.N - 1:
        if True:
            pos_x = x[0]
            pos_y = x[1]
            pos = x[:2]
            psi = x[2]
            v = x[3]
            w = x[4]

            a = u[0]
            alpha = u[1]

            # Parameters
            x_goal = getattr(settings.params, "x_goal")
            y_goal = getattr(settings.params, "y_goal")
            psi_goal = getattr(settings.params, "psi_goal")
            Wrepulsive = getattr(settings.params, "Wrepulsive")

            Wx = getattr(settings.params, "Wx")
            Wy = getattr(settings.params, "Wy")
            Wpsi = getattr(settings.params, "Wpsi")

            Wv = getattr(settings.params, "Wv")
            Ww = getattr(settings.params, "Ww")
            Wforward = getattr(settings.params, "Wforward")

            Walpha = getattr(settings.params, "Walpha")
            Wa = getattr(settings.params, "Wa")

            Ws = getattr(settings.params, "Ws")

            # Derive position error
            x_error = pos_x - x_goal
            y_error = pos_y - y_goal
            psi_error = psi - psi_goal

            disToGoal = casadi.norm_2(pos- casadi.vertcat(x_goal, y_goal))
            disToGoal = casadi.fmax(disToGoal, self.config.NEAR_GOAL_THRESHOLD) # in case arriving at goal posistion

            max_v_range = self.system.upper_bound['v'] - self.system.lower_bound['v']
            max_w_range = self.system.upper_bound['w'] - self.system.lower_bound['w']
            max_acc_range = self.system.upper_bound['a'] - self.system.lower_bound['a']
            max_alpha_range = self.system.upper_bound['alpha'] - self.system.lower_bound['alpha']

            if u.shape[0] >2: #Todo check meaning
                if stage_idx == self.config.FORCES_N+1: #Todo parameter N
                    cost = Wx*x_error ** 2 / disToGoal + Wy*y_error ** 2/disToGoal + Wpsi *psi_error ** 2   #+ Wv*v*v/max_v_range + Ww*w*w/max_w_range
                else:
                    cost = 0 #Wa * a * a / max_acc_range + Walpha * alpha * alpha / max_alpha_range + Wv * v * v / max_v_range + Ww * w * w / max_w_range
            else: print("not implemented yet")

        return cost




def objective(z, param, model, settings, stage_idx):
    # print("stage idx in jackal_objective: {}".format(stage_idx))
    cost = 0.
    settings.params.load_params(param)

    # Retrieve variables
    x = z[model.nu:model.nu + model.nx]
    u = z[0:model.nu]



    # Weights
    settings.weights.set_weights(param)

    for module in settings.modules.modules:
        if module.type == "objective":
            for module_objective in module.objectives:
                cost += module_objective.get_value(x, u, settings, stage_idx)

    return cost



