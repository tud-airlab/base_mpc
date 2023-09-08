import casadi
import numpy as np
import helpers

""" 
Defines inequality constraints of different types. 
See control_modules.py for their integration into the controller 
"""

# Class to aggregate the number of constraints and nh, nparam
class Constraints:

    def __init__(self, params):
        self.upper_bound = []
        self.lower_bound = []
        self.constraints = []

        self.constraint_modules = []

        self.nh = 0
        # self.npar = 0
        self.params = params
        # self.param_idx = param_idx

    def add_constraint(self, constraint):
        self.constraints.append(constraint) # param_idx keeps track of the indices
        constraint.append_upper_bound(self.upper_bound)
        constraint.append_lower_bound(self.lower_bound)

        self.nh += constraint.nh

    def inequality(self, z, param, settings, model):
        result = []

        for constraint in self.constraints:
            constraint.append_constraints(result, z, param, settings, model)

        return result

class LinearConstraints:

    def __init__(self, params, n_discs, num_constraints):
        self.num_constraints = num_constraints
        self.n_discs = n_discs
        self.params = params

        self.nh = num_constraints * n_discs

        # @Todo: Be able to add multiple sets of constraints
        for disc in range(n_discs):
            for i in range(num_constraints):
                params.add_parameter(self.constraint_name(disc, i) + "_a1")
                params.add_parameter(self.constraint_name(disc, i) + "_a2")
                params.add_parameter(self.constraint_name(disc, i) + "_b")

    def constraint_name(self, disc_idx, constraint_idx):
        return "disc_"+str(disc_idx)+"_linear_constraint_"+str(constraint_idx)

    def append_lower_bound(self, lower_bound):
        for scenario in range(0, self.num_constraints):
            for disc in range(0, self.n_discs):
                lower_bound.append(-np.inf)

    def append_upper_bound(self, upper_bound):
        for scenario in range(0, self.num_constraints):
            for disc in range(0, self.n_discs):
                upper_bound.append(0.0)

    def append_constraints(self, constraints, z, param, settings, model):
        settings.params.load_params(param)

        # Retrieve variables
        x = z[model.nu:model.nu + model.nx]
        u = z[0:model.nu]

        # States
        pos = np.array([x[0], x[1]])
        slack = u[2]
        psi = x[2]

        rotation_car = helpers.rotation_matrix(psi)
        for disc_it in range(0, self.n_discs):
            disc_x = getattr(settings.params, "disc_offset_" + str(disc_it))
            disc_relative_pos = np.array([disc_x, 0])
            disc_pos = pos #+ rotation_car.dot(disc_relative_pos)

            # A'x <= b
            for constraint_it in range(0, self.num_constraints):
                a1 = getattr(settings.params, self.constraint_name(disc_it, constraint_it) + "_a1")
                a2 = getattr(settings.params, self.constraint_name(disc_it, constraint_it) + "_a2")
                b = getattr(settings.params, self.constraint_name(disc_it, constraint_it) + "_b")

                constraints.append(a1 * disc_pos[0] + a2 * disc_pos[1] - b)


class EllipsoidConstraints:

    def __init__(self, n_discs, max_obstacles, params, rotation_clockwise=True):
        self.max_obstacles = max_obstacles
        self.n_discs = n_discs

        self.nh = max_obstacles * n_discs

        # Add parameters
        for obs_id in range(max_obstacles):
            params.add_parameter("ellipsoid_obst_" + str(obs_id) + "_x")
            params.add_parameter("ellipsoid_obst_" + str(obs_id) + "_y")
            params.add_parameter("ellipsoid_obst_" + str(obs_id) + "_psi")
            params.add_parameter("ellipsoid_obst_" + str(obs_id) + "_major")
            params.add_parameter("ellipsoid_obst_" + str(obs_id) + "_minor")
            params.add_parameter("ellipsoid_obst_" + str(obs_id) + "_chi")
            params.add_parameter("ellipsoid_obst_" + str(obs_id) + "_r")



        self.rotation_clockwise = rotation_clockwise

    def append_lower_bound(self, lower_bound):
        for obs in range(0, self.max_obstacles):
            for disc in range(0, self.n_discs):
                lower_bound.append(1.0)

    def append_upper_bound(self, upper_bound):
        for obs in range(0, self.max_obstacles):
            for disc in range(0, self.n_discs):
                upper_bound.append(np.Inf)

    def append_constraints(self, constraints, z, param, settings, model):

        settings.params.load_params(param)

        # Retrieve variables
        x = z[model.nu:model.nu + model.nx]
        u = z[0:model.nu]

        # States
        pos = np.array([x[0], x[1]])
        psi = x[2]
        slack = u[-1]

        rotation_car = helpers.rotation_matrix(psi)

        r_disc = getattr(settings.params, 'disc_r') #param[self.start_param]

        # Constraint for dynamic obstacles
        for obstacle_it in range(0, self.max_obstacles):
            # Retrieve parameters
            obst_x = getattr(settings.params, "ellipsoid_obst_" + str(obstacle_it) + "_x")
            obst_y = getattr(settings.params, "ellipsoid_obst_" + str(obstacle_it) + "_y")
            obst_psi = getattr(settings.params, "ellipsoid_obst_" + str(obstacle_it) + "_psi")
            obst_major = getattr(settings.params, "ellipsoid_obst_" + str(obstacle_it) + "_major")
            obst_minor = getattr(settings.params, "ellipsoid_obst_" + str(obstacle_it) + "_minor")
            obst_r = getattr(settings.params, "ellipsoid_obst_" + str(obstacle_it) + "_r")

            # multiplier for the risk when obst_major, obst_major only denote the covariance
            # (i.e., the smaller the risk, the larger the ellipsoid).
            # This value should already be a multiplier (using exponential cdf).
            chi = getattr(settings.params, "ellipsoid_obst_" + str(obstacle_it) + "_chi")

            # obstacle computations
            obstacle_cog = np.array([obst_x, obst_y])

            # Compute ellipse matrix
            obst_major *= casadi.sqrt(chi)
            obst_minor *= casadi.sqrt(chi)
            ab = np.array([[1. / ((obst_major + (r_disc + obst_r)) ** 2), 0],
                           [0, 1. / ((obst_minor + (r_disc + obst_r)) ** 2)]])

            # In the original LMPCC paper the angle of the obstacles is defined clockwise
            # While it could also make sense to define this anti-clockwise, just like the orientation of the Roboat
            if self.rotation_clockwise:
                obstacle_rotation = helpers.rotation_matrix(obst_psi)
            else:
                obstacle_rotation = helpers.rotation_matrix(-obst_psi)

            obstacle_ellipse_matrix = obstacle_rotation.transpose().dot(ab).dot(obstacle_rotation)

            for disc_it in range(0, self.n_discs):
                # Get and compute the disc position
                disc_x = getattr(settings.params, 'disc_offset_' + str(disc_it))
                disc_relative_pos = np.array([disc_x, 0])
                disc_pos = pos + rotation_car.dot(disc_relative_pos)

                # construct the constraint and append it
                disc_to_obstacle = disc_pos - obstacle_cog
                c_disc_obstacle = disc_to_obstacle.transpose().dot(obstacle_ellipse_matrix).dot(disc_to_obstacle)
                constraints.append(c_disc_obstacle + slack)

