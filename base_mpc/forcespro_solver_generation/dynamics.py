import copy

import numpy as np
import casadi
from typing import List
import forcespro.nlp
import casadi as ca


# Returns discretized dynamics of a given model (see below)
def discrete_dynamics(z, model, integrator_stepsize):
    """

    @param z: state vector (u, x)
    @param model: Model of the system
    @param integrator_stepsize: Integrator stepsize in seconds
    @return:
    """
    # We use an explicit RK4 integrator here to discretize continuous dynamics

    result = forcespro.nlp.integrate(
        model.continuous_model,
        z[model.nu:model.nu+model.nx],
        z[0:model.nu],
        integrator=forcespro.nlp.integrators.RK4,
        stepsize=integrator_stepsize)

    return result

def add_dual(model, nh):
    model.nu += nh
    model.continuous_model = lambda x, u: casadi.vstack(model.continuous_model(x, u), np.zeros((nh, 1))) # lambdas have no dynamics

# Dynamics, i.e. equality constraints #
# This class contains models to choose from
# They can be coupled with physical limits using Systems defined in systems.py
# See Bicycle model for an example of the required parameters
class DynamicModel:

    def __init__(self, system):
        self.nvar = self.nu + self.nx
        self.system = system

    def __str__(self):
        result = 'Dynamical Model: ' + str(type(self)) + '\n' +\
               'System: ' + str(self.system) + '\n'

        if hasattr(self, 'interfaces'):
            result += 'Interfaces: '

            for interface in self.interfaces:
                result += interface + " "

            result += "\n"

        result += 'States: ' + str(self.states) + '\n'
        result += 'Inputs: ' + str(self.inputs) + '\n'
        return result

    # Appends upper bounds from system
    def upper_bound(self):
        result = np.array([])

        for input in self.inputs:
            result = np.append(result, self.system.upper_bound[input])

        for state in self.states:
            result = np.append(result, self.system.upper_bound[state])

        return result

    # Appends lower bounds from system
    def lower_bound(self):
        result = np.array([])

        for input in self.inputs:
            result = np.append(result, self.system.lower_bound[input])

        for state in self.states:
            result = np.append(result, self.system.lower_bound[state])

        return result


# Second-order Bicycle model
class BicycleModel(DynamicModel):

    def __init__(self, system):
        self.nu = 3 # number of control variables
        self.nx = 5 # number of states

        super(BicycleModel, self).__init__(system)

        self.states = ['x', 'y', 'psi', 'v', 'w']  # , 'ax', 'ay'
        self.states_from_sensor = [True, True, True, True, True]  # , True, True
        self.states_from_sensor_at_infeasible = [True, True, True, True, True]  # False variables are guessed 0 at infeasible

        self.inputs = ['a', 'alpha', 'slack']
        self.inputs_to_vehicle = [True, True, False]
        self.possible_inputs_to_vehicle = ['v', 'a' ,'alpha','w', 'psi']

    def continuous_model(self, x, u):
        a = u[0]
        alpha = u[1]
        psi = x[2]
        v = x[3]
        w = x[4]

        return np.array([v * casadi.cos(psi),
                         v * casadi.sin(psi),
                         w,
                         a,
                         alpha])

class PointMass_1order_Model(DynamicModel):

    def __init__(self, system):
        self.nu = 3 # number of control variables
        self.nx = 5 # number of states

        super(PointMass_1order_Model, self).__init__(system)

        self.states = ['x', 'y', 'psi', 'v_x', 'v_y']  # , 'ax', 'ay'
        self.states_from_sensor = [True, True, True, True, True]  # , True, True
        self.states_from_sensor_at_infeasible = [True, True, True, True, True]  # False variables are guessed 0 at infeasible

        self.inputs = ['a_x', 'a_y', 'slack']
        self.inputs_to_vehicle = [True, True, False]
        self.possible_inputs_to_vehicle = ['a_x', 'a_y']

    def continuous_model(self, x, u):


        a_x = u[0]
        a_y = u[1]
        psi = x[2]
        v_x = x[3]
        v_y = x[4]

        return np.array([v_x,
                         v_y,
                         ca.arctan(v_y,v_x),
                         a_x,
                         a_y])


    def upper_bound(self):
        result = np.array([])

        for model in [self.models[0]]:
            for input in model.inputs:
                result = np.append(result, model.system.upper_bound[input])

        for model in self.models:
            for state in model.states:
                result = np.append(result, model.system.upper_bound[state])

        return result

    # Appends lower bounds from system
    def lower_bound(self):
        result = np.array([])

        for model in [self.models[0]]:
            for input in model.inputs:
                result = np.append(result, model.system.lower_bound[input])

        for model in self.models:
            for state in model.states:
                result = np.append(result, model.system.lower_bound[state])


        return result





