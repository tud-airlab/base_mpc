import sys
sys.path.append('..')
import control_modules
import helpers
from config import AlbertConfig as Robot
from config import MPCConfig as MPCConfig

N = 10                    # MPC Horizon
integrator_stepsize = 0.2   # Timestep of the integrator
n_discs = 1  # Number of discs modeling the vehicle collision region
system = Robot()

use_sqp_solver = False      # Note: SQP for scenario has max_it = 1
enable_repulsive = False    # Enable repulsive fields in the objective?

# --- Constraint Selection --- #
use_scenario_constraints = False      # Scenario approach
use_ellipsoid_constraints = False    # Ellipsoid obstacle models
use_gaussian_constraints = False
use_linear_constraints = False

module_selection = 1 # 1: GO-MPC

if module_selection == 1:
    use_linear_constraints = True
    use_ellipsoid_constraints = True

# --- Interface Selection --- #
interfaces = ['AlbertSimulator', 'Albert']  # Define the interfaces that this solver can run with
# @Note: With more than one interface, the IDE cannot automatically resolve the interface, giving errors

# --- Ellipsoid Settings --- #
if use_ellipsoid_constraints or use_gaussian_constraints:
    modes = 1
    n_max_obstacles = 5     # Maximum dynamic obstacles to evade with the planner
    n_max_obstacles *= modes  # To account for the number of modes, we need an ellipsoid per mode!

if use_linear_constraints:
    num_constraints = 8

# --- SQP Settings --- #
if use_sqp_solver:
    print_init_bfgs = True

# === Constraint Definition === #
# ! DO NOT CHANGE THE ORDER ANY PARTS BELOW ! (i.e., parameters first, then inequalities!)
params = helpers.RuntimeParameterStructure()
modules = control_modules.ModuleManager(params)

modules.add_module(control_modules.BASE_MPCModule(params, system, MPCConfig))  # Track a reference path

# All added weights must also be in the .cfg file to function with the lmpcc_controller.cpp code!

weight_list = list()

weights = helpers.WeightStructure(params, weight_list)

# --- Inequality Constraints --- #
# Parameters for the vehicle collision region
params.add_parameter("disc_r")
params.add_multiple_parameters("disc_offset", n_discs)


#
if use_ellipsoid_constraints:
    modules.add_module(control_modules.EllipsoidalConstraintModule(params, n_discs=n_discs, n_max_obstacles=n_max_obstacles))

if use_linear_constraints:
    modules.add_module(control_modules.LinearConstraintModule(params, n_discs=n_discs, num_constraints=num_constraints))

# === Collect Constraint Data === #
print(params)
npar = params.n_par()
nh = modules.number_of_constraints()

print(' ')