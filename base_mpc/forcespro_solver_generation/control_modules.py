import inequality
import objective

class ModuleManager:
    """
    The idea of modules is that they can include multiple constraint sets if necessary
    In addition, they are directly linked to the c++ code module
    """

    def __init__(self, params):
        self.constraint_manager = inequality.Constraints(params)
        self.modules = []

        self.params = params

    def add_module(self, module):
        self.modules.append(module)

        if module.type == "constraint":
            for constraint in module.constraints:
                self.constraint_manager.add_constraint(constraint)

    def inequality_constraints(self, z, param, settings, model):
        return self.constraint_manager.inequality(z, param, settings, model)

    def number_of_constraints(self):
        return self.constraint_manager.nh

    def __str__(self):
        result = "--- MPC Modules ---\n"
        for module in self.modules:
            result += str(module) + "\n"

        return result


class Module:

    def __init__(self):
        self.module_name = "UNDEFINED"

    def __str__(self):
        result = self.type.capitalize() + ": " + self.module_name
        return result


""" OBJECTIVE MODULES """
class BASE_MPCModule(Module):

    """
    Navigate to a subgoal/goal
    """

    def __init__(self, params, system, config):
        self.module_name = "NOTDEFINED"  # Needs to correspond to the c++ name of the module
        self.import_name = "NOTDEFINED"
        self.type = "objective"
        self.system = system

        self.objectives = []
        self.objectives.append(objective.BASE_MPCObjective(params, system, config))


""" CONSTRAINT MODULES """


class EllipsoidalConstraintModule(Module):

    """
    Linear constraints for scenario-based motion planning
    """

    def __init__(self, params, n_discs, n_max_obstacles):
        self.module_name = "EllipsoidalConstraints"  # Needs to correspond to the c++ name of the module
        self.import_name = "modules_constraints/ellipsoidal_constraints.h"
        self.type = "constraint"

        self.constraints = []
        self.constraints.append(inequality.EllipsoidConstraints(n_discs, n_max_obstacles, params))

class MultiEllipsoidalConstraintModule(Module):

    """
    Linear constraints for scenario-based motion planning
    """

    def __init__(self, params, n_discs, max_obstacles):
        self.module_name = "MultiEllipsoidalConstraints"  # Needs to correspond to the c++ name of the module
        self.import_name = "modules_constraints/ellipsoidal_constraints.h"
        self.type = "constraint"

        self.constraints = []
        self.constraints.append(inequality.MultiEllipsoidConstraints(n_discs, max_obstacles, params))

class LinearConstraintModule(Module):

    """
    Linear constraints for scenario-based motion planning
    """

    def __init__(self, params, n_discs, num_constraints):
        self.module_name = "TOBEDEFINED"  # Needs to correspond to the c++ name of the module
        self.import_name = "TOBEDEFINED"
        self.type = "constraint"

        self.constraints = []
        self.constraints.append(inequality.LinearConstraints(params,n_discs, num_constraints))


class InteractiveLinearConstraintModule(Module):

    """
    Linear constraints for scenario-based motion planning
    """

    def __init__(self, params, n_discs, max_obstacles):
        self.module_name = "TOBEDEFINED"  # Needs to correspond to the c++ name of the module
        self.import_name = "TOBEDEFINED"
        self.type = "constraint"

        self.constraints = []
        self.constraints.append(inequality.InteractiveLinearConstraints(params,n_discs, max_obstacles))

