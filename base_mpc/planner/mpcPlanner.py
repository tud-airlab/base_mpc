import sys
sys.path.append('/home/luzia/code/forces_pro_client/')  # todo remove
import pickle
import forcespro.nlp

class MPCPlanner(object):

    def __init__(self, robot_type, solver_dir):


        self.solver = forcespro.nlp.Solver.from_directory(solver_dir + "/AlbertFORCESNLPsolver") #todo add robot_type

        with open(solver_dir + "/Albertparams.pkl", 'rb') as file:
            self.parameters = pickle.load(file)



    def updateDynamicObstacles(self, obstArray):
        print('not implemented yet')
    def setx0(self, xinit):
        print('not implemented yet')
    def solve(self, current_state):
        #self._xinit = current_state[0: self._nx]
        self.setX0(self._xinit)
        print('not implemented yet')