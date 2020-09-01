import numpy as np
from sim.sim2d import sim_run
import math

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['OBSTACLES'] = False

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 20
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference1 = [10, 10, 0]
        self.reference2 = [10, 2, 3.14/2]

    def plant_model(self,prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3]
        x_t_1 = x_t + v_t*np.cos(psi_t)*dt
        y_t_1 = y_t + v_t*np.sin(psi_t)*dt
        a_t = pedal
        v_t_1 = v_t + a_t*dt - v_t/25
        psi_t_1 = psi_t + v_t*np.tan(steering)/2.5*dt
        return [x_t_1, y_t_1, psi_t_1, v_t_1]

    def cost_function(self,u,*args):
        state = args[0]
        ref = args[1]
        cost = 0
        for k in range(self.horizon):
            state = self.plant_model(state,self.dt,u[k],u[k+1])
            cost += abs(ref[0]-state[0])**2
            cost += abs(ref[1]-state[1])**2
            cost += abs(ref[2]-state[2])**2
                      
        return cost

sim_run(options, ModelPredictiveControl)

         
#  '''
#             cost = cost + state[3]*(state[0]-ref[0]) + state[3]*(state[1]-ref[1])
#             if state[0] >= 0:
#                 cost += abs(state[2] - 0.88)
#             if state[0] > 9 and state[1] > 9:
#                 cost = state[3]*(state[2]+0.88)
#             if state[0] == 10 and state[1] == 10 and state[2] == 0:
#                 cost = 0
#             '''