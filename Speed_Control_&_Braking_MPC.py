import numpy as np
from sim.sim1d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['FULL_RECALCULATE'] = False

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 20
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference = [50, 0, 0]

    def plant_model(self, prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        v_t = prev_state[3] # m/s
        x_t_1 = x_t + v_t*dt
        a_t = pedal
        v_t_1 = v_t + a_t*dt - v_t/25
        return [x_t_1, 0, 0, v_t_1]

    def cost_function(self,u,*args):
        state = args[0] #[]
        ref = args[1]
        # x_vect = [1,0,0,0]
        cost = 0        
        for k in range(self.horizon):
            # x_vect = self.plant_model(x_vect,self.dt,u[k],0)
            state = self.plant_model(state,self.dt,u[k],0)
            # cost = cost + (self.reference[0]-x_vect[0])**2
            cost = cost + (self.reference[0]-state[0])**2 
            
            if(state[3]>2.77778):
                cost += state[3]*1000
            
            # cost = cost + (ref[0]-state[0])**2  
            


        return cost

sim_run(options, ModelPredictiveControl)
