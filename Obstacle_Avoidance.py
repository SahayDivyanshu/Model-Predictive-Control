import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['OBSTACLES'] = True

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 15
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference1 = [10, 0, 0]
        self.reference2 = None

        self.x_obs = 5
        self.y_obs = 0.1

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
        # for k in range(self.horizon):
        #     state = self.plant_model(state,self.dt,u[2*k],u[k+1])
        #     cost += abs(ref[0]-state[0])**2
        #     cost += abs(ref[1]-state[1] - 0.3)**2
        #     cost += abs(ref[2]-state[2])**2
        #     if(state[0]== 4.0 and state[1] == 0.1):
        #         cost = state[3]*100
        # return cost


        #cost function from the solution
        for k in range(self.horizon):
             state = self.plant_model(state,self.dt,u[2*k],u[2*k+1])
             cost += abs(ref[0]-state[0])**2
             cost += abs(ref[1]-state[1])**2
             cost += abs(ref[2]-state[2])**2
             cost+=self.obstacle_cost(state[0],state[1])
        return cost

    def obstacle_cost(self,x,y):
        dist = (x-self.x_obs)**2 + (y-self.y_obs)**2
        dist =  np.sqrt(dist)
        if(dist>2):
            return 15
        else:
            return 1/dist*30    

sim_run(options, ModelPredictiveControl)
