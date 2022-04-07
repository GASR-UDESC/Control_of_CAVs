from rps import robotarium
from rps.utilities.misc import *
from rps.utilities.controllers import *

class Simulator:
    def __init__(self, number_of_agents, figure, initial_points, real_time):
        self.n = number_of_agents
        self.robots = robotarium.Robotarium(number_of_robots = number_of_agents,
        show_figure = figure, initial_conditions = initial_points, 
        sim_in_real_time = real_time)

        self.single_integrator_position_controller = create_si_position_controller()
        self.__, self.uni_to_si_states = create_si_to_uni_mapping()
        self.si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion()
        self.x_poses = self.robots.get_poses()
        self.x_states = self.uni_to_si_states(self.x_poses)

        self.robots.step()
    
        
    def step_begin(self):
        self.x_poses = self.robots.get_poses()
        self.x_states = self.uni_to_si_states(self.x_poses)

        return(self.x_poses, self.x_states)
    

    def step_end(self, goal):
        dxi = self.single_integrator_position_controller(self.x_states, goal[:2][:])
        dxu = self.si_to_uni_dyn(dxi, self.x_poses)
        self.robots.set_velocities(np.arange(self.n), dxu)

        self.robots.step()


#Allways call Simulator.robots.call_at_scripts_end() at the end of each simulation

