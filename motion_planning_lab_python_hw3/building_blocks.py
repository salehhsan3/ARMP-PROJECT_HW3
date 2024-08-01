import numpy as np
import random

def sphere_collision(p1,p2,r1,r2):
    return np.linalg.norm(np.array(p1)-np.array(p2)) < r1 + r2

def max_angle_difference(conf1, conf2):
    max_difference = 0
    for angle1, angle2 in zip(conf1, conf2):
        # normalize angles to [0, 2 * np.pi] range
        angle1, angle2 = angle1 % (2 * np.pi), angle2 % (2 * np.pi) 
        difference = abs(angle1 - angle2)
        # Consider the circular nature of angles
        difference = min(difference % (2 * np.pi), (2 * np.pi - difference) % (2 * np.pi))
        max_difference = max(max_difference, difference)
    return max_difference

def get_closest_config(conflist, base_config):
    best_dist = float('inf')
    best_conf = []
    for config in conflist:
        dist = sum(pow(a-b,2) for a, b in zip(base_config,config))
        if(dist < best_dist):
            dist=best_dist
            best_conf = config
    return best_conf
    
class Building_Blocks(object):
    '''
    @param resolution determines the resolution of the local planner(how many intermidiate configurations to check)
    @param p_bias determines the probability of the sample function to return the goal configuration
    '''
    def __init__(self, transform, ur_params, env, resolution=0.1, p_bias=0.05):
        self.transform = transform
        self.ur_params = ur_params
        self.env = env
        self.resolution = resolution
        self.p_bias = p_bias
        self.cost_weights = np.array([0.4, 0.3 ,0.2 ,0.1 ,0.07 ,0.05])
        
    def sample(self, goal_conf) -> np.array:
        """
        sample random configuration
        @param goal_conf - the goal configuration
        """
        # DONE
        conf = []
        if random.random() < self.p_bias:
            return np.array(goal_conf)
        
        for joint, range in self.ur_params.mechanical_limits.items():
            conf.append(random.uniform(range[0],range[1]))
        
        return np.array(conf)
        

    def is_in_collision(self, conf) -> bool:
        """check for collision in given configuration, arm-arm and arm-obstacle
        return True if in collision
        @param conf - some configuration 
        """
        # TODO 
        # hint: use self.transform.conf2sphere_coords(), self.ur_params.sphere_radius, self.env.obstacles
        global_sphere_coords = self.transform.conf2sphere_coords(conf)
        # arm - arm collision

        for i in range(len(self.ur_params.ur_links) - 2):
            for j in range(i+2,len(self.ur_params.ur_links)): # maybe change to (i+1) because of what was said in the group chat?
                spheres = global_sphere_coords[self.ur_params.ur_links[i]]
                other_spheres = global_sphere_coords[self.ur_params.ur_links[j]]
                for s in spheres:
                    for s2 in other_spheres:
                        if sphere_collision(s[0:3],s2[0:3],self.ur_params.sphere_radius[self.ur_params.ur_links[i]],self.ur_params.sphere_radius[self.ur_params.ur_links[j]]):
                            return True
        
        # arm - obstacle collision  
        for joint, spheres in global_sphere_coords.items():
            for sphere in spheres:
                for obstacle in self.env.obstacles:
                    
                    if sphere_collision(sphere[0:3],obstacle,self.ur_params.sphere_radius[joint],self.env.radius):
                        return True

        # arm - floor collision
        for joint, spheres in global_sphere_coords.items(): #temp base always hits floor
            if(joint == self.ur_params.ur_links[0]):
                continue
            for sphere in spheres:
                if sphere[2] < self.ur_params.sphere_radius[joint]:
                    return True
        
        # x - axis limit
        for joint, spheres in global_sphere_coords.items(): #temp base always hits floor
            for sphere in spheres:
                if sphere[0] + self.ur_params.sphere_radius[joint] > 0.4:
                    return True
        return False
        
    
    def local_planner(self, prev_conf ,current_conf) -> bool:
        '''check for collisions between two configurations - return True if trasition is valid
        @param prev_conf - some configuration
        @param current_conf - current configuration 
        '''
        #TODO do bound interpolation (-10 to 350 degrees should be 0 not 180)
        # number_of_configurations_to_check = 3
        number_of_configurations_to_check = max( 3, int(max_angle_difference(prev_conf, current_conf) / self.resolution) )
        return not any([self.is_in_collision(np.array(conf)) for conf in np.linspace(prev_conf,current_conf, number_of_configurations_to_check, endpoint=True)])
        
    
    def edge_cost(self, conf1, conf2):
        '''
        Returns the Edge cost- the cost of transition from configuration 1 to configuration 2
        @param conf1 - configuration 1
        @param conf2 - configuration 2
        '''
        return np.dot(self.cost_weights, np.power(conf1-conf2,2)) ** 0.5