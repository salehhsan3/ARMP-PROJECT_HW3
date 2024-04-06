import numpy as np
import os
from environment import Environment
from kinematics import UR5e_PARAMS, Transform
from planners import RRT_STAR
from building_blocks import Building_Blocks
from visualizer import Visualize_UR


def startVisualization():
    ur_params = UR5e_PARAMS(inflation_factor=1)
    transform = Transform(ur_params)
    
    #TODO - change the position of the cubes during the task
    cube1_coords = [-0.10959248574268822, -0.6417732149769166, 0.1390226933317033]
    cube2_coords = [0.08539928976845282, -0.8370930220946053, 0.13813472317717034]
    cube3_coords =  [-0.008445229140271685, -0.7365370847309188, 0.00955541284784159]
    cube4_coords = [0.23647185443765273 ,-0.769747539513382, 0.03971366463235271]
    cube5_coords =[0.26353072323141574 ,-0.4629969534200313, 0.2651034131371637]
    cube6_coords =  [0.26940059242703984, -0.4730222745248458, 0.021688493137064376]
    initial_cubes_coords = [cube1_coords,cube2_coords,cube3_coords,cube4_coords,cube5_coords,cube6_coords]
    
    ##############################################################################################
    fictional_ground = 0.1
    # for Paulo - constructing a: P
    cube1_final = [-0.25,-0.33, fictional_ground]
    cube2_final = [-0.25,-0.50, fictional_ground]
    cube3_final = [-0.25,-0.25, fictional_ground]
    cube4_final = [-0.175,-0.33, fictional_ground]
    cube5_final = [-0.175,-0.25, fictional_ground]
    cube6_final = [-0.25,-0.4, fictional_ground]
    dir_name = 'P-path'
    ##############################################################################################
    # for Saleh - constructing a: S
    # cube1_final = [-0.08, -0.42, fictional_ground]
    # cube2_final = [-0.15, -0.48, fictional_ground]
    # cube3_final = [-0.23, -0.43, fictional_ground]
    # cube4_final = [-0.14, -0.35, fictional_ground]
    # cube5_final = [-0.2, -0.28, fictional_ground]
    # cube6_final = [-0.12, -0.24, fictional_ground]
    # dir_name = 'C:\\Users\\Saleh\\Desktop\\ARMP\\ARMP-PROJECT_HW3\\S-path'
    ##############################################################################################
    
    final_cubes_coords = [cube1_final,cube2_final,cube3_final,cube4_final,cube5_final,cube6_final]
    interim_cube_coords = [c for c in initial_cubes_coords]
    
    env = Environment(env_idx=3, cube_coords=initial_cubes_coords)
    bb = Building_Blocks(transform=transform, ur_params=ur_params, env=env, resolution=0.1, p_bias=0.05,)
    visualizer = None
    
    at_cube = False
    i = 0
    while i < len(final_cubes_coords):
        if at_cube:
            interim_cube_coords[i] = final_cubes_coords[i]
            filename = f'cube{i+1}Tocube{i+1}_goal'
            i += 1
        else:
            if i == 0:
                filename = f'homeTocube{i+1}'
            else:
                filename = f'{"cube" + str(i) + "_goal" if i > 0 else "home"}Tocube{i+1}_approach'
        at_cube = not at_cube
        env = Environment(env_idx=3, cube_coords=initial_cubes_coords)
        bb = Building_Blocks(transform=transform, ur_params=ur_params, env=env, resolution=0.1, p_bias=0.05,)
        visualizer = Visualize_UR(ur_params, env=env, transform=transform, bb=bb)
        try:
            print(os.path.join(dir_name, filename) + '_path.npy')
            path = np.load(os.path.join(dir_name, filename) + '_path.npy')
            visualizer.show_path(path)
            # visualizer.show_conf(path[-1])
        except:
            print('No Path Found')
        initial_cubes_coords = [c for c in interim_cube_coords]
    visualizer.show_conf(path[-1])
if __name__ == '__main__':
    startVisualization()()



