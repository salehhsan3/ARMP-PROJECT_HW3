import numpy as np
import os
from environment import Environment
from kinematics import UR5e_PARAMS, Transform
from planners import RRT_STAR
from building_blocks import Building_Blocks
from visualizer import Visualize_UR
from building_blocks import get_closest_config
from inverse_kinematics import get_valid_inverse_solutions

def main():
    ur_params = UR5e_PARAMS(inflation_factor=1)
    transform = Transform(ur_params)

    # --------- configurations-------------
    home = np.deg2rad([0, -90, 0, -90, 0,0 ])

    cube1_approach = np.deg2rad([68.8,-68.3, 84.2,-107.1,-90,-18]) #approach without collision
    cube1 = np.deg2rad([69,-63,85, -107.7,-91.3,-18.2]) #actual position of the cube 
    # cube1_goal example - change as you wish #TODO
    # cube1_goal = np.array([-1.20223872,-1.434,-2.3944, -0.883,1.5707,-2.7730])#found by inverse kinematics
    cube1_goal = None

    cube2_approach = np.deg2rad([87.5, -45.5, 47.7, -102, -90.6, 3.3]) #approach without collision
    cube2 = np.deg2rad([86.9, -40.1, 47.7, -102, -90.6, 3.3]) #actual position of the cube 
    #TODO# cube2_goal = #found by inverse kinematics
    cube2_goal = None

    cube3_approach = np.deg2rad([79.7,-46.9,69.7,-105.1,-92.6,-10.1]) #approach without collision
    cube3 = np.deg2rad([80.2, -43.4, 68.9, -107.1,-93.9, -9.4]) #actual position of the cube 
    #TODO# cube3_goal = #found by inverse kinematics
    cube3_goal = None

    # cube4_approach = np.deg2rad([97.6, -38.3,-52.1, -100.8, -90.1, 8.5]) #approach without collision
    cube4_approach = np.deg2rad([97.6, -38.3, 52.1, -100.8, -90.1, 8.5]) 
    cube4 = np.deg2rad([97.6, -34.6, 52.3, -100.8, -90.1, 8.5]) #actual position of the cube 
    #TODO# cube4_goal = #found by inverse kinematics
    cube4_goal = None

    cube5_approach = np.deg2rad([104.6, -85.3,87.7, -90.5,-88.3, 17]) #approach without collision
    cube5 = np.deg2rad([105.1, -86.3, 97.4, -102, -89.8, 19.3]) #actual position of the cube 
    #TODO# cube5_goal = #found by inverse kinematics
    cube5_goal = None
    
    cube6_approach = np.deg2rad([78.3, -61.7, 120.9, -87.6,-12.9, 27.7]) #approach without collision
    cube6 = np.deg2rad([78,-56.6,120.9, -93,-12.7,29.4]) #actual position of the cube 
    #TODO# cube6_goal =#found by inverse kinematics 
    cube6_goal = None
    
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
    # cube1_final = [-0.106, -0.483, fictional_ground]
    # cube2_final = [-0.106, -0.29, fictional_ground]
    # cube3_final = [-0.106, -0.385, fictional_ground]
    # cube4_final = [-0.176, -0.483, fictional_ground]
    # cube5_final = [-0.166, -0.383, fictional_ground] # always fails with inverse kinematics
    # cube6_final = [-0.22, -0.43, fictional_ground]
    # cube1_final = [-0.25,-0.33, fictional_ground]
    # cube2_final = [-0.25,-0.50, fictional_ground]
    # cube3_final = [-0.25,-0.25, fictional_ground]
    # cube4_final = [-0.175,-0.33, fictional_ground]
    # cube5_final = [-0.175,-0.25, fictional_ground]
    # cube6_final = [-0.25,-0.4, fictional_ground]
    # dir_name = 'P-path'
    ##############################################################################################
    # for Saleh - constructing a: S
    cube1_final = [-0.08, -0.42, fictional_ground]
    cube2_final = [-0.15, -0.48, fictional_ground]
    cube3_final = [-0.23, -0.43, fictional_ground]
    cube4_final = [-0.14, -0.35, fictional_ground]
    cube5_final = [-0.2, -0.28, fictional_ground]
    cube6_final = [-0.12, -0.24, fictional_ground]
    # cube1_final = [-0.1,-0.25, fictional_ground]
    # cube2_final = [-0.20,-0.25, fictional_ground]
    # cube3_final = [-0.13,-0.35, fictional_ground]
    # cube4_final = [-0.07,-0.43, fictional_ground]
    # cube5_final = [-0.01,-0.50, fictional_ground]
    # cube6_final = [-0.15,-0.50, fictional_ground]
    dir_name = 'S-Path-new'
    ##############################################################################################
    final_cubes_coords = [cube1_final,cube2_final,cube3_final,cube4_final,cube5_final,cube6_final]

    # example of how to find coordinates x-y-z from given configuration
    # cubex_goal = np.array([-1.20223872,-1.434,-2.3944, -0.883,1.5707,-2.7730])
    # manipulator_spheres = transform.conf2sphere_coords(cubex_goal) # forward kinematics
    # cube_coords = manipulator_spheres['wrist_3_link'][-1][:3] # take the position of the cube
    
    # manipulator_spheres = transform.conf2sphere_coords(cube4) # forward kinematics
    # cube_coords = manipulator_spheres['wrist_3_link'][-1][:3] # take the position of the cube
    # print(cube4)
    
    cubes = [cube1, cube2, cube3, cube4, cube5, cube6]
    cube_approaches = [cube1_approach, cube2_approach, cube3_approach, cube4_approach, cube5_approach, cube6_approach]
    cube_goals = [cube1_goal, cube2_goal, cube3_goal, cube4_goal, cube5_goal, cube6_goal]
    
    
    
    ########################################
    # rrt_start = home
    # rrt_goal = cube1_approach
    # env = Environment(env_idx=3, cube_coords=initial_cubes_coords)
    # bb = Building_Blocks(transform=transform, ur_params=ur_params, env=env, resolution=0.1, p_bias=0.05,)
    # rrt_star_planner = RRT_STAR(max_step_size=0.5, max_itr=10000, bb=bb)
    # filename = f'homeTocube1'
    # rrt_path = rrt_star_planner.find_path(start_conf=rrt_start, goal_conf=rrt_goal, filename=filename,)
    # add_before = None
    # add_after = cube1
    # if bb.is_in_collision(rrt_start):
    #     print('start in collision:  ')
    # if bb.is_in_collision(rrt_goal): 
    #     print('goal in collision')
    # path = []
    # if add_before is not None:
    #     path.append(add_before)
    # for conf in rrt_path:
    #     path.append(conf)
    # if add_after is not None:
    #     path.append(add_after)
    # print(path)
    # np.save(os.path.join(dir_name,filename+'_path'), np.array(path))
    
    #############################################
    
    # rrt_start = cube1_approach
    # cube1_goal = np.array(get_closest_config(get_valid_inverse_solutions(*final_cubes_coords[0],bb=bb), cubes[0]))
    # print('cube1_goal: ', cube1_goal)
    # rrt_goal = cube1_goal
    # env = Environment(env_idx=3, cube_coords=initial_cubes_coords)
    # bb = Building_Blocks(transform=transform, ur_params=ur_params, env=env, resolution=0.1, p_bias=0.05,)
    # rrt_star_planner = RRT_STAR(max_step_size=0.5, max_itr=10000, bb=bb)
    # filename = f'cube{1}Tocube{1}_goal'
    # rrt_path = rrt_star_planner.find_path(start_conf=rrt_start, goal_conf=rrt_goal, filename=filename,)
    # add_before = cube1
    # add_after = None
    # if bb.is_in_collision(rrt_start):
    #     print('start in collision:  ')
    # if bb.is_in_collision(rrt_goal): 
    #     print('goal in collision')
    # path = []
    # if add_before is not None:
    #     path.append(add_before)
    # for conf in rrt_path:
    #     path.append(conf)
    # if add_after is not None:
    #     path.append(add_after)
    # print(filename + '_path: \n', path)
    # np.save(os.path.join(dir_name,filename+'_path'), np.array(path))
    
    # #############################################
    
    # rrt_start = np.array([-1.38036905, -1.67990399, -1.81207705, -0.62704438,  1.39280031, -2.38466447])
    # rrt_goal = cube2_approach
    # initial_cubes_coords[0] = cube1_final
    # env = Environment(env_idx=3, cube_coords=initial_cubes_coords)
    # bb = Building_Blocks(transform=transform, ur_params=ur_params, env=env, resolution=0.1, p_bias=0.05,)
    # rrt_star_planner = RRT_STAR(max_step_size=0.5, max_itr=10000, bb=bb)
    # filename = f'cube{1}_goalTocube{2}_approach'
    # rrt_path = rrt_star_planner.find_path(start_conf=rrt_start, goal_conf=rrt_goal, filename=filename,)
    # add_before = cube1_goal
    # add_after = cube2
    # if bb.is_in_collision(rrt_start):
    #     print('start in collision:  ')
    # if bb.is_in_collision(rrt_goal): 
    #     print('goal in collision')
    # path = []
    # if add_before is not None:
    #     path.append(add_before)
    # for conf in rrt_path:
    #     path.append(conf)
    # if add_after is not None:
    #     path.append(add_after)
    # print(path)
    # np.save(os.path.join(dir_name,filename+'_path'), np.array(path))
    
    # #############################################
    
    # rrt_start = cube2_approach
    # cube2_goal = np.array(get_closest_config(get_valid_inverse_solutions(*final_cubes_coords[1],bb=bb), cubes[1]))
    # rrt_goal = cube2_goal
    # env = Environment(env_idx=3, cube_coords=initial_cubes_coords)
    # bb = Building_Blocks(transform=transform, ur_params=ur_params, env=env, resolution=0.1, p_bias=0.05,)
    # rrt_star_planner = RRT_STAR(max_step_size=0.5, max_itr=10000, bb=bb)
    # filename = f'cube{2}Tocube{2}_goal'
    # rrt_path = rrt_star_planner.find_path(start_conf=rrt_start, goal_conf=rrt_goal, filename=filename,)
    # add_before = cube2
    # add_after = None
    # if bb.is_in_collision(rrt_start):
    #     print('start in collision:  ')
    # if bb.is_in_collision(rrt_goal): 
    #     print('goal in collision')
    # path = []
    # if add_before is not None:
    #     path.append(add_before)
    # for conf in rrt_path:
    #     path.append(conf)
    # if add_after is not None:
    #     path.append(add_after)
    # print(path)
    # np.save(os.path.join(dir_name,filename+'_path'), np.array(path))
    
    
    # #############################################
    
    # rrt_start = np.array([-1.57675188, -1.72734683, -1.98413587, -0.70692361,  1.42567562, 2.78277216])
    # rrt_goal = cube3_approach
    # initial_cubes_coords[1] = cube2_final
    # env = Environment(env_idx=3, cube_coords=initial_cubes_coords)
    # bb = Building_Blocks(transform=transform, ur_params=ur_params, env=env, resolution=0.1, p_bias=0.05,)
    # rrt_star_planner = RRT_STAR(max_step_size=0.5, max_itr=10000, bb=bb)
    # filename = f'cube{2}_goalTocube{3}_approach'
    # rrt_path = rrt_star_planner.find_path(start_conf=rrt_start, goal_conf=rrt_goal, filename=filename,)
    # add_before = cube2_goal
    # add_after = cube3
    # if bb.is_in_collision(rrt_start):
    #     print('start in collision:  ')
    # if bb.is_in_collision(rrt_goal): 
    #     print('goal in collision')
    # path = []
    # if add_before is not None:
    #     path.append(add_before)
    # for conf in rrt_path:
    #     path.append(conf)
    # if add_after is not None:
    #     path.append(add_after)
    # print(path)
    # np.save(os.path.join(dir_name,filename+'_path'), np.array(path))
    
    # #############################################
    
    # rrt_start = cube3_approach
    # cube3_goal = np.array(get_closest_config(get_valid_inverse_solutions(*final_cubes_coords[2],bb=bb), cubes[2]))
    # rrt_goal = cube3_goal
    # env = Environment(env_idx=3, cube_coords=initial_cubes_coords)
    # bb = Building_Blocks(transform=transform, ur_params=ur_params, env=env, resolution=0.1, p_bias=0.05,)
    # rrt_star_planner = RRT_STAR(max_step_size=0.5, max_itr=10000, bb=bb)
    # filename = f'cube{3}Tocube{3}_goal'
    # rrt_path = rrt_star_planner.find_path(start_conf=rrt_start, goal_conf=rrt_goal, filename=filename,)
    # add_before = cube3
    # add_after = None
    # if bb.is_in_collision(rrt_start):
    #     print('start in collision:  ')
    # if bb.is_in_collision(rrt_goal): 
    #     print('goal in collision')
    # path = []
    # if add_before is not None:
    #     path.append(add_before)
    # for conf in rrt_path:
    #     path.append(conf)
    # if add_after is not None:
    #     path.append(add_after)
    # print(path)
    # np.save(os.path.join(dir_name,filename+'_path'), np.array(path))
    
    
    # #############################################
    
    # rrt_start = np.array([-1.693318591940113, -1.7426600324299304, -2.142571180043586, -0.37944108770860074, 1.259764332989771, 2.171212959306417])
    # rrt_goal = cube4_approach
    # initial_cubes_coords[2] = cube3_final
    # env = Environment(env_idx=3, cube_coords=initial_cubes_coords)
    # bb = Building_Blocks(transform=transform, ur_params=ur_params, env=env, resolution=0.1, p_bias=0.05,)
    # rrt_star_planner = RRT_STAR(max_step_size=0.5, max_itr=10000, bb=bb)
    # filename = f'cube{3}_goalTocube{4}_approach'
    # rrt_path = rrt_star_planner.find_path(start_conf=rrt_start, goal_conf=rrt_goal, filename=filename,)
    # add_before = cube3_goal
    # add_after = cube4
    # if bb.is_in_collision(rrt_start):
    #     print('start in collision:  ')
    # if bb.is_in_collision(rrt_goal): 
    #     print('goal in collision')
    # path = []
    # if add_before is not None:
    #     path.append(add_before)
    # for conf in rrt_path:
    #     path.append(conf)
    # if add_after is not None:
    #     path.append(add_after)
    # print(path)
    # np.save(os.path.join(dir_name,filename+'_path'), np.array(path))
    
    # #############################################
    
    # rrt_start = cube4_approach
    # cube4_goal = np.array(get_closest_config(get_valid_inverse_solutions(*final_cubes_coords[3],bb=bb), cubes[3]))
    # rrt_goal = cube4_goal
    # env = Environment(env_idx=3, cube_coords=initial_cubes_coords)
    # bb = Building_Blocks(transform=transform, ur_params=ur_params, env=env, resolution=0.1, p_bias=0.05,)
    # rrt_star_planner = RRT_STAR(max_step_size=0.5, max_itr=10000, bb=bb)
    # filename = f'cube{4}Tocube{4}_goal'
    # rrt_path = rrt_star_planner.find_path(start_conf=rrt_start, goal_conf=rrt_goal, filename=filename,)
    # add_before = cube4
    # add_after = None
    # if bb.is_in_collision(rrt_start):
    #     print('start in collision:  ')
    # if bb.is_in_collision(rrt_goal): 
    #     print('goal in collision')
    # path = []
    # if add_before is not None:
    #     path.append(add_before)
    # for conf in rrt_path:
    #     path.append(conf)
    # if add_after is not None:
    #     path.append(add_after)
    # print(path)
    # np.save(os.path.join(dir_name,filename+'_path'), np.array(path))
    
    
    # #############################################
    
    # rrt_start = TOFILL
    # rrt_goal = cube5_approach
    # initial_cubes_coords[3] = cube4_final
    # env = Environment(env_idx=3, cube_coords=initial_cubes_coords)
    # bb = Building_Blocks(transform=transform, ur_params=ur_params, env=env, resolution=0.1, p_bias=0.05,)
    # rrt_star_planner = RRT_STAR(max_step_size=0.5, max_itr=10000, bb=bb)
    # filename = f'cube{4}_goalTocube{5}_approach'
    # rrt_path = rrt_star_planner.find_path(start_conf=rrt_start, goal_conf=rrt_goal, filename=filename,)
    # add_before = None
    # add_after = cube5
    # if bb.is_in_collision(rrt_start):
    #     print('start in collision:  ')
    # if bb.is_in_collision(rrt_goal): 
    #     print('goal in collision')
    # path = []
    # if add_before is not None:
    #     path.append(add_before)
    # for conf in rrt_path:
    #     path.append(conf)
    # if add_after is not None:
    #     path.append(add_after)
    # print(path)
    # np.save(os.path.join(dir_name,filename+'_path'), np.array(path))
    
    # #############################################
    
    # rrt_start = cube5_approach
    # cube5_goal = np.array(get_closest_config(get_valid_inverse_solutions(*final_cubes_coords[4],bb=bb), cubes[4]))
    # rrt_goal = cube5_goal
    # env = Environment(env_idx=3, cube_coords=initial_cubes_coords)
    # bb = Building_Blocks(transform=transform, ur_params=ur_params, env=env, resolution=0.1, p_bias=0.05,)
    # rrt_star_planner = RRT_STAR(max_step_size=0.5, max_itr=10000, bb=bb)
    # filename = f'cube{5}Tocube{5}_goal'
    # rrt_path = rrt_star_planner.find_path(start_conf=rrt_start, goal_conf=rrt_goal, filename=filename,)
    # add_before = cube5
    # add_after = None
    # if bb.is_in_collision(rrt_start):
    #     print('start in collision:  ')
    # if bb.is_in_collision(rrt_goal): 
    #     print('goal in collision')
    # path = []
    # if add_before is not None:
    #     path.append(add_before)
    # for conf in rrt_path:
    #     path.append(conf)
    # if add_after is not None:
    #     path.append(add_after)
    # print(path)
    # np.save(os.path.join(dir_name,filename+'_path'), np.array(path))
    
    
    # #############################################
    
    # rrt_start = TOFILL-cube5goal
    # rrt_goal = cube6_approach
    # env = Environment(env_idx=3, cube_coords=initial_cubes_coords)
    # bb = Building_Blocks(transform=transform, ur_params=ur_params, env=env, resolution=0.1, p_bias=0.05,)
    # rrt_star_planner = RRT_STAR(max_step_size=0.5, max_itr=10000, bb=bb)
    # filename = f'cube{5}_goalTocube{6}_approach'
    # rrt_path = rrt_star_planner.find_path(start_conf=rrt_start, goal_conf=rrt_goal, filename=filename,)
    # add_before = cube5_goal
    # add_after = cube6
    # if bb.is_in_collision(rrt_start):
    #     print('start in collision:  ')
    # if bb.is_in_collision(rrt_goal): 
    #     print('goal in collision')
    # path = []
    # if add_before is not None:
    #     path.append(add_before)
    # for conf in rrt_path:
    #     path.append(conf)
    # if add_after is not None:
    #     path.append(add_after)
    # print(path)
    # np.save(os.path.join(dir_name,filename+'_path'), np.array(path))
    
    # #############################################
    
    # rrt_start = cube6_approach
    # cube6_goal = np.array(get_closest_config(get_valid_inverse_solutions(*final_cubes_coords[5],bb=bb), cubes[5]))
    # rrt_goal = cube6_goal
    # env = Environment(env_idx=3, cube_coords=initial_cubes_coords)
    # bb = Building_Blocks(transform=transform, ur_params=ur_params, env=env, resolution=0.1, p_bias=0.05,)
    # rrt_star_planner = RRT_STAR(max_step_size=0.5, max_itr=10000, bb=bb)
    # filename = f'cube{6}Tocube{6}_goal'
    # rrt_path = rrt_star_planner.find_path(start_conf=rrt_start, goal_conf=rrt_goal, filename=filename,)
    # add_before = cube1
    # add_after = None
    # if bb.is_in_collision(rrt_start):
    #     print('start in collision:  ')
    # if bb.is_in_collision(rrt_goal): 
    #     print('goal in collision')
    # path = []
    # if add_before is not None:
    #     path.append(add_before)
    # for conf in rrt_path:
    #     path.append(conf)
    # if add_after is not None:
    #     path.append(add_after)
    # print(path)
    # np.save(os.path.join(dir_name,filename+'_path'), np.array(path))
    
    #############################################
    
    #cube3_goal->cube_4_approach
    rrt_start = np.array([0.8288548452878606, -1.247008414469712, 1.8762297876817924, 0.9415749535828163, 1.5707963267948968, -0.7419414815070361])
    rrt_goal = cube4_approach
    
    # cube5_goal ->cube6_approach:
    # rrt_start = np.array([-1.6521184004291039, -1.1080418640327643, -1.9090524044737762, -0.689894957963152, 1.312583121735577, 2.4209683549554564])
    # rrt_start = np.array(get_closest_config(get_valid_inverse_solutions(*final_cubes_coords[4],bb=bb), cubes[4]))
    # rrt_goal = cube6_approach
    
    initial_cubes_coords[0] = cube1_final
    initial_cubes_coords[1] = cube2_final
    initial_cubes_coords[2] = cube3_final
    # initial_cubes_coords[3] = cube4_final
    # initial_cubes_coords[4] = cube5_final
    env = Environment(env_idx=3, cube_coords=initial_cubes_coords)
    bb = Building_Blocks(transform=transform, ur_params=ur_params, env=env, resolution=0.1, p_bias=0.05,)
    
    # rrt_start = np.array(get_closest_config(get_valid_inverse_solutions(*final_cubes_coords[4],bb=bb), cubes[4]))
    # rrt_goal = cube6_approach
    
    rrt_star_planner = RRT_STAR(max_step_size=0.5, max_itr=10000, bb=bb)
    # filename = f'cube{5}_goalTocube{6}_approach'
    filename = f'cube{3}_goalTocube{4}_approach'
    rrt_path = rrt_star_planner.find_path(start_conf=rrt_start, goal_conf=rrt_goal, filename=filename,)
    add_before = cube3_goal
    add_after = cube4
    if bb.is_in_collision(rrt_start):
        print('start in collision:  ')
    if bb.is_in_collision(rrt_goal): 
        print('goal in collision')
    path = []
    if add_before is not None:
        path.append(add_before)
    for conf in rrt_path:
        path.append(conf)
    if add_after is not None:
        path.append(add_after)
    print(path)
    np.save(os.path.join(dir_name,filename+'_path'), np.array(path))

    
    
    
if __name__ == '__main__':
    main()
