import numpy as np
from environment import Environment
from kinematics import UR5e_PARAMS, Transform
from planners import RRT_STAR
from building_blocks import Building_Blocks
from visualizer import Visualize_UR
from building_blocks import get_closest_config
from inverse_kinematics import get_valid_inverse_solutions


def main():
    
    #TODO - change the position of the cubes during the task
    cube1_coords = [-0.10959248574268822, -0.6417732149769166, 0.1390226933317033]
    cube2_coords = [0.08539928976845282, -0.8370930220946053, 0.13813472317717034]
    cube3_coords =  [-0.008445229140271685, -0.7365370847309188, 0.00955541284784159]
    cube4_coords = [0.23647185443765273 ,-0.769747539513382, 0.03971366463235271]
    cube5_coords =[0.26353072323141574 ,-0.4629969534200313, 0.2651034131371637]
    cube6_coords =  [0.26940059242703984, -0.4730222745248458, 0.021688493137064376]
    initial_cubes_coords = [cube1_coords,cube2_coords,cube3_coords,cube4_coords,cube5_coords,cube6_coords] 

    ur_params = UR5e_PARAMS(inflation_factor=1)
    transform = Transform(ur_params)

    env = Environment(env_idx=3, cube_coords=initial_cubes_coords)
    
    bb = Building_Blocks(transform=transform, 
                        ur_params=ur_params, 
                        env=env,
                        resolution=0.1, 
                        p_bias=0.05,)
    ##############################################################################################
    fictional_ground = 0.1
    # for Paulo - constructing a: P
    cube1_final = [-0.106, -0.483, fictional_ground]
    cube2_final = [-0.106, -0.29, fictional_ground]
    cube3_final = [-0.106, -0.385, fictional_ground]
    cube4_final = [0.176, -0.483, fictional_ground]
    cube5_final = [-0.166, -0.383, fictional_ground]
    cube6_final = [-0.22, -0.43, fictional_ground]
    ##############################################################################################
    # for Saleh - constructing a: S
    cube1_final = [-0.08, -0.42, fictional_ground] # np.array([1.0655136671983865, -1.140326206296458, 1.7178655987338225, 0.9932569343575324, 1.5707963267948966, -0.5052826595965101])
    cube2_final = [-0.15, -0.48, fictional_ground] # np.array([1.0655136671983865,-1.140326206296458,1.7178655987338225,0.9932569343575324,1.5707963267948966,-0.5052826595965101])
    cube3_final = [-0.23, -0.43, fictional_ground] # np.array([1.0655136671983865,-1.140326206296458,1.7178655987338225,0.9932569343575324,1.5707963267948966,-0.5052826595965101])
    cube4_final = [-0.14, -0.35, fictional_ground]
    cube5_final = [-0.2, -0.28, fictional_ground]
    cube6_final = [-0.12, -0.24, fictional_ground]
    ##############################################################################################
    final_cubes_coords = [cube1_final,cube2_final,cube3_final,cube4_final,cube5_final,cube6_final]


    # --------- configurations-------------
    home = np.deg2rad([0, -90, 0, -90, 0,0 ])

    cube1_approach = np.deg2rad([68.8,-68.3, 84.2,-107.1,-90,-18]) #approach without collision
    cube1 = np.deg2rad([69,-63,85, -107.7,-91.3,-18.2]) #actual position of the cube 
    cube1_goal = np.array(get_closest_config(get_valid_inverse_solutions(*cube1_final,bb=bb), cube1)) #closest inverse kin to the last point
  

    cube2_approach = np.deg2rad([87.5, -45.5, 47.7, -102, -90.6, 3.3]) #approach without collision
    cube2 = np.deg2rad([86.9, -40.1, 47.7, -102, -90.6, 3.3]) #actual position of the cube 
    cube2_goal = np.array(get_closest_config(get_valid_inverse_solutions(*cube2_final,bb=bb), cube2)) #closest inverse kin to the last point


    cube3_approach = np.deg2rad([79.7,-46.9,69.7,-105.1,-92.6,-10.1]) #approach without collision
    cube3 = np.deg2rad([80.2, -43.4, 68.9, -107.1,-93.9, -9.4]) #actual position of the cube 
    cube3_goal = np.array(get_closest_config(get_valid_inverse_solutions(*cube3_final,bb=bb), cube3)) #closest inverse kin to the last point
 

    cube4_approach = np.deg2rad([97.6, -38.3,-52.1, -100.8, -90.1, 8.5]) #approach without collision
    cube4 = np.deg2rad([97.6, -34.6, 52.3, -100.8, -90.1, 8.5]) #actual position of the cube 
    cube4_goal = np.array(get_closest_config(get_valid_inverse_solutions(*cube4_final,bb=bb), cube4)) #closest inverse kin to the last point
 

    cube5_approach = np.deg2rad([104.6, -85.3,87.7, -90.5,-88.3, 17]) #approach without collision
    cube5 = np.deg2rad([105.1, -86.3, 97.4, -102, -89.8, 19.3]) #actual position of the cube 
    cube5_goal = np.array(get_closest_config(get_valid_inverse_solutions(*cube5_final,bb=bb), cube5)) #closest inverse kin to the last point

    cube6_approach = np.deg2rad([78.3, -61.7, 120.9, -87.6,-12.9, 27.7]) #approach without collision
    cube6 = np.deg2rad([78,-56.6,120.9, -93,-12.7,29.4]) #actual position of the cube 
    cube6_goal = np.array(get_closest_config(get_valid_inverse_solutions(*cube6_final,bb=bb), cube6)) #closest inverse kin to the last point 

    print("CUBEGOALS", cube1_goal,cube2_goal,cube3_goal,cube4_goal,cube5_goal,cube6_goal)
    # example of how to find coordinates x-y-z from given configuration
    cubex_goal = np.array([-1.20223872,-1.434,-2.3944, -0.883,1.5707,-2.7730])
    manipulator_spheres = transform.conf2sphere_coords(cubex_goal) # forward kinematics
    cube_coords = manipulator_spheres['wrist_3_link'][-1][:3] # take the position of the cube
    print(cube_coords)
    
    visualizer = Visualize_UR(ur_params, env=env, transform=transform, bb=bb)
    visualizer.show_conf(home)

    # ---------Example-----------
    # go to cube 1
    rrt_start = home
    rrt_goal = cube1_approach 
    add_before = None #add before path
    add_after = cube1 #add after path
    filename = 'home2cube1'

    # go to cube1_goal
    # rrt_start = cube1_approach
    # rrt_goal = cube1_goal
    # add_before = cube1
    # add_after = None
    # filename = 'cube1_to_cube1goal'

    nodes = [home, cube1_approach,cube1_goal,cube2_approach,cube2_goal,cube3_approach,cube3_goal,cube4_approach,cube4_goal,cube5_approach,cube5_goal,cube6_approach,cube6_goal]
    cubes = [cube1,cube2,cube3,cube4,cube5,cube6]
    at_cube = False
    i = 0
    for rrt_start,rrt_goal in zip(nodes,nodes[1:]):
        
        rrt_star_planner = RRT_STAR(max_step_size=0.5,
                                max_itr=10, 
                                bb=bb)

        filename = 'path_'+ str(i) + ('_start' if not at_cube else '_end')
        add_before = None
        add_after = None
        if(at_cube is False):
            add_after = cubes[i]
        else:
            add_before = cubes[i]
            i += 1
        at_cube = not at_cube

        print(filename,rrt_start, rrt_goal, add_before, add_after,at_cube)
        if bb.is_in_collision(rrt_start):
            print('start in collision')
        if bb.is_in_collision(rrt_goal): 
            print('goal in collision')
        
        rrt_path = rrt_star_planner.find_path(start_conf=rrt_start,
                                            goal_conf=rrt_goal,
                                            filename=filename, 
                                            )

        path = []
        if add_before is not None:
            path.append(add_before)
        for conf in rrt_path:
            path.append(conf)
        if add_after is not None:
            path.append(add_after)
        print(path)
        np.save(filename+'_path', np.array(path))

    try:
        path = np.load(filename+'_path.npy')
        visualizer.show_path(path)
        visualizer.show_conf(path[-1])
    except:
        print('No Path Found')
   

    plan_list = ['open', 
            'home2cube1_path.npy', 'close',
            'cube1_to_cube1goal_path.npy', 'open',
            #TODO
            ]
if __name__ == '__main__':
    main()



