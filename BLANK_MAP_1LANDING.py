from ast import Global
from re import X
from sre_constants import SUCCESS
from sre_parse import State
import py_trees
import time
import py_trees.decorators
import py_trees.display
from drone_trees import leaf_nodes as lf
from drone_trees import flight_idioms as im
from gridmap import OccupancyGridMap
import matplotlib.pyplot as plt
from a_star import a_star
from utils import plot_path
import random
from utils import dist2d
import math


##############################
#Globals
##############################

#############################Real Distance Calculator

def calc_points_distance(path_px):
        res = 0
        for i in range(len(path_px)-1):
            res += math.dist(path_px[i],path_px[i+1])
        return res


##############################################
# SIMULATION
##############################################


###################
#DRONE 1
###################


class PathGeneratorDrone1(py_trees.behaviour.Behaviour):
   
   def update(self,path=1):
    if __name__ == '__main__':
    
    ############################
    #Loiter PHASE
    ############################

    # load the map
     gmap = OccupancyGridMap.from_png('maps/BLANK_MAP.png', 1)
    
    # set a start and an end node (in meters)
    start_node = (496, 200)
    goal_node = (200, 200)
    
    #deterine drone travel time for planned route
    
    global DRONE1_Travel_Time_Loiter_Seconds

    Travel_Time_Loiter = (dist2d(start_node, goal_node)/ 16.5)
    float_str = float(Travel_Time_Loiter)
    DRONE1_Travel_Time_Loiter_Seconds = int(float_str)


    # run A*
    path, path_px = a_star(start_node, goal_node, gmap, movement='8N')
    
    gmap.plot()
    
    if path:
        # plot resulting path in pixels over the map
        plot_path(path_px)
        print('DRONE1--Loiter_Distance METERS')
        print(Travel_Time_Loiter*16.5 )
        
        print('DRONE1--Loiter_Travel_Time INTEGER_SECONDS')
        print(DRONE1_Travel_Time_Loiter_Seconds)
        print('---------------------------------')
        #print(path_px)

    else:
        print('DRONE1--Path cannont be traveresed')

        # plot start and goal points over the map (in pixels)
        start_node_px = gmap.get_index_from_coordinates(start_node[0], start_node[1])
        goal_node_px = gmap.get_index_from_coordinates(goal_node[0], goal_node[1])

        plt.plot(start_node_px[0], start_node_px[1], 'ro')
        plt.plot(goal_node_px[0], goal_node_px[1], 'go')

    #plt.show(block=None)
    #plt.savefig('DRONE1--FlightPlan_Loiter.pdf')

    ############################
    #FLIGHT TO MERGING POINT
    ############################

    # load the map
    gmap = OccupancyGridMap.from_png('maps/BLANK_MAP.png', 1)
    
    # set a start and an end node (in meters)
    start_node = (200, 200)
    goal_node = (496, 500)
    
    #deterine drone travel time for planned route
    # run A*
    path, path_px = a_star(start_node, goal_node, gmap, movement='8N')
    
    Travel_Time_Flight_EUCLEDIAN= (dist2d(start_node, goal_node)/16.5)
    
    global DRONE1_Travel_Time_Flight_Seconds
    
    Travel_Time_Flight=(calc_points_distance(path_px)/16.5)
    float_str = float(Travel_Time_Flight)
    DRONE1_Travel_Time_Flight_Seconds = int(float_str)
    
    print('DRONE1--MergingPoint_Distance METERS')
    print(Travel_Time_Flight*16.5)
    #print(Travel_Time_Flight_EUCLEDIAN*16.5)
    print('DRONE1--MergingPoint_Travel_Time INTEGER_SECONDS')
    print(DRONE1_Travel_Time_Flight_Seconds)
    print('---------------------------------')

    gmap.plot()
    
    if path:
        # plot resulting path in pixels over the map
        plot_path(path_px)
        #print(path_px)

    else:
        print('DRONE1--Path cannont be traveresed')

        # plot start and goal points over the map (in pixels)
        start_node_px = gmap.get_index_from_coordinates(start_node[0], start_node[1])
        goal_node_px = gmap.get_index_from_coordinates(goal_node[0], goal_node[1])

        plt.plot(start_node_px[0], start_node_px[1], 'ro')
        plt.plot(goal_node_px[0], goal_node_px[1], 'go')

    #plt.savefig('DRONE1--FlightPlan_MERGING.pdf')

    ############################
    #FLIGHT TO LANDING
    ############################

    # load the map
    gmap = OccupancyGridMap.from_png('maps/BLANK_MAP.png', 1)
    
    # set a start and an end node (in meters)
    start_node = (496, 500)
    goal_node = (496, 800)
    
    #deterine drone travel time for planned route
    
    global DRONE1_Travel_Time_Landing_Seconds

    Travel_Time_Landing = (dist2d(start_node, goal_node)/ 16.5)
    float_str = float(Travel_Time_Landing)
    DRONE1_Travel_Time_Landing_Seconds = int(float_str)
    

    # run A*
    path, path_px = a_star(start_node, goal_node, gmap, movement='8N')
    
    gmap.plot()
    
    if path:
        # plot resulting path in pixels over the map
        plot_path(path_px)
        print('DRONE1--FlyToLanding_Travel_Distance METERS')
        print(Travel_Time_Landing*16.5)

        print('DRONE1--FlyToLanding_Travel_Time INTEGER_SECONDS')
        print( DRONE1_Travel_Time_Landing_Seconds)
        print('---------------------------------')
        print('---------------------------------')
        #print(path_px)

    else:
        print('DRONE1--Path cannont be traveresed')

        # plot start and goal points over the map (in pixels)
        start_node_px = gmap.get_index_from_coordinates(start_node[0], start_node[1])
        goal_node_px = gmap.get_index_from_coordinates(goal_node[0], goal_node[1])

        plt.plot(start_node_px[0], start_node_px[1], 'ro')
        plt.plot(goal_node_px[0], goal_node_px[1], 'go')

    plt.savefig('DRONE1--FlightPlan.pdf')

    #plt.show()

    if path:
     self.feedback_message = 'Great succes eh? {}'
     return py_trees.common.Status.SUCCESS
    else:
     self.feedback_message = 'Failutre to reach next waypoint {}'
    return py_trees.common.Status.FAILURE

TASK_PathGeneratorDrone1 = py_trees.decorators.OneShot(PathGeneratorDrone1("Drone1PathGeneration"))     

###################
#DRONE 2
###################


class PathGeneratorDrone2(py_trees.behaviour.Behaviour):
   
   def update(self,path=1):
    if __name__ == '__main__':
    
    ############################
    #Loiter PHASE
    ############################

    # load the map
     gmap = OccupancyGridMap.from_png('maps/BLANK_MAP.png', 1)
    
    # set a start and an end node (in meters)
    start_node = (496, 190)
    goal_node = (792, 190)
    
    #deterine drone travel time for planned route
    
    global DRONE2_Travel_Time_Loiter_Seconds

    Travel_Time_Loiter = (dist2d(start_node, goal_node)/ 16.5)
    float_str = float(Travel_Time_Loiter)
    DRONE2_Travel_Time_Loiter_Seconds = int(float_str)


    # run A*
    path, path_px = a_star(start_node, goal_node, gmap, movement='8N')
    
    gmap.plot()
    
    if path:
        # plot resulting path in pixels over the map
        plot_path(path_px)
        print('DRONE2--Loiter_Distance METERS')
        print(Travel_Time_Loiter*16.5 )
        
        print('DRONE2--Loiter_Travel_Time INTEGER_SECONDS')
        print(DRONE2_Travel_Time_Loiter_Seconds)
        print('---------------------------------')
        #print(path_px)

    else:
        print('DRONE2--Path cannont be traveresed')

        # plot start and goal points over the map (in pixels)
        start_node_px = gmap.get_index_from_coordinates(start_node[0], start_node[1])
        goal_node_px = gmap.get_index_from_coordinates(goal_node[0], goal_node[1])

        plt.plot(start_node_px[0], start_node_px[1], 'ro')
        plt.plot(goal_node_px[0], goal_node_px[1], 'go')

    #plt.show(block=None)
    #plt.savefig('DRONE2--FlightPlan_Loiter.pdf')

    ############################
    #FLIGHT TO MERGING POINT
    ############################

    # load the map
    gmap = OccupancyGridMap.from_png('maps/BLANK_MAP.png', 1)
    
    # set a start and an end node (in meters)
    start_node = (792, 190)
    goal_node = (496, 500)
    
    #deterine drone travel time for planned route
    # run A*
    path, path_px = a_star(start_node, goal_node, gmap, movement='8N')
    
    Travel_Time_Flight_EUCLEDIAN= (dist2d(start_node, goal_node)/16.5)
    
    global DRONE2_Travel_Time_Flight_Seconds
    
    Travel_Time_Flight=(calc_points_distance(path_px)/16.5)
    float_str = float(Travel_Time_Flight)
    DRONE2_Travel_Time_Flight_Seconds = int(float_str)
    
    print('DRONE2--MergingPoint_Distance METERS')
    print(Travel_Time_Flight*16.5)
    #print(Travel_Time_Flight_EUCLEDIAN*16.5)
    print('DRONE2--MergingPoint_Travel_Time INTEGER_SECONDS')
    print(DRONE2_Travel_Time_Flight_Seconds)
    print('---------------------------------')

    
    gmap.plot()
    
    if path:
        # plot resulting path in pixels over the map
        plot_path(path_px)
        #print(path_px)

    else:
        print('DRONE2--Path cannont be traveresed')

        # plot start and goal points over the map (in pixels)
        start_node_px = gmap.get_index_from_coordinates(start_node[0], start_node[1])
        goal_node_px = gmap.get_index_from_coordinates(goal_node[0], goal_node[1])

        plt.plot(start_node_px[0], start_node_px[1], 'ro')
        plt.plot(goal_node_px[0], goal_node_px[1], 'go')

    #plt.savefig('DRONE2--FlightPlan_MERGING.pdf')

    ############################
    #FLIGHT TO LANDING
    ############################

    # load the map
    gmap = OccupancyGridMap.from_png('maps/BLANK_MAP.png', 1)
    
    # set a start and an end node (in meters)
    start_node = (496, 500)
    goal_node = (496, 800)
    
    #deterine drone travel time for planned route
    
    global DRONE2_Travel_Time_Landing_Seconds

    Travel_Time_Landing = (dist2d(start_node, goal_node)/ 16.5)
    float_str = float(Travel_Time_Landing)
    DRONE2_Travel_Time_Landing_Seconds = int(float_str)
    

    # run A*
    path, path_px = a_star(start_node, goal_node, gmap, movement='8N')
    
    gmap.plot()
    
    if path:
        # plot resulting path in pixels over the map
        plot_path(path_px)
        print('DRONE2--FlyToLanding_Travel_Distance METERS')
        print(Travel_Time_Landing*16.5)

        print('DRONE2--FlyToLanding_Travel_Time INTEGER_SECONDS')
        print( DRONE2_Travel_Time_Landing_Seconds)
        print('---------------------------------')
        print('---------------------------------')

    else:
        print('DRONE2--Path cannont be traveresed')

        # plot start and goal points over the map (in pixels)
        start_node_px = gmap.get_index_from_coordinates(start_node[0], start_node[1])
        goal_node_px = gmap.get_index_from_coordinates(goal_node[0], goal_node[1])

        plt.plot(start_node_px[0], start_node_px[1], 'ro')
        plt.plot(goal_node_px[0], goal_node_px[1], 'go')

    plt.savefig('DRONE2--FlightPlan.pdf')

    #plt.show()

    if path:
     self.feedback_message = 'Great succes eh? {}'
     return py_trees.common.Status.SUCCESS
    else:
     self.feedback_message = 'Failutre to reach next waypoint {}'
    return py_trees.common.Status.FAILURE

TASK_PathGeneratorDrone2 = py_trees.decorators.OneShot(PathGeneratorDrone2("Drone2PathGeneration"))     

###################
#DRONE 3
###################


class PathGeneratorDrone3(py_trees.behaviour.Behaviour):
   
   def update(self,path=1):
    if __name__ == '__main__':
    
    ############################
    #Loiter PHASE
    ############################

    # load the map
     gmap = OccupancyGridMap.from_png('maps/BLANK_MAP.png', 1)
    
    # set a start and an end node (in meters)
    start_node = (496, 180)
    goal_node = (200, 180)
    
    #deterine drone travel time for planned route
    
    global DRONE3_Travel_Time_Loiter_Seconds

    Travel_Time_Loiter = (dist2d(start_node, goal_node)/ 16.5)
    float_str = float(Travel_Time_Loiter)
    DRONE3_Travel_Time_Loiter_Seconds = int(float_str)


    # run A*
    path, path_px = a_star(start_node, goal_node, gmap, movement='8N')
    
    gmap.plot()
    
    if path:
        # plot resulting path in pixels over the map
        plot_path(path_px)
        print('DRONE3--Loiter_Distance METERS')
        print(Travel_Time_Loiter*16.5 )
        
        print('DRONE3--Loiter_Travel_Time INTEGER_SECONDS')
        print(DRONE3_Travel_Time_Loiter_Seconds)
        print('---------------------------------')
        #print(path_px)

    else:
        print('DRONE3--Path cannont be traveresed')

        # plot start and goal points over the map (in pixels)
        start_node_px = gmap.get_index_from_coordinates(start_node[0], start_node[1])
        goal_node_px = gmap.get_index_from_coordinates(goal_node[0], goal_node[1])

        plt.plot(start_node_px[0], start_node_px[1], 'ro')
        plt.plot(goal_node_px[0], goal_node_px[1], 'go')

    #plt.show(block=None)
    #plt.savefig('DRONE3--FlightPlan_Loiter.pdf')

    ############################
    #FLIGHT TO MERGING POINT
    ############################

    # load the map
    gmap = OccupancyGridMap.from_png('maps/BLANK_MAP.png', 1)
    
    # set a start and an end node (in meters)
    start_node = (200,180)
    goal_node = (496, 500)
    
    #deterine drone travel time for planned route
    # run A*
    path, path_px = a_star(start_node, goal_node, gmap, movement='8N')
    
    Travel_Time_Flight_EUCLEDIAN= (dist2d(start_node, goal_node)/16.5)
    
    global DRONE3_Travel_Time_Flight_Seconds
    
    Travel_Time_Flight=(calc_points_distance(path_px)/16.5)
    float_str = float(Travel_Time_Flight)
    DRONE3_Travel_Time_Flight_Seconds = int(float_str)
    
    print('DRONE3--MergingPoint_Distance METERS')
    print(Travel_Time_Flight*16.5)
    #print(Travel_Time_Flight_EUCLEDIAN*16.5)
    print('DRONE3--MergingPoint_Travel_Time INTEGER_SECONDS')
    print(DRONE3_Travel_Time_Flight_Seconds)
    print('---------------------------------')

    
    gmap.plot()
    
    if path:
        # plot resulting path in pixels over the map
        plot_path(path_px)
        #print(path_px)

    else:
        print('DRONE3--Path cannont be traveresed')

        # plot start and goal points over the map (in pixels)
        start_node_px = gmap.get_index_from_coordinates(start_node[0], start_node[1])
        goal_node_px = gmap.get_index_from_coordinates(goal_node[0], goal_node[1])

        plt.plot(start_node_px[0], start_node_px[1], 'ro')
        plt.plot(goal_node_px[0], goal_node_px[1], 'go')

    #plt.savefig('DRONE3--FlightPlan_MERGING.pdf')

    ############################
    #FLIGHT TO LANDING
    ############################

    # load the map
    gmap = OccupancyGridMap.from_png('maps/BLANK_MAP.png', 1)
    
    # set a start and an end node (in meters)
    start_node = (496, 500)
    goal_node = (496, 800)
    
    #deterine drone travel time for planned route
    
    global DRONE3_Travel_Time_Landing_Seconds

    Travel_Time_Landing = (dist2d(start_node, goal_node)/ 16.5)
    float_str = float(Travel_Time_Landing)
    DRONE3_Travel_Time_Landing_Seconds = int(float_str)
    

    # run A*
    path, path_px = a_star(start_node, goal_node, gmap, movement='8N')
    
    gmap.plot()
    
    if path:
        # plot resulting path in pixels over the map
        plot_path(path_px)
        print('DRONE3--FlyToLanding_Travel_Distance METERS')
        print(Travel_Time_Landing*16.5)

        print('DRONE3--FlyToLanding_Travel_Time INTEGER_SECONDS')
        print( DRONE3_Travel_Time_Landing_Seconds)
        print('---------------------------------')
        print('---------------------------------')
        #print(path_px)

    else:
        print('DRONE3--Path cannont be traveresed')

        # plot start and goal points over the map (in pixels)
        start_node_px = gmap.get_index_from_coordinates(start_node[0], start_node[1])
        goal_node_px = gmap.get_index_from_coordinates(goal_node[0], goal_node[1])

        plt.plot(start_node_px[0], start_node_px[1], 'ro')
        plt.plot(goal_node_px[0], goal_node_px[1], 'go')

    plt.savefig('DRONE3--FlightPlan.pdf')

    #plt.show()

    if path:
     self.feedback_message = 'Great succes eh? {}'
     return py_trees.common.Status.SUCCESS
    else:
     self.feedback_message = 'Failutre to reach next waypoint {}'
    return py_trees.common.Status.FAILURE

TASK_PathGeneratorDrone3 = py_trees.decorators.OneShot(PathGeneratorDrone3("Drone3PathGeneration")) 


##############################################
# TRAFFIC CONTROLLER 
##############################################


##############################Estimated Flight Times
#SIMULATION INPUTS DIRECT IN GLOBAL VARIABLES FOR EASY READING

DRONE1_Travel_Time_Loiter_Seconds = 0
DRONE1_Travel_Time_Flight_Seconds = 0
DRONE1_Travel_Time_Landing_Seconds = 0
DRONE1__TOTALFLIGHT= (DRONE1_Travel_Time_Loiter_Seconds+DRONE1_Travel_Time_Flight_Seconds+DRONE1_Travel_Time_Landing_Seconds)
print("DRONE1__TOTALFLIGHT_TIME SECONDS")
print(DRONE1__TOTALFLIGHT)

DRONE2_Travel_Time_Loiter_Seconds = 0
DRONE2_Travel_Time_Flight_Seconds = 0
DRONE2_Travel_Time_Landing_Seconds = 0
DRONE2__TOTALFLIGHT= (DRONE2_Travel_Time_Loiter_Seconds+DRONE2_Travel_Time_Flight_Seconds+DRONE2_Travel_Time_Landing_Seconds)
print("DRONE2__TOTALFLIGHT_TIME SECONDS")
print(DRONE2__TOTALFLIGHT)

DRONE3_Travel_Time_Loiter_Seconds = 0
DRONE3_Travel_Time_Flight_Seconds = 0
DRONE3_Travel_Time_Landing_Seconds = 0
DRONE3__TOTALFLIGHT= (DRONE3_Travel_Time_Loiter_Seconds+DRONE3_Travel_Time_Flight_Seconds+DRONE3_Travel_Time_Landing_Seconds)
print("DRONE3__TOTALFLIGHT_TIME SECONDS")
print(DRONE3__TOTALFLIGHT)

Merging_Offset=2 #an offset of two seconds for merging resulting in a distance of 40 meters between each drone on the loitering line

TotalRunTime = (Merging_Offset+Merging_Offset+DRONE3__TOTALFLIGHT)
print("TotalRunTime SECONDS")
print(TotalRunTime)

#int (Travel_Time_Loyte)



#DRONE1!!! TRAFFIC CONTROLLER
#############################################################################///DRONE1
Drone1_Loiter=  py_trees.decorators.OneShot(py_trees.behaviours.Count(
  name='Drone1-Fly_to_Loiter', 
  fail_until=0, 
  running_until=1+DRONE1_Travel_Time_Loiter_Seconds, 
  success_until=TotalRunTime ,))

Drone1_Merging=  py_trees.decorators.OneShot(py_trees.behaviours.Count(
  name='Drone1-Fly_to_Merging', 
  fail_until=0, 
  running_until=DRONE1_Travel_Time_Flight_Seconds, 
  success_until=TotalRunTime ,))

Drone1_Landing=  py_trees.decorators.OneShot(py_trees.behaviours.Count(
  name='Drone1-Fly_to_Landing', 
  fail_until=0, 
  running_until=DRONE1_Travel_Time_Landing_Seconds, 
  success_until=TotalRunTime ,))

#############################################################################
#DRONE2!!! TRAFFIC CONTROLLER
#############################################################################///DRONE2
Drone2_Loiter=  py_trees.decorators.OneShot(py_trees.behaviours.Count(
  name='Drone2-Fly_to_Loiter', 
  fail_until=0, 
  running_until=(DRONE1_Travel_Time_Loiter_Seconds+Merging_Offset), 
  success_until=TotalRunTime ,))

Drone2_Merging=  py_trees.decorators.OneShot(py_trees.behaviours.Count(
  name='Drone2-Fly_to_Merging', 
  fail_until=0, 
  running_until=DRONE2_Travel_Time_Flight_Seconds, 
  success_until=TotalRunTime ,))

Drone2_Landing=  py_trees.decorators.OneShot(py_trees.behaviours.Count(
  name='Drone2-Fly_to_Landing', 
  fail_until=0, 
  running_until=DRONE2_Travel_Time_Landing_Seconds, 
  success_until=TotalRunTime ,))

#############################################################################
#DRONE3!!! TRAFFIC CONTROLLER
#############################################################################///DRONE3

Drone3_Loiter=  py_trees.decorators.OneShot(py_trees.behaviours.Count(
  name='Drone3-Fly_to_Loiter', 
  fail_until=0, 
  running_until=(DRONE1_Travel_Time_Loiter_Seconds+Merging_Offset+Merging_Offset), 
  success_until=TotalRunTime ,))

Drone3_Merging=  py_trees.decorators.OneShot(py_trees.behaviours.Count(
  name='Drone3-Fly_to_Merging', 
  fail_until=0, 
  running_until=DRONE3_Travel_Time_Flight_Seconds, 
  success_until=TotalRunTime ,))

Drone3_Landing=  py_trees.decorators.OneShot(py_trees.behaviours.Count(
  name='Drone3-Fly_to_Landing', 
  fail_until=0, 
  running_until=DRONE3_Travel_Time_Landing_Seconds, 
  success_until=TotalRunTime ,))


##############################################
# MAIN TREES
##############################################

Simulation = py_trees.composites.Parallel(name="Simulations",
                                                    children=[TASK_PathGeneratorDrone1,TASK_PathGeneratorDrone2,TASK_PathGeneratorDrone3,
                                                            ])

#REMINDER MEMORY MUST BE TRUE FOR RUN AND FALSE FOR MAP PRINT!!!
py_trees.decorators.EternalGuard #to add in traffic controller tree when running with 0 global variables

Drone1_FlightControll = py_trees.composites.Sequence(name="Drone1Controll", memory=True,
                                                    children=[Drone1_Loiter,Drone1_Merging,Drone1_Landing
                                                            ])

Drone2_FlightControll = py_trees.composites.Sequence(name="Drone2Controll", memory=True,
                                                    children=[Drone2_Loiter,Drone2_Merging,Drone2_Landing
                                                            ])

Drone3_FlightControll = py_trees.composites.Sequence(name="Drone3Controll", memory=True,
                                                    children=[Drone3_Loiter,Drone3_Merging,Drone3_Landing
                                                            ])

###############################################
#MAIN_FLIGHR_CONTROLLER
###############################################

root_main = py_trees.composites.Parallel(name="ROOT_Main",
                                    children=[Simulation,
                                    Drone1_FlightControll, Drone2_FlightControll, Drone3_FlightControll])


#DEBUGGER
#///////////////
#py_trees.logging.level = py_trees.logging.Level.DEBUG
                                                                                                                
behaviour_tree_root = py_trees.trees.BehaviourTree(root_main)
# Add tick handler for terminal visualisation
snapshot_visitor = py_trees.visitors.SnapshotVisitor()
behaviour_tree_root.visitors.append(snapshot_visitor)
# ---- Run BT for 10 seconds and print status
#py_trees.display.render_dot_tree (root_main)
for ii in range(6+TotalRunTime):
    py_trees.console.banner(f"{ii}")
    behaviour_tree_root.tick()
    unicode_tree = py_trees.display.unicode_tree(behaviour_tree_root.root,
                                            visited=snapshot_visitor.visited,
                                            previously_visited=snapshot_visitor.visited)
    #print(blackboard)
    print(unicode_tree)
    #print(unicode_blackboard())
    # pause
    time.sleep(1)

