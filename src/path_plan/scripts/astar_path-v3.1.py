#!/usr/bin/env python

import rospy
import math
import numpy as np
from matplotlib import pyplot as plt
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped

class nodE():

    def __init__(s, pos = None, parent = None) :
        s.pos = pos
        s.parent = parent

        s.g = 0
        s.h = 0
        s.f = 0

def a_star_path_plan(the_matrix, start, goal):

    total_rows = len(the_matrix)
    total_cols = len( the_matrix[0] )
    
    #create start and goal nodes
    start_n = nodE(start)
    goal_n = nodE(pos = goal)

    start_n.g = start_n.f = start_n.h = 0
    goal_n.g = goal_n.f = goal_n.h = 0

    #initializing open and closed lists
    open_nodes = []
    closed_nodes = []

    #open start node
    open_nodes.append(start_n)

    
    #Keep L00Ping till goal is found
    i = 0
    while len(open_nodes)>0 :
        
        #Get to the next best node - one with least f_cost
        where_i_am = open_nodes[0]   #assume it's the fist one
        nb_idx = 0
        
        for idx, node in enumerate(open_nodes):
            if node.f < where_i_am.f :
                where_i_am = node
                nb_idx = idx
        
        # Now we know where_i_am is the best node to go to next (least f). 
        # So let's close it
        open_nodes.pop(nb_idx)
        closed_nodes.append(where_i_am)

        #Have we reached yet...?
        if where_i_am.pos == goal_n.pos :
            #Yayy, so let's trace back our path
            path = []
            c = where_i_am
            while c is not None :
                path.append(c.pos)
                c = c.parent
            #The reverse of which, is what we required!
            return path[::-1]


        
        #find all possible next moves (children)
        children = []

        dirs = [
                (1,0), (0, 1),
                (-1, 0), (0, -1),
                (1, 1), (1, -1),
                (-1, 1), (-1, -1)
            ]
        
        #Considering each possible child at a time ~
        for (r, c) in dirs:
            #Where is this child?
            child_pos = ( where_i_am.pos[0]+r, where_i_am.pos[1]+c )

            #Is the chiild real?
            be_real = True

            #Is the child in the_matrix?
            if (child_pos[0] < 0) or (child_pos[0] > (total_rows - 1)):
                be_real = False
                continue
            if (child_pos[1] < False) or (child_pos[1] > (total_cols - 1)):
                be_real = False
                continue

            #Is the child a wall?
            if (the_matrix[ child_pos[0] ][ child_pos[1] ] == 100):
                be_real = False
                continue
            
            if be_real:
                #Add child as a node
                child_node = nodE(child_pos, where_i_am)
                children.append(child_node)



        #L00Ping through the children
        for child in children:
            #flag
            f = 1

            #Closed before?
            for closed in closed_nodes:
                if (child.pos[0] == closed.pos[0]) and (child.pos[1] == closed.pos[1]):
                    f = 0
                    continue
                    
                    
            
            #Compute f, g, h of the child
            #dx = child.pos[0] - where_i_am.pos[0]
            #dy = child.pos[1] - where_i_am.pos[1]
            
            #child.g = where_i_am.g + math.sqrt((dx**2) + (dy**2))
            if (child.pos[0] == where_i_am.pos[0]) or (child.pos[1] == where_i_am.pos[1]):
                child.g = where_i_am.g + 1
            else:
                child.g = where_i_am.g + 1.44


            dx = child.pos[0] - goal_n.pos[0]
            dy = child.pos[1] - goal_n.pos[1]
            #child.h = ((dx**2) + (dy**2))      #Euclidean distance
            child.h = abs(dx) + abs(dy)

            child.f = child.g + child.h

            #Have I seen this child before?
            for idx, seen in enumerate(open_nodes) :
                if child.pos == seen.pos:
                    f = 0
                    #Is the child closer to his goal?
                    if child.g < seen.g:                       
                        open_nodes.pop(idx)
                        open_nodes.append(child)
                        break


            #If a child comes this far, it shall be opened
            if f==1: 
                open_nodes.append(child)
            
            # ~ Trouble-shooting ~
            '''
            i+=1
            if i<50000:
                print('itr = ', i)
                print('curr_pt = ', where_i_am.pos)
                #print('open - ')
                for n in open_nodes:
                    #print(n.pos)
                    c = 0
                    for n2 in open_nodes:
                        if n.pos == n2.pos:
                            c+=1
                            if c==2:
                                print('open repeating!')
                                break
                #print('c - ')
                for n in closed_nodes:
                    #print(n.pos)
                    c = 0
                    for n2 in open_nodes:
                        if n.pos == n2.pos:
                            c+=1
                            if c==2:
                                print('close repeating!')
                                break
            if i>80000:
                print('not completed! closed =')
                cl = []
                for c in closed_nodes:
                    cl.append(c.pos)
                return cl
            '''
                



def map_callback(map_msg):

    # Extract the occupancy grid data from the message
    width = map_msg.info.width
    height = map_msg.info.height
    resolution = map_msg.info.resolution
    origin = map_msg.info.origin
    occupancy_data = map_msg.data

    print(resolution)

    # Convert the occupancy grid data into a 2D array
    the_matrix = np.array(map_msg.data)
    the_matrix = the_matrix.reshape(height, width)

    '''
    for i in range(height):
        row = []

        for j in range(width):
            index = i * width + j
            value = occupancy_data[index]
            row.append(value)

        the_matrix.append(row)
        print(the_matrix)'''
    
    return height, width, resolution, the_matrix


#   ~ MAIN ~
if __name__ == '__main__':

    rospy.init_node('path_planner')
    rospy.loginfo('\nHi. Path planner node initiated!\n')

    rospy.loginfo("Getting the map from the servers")
    rospy.wait_for_service('/static_map')
    service = rospy.ServiceProxy("/static_map", GetMap)

    response = service()
    map = response.map
    h, w, r, the_matrix = map_callback(map)

            

    print('Map dimensions = ', h,'\t', w)

    print(the_matrix)


    start = (0, 0)
    end = (49, 49)

    start = (35, 5)
    end = (15, 49) 

    start = (49, 8)
    end = (15, 49)    

    p = a_star_path_plan(the_matrix, start, end)


    path_pub = rospy.Publisher('path_topic', Path, queue_size=10)


    print('the path - ', p)

    for (x, y) in   p :
        the_matrix[x][y] = 50

    plt.imshow(the_matrix)
    plt.show()

    path = Path()
    path.header.frame_id = 'map'

    for (r, c) in p:
        pose_i = PoseStamped()
        pose_i.pose.position.x = c
        pose_i.pose.position.y = r
        path.poses.append(pose_i)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        path_pub.publish(path)
        rate.sleep()

    #map_sub = rospy.Subscriber('/map', OccupancyGrid, map_callback)
   
    #rospy.Subscriber('map', OccupancyGrid, map_callback)
    #path_pub.publish(p)
    rospy.loginfo('Published Path Successfully')
    #rospy.spin()