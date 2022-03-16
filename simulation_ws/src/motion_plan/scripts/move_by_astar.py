#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

from math import atan2
from geometry_msgs.msg import Twist, Point


import math

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)




maze  = [[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
         [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
         [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
         [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
         [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
         [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
         [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
         [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
         [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
         [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
         [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]]


x_value = {}
y_value = {}
j =-1
for i in range(20):
    if i < 10 :
        x_value[str(i)] = 10-i
        y_value[str(i)] = 10-i
    else:
        x_value [str(i)] = j
        y_value [str(i)] = j
        j = j - 1

print(x_value)
print(y_value)
in_x = 9
in_y = 2

start_x = x = float(x_value[str(in_x)])
start_y = y = float(y_value[str(in_y)])


theta = 0.0
command = "STOP"
def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])



rospy.init_node('speed_controller')
sub = rospy.Subscriber("/odom",Odometry,newOdom)
pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)


r = rospy.Rate(20)

desired_position_ = Point()

desired_position_.x = 3
desired_position_.y = 18

# g_x = int(goal.x * 2)
# g_y = int(goal.y * 2)


start = (in_x, in_y)
end = (desired_position_.x, desired_position_.y)
path = astar(maze, start, end)

print(path)

path = list(path)
# an_array = np.array(path)
# path = an_array * 0.5
path = path[::-1]
path = list(path)
# removed = path.pop()

# print(path)


speed = Twist()

dist_precision_ = 0.6

is_orention_corrected = False

while not rospy.is_shutdown():
    

    

    
          
       
        # inc_x =  float(x_value[str(desired_position_.x)])- float(x_value[str(math.ceil(x))])
        # inc_y = float(y_value[str(desired_position_.y)])- float(y_value[str(math.ceil(y))])
        

        inc_x =  float(x_value[str(desired_position_.x)])- x
        inc_y = float(y_value[str(desired_position_.y)])- y

        angle_to_goal= atan2(inc_y, inc_x)

        print("x goal %f" , x)
        print("y goal %f" , y)
        

        print("x  ceil goal %f" , float(x_value[str(math.ceil(x))]))
        print("y goal %f" , float(y_value[str(desired_position_.y)]))

        if abs(angle_to_goal - theta) > 0.1:
            speed.linear.x = 0.0
            speed.angular.z = 0.3
            print("Oranation is performed theta:%f", abs(angle_to_goal - theta) )
        else:
            if  path:
                removed = path.pop()
                local_x = removed[0]
                local_y = removed[1]

            print("Poped [%s]",removed)
            # x = float(x_value[str(in_x)])
            # y = float(y_value[str(in_y)])

            # is_orention_corrected = True
            print("Move forward %f", theta)
            
        
            desired_yaw = math.atan2(inc_y, inc_x)
            err_yaw = desired_yaw - theta
            err_pos = math.sqrt(pow(inc_y, 2) + pow(inc_x, 2))
        

            if err_pos > dist_precision_:
                speed.linear.x =x_value[str(math.ceil(local_x))]/10+0.002
                print ('X Value: [%s]' % speed.linear.x)
                speed.linear.y =y_value[str(math.ceil(local_y))]/10+0.002
                speed.angular.z = 0.0
                print(path)

            else:
                print ('Position error: [%s]' % err_pos)
                speed.linear.x = 0.0
                speed.linear.y = 0.0
                speed.angular.z = 0.0


         

            desired_position_.x =local_x
            desired_position_.y =local_y

        pub.publish(speed)
        r.sleep()