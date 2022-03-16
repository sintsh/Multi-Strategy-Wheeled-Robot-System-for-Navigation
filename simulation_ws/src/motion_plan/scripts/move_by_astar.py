#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

from math import atan2
from geometry_msgs.msg import Twist, Point
from astar_algo import astar

import math

def map_grid_to_world():
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

    return maze, x_value, y_value



theta = 0.0

def clbk_odom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

 




def main():


    rospy.init_node('speed_controller')
    sub = rospy.Subscriber("/odom",Odometry,clbk_odom)
    pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)

    maze ,x_value , y_value = map_grid_to_world()
    # Model starting point
    in_x = 9
    in_y = 2

    x = float(x_value[str(in_x)])
    y = float(y_value[str(in_y)])


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

   
    path = path[::-1]
    path = list(path)
  
    print(path)

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

if __name__ == '__main__':
    main()