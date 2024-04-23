"""
.. module:: srvNode_c

   :platform: Unix
   :synopsis: Service ROS Node for robot simulator

.. moduleauthor:: Ami Sofia Quijano Shimizu

This node implements 
1) A service that subscribes to the robot’s position and velocity (using the custom message) and 
2) Implements a server to retrieve the distance of the robot from the target and the robot’s average speed. The size of the averaging window is set as a parameter in the launch file.

Subscribes to:
**/PosVel** : Topic which 
**/reaching_goal/goal**: 

Service:
**get_dist_speed** : Service which 
"""

#!/usr/bin/env python3

# Ami Sofia Quijano Shimizu
# Research Track 1 - Assignment 2 c)

# Instructions: Create a service node that subscribes to the robot’s position and velocity (using the custom message) and implements a server to retrieve the distance of the robot from the target and the robot’s average speed.
# The size of the averaging window is set as a parameter in the launch file.

# Useful imports
import rospy
import math
from ros_robot_simulation_pkg.srv import GetDistSpeed, GetDistSpeedResponse  # Import the custom service message
from ros_robot_simulation_pkg.msg import PosVel # Import the custom message type
from ros_robot_simulation_pkg.msg import PlanningActionGoal # Import the message type from /reaching_goal/goal topic
from nav_msgs.msg import Odometry # Import message type of /odom topic
from geometry_msgs.msg import Point
from collections import deque  # Import deque for efficient averaging
#from srvNode_b import target_callback # Import the target callback used in srvNode_b.py


class GetDistSpeedService:
    """
    Description:
        {description}
    """

    def __init__(self):
        """
        Description:
            Initialization function for the ROS service, subscriber, and variables as required

        Args:
            {args}
        """

        # Variable to store the last target coordinates
        self.last_target = []

        # Variable to store the robot's current position and velocity
        self.posvel = PosVel()

        # Size of the averaging window
        self.window_size = rospy.get_param("/window_size")

        # Queue to store the last N velocities for averaging
        self.avg_vel_x_queue = deque(maxlen=self.window_size)
        self.avg_vel_z_queue = deque(maxlen=self.window_size)

        # Create a service named 'get_dist_speed' with its custom service message type and callback function
        self.service = rospy.Service('get_dist_speed', GetDistSpeed, self.handle_get_dist_speed)

        # Subscribe to the '/PosVel' topic to get the robot's current position and velocity
        rospy.Subscriber('/PosVel', PosVel, self.posvel_callback)
        
        # Subscribe to the '/reaching_goal/goal' topic to get the last target coordinates
        rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, self.target_callback)


    def target_callback(self, msg):
        """
        Description:
            Subscriber callback function to update the last_target variable when a new target is received in the /reaching_goal/goal topic

        Args:
            ``msg``:
            ``self``:
        """
        
        # Save in last_target variable the x and y position message from /reaching_goal/goal topic
        if self.last_target == []:
            self.last_target.append(msg.goal.target_pose.pose.position.x)
            self.last_target.append(msg.goal.target_pose.pose.position.y)
        
        else:
            self.last_target[0] = msg.goal.target_pose.pose.position.x
            self.last_target[1] = msg.goal.target_pose.pose.position.y
            
            
    def posvel_callback(self, msg):
        """
        Description:
            Subscriber callback function to update the robot's position and velocity variables when a new message is received in the /PosVel topic

        Args:
            ``msg``:
            ``self``:
        """

        # Update the robot's current position
        self.posvel.x = msg.x
        self.posvel.y = msg.y

        # Update the robot's current velocity
        self.posvel.vel_x = msg.vel_x
        self.posvel.vel_z = msg.vel_z

        # Append the velocity to the queue for averaging
        self.avg_vel_x_queue.append(msg.vel_x)
        self.avg_vel_z_queue.append(msg.vel_z)

    def handle_get_dist_speed(self, req):
        """
        Description:
            Service callback function to handle requests for the robot's distance from the target and average speed

        Args:
            ``req``:
            ``self``:
        """

        # Create a response object using the custom service message type
        response = GetDistSpeedResponse()
        
        # If a last target exists, then populate the distance responses
        if self.last_target:        
            
            # Calculate the cartesian and euclidean distances from the target
            response.dist_x = self.posvel.x - self.last_target[0]
            response.dist_y = self.posvel.y - self.last_target[1]
            response.dist_eucl = math.sqrt(response.dist_x**2 + response.dist_y**2)
            
            # Set the distance success field to True
            response.success_dist = True
            
            # If there are enough registered speeds according to the window size
            if len(self.avg_vel_x_queue) == self.window_size and len(self.avg_vel_z_queue) == self.window_size:

                # Calculate the average speed using the velocities in the queue
                response.avg_vel_x = sum(self.avg_vel_x_queue) / len(self.avg_vel_x_queue)
                response.avg_vel_z = sum(self.avg_vel_z_queue) / len(self.avg_vel_z_queue)

                # Set the velocity success field to True
                response.success_avg_vel = True
            
            # If there aren't enough readings,     
            else:
                # Set the velocity success field to False
                response.success_avg_vel = False
            
        # If a target doesn't exist, 
        else:
            # Set the position success field to False
            response.success_dist = False
            
            # If there are enough registered speeds according to the window size
            if len(self.avg_vel_x_queue) == self.window_size and len(self.avg_vel_z_queue) == self.window_size:

                # Calculate the average speed using the velocities in the queue
                response.avg_vel_x = sum(self.avg_vel_x_queue) / len(self.avg_vel_x_queue)
                response.avg_vel_z = sum(self.avg_vel_z_queue) / len(self.avg_vel_z_queue)

                # Set the velocity success field to True
                response.success_avg_vel = True
            
            # If there aren't enough readings,    
            else:
                # Set the velocity success field to False
                response.success_avg_vel = False
            
        # Return the response to the service caller
        return response

if __name__ == '__main__':

    try:
        # Initialize the ROS node
        rospy.init_node('get_dist_speed_server')

        # Instantiate the RobotInfoService class and start the ROS node
        get_dist_speed_service = GetDistSpeedService()

        # Keep the script running until the node is shut down
        rospy.spin()

    except rospy.ROSInterruptException:
        # If for some issue the previous lines couldn't be executed, print this message:
        print("Program interrupted before completion", file=sys.stderr)

