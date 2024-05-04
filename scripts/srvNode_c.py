#!/usr/bin/env python3

"""
.. module:: srvNode_c

   :platform: Unix
   :synopsis: Service ROS Node for robot simulator

.. moduleauthor:: Ami Sofia Quijano Shimizu

This node subscribes to the robot’s position and velocity (using the custom message) and implements a service to retrieve the distance of the robot from the target and the robot’s average speed. 
The size of the averaging window is set as a parameter in the launch file.

Subscribes to:

**/PosVel**: Topic which receives the robot's position (in x and y) and velocity (in x and z) as a custom message of type ``PosVel``.

**/reaching_goal/goal**: Topic which receives information about the last goal sent including header, goal ID, target position and target orientation

Service:

**get_dist_speed**: Service which replies with the x, y and euclidean distances from the robot to the current target, the average linear speed in x, 
the average angular speed in and z, a boolean which indicates if the service request was successful for distance calculation and another boolean which indicates 
if the service request was successful for velocity calculation
"""

# Ami Sofia Quijano Shimizu
# Research Track 1 - Assignment 2 c)

# Instructions: Create a service node that subscribes to the robot’s position and velocity (using the custom message) and implements a server to retrieve the distance of the robot from the target and the robot’s average speed.
# The size of the averaging window is set as a parameter in the launch file.

# Useful imports
import rospy
import math
from ros_robot_sim_pkg.srv import GetDistSpeed, GetDistSpeedResponse  # Import the custom service message
from ros_robot_sim_pkg.msg import PosVel # Import the custom message type
from ros_robot_sim_pkg.msg import PlanningActionGoal # Import the message type from /reaching_goal/goal topic
from nav_msgs.msg import Odometry # Import message type of /odom topic
from geometry_msgs.msg import Point
from collections import deque  # Import deque for efficient averaging
#from srvNode_b import target_callback # Import the target callback used in srvNode_b.py


class GetDistSpeedService:
    """
    Brief:
        Class for representing the GetDistSpeedService node
    """

    def __init__(self):
        """
        Brief:
            Initialization function for the ROS service, subscriber, and variables as required

        Detailed Description:
            1. Creates a list variable ``last_target`` to store the last target coordinates sent as [x y]
            2. Creates a variable ``posvel`` of the message type ``PosVel`` to store the robot's current xy position and  xz velocity
            3. Creates the ``window_size`` variable which corresponds to the N velocities that will be averaged. It is a parameter from the launch file  
            4. Creates the variables ``avg_vel_x_queue`` and ``avg_vel_z_queue`` to queue and store the last N velocities in x and z
            5. Creates the service ``get_dist_speed`` with service message type ``GetDistSpeed``
            6. Creates a subscriber for the ``/reaching_goal/goal`` topic
            7. Creates a subscriber for the ``/PosVel`` topic
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
        Brief:
            Subscriber callback function that updates the last target coordinates when the user enters a new target

        Detailed Description:
            1. Saves the x and y coordinates of the last target in the variable ``last_target`` by splitting the message received in the ``/reaching_goal/goal`` topic 

        Args:
            ``msg``: Message received in the ``/reaching_goal/goal`` topic
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
        Brief:
            Subscriber callback function that updates the robot's current x,y positions and x,z velocities and stores it in the queue the velocities

        Detailed Description:
            1. Saves the current x and y coordinates and the current x and z velocities in the variable ``posvel`` by splitting the message received in the ``/PosVel`` topic 
            2. Appends in the queue list variables ``avg_vel_x_queue`` and ``avg_vel_z_queue`` the x and z velocities (for average caculation)

        Args:
            ``msg``: Message received in the ``/PosVel`` topic
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
        Brief:
            Service callback function to handle requests for the robot's distance from the target and average speed

        Detailed Description:
            1. Creates a response object for the service message ``GetDistSpeed``
            2. If at least one target has been sent by user, the cartesian x and y distances and euclidean distance from the robot to the target are computed (with values obtained from the previous 2 Subscriber callback functions) and sent as a service response message together with a successful request confirmation for the distance calculation
            3. If no target has been sent at all, an unsuccessful request confirmation for the distance calculation is sent as the service response message
            4. If there are N registered speeds according to the window size, the average of x and z velocities are computed and sent as a service response message together with a successful request confirmation for the average speed calculation
            5. If there are less than N registered speeds according to the window size, an unsuccessful request confirmation for the average speed calculation is sent as a service response message 

        Args:
            req: Request message sent to the server
        
        Returns:
            response: ``GetDistSpeed`` service response message
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


def main():
    """
    Brief:
        Initialization of the service node
    """
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


if __name__ == '__main__':
    main()

    

