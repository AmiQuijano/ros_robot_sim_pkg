#!/usr/bin/env python3

"""
.. module:: srvNode_b

   :platform: Unix
   :synopsis: Service ROS Node for robot simulator

.. moduleauthor:: Ami Sofia Quijano Shimizu

This node implements a service node that, when called, returns the coordinates of the last target sent by the user. 

Subscribes to:

**/reaching_goal/goal** : Topic which receives coordinates of the last target sent

Service:

**get_last_target** : Service which replies with the x and y coordinates of the last target sent as well as a boolean which indicates if the service request was successful or not.
"""

# Ami Sofia Quijano Shimizu
# Reseach Track 1 - Assignment 2 b)

# Instructions: Create a service node that, when called, returns the coordinates of the last target sent by the user. 


# Useful imports
import rospy
from ros_robot_sim_pkg.srv import GetLastTarget, GetLastTargetResponse  # Import the custom service message
from ros_robot_sim_pkg.msg import PlanningActionGoal # Import the message type from /reaching_goal/goal topic


class GetLastTargetService:
    """
    Brief:
        Class for representing the GetLastTargetService node
    """

    def __init__(self):
        """
        Brief:
            Initialization function for the ROS service, Subscriber and variables as required

        Detailed Description:
            1. Creates a list variable ``last_target`` to store the last target coordinates sent as [x y]
            2. Creates the service ``get_last_target`` with service message type ``GetLastTarget``
            3. Creates a subscriber for the topic ``/reaching_goal/goal``
        """
    
        # Variable to store the last target coordinates
        self.last_target = []

        # Create a service named 'get_last_target' with the custom service message type
        self.service = rospy.Service('get_last_target', GetLastTarget, self.handle_get_last_target)

        # Subscribe to the '/reaching_goal/goal' topic to get the last target coordinates
        self.target_subscriber = rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, self.target_callback)
    

    def target_callback(self, msg):
        """
        Brief:
            Subscriber callback function that updates the last target coordinates when the user enters a new target

        Detailed Description:
            1. Saves the x and y coordinates of the last target in the variable ``last_target`` by splitting the message received in the ``/reaching_goal/goal`` topic 

        Args:
            msg: Message received in the ``/reaching_goal/goal`` topic
        """
        
        # Save in last_target variable the x and y position message from /reaching_goal/goal topic
        if self.last_target == []:
            self.last_target.append(msg.goal.target_pose.pose.position.x)
            self.last_target.append(msg.goal.target_pose.pose.position.y)
        
        else:
            self.last_target[0] = msg.goal.target_pose.pose.position.x
            self.last_target[1] = msg.goal.target_pose.pose.position.y


    def handle_get_last_target(self, req):
        """
        Brief:
            Service callback function to handle requests for the last target coordinates
            
        Detailed Description:
            1. Creates a response object for the service message ``GetLastTarget``  
            2. If at least one target has been sent by user, the x and y coordinates of the last target (obtained from the Subscriber callback function) as well as a successful request confirmation are set as the service response message  
            3. If no target has been sent at all, an unsuccessful request confirmation is sent as the service response message

        Args:
            req: Request message sent to the server

        Returns:

        **response**: ``GetLastTarget`` service response message
        """
        
        # Create a response object using the custom service message type
        response = GetLastTargetResponse()  
        
        # If a last target exists, populate the response fields with its coordinates and the success confirmation  
        if self.last_target:  
            response.x = self.last_target[0]
            response.y = self.last_target[1]
            response.success = True
        
        # If no last target exists, set the success field to False
        else:
            response.success = False
        
        # Return the response to the service caller
        return response  


def main():
    """
    Description:
        Initialization of the service node
    """

    try:
        # Initialize the ROS node
        rospy.init_node('get_last_target_server')
    
        # Instantiate the GetLastTargetService class and start the ROS node
        get_last_target_service = GetLastTargetService()
    
        # Keep the script running until the node is shut down
        rospy.spin()  
    
    except rospy.ROSInterruptException:
        # If for some issue the previous lines could't be executed, print this message:
        print("Program interrupted before completion", file=sys.stderr)


if __name__ == '__main__':
    main()
    
    

