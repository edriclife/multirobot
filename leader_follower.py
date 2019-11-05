#! /usr/bin/env python

import rospy
import time
import actionlib
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


# Create constants to indicate action status
PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

# Create float64 variables to store criminal odom-related information
count = 0
OFFSET = 0.35
leader_x = 0.5
leader_y = 0.5
leader_z = 0.0
leader_w = 0.1
current_leader_x = 0.5
current_leader_y = 0.5
current_leader_z = 0.0
current_leader_w = 0.1


def start_operation():

    # Initialize the get main node-------------------------------------------------------------------------------# Initialize main node START

    rospy.init_node('leader_follower')

    #------------------------------------------------------------------------------------------------------------# Initialize main node END


	
    # Define the callback to get odometry information of the leader--------------------------------------------# Create leader odom subscriber START

    def callback(msg):
	global count
	global leader_x
	global leader_y
	global leader_z
	global leader_w
	global current_leader_x
	global current_leader_y	
	global current_leader_z
	global current_leader_w

	leader_x = msg.pose.pose.position.x
	leader_y = msg.pose.pose.position.y
	leader_z = msg.pose.pose.orientation.z
	leader_w = msg.pose.pose.orientation.w
	if count == 0:
	    current_leader_x = leader_x
	    current_leader_y = leader_y
	    current_leader_z = leader_z
	    current_leader_w = leader_w

	count+=1


    # Create an object to subscribe to the leader's odom-related information
    odom_sub = rospy.Subscriber('tb3_1/odom',Odometry,callback)

    #---------------------------------------------------------------------------------------------------------- # Create leader odom subscriber END



    # Define the feedback callback function. Will be called when receive feedback from action server----------- # Create leader follower START 

    def feedback_callback(feedback):
        #print("Haven't reached the goal yet.")
	pass


    # Initialize the action client 
    client = actionlib.SimpleActionClient("tb3_0/move_base",MoveBaseAction)


    # Wait and check for the action server to be active
    rospy.loginfo("Waiting for action sever 'move_base' to be active.")
    client.wait_for_server()
    rospy.loginfo("Action server found and is active.")


    # Wait for criminal to start moving
    print("Waiting for leader to move...")
#changed
    while abs(current_leader_x - leader_x)<0.1 and abs(current_leader_y - leader_y)<0.1:
	pass


    # Create a goal message
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w = computeNewCoor()
    client.send_goal(goal, feedback_cb=feedback_callback)


    # Get the state of the current action and define further actions that needs to be taken
    state_result = client.get_state()
    rate = rospy.Rate(1)

    while state_result < DONE:
        rate.sleep()
        state_result = client.get_state()

	# If leader is still moving, send a new goal to override the old one
	print("Current leader coordinates: x="+str(leader_x)+", y="+str(leader_y))
	print("Currently processed/known: z="+str(leader_z)+", w="+str(leader_w))
#changed
	if abs(current_leader_x - leader_x)>=0.05 or abs(current_leader_y - leader_y)>=0.05:
	    next_goal = MoveBaseGoal()
    	    goal.target_pose.header.frame_id = "map"
    	    goal.target_pose.header.stamp = rospy.Time.now()
    	    goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w = computeNewCoor()
	    client.send_goal(goal, feedback_cb=feedback_callback)
	    print("New goal sent!")
	

    rospy.loginfo("Result state: "+str(state_result))
    #if state_result == ERROR:
        #rospy.logerr("Something went wrong in the server.")
    #if state_result == WARN:
        #rospy.logwarn("THere is a warning in the server.")

    #---------------------------------------------------------------------------------------------------------- # leader follower action client END



# Method to compute new x and y coordinates for the follower based on the changing coordinates of leader
def computeNewCoor():
    global leader_x
    global leader_y
    global leader_z
    global leader_w	
    global current_leader_x
    global current_leader_y
    global current_leader_z
    global current_leader_w
    print("Currently processed/known: x="+str(leader_x)+", y="+str(leader_y))
    print("Currently processed/known: z="+str(leader_z)+", w="+str(leader_w))
    x_coor = 0.0
    y_coor = 0.0
    z_coor = 0.0
    w_coor = 0.0

	
    # NORTH, SOUTH, EAST, WEST
    if leader_z > 0.695 and leader_z < 0.705:
        x_coor = leader_x
	y_coor = leader_y - OFFSET
    elif leader_z < -0.705 and leader_z < -0.695:
        x_coor = leader_x
	Y_COOR = leader_y + OFFSET
    elif leader_z > -0.005 and leader_z < 0.005:
        x_coor = leader_x - OFFSET
	y_coor = leader_y
    elif leader_z > 0.995 and leader_z < -0.995:
        x_coor = leader_x + OFFSET
	y_coor = leader_y
    # NORTH-EAST, NORTH-WEST, SOUTH-WEST, SOUTH-EAST
    elif leader_z > 0.420 and leader_z < 0.430:
        x_coor = leader_x - OFFSET
	y_coor = leader_y - OFFSET
    elif leader_z > 0.920 and leader_z < 0.930:
	x_coor = leader_x + OFFSET
	y_coor = leader_y - OFFSET
    elif leader_z > -0.898 and leader_z < -0.888:
	x_coor = leader_x + OFFSET
        y_coor = leader_y + OFFSET
    elif leader_z > -0.355 and leader_z < -0.345:
	x_coor = leader_x - OFFSET
        y_coor = leader_y + OFFSET
    # MORE ORIENTATIONS FOR MORE ACCURACY (ANTI-CLOCKWISE)
    elif leader_z >= 0.005  and leader_z < 0.2125:
	x_coor = leader_x - OFFSET
	y_coor = leader_y - OFFSET
    elif leader_z >= 0.2125 and leader_z <= 0.420:
	x_coor = leader_x - OFFSET
	y_coor = leader_y - OFFSET
    elif leader_z >= 0.430 and leader_z < 0.5625:
	x_coor = leader_x - OFFSET
	y_coor = leader_y - OFFSET
    elif leader_z >= 0.5625 and leader_z <= 0.695:
	x_coor = leader_x - OFFSET
	y_coor = leader_y - OFFSET

    elif leader_z >= 0.705 and leader_z < 0.8125:
	x_coor = leader_x + OFFSET
	y_coor = leader_y - OFFSET
    elif leader_z >= 0.8125 and leader_z <= 0.920:
	x_coor = leader_x + OFFSET
	y_coor = leader_y - OFFSET
    elif leader_z >= 0.930 and leader_z < 0.9625:
	x_coor = leader_x + OFFSET
	y_coor = leader_y - OFFSET
    elif leader_z >= 0.9625 and leader_z <= 0.995:
	x_coor = leader_x + OFFSET
	y_coor = leader_y - OFFSET

    elif leader_z >= -0.995 and leader_z < -0.9465:
	x_coor = leader_x + OFFSET
	y_coor = leader_y + OFFSET
    elif leader_z >= -0.9465 and leader_z <= -0.898:
	x_coor = leader_x + OFFSET
	y_coor = leader_y + OFFSET
    elif leader_z >= -0.888 and leader_z < -0.7965:
	x_coor = leader_x + OFFSET
	y_coor = leader_y + OFFSET
    elif leader_z >= -0.7965 and leader_z <= -0.705:
	x_coor = leader_x + OFFSET
	y_coor = leader_y + OFFSET

    elif leader_z >= -0.705 and leader_z < -0.525:
	x_coor = leader_x - OFFSET
	y_coor = leader_y + OFFSET
    elif leader_z >= -0.525 and leader_z <= -0.355:
	x_coor = leader_x - OFFSET
	y_coor = leader_y + OFFSET
    elif leader_z >= -0.345 and leader_z < -0.175:
	x_coor = leader_x - OFFSET
	y_coor = leader_y + OFFSET
    elif leader_z >= -0.175 and leader_z <= -0.005:
	x_coor = leader_x - OFFSET
	y_coor = leader_y + OFFSET

    current_leader_x = leader_x
    current_leader_y = leader_y
    current_leader_z = leader_z
    current_leader_w = leader_w

    z_coor = leader_z
    w_coor = leader_w

    return x_coor,y_coor,z_coor,w_coor



# Main method to start the program
if __name__ == "__main__":
    try:
        start_operation()
    except rospy.ROSInterruptException:
	rospy.loginfo("Operation interrupted, aborting operation...")
        








