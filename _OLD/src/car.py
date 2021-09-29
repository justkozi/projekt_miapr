#! /usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path

import math



path = []

def callback(data):
    # rospy.loginfo(rospy.get_name()+"I heard %s",data)
    path = data.poses
    # print(path)
    # print(len(path))
    point = None
    while(len(path) > 0):
            
        point = path.pop()
        print(point.pose.position)


# def main():
#     rospy.spin()

    # rospy.init_node('go_to_point')
    
    # pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    # sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    


    # rate = rospy.Rate(20)
    # while not rospy.is_shutdown():
    #     if state_ == 0:
    #         fix_yaw(desired_position_)
    #     elif state_ == 1:
    #         go_straight_ahead(desired_position_)
    #     elif state_ == 2:
    #         done()
    #         pass
    #     else:
    #         rospy.logerr('Unknown state!')
    #         pass
    #     rate.sleep()

# if __name__ == '__main__':
#     rospy.init_node('listener', anonymous=True)
#     rospy.Subscriber("path", Path, callback)
#     main()


class GoToPose():
    def __init__(self):

        self.goal_sent = False

	# What to do if shut down (e.g. Ctrl-C or failure)
	rospy.on_shutdown(self.shutdown)
	
	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 60 seconds to complete task
	success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=False)
        navigator = GoToPose()

        # Customize the following values so they are appropriate for your location
        position = {'x': 1.22, 'y' : 2.56}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = navigator.goto(position, quaternion)

        if success:
            rospy.loginfo("Hooray, reached the desired pose")
        else:
            rospy.loginfo("The base failed to reach the desired pose")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")