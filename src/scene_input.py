import sys
import copy
import rospy
import moveit_commander
from moveit_commander import roscpp_initialize, roscpp_shutdown
import moveit_msgs.msg
import geometry_msgs.msg



if __name__ == "__main__":
	roscpp_initialize(sys.argv)
	rospy.init_node('scene input', anonymous=True)
	rospy.loginfo("Starting scene input")
	# add some code here

	#Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
	scene = moveit_commander.PlanningSceneInterface()
	rospy.sleep(1) 

	# clean the scene
	scene.remove_world_object("pole")
	scene.remove_world_object("table")
	scene.remove_world_object("part")
	rospy.sleep(1) 

	# publish a demo scene
	p = geometry_msgs.msg.PoseStamped()
	p.header.frame_id = "world"
	p.pose.position.x = 0.7
	p.pose.position.y = -0.4
	p.pose.position.z = 0.85
	p.pose.orientation.w = 1.0
	scene.add_box("pole", p, (0.3, 0.1, 1.0))

	p.pose.position.y = -0.2
	p.pose.position.z = 0.175
	scene.add_box("table", p, (0.5, 1.5, 0.35))

	p.pose.position.x = 0.6
	p.pose.position.y = -0.7
	p.pose.position.z = 0.5
	scene.add_box("part", p, (0.15, 0.1, 0.3))
	    
	rospy.spin()
	roscpp_shutdown()
	rospy.loginfo("Stopping scene input")





