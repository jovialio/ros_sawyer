import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg



#First initialize moveit_commander and rospy.
print "============ Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

#Instantiate a RobotCommander object. This object is an interface to the robot as a whole.
robot = moveit_commander.RobotCommander()

#Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
scene = moveit_commander.PlanningSceneInterface()

# clean the scene
scene.remove_world_object("pole")
scene.remove_world_object("table")
scene.remove_world_object("part")

# publish a demo scene
p = geometry_msgs.msg.PoseStamped()
p.header.frame_id = robot.get_planning_frame()
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

#Instantiate a MoveGroupCommander object. This object is an interface to one group of joints. 
#In this case the group is the joints in the left arm. This interface can be used to plan and execute 
#motions on the left arm.
group = moveit_commander.MoveGroupCommander("manipulator") # use right_arm for actual sawyer, manipulator for sim

#create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=20)

#Wait for RVIZ to initialize
print "============ Waiting for RVIZ..."
rospy.sleep(10)
print "============ Starting tutorial "

#get the name of the reference frame for this robot
print "============ Reference frame: %s" % group.get_planning_frame()

#print the name of the end-effector link for this group
print "============ End effector: %s" % group.get_end_effector_link()

#get a list of all the groups in the robot
print "============ Robot Groups:"
print robot.get_group_names()

#or debugging it is useful to print the entire state of the robot.
print "============ Printing robot state"
print robot.get_current_state()
print "============"

#get robot current pose
print "============ Printing robot state"
print group.get_current_pose()
print "============"


#plan a motion for this group to a desired pose for the end-effector
print "============ Generating plan 1"
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 1.0
pose_target.position.x = 0.7
pose_target.position.y = -0.05
pose_target.position.z = 1.1
group.set_pose_target(pose_target)

#all the planner to compute the plan and visualize it if successful 
#Note that we are just planning, not asking move_group to actually move the robot
plan1 = group.plan()
print "============ Waiting while RVIZ displays plan1..."
rospy.sleep(5)

# Uncomment below line when working with a real robot
# group.go(wait=True)

# Use execute instead if you would like the robot to follow
# the plan that has already been computed
group.execute(plan1)