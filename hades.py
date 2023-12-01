#!/usr/bin/env python3

# Students: Jordan Mosakowski, Maya Murphy, Chris Bird
# Final Project: Project Hades
# Date: December 1st, 2023
# Acknowledgements: Code partially used to generate marker arrays: https://answers.ros.org/question/373802/minimal-working-example-for-rviz-marker-publishing/
# Furthermore, we used the code for the Move Group Python Interface tutorial as a base, but heavily modified it to do what we accomplished. We have included the lecense for it below.

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman



import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Quaternion,PoseStamped
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

tau = pi * 2

def all_close(goal, actual, tolerance):
 """
 Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
 @param: goal       A list of floats, a Pose or a PoseStamped
 @param: actual     A list of floats, a Pose or a PoseStamped
 @param: tolerance  A float
 @returns: bool
 """
 all_equal = True
 if type(goal) is list:
   for index in range(len(goal)):
     if abs(actual[index] - goal[index]) > tolerance:
       return False


 elif type(goal) is geometry_msgs.msg.PoseStamped:
   return all_close(goal.pose, actual.pose, tolerance)


 elif type(goal) is geometry_msgs.msg.Pose:
   return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)


 return True

# Helper function to make pose given an x,y,z position and orientation w
def make_pose(x,y,z,w):
   pose = PoseStamped()
   pose.header.frame_id = "panda_link0"
   pose.pose.position.x = x
   pose.pose.position.y = y
   pose.pose.position.z = z
   pose.pose.orientation.w = w
   return pose


class ProjectHades(object):
 """ProjectHades"""
 def __init__(self):
   super(ProjectHades, self).__init__()

   ## First initialize `moveit_commander`_ and a `rospy`_ node:
   moveit_commander.roscpp_initialize(sys.argv)
   rospy.init_node('project_hades',
                   anonymous=True)


   ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
   ## the robot:
   robot = moveit_commander.RobotCommander()


   ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
   ## to the world surrounding the robot:
   scene = moveit_commander.PlanningSceneInterface()


   ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
   ## to one group of joints.  In this case the group is the joints in the Panda
   ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
   ## you should change this value to the name of your robot arm planning group.
   ## This interface can be used to plan and execute motions on the Panda:
   group_name = "panda_arm"
   group = moveit_commander.MoveGroupCommander(group_name)


   ## We create a `DisplayTrajectory`_ publisher which is used later to publish
   ## trajectories for RViz to visualize:
   display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                  moveit_msgs.msg.DisplayTrajectory,
                                                  queue_size=20)


   markers_pub = rospy.Publisher(
           "visualization_marker_array",
           MarkerArray,
           queue_size=1)
   rospy.sleep(2)
          
   # Allow the functions to call the variables
   self.markers_pub = markers_pub
   self.markers = MarkerArray()
   self.robot = robot
   self.scene = scene
   self.group = group
   self.display_trajectory_publisher = display_trajectory_publisher
   self.eef_link = group.get_end_effector_link()

 # Helper function to open the hand
 def open_gripper(self, amt=0.03):
   grabber_group = moveit_commander.MoveGroupCommander("panda_hand")
   grabber_goal = grabber_group.get_current_joint_values()
   grabber_goal[0] = amt
   grabber_goal[1] = amt
   grabber_group.go(grabber_goal, wait = True)

 # Helper function to close the hand
 def close_gripper(self, amt=0.01):
   grabber_group = moveit_commander.MoveGroupCommander("panda_hand")
   grabber_goal = grabber_group.get_current_joint_values()
   grabber_goal[0] = amt
   grabber_goal[1] = amt
   grabber_group.go(grabber_goal, wait = True)




 def add_objects(self):
   # create the surface that the samples sit on
   self.scene.add_box("table",make_pose(0.5,0,0.39,1.0),(0.2,0.6,0.02))

   # create the object for collecting the water samples
   self.scene.add_box("watersampler",make_pose(0.5,0.2,0.5,1.0),(0.02,0.02,0.2))

   # create the object to represent the surface sampler, which consists of a vertical stick and horizontal cylinder
   self.scene.add_box("surfacesampler",make_pose(0.5,0,0.5,1.0),(0.02,0.02,0.2))
   pose = make_pose(0.5,0,0.41,1.0)
   orientation = quaternion_from_euler(-tau/4, 0, 0)
   pose.pose.orientation = Quaternion(*orientation)
   self.scene.add_cylinder("surfacesampler2",pose,0.14, 0.01)

   # create the object to take air measurements
   self.scene.add_box("airsampler",make_pose(0.5,-0.2,0.45,1.0), (0.04,0.04,0.1))


   # Create a lake
   marker = Marker()
   marker.header.frame_id = "panda_link0"
   marker.lifetime = rospy.Duration.from_sec(300)
   marker.type = marker.CUBE
   marker.action = marker.ADD
   marker.header.stamp = rospy.Time.now()
   marker.pose.position.x = 0
   marker.pose.position.y = 0.75
   marker.pose.position.z = 0
   marker.pose.orientation.w = 1.0
   marker.scale.x = 1
   marker.scale.y = 1
   marker.scale.z = 0.001
   marker.color.r = 0.0
   marker.color.g = 0.5
   marker.color.b = 1.0
   marker.color.a = 0.5
   marker.id = 0
   print("Adding Marker")
   self.markers.markers.append(marker)

   # Create grass
   marker = Marker()
   marker.header.frame_id = "panda_link0"
   marker.lifetime = rospy.Duration.from_sec(300)
   marker.type = marker.CUBE
   marker.action = marker.ADD
   marker.header.stamp = rospy.Time.now()
   marker.pose.position.x = 0
   marker.pose.position.y = -0.75
   marker.pose.position.z = 0
   marker.pose.orientation.w = 1.0
   marker.scale.x = 2
   marker.scale.y = 2
   marker.scale.z = 0.001
   marker.color.r = 0.0
   marker.color.g = 0.7
   marker.color.b = 0.0
   marker.color.a = 0.5
   marker.id = 1
   print("Adding Marker")
   self.markers.markers.append(marker)
   # while not rospy.is_shutdown():
   # self.markers_pub.publish(marker)
   self.markers_pub.publish(self.markers)


 def clear(self):
   self.scene.clear()


 def go_to_joint_state(self):
   # Copy class variables to local variables to make the web tutorials more clear.
   # In practice, you should use the class variables directly unless you have a good
   # reason not to.
   group = self.group


   ## BEGIN_SUB_TUTORIAL plan_to_joint_state
   ##
   ## Planning to a Joint Goal
   ## ^^^^^^^^^^^^^^^^^^^^^^^^
   ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
   ## thing we want to do is move it to a slightly better configuration.
   # We can get the joint values from the group and adjust some of the values:
   joint_goal = group.get_current_joint_values()
   joint_goal[0] = 0
   joint_goal[1] = -pi/4
   joint_goal[2] = 0
   joint_goal[3] = -pi/2
   joint_goal[4] = 0
   joint_goal[5] = pi/3
   joint_goal[6] = 0


   # The go command can be called with joint values, poses, or without any
   # parameters if you have already set the pose or joint target for the group
   group.go(joint_goal, wait=True)


   # Calling ``stop()`` ensures that there is no residual movement
   group.stop()


   ## END_SUB_TUTORIAL


   # For testing:
   # Note that since this section of code will not be included in the tutorials
   # we use the class variable rather than the copied state variable
   current_joints = self.group.get_current_joint_values()
   return all_close(joint_goal, current_joints, 0.01)

 # Take measurements of turbidity and benzene in the water.
 def sample_water(self):
   self.go_to_joint_state()
   self.open_gripper()

   waypoints = []
   wpose = self.group.get_current_pose().pose
   # orientation is from the side of the box
   orientation = quaternion_from_euler(-tau / 4, -tau / 8, -tau / 4)
   wpose.orientation = Quaternion(*orientation)
   wpose.position.x = 0.4
   wpose.position.y = 0.2
   wpose.position.z = 0.57
   waypoints.append(copy.deepcopy(wpose))
   wpose = self.group.get_current_pose().pose
   # rotate the orientation so it grabs from the top instead of the side
   orientation = quaternion_from_euler(-tau / 4, -tau / 8, -tau / 4)
   wpose.orientation = Quaternion(*orientation)
   wpose.position.x = 0.5
   wpose.position.y = 0.2
   wpose.position.z = 0.57
   waypoints.append(copy.deepcopy(wpose))
   (plan, fraction) = self.group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
   self.group.execute(plan, wait=True)
   self.close_gripper()
   self.attach_box("watersampler")
   self.go_to_joint_state()
   waypoints = []
   wpose = self.group.get_current_pose().pose
   # orientation is from the side of the box
   orientation = quaternion_from_euler(-tau / 4, -tau / 8, -tau / 4)
   wpose.orientation = Quaternion(*orientation)
   wpose.position.x = 0.0
   wpose.position.y = 0.6
   wpose.position.z = 0.3
   waypoints.append(copy.deepcopy(wpose))
   wpose = self.group.get_current_pose().pose
   # rotate the orientation so it grabs from the top instead of the side
   orientation = quaternion_from_euler(-tau / 4, -tau / 8, -tau / 4)
   wpose.orientation = Quaternion(*orientation)
   wpose.position.x = 0.0
   wpose.position.y = 0.6
   wpose.position.z = 0.1
   waypoints.append(copy.deepcopy(wpose))
   (plan, fraction) = self.group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
   self.group.execute(plan, wait=True)
   print("----------------\nTAKING WATER MEASUREMENTS")
   rospy.sleep(3)
   print("Benzeze: 18ppb")
   print("Turbidity: 75 NTU")
   self.go_to_joint_state()
   waypoints = []
   wpose = self.group.get_current_pose().pose
   # orientation is from the side of the box
   orientation = quaternion_from_euler(-tau / 4, -tau / 8, -tau / 4)
   wpose.orientation = Quaternion(*orientation)
   wpose.position.x = 0.4
   wpose.position.y = 0.2
   wpose.position.z = 0.6
   waypoints.append(copy.deepcopy(wpose))
   wpose = self.group.get_current_pose().pose
   # rotate the orientation so it grabs from the top instead of the side
   orientation = quaternion_from_euler(-tau / 4, -tau / 8, -tau / 4)
   wpose.orientation = Quaternion(*orientation)
   wpose.position.x = 0.5
   wpose.position.y = 0.2
   wpose.position.z = 0.6
   waypoints.append(copy.deepcopy(wpose))
   (plan, fraction) = self.group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
   self.group.execute(plan, wait=True)
   self.detach_box("watersampler")
   self.open_gripper()
   print("FINISHED WATER MEASUREMENTS\n\n\n")

 # Collect a sample of the surface material
 def sample_surface(self):
       self.go_to_joint_state()
       self.open_gripper()
       self.close_gripper(0.02)
       waypoints = []
       wpose = self.group.get_current_pose().pose
       # orientation is from the top
       orientation = quaternion_from_euler(0, -tau/2, 0)
       wpose.orientation = Quaternion(*orientation)
       wpose.position.x = 0.5
       wpose.position.y = 0
       wpose.position.z = 0.8
       waypoints.append(copy.deepcopy(wpose))
       wpose = self.group.get_current_pose().pose
       orientation = quaternion_from_euler(0, -tau/2, 0)
       wpose.orientation = Quaternion(*orientation)
       wpose.position.x = 0.5
       wpose.position.y = 0
       wpose.position.z = 0.65
       waypoints.append(copy.deepcopy(wpose))
       (plan, fraction) = self.group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold
       self.group.execute(plan, wait=True)
       # self.close_gripper()
       self.attach_box("surfacesampler",wait=False)
       self.attach_box("surfacesampler2")
       self.go_to_joint_state()

       waypoints = []
       wpose = self.group.get_current_pose().pose
       # orientation is from the side of the box
       orientation = quaternion_from_euler(0, -tau/2, tau/4)
       wpose.orientation = Quaternion(*orientation)
       wpose.position.x = 0.0
       wpose.position.y = -0.3
       wpose.position.z = 0.5
       waypoints.append(copy.deepcopy(wpose))
       wpose = self.group.get_current_pose().pose
       # rotate the orientation so it grabs from the top instead of the side
       orientation = quaternion_from_euler(0, -tau/2, tau/4)
       wpose.orientation = Quaternion(*orientation)
       wpose.position.x = 0.0
       wpose.position.y = -0.3
       wpose.position.z = 0.3
       waypoints.append(copy.deepcopy(wpose))
       (plan, fraction) = self.group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold
       self.group.execute(plan, wait=True)
       print("----------------\nCOLLECTING SAMPLE OF SURFACE")
       rospy.sleep(1)
       waypoints = []
       wpose = self.group.get_current_pose().pose
       # orientation is from the side of the box
       orientation = quaternion_from_euler(0, -tau/2, tau/4)
       wpose.orientation = Quaternion(*orientation)
       wpose.position.x = 0.0
       wpose.position.y = -0.4
       wpose.position.z = 0.3
       waypoints.append(copy.deepcopy(wpose))
       wpose = self.group.get_current_pose().pose
       # rotate the orientation so it grabs from the top instead of the side
       orientation = quaternion_from_euler(0, -tau/2, tau/4)
       wpose.orientation = Quaternion(*orientation)
       wpose.position.x = 0.0
       wpose.position.y = -0.65
       wpose.position.z = 0.3
       waypoints.append(copy.deepcopy(wpose))
       (plan, fraction) = self.group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold
       self.group.execute(plan, wait=True)
       rospy.sleep(3)


       self.go_to_joint_state()
       waypoints = []
       wpose = self.group.get_current_pose().pose
       # orientation is from the top
       orientation = quaternion_from_euler(0, -tau/2, 0)
       wpose.orientation = Quaternion(*orientation)
       wpose.position.x = 0.5
       wpose.position.y = 0
       wpose.position.z = 0.8
       waypoints.append(copy.deepcopy(wpose))
       wpose = self.group.get_current_pose().pose
       orientation = quaternion_from_euler(0, -tau/2, 0)
       wpose.orientation = Quaternion(*orientation)
       wpose.position.x = 0.5
       wpose.position.y = 0
       wpose.position.z = 0.67
       waypoints.append(copy.deepcopy(wpose))
       (plan, fraction) = self.group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold
       self.group.execute(plan, wait=True)
       self.detach_box("surfacesampler",wait=False)
       self.detach_box("surfacesampler2")
       self.open_gripper()
       print("FINISHED SURFACE SAMPLE COLLECTION\n\n\n")

 # measure the particulate matter and carbon monoxide levels in the air at different heights
 def sample_air(self):
       self.go_to_joint_state()
       self.open_gripper()
       waypoints = []
       wpose = self.group.get_current_pose().pose
       # orientation is from the top
       orientation = quaternion_from_euler(-tau / 4, -tau / 8, -tau / 4)
       wpose.orientation = Quaternion(*orientation)
       wpose.position.x = 0.4
       wpose.position.y = -0.2
       wpose.position.z = 0.52
       waypoints.append(copy.deepcopy(wpose))
       wpose = self.group.get_current_pose().pose
       orientation = quaternion_from_euler(-tau / 4, -tau / 8, -tau / 4)
       wpose.orientation = Quaternion(*orientation)
       wpose.position.x = 0.5
       wpose.position.y = -0.2
       wpose.position.z = 0.52
       waypoints.append(copy.deepcopy(wpose))
       (plan, fraction) = self.group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold
       self.group.execute(plan, wait=True)
       self.close_gripper(0.02)
       print("TAKING AIR QUALITY MEASUREMENTS")
       self.attach_box("airsampler")
       rospy.sleep(1)
      
       print("---------------\nHeight: 0.6m")
       print("Particulate Matter 2.5: 33.8µg/m^3")
       print("Carbon Monoxide: 1575ppm")
       waypoints = []
       wpose = self.group.get_current_pose().pose
       orientation = quaternion_from_euler(-tau / 4, -tau / 8, -tau / 4)
       wpose.orientation = Quaternion(*orientation)
       wpose.position.x = 0.4
       wpose.position.y = -0.2
       wpose.position.z = 0.6
       waypoints.append(copy.deepcopy(wpose))
       (plan, fraction) = self.group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold
       self.group.execute(plan, wait=True)
       rospy.sleep(2)
      
       print("---------------\nHeight: 0.7m")
       print("Particulate Matter 2.5: 35.1µg/m^3")
       print("Carbon Monoxide: 1600ppm")
       waypoints = []
       wpose = self.group.get_current_pose().pose
       orientation = quaternion_from_euler(-tau / 4, -tau / 8, -tau / 4)
       wpose.orientation = Quaternion(*orientation)
       wpose.position.x = 0.4
       wpose.position.y = -0.2
       wpose.position.z = 0.7
       waypoints.append(copy.deepcopy(wpose))
       (plan, fraction) = self.group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold
       self.group.execute(plan, wait=True)
       rospy.sleep(2)
      
       print("---------------\nHeight: 0.8m")
       print("Particulate Matter 2.5: 36.4µg/m^3")
       print("Carbon Monoxide: 1597ppm")
       waypoints = []
       wpose = self.group.get_current_pose().pose
       orientation = quaternion_from_euler(-tau / 4, -tau / 8, -tau / 4)
       wpose.orientation = Quaternion(*orientation)
       wpose.position.x = 0.35
       wpose.position.y = -0.12
       wpose.position.z = 0.8
       waypoints.append(copy.deepcopy(wpose))
       (plan, fraction) = self.group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold
       self.group.execute(plan, wait=True)
       rospy.sleep(2)
      
       print("---------------\nHeight: 0.9m")
       print("Particulate Matter 2.5: 36.9µg/m^3")
       print("Carbon Monoxide: 1607ppm")
       waypoints = []
       wpose = self.group.get_current_pose().pose
       orientation = quaternion_from_euler(-tau / 4, -tau / 8, -tau / 4)
       wpose.orientation = Quaternion(*orientation)
       wpose.position.x = 0.30
       wpose.position.y = -0.05
       wpose.position.z = 0.9
       waypoints.append(copy.deepcopy(wpose))
       (plan, fraction) = self.group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold
       self.group.execute(plan, wait=True)
       rospy.sleep(2)


       print("---------------\nHeight: 1.0m")
       print("Particulate Matter 2.5: 38.2µg/m^3")
       print("Carbon Monoxide: 1684ppm")
       waypoints = []
       wpose = self.group.get_current_pose().pose
       orientation = quaternion_from_euler(-tau / 4, -tau / 8, -tau / 4)
       wpose.orientation = Quaternion(*orientation)
       wpose.position.x = 0.3
       wpose.position.y = 0.0
       wpose.position.z = 1.0
       waypoints.append(copy.deepcopy(wpose))
       (plan, fraction) = self.group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold
       self.group.execute(plan, wait=True)
       rospy.sleep(2)



       print("---------------\nHeight: 1.1m")
       print("Particulate Matter 2.5: 37.6µg/m^3")
       print("Carbon Monoxide: 1643ppm")
       waypoints = []
       wpose = self.group.get_current_pose().pose
       orientation = quaternion_from_euler(-tau / 4, -tau / 8, -tau / 4)
       wpose.orientation = Quaternion(*orientation)
       wpose.position.x = 0.25
       wpose.position.y = 0.0
       wpose.position.z = 1.1
       waypoints.append(copy.deepcopy(wpose))
       (plan, fraction) = self.group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold
       self.group.execute(plan, wait=True)
       rospy.sleep(2)

       print("---------------\nHeight: 1.2m")
       print("Particulate Matter 2.5: 37.3µg/m^3")
       print("Carbon Monoxide: 1656ppm")
       waypoints = []
       wpose = self.group.get_current_pose().pose
       orientation = quaternion_from_euler(-tau / 4, -tau / 8, -tau / 4)
       wpose.orientation = Quaternion(*orientation)
       wpose.position.x = 0.20
       wpose.position.y = 0.0
       wpose.position.z = 1.2
       waypoints.append(copy.deepcopy(wpose))
       (plan, fraction) = self.group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold
       self.group.execute(plan, wait=True)
       rospy.sleep(2)


       self.go_to_joint_state()
       # return
       waypoints = []
       wpose = self.group.get_current_pose().pose
       orientation = quaternion_from_euler(-tau / 4, -tau / 8, -tau / 4)
       wpose.orientation = Quaternion(*orientation)
       wpose.position.x = 0.4
       wpose.position.y = -0.2
       wpose.position.z = 0.5
       waypoints.append(copy.deepcopy(wpose))
       wpose = self.group.get_current_pose().pose
       orientation = quaternion_from_euler(-tau / 4, -tau / 8, -tau / 4)
       wpose.orientation = Quaternion(*orientation)
       wpose.position.x = 0.5
       wpose.position.y = -0.2
       wpose.position.z = 0.5
       waypoints.append(copy.deepcopy(wpose))
       (plan, fraction) = self.group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold
       self.group.execute(plan, wait=True)
       self.detach_box("airsampler")
       self.open_gripper()
       print("FINISHED AIR QUALITY MEASUREMENTS\n\n\n")
 def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
   # Copy class variables to local variables to make the web tutorials more clear.
   # In practice, you should use the class variables directly unless you have a good
   # reason not to.
   scene = self.scene


   ## BEGIN_SUB_TUTORIAL wait_for_scene_update
   ##
   ## Ensuring Collision Updates Are Receieved
   ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   ## If the Python node dies before publishing a collision object update message, the message
   ## could get lost and the box will not appear. To ensure that the updates are
   ## made, we wait until we see the changes reflected in the
   ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
   ## For the purpose of this tutorial, we call this function after adding,
   ## removing, attaching or detaching an object in the planning scene. We then wait
   ## until the updates have been made or ``timeout`` seconds have passed
   start = rospy.get_time()
   seconds = rospy.get_time()
   while (seconds - start < timeout) and not rospy.is_shutdown():
     # Test if the box is in attached objects
     attached_objects = scene.get_attached_objects([''])
     is_attached = len(attached_objects.keys()) > 0


     # Test if the box is in the scene.
     # Note that attaching the box will remove it from known_objects
     is_known = '' in scene.get_known_object_names()


     # Test if we are in the expected state
     if (box_is_attached == is_attached) and (box_is_known == is_known):
       return True


     # Sleep so that we give other threads time on the processor
     rospy.sleep(0.1)
     seconds = rospy.get_time()


   # If we exited the while loop without returning then we timed out
   return False
   ## END_SUB_TUTORIAL


 def attach_box(self,box_name,wait=True, timeout=4):
   # Copy class variables to local variables to make the web tutorials more clear.
   # In practice, you should use the class variables directly unless you have a good
   # reason not to.
   robot = self.robot
   scene = self.scene
   eef_link = self.eef_link


   ## BEGIN_SUB_TUTORIAL attach_object
   ##
   ## Attaching Objects to the Robot
   ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
   ## robot be able to touch them without the planning scene reporting the contact as a
   ## collision. By adding link names to the ``touch_links`` array, we are telling the
   ## planning scene to ignore collisions between those links and the box. For the Panda
   ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
   ## you should change this value to the name of your end effector group name.
   grasping_group = 'panda_hand'
   touch_links = robot.get_link_names(group=grasping_group)
   scene.attach_box(eef_link, box_name, touch_links=touch_links)
   ## END_SUB_TUTORIAL
   if(wait):
       # We wait for the planning scene to update.
       return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


 def detach_box(self, box_name,wait=True, timeout=4):
   # Copy class variables to local variables to make the web tutorials more clear.
   # In practice, you should use the class variables directly unless you have a good
   # reason not to.
   scene = self.scene
   eef_link = self.eef_link


   ## BEGIN_SUB_TUTORIAL detach_object
   ##
   ## Detaching Objects from the Robot
   ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   ## We can also detach and remove the object from the planning scene:
   scene.remove_attached_object(eef_link, name=box_name)
   ## END_SUB_TUTORIAL
   if(wait):
       # We wait for the planning scene to update.
       return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)



def main():
 try:
   hades = ProjectHades()


   hades.add_objects()

   rospy.sleep(1)
  
   hades.sample_air()
  #  print("Water")
   hades.sample_water()
  #  print("Surface")
   hades.sample_surface()

   rospy.sleep(5)

   hades.go_to_joint_state()
   hades.clear()
   print("============ Python tutorial demo complete!")
 except rospy.ROSInterruptException:
   return
 except KeyboardInterrupt:
   return


if __name__ == '__main__':
 main()
