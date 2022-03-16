#!/usr/bin/env python
# use moveit_commander (the Python MoveIt user interfaces )
from math import pi
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
#from ur_move_test.msg import Coordinate_list
from ur_move_test.srv import obj_coordinate,obj_coordinateRequest,obj_coordinateResponse
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion



## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):

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



class MoveGroupTutorial(object):
  """MoveGroupTutorial"""
  def __init__(self):
    # super(MoveGroupTutorial, self).__init__()
    # moveit_commander.roscpp_initialize(sys.argv)
    # #rospy.init_node('move_group_tutorial_ur5', anonymous=True)
 
    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()
    # group_name = "arm" 
    # group = moveit_commander.MoveGroupCommander(group_name)
    # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
    #                                                moveit_msgs.msg.DisplayTrajectory,
    #                                                queue_size=20)
    # reference_frame = 'base_link'
    # group.set_pose_reference_frame(reference_frame)
    # ee_link = group.get_end_effector_link()
    
    # group.set_end_effector_link(ee_link)

  
    # group.set_max_acceleration_scaling_factor(0.1)
    # group.set_max_velocity_scaling_factor(0.1)
    
    # self.box_name = ''
    # self.robot = robot
    # self.scene = scene
    # self.group = group
    # self.reference_frame = reference_frame
    # self.end_effector_link = ee_link

    self.origin_degree = [0, 0, 0]
    self.target_angle = 0
    self.target_x = 0
    self.target_y = 0
    #rospy.Service('obj_location', obj_coordinate, pass_info_to_server)

  ###====================== home ======================###
  def pose(self): 
    group = self.group
    group.set_max_acceleration_scaling_factor(0.1)
    group.set_max_velocity_scaling_factor(0.1)
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = -pi*3/2
    joint_goal[1] = -pi/2
    joint_goal[2] =  pi/2
    joint_goal[3] = -pi/2
    joint_goal[4] = -pi/2
    joint_goal[5] =  pi/2    
    group.go(joint_goal, wait=True)
    group.stop()
    group.clear_pose_targets()
    current_joints = group.get_current_joint_values()
    origin_orientation =  group.get_current_pose().pose.orientation
    origindegree =  euler_from_quaternion([origin_orientation.x, origin_orientation.y, origin_orientation.z, origin_orientation.w]) 

    
    self.origin_degree[0] = origindegree[0]/3.14*180.0
    self.origin_degree[1] = origindegree[1]/3.14*180.0
    self.origin_degree[2] = origindegree[2]/3.14*180.0
    return all_close(joint_goal, current_joints, 0.01)

  ###====================== obj_location ======================###
  # def pose1(self):
  #   obj_x = 209
  #   obj_y = 173

  #   #obj_x, obj_y = self.pass_info_to_server()

  #   group = self.group
  #   pose_goal = geometry_msgs.msg.Pose()
  #   pose_goal.position.x = ((234.16 - obj_x)*4.16 -108.8)/1000 #-0.108
  #   pose_goal.position.y = (487 -70 + (obj_y*2.27))/1000 #0.630
  #   pose_goal.position.z = 0.146  #0.106  #0.0823

  #   calibrate_x = ((234.16 - obj_x)*4.16 -108.8)/1000
  #   calibrate_y = (487 -70 + (obj_y*2.27))/1000
  #   print("x: ",calibrate_x,"y: ",calibrate_y)
    
  #   quaternion = quaternion_from_euler(np.radians(-180),np.radians(0), np.radians(-90))   #roll_angle, pitch_angle, yaw_angle  
    


  #   pose_goal.orientation.x = quaternion[0]
  #   pose_goal.orientation.y = quaternion[1]
  #   pose_goal.orientation.z = quaternion[2]
  #   pose_goal.orientation.w = quaternion[3]

  #   group.set_pose_target(pose_goal, self.end_effector_link)
  #   plan = group.go(wait=True)
  #   group.stop()
  #   group.clear_pose_targets()

  def test_communication(self):
      obj_x, obj_y = self.pass_info_to_server()

      calibrate_x = ((234.16 - obj_x)*4.16 -108.8)/1000
      calibrate_y = (487 -70 + (obj_y*2.27))/1000
      print("x: ",obj_x,"y: ",obj_y)


  def pose1(self):
    group = self.group
    group.set_max_acceleration_scaling_factor(0.1)
    group.set_max_velocity_scaling_factor(0.1)
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = -276.88*(3.14/180)
    joint_goal[1] = -28.67*(3.14/180)
    joint_goal[2] = 56.72*(3.14/180)
    joint_goal[3] = -118.02*(3.14/180)
    joint_goal[4] = -89.66*(3.14/180)
    joint_goal[5] = 90*(3.14/180)
    group.go(joint_goal, wait=True)
    group.stop()
    group.clear_pose_targets()
    current_joints = group.get_current_joint_values()
    origin_orientation =  group.get_current_pose().pose.orientation
    origindegree =  euler_from_quaternion([origin_orientation.x, origin_orientation.y, origin_orientation.z, origin_orientation.w]) 

    
    self.origin_degree[0] = origindegree[0]/3.14*180.0
    self.origin_degree[1] = origindegree[1]/3.14*180.0
    self.origin_degree[2] = origindegree[2]/3.14*180.0
    return all_close(joint_goal, current_joints, 0.01)


  ###====================== upper cabin ======================###
  def pose2(self):
    group = self.group
    group.set_max_acceleration_scaling_factor(0.1)
    group.set_max_velocity_scaling_factor(0.1)
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = (103.61 -360)*(3.14/180)
    joint_goal[1] = -64.36*(3.14/180)
    joint_goal[2] = 78.97*(3.14/180)
    joint_goal[3] = -16.05*(3.14/180)
    joint_goal[4] = 13.62*(3.14/180)
    joint_goal[5] = -88.53*(3.14/180)
    group.go(joint_goal, wait=True)
    group.stop()
    group.clear_pose_targets()
    current_joints = group.get_current_joint_values()
    origin_orientation =  group.get_current_pose().pose.orientation
    origindegree =  euler_from_quaternion([origin_orientation.x, origin_orientation.y, origin_orientation.z, origin_orientation.w]) 

    
    self.origin_degree[0] = origindegree[0]/3.14*180.0
    self.origin_degree[1] = origindegree[1]/3.14*180.0
    self.origin_degree[2] = origindegree[2]/3.14*180.0
    return all_close(joint_goal, current_joints, 0.01)

 ###====================== lower cabin ======================###    
  def pose3(self):
    group = self.group
    group.set_max_acceleration_scaling_factor(0.1)
    group.set_max_velocity_scaling_factor(0.1)
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = (103.65 -360)*(3.14/180)
    joint_goal[1] = -31.66*(3.14/180)
    joint_goal[2] = 91.39*(3.14/180)
    joint_goal[3] = -60.7*(3.14/180)
    joint_goal[4] = 13.88*(3.14/180)
    joint_goal[5] = -88.53*(3.14/180)  
    group.go(joint_goal, wait=True)
    group.stop()
    group.clear_pose_targets()
    current_joints = group.get_current_joint_values()
    origin_orientation =  group.get_current_pose().pose.orientation
    origindegree =  euler_from_quaternion([origin_orientation.x, origin_orientation.y, origin_orientation.z, origin_orientation.w]) 

    
    self.origin_degree[0] = origindegree[0]/3.14*180.0
    self.origin_degree[1] = origindegree[1]/3.14*180.0
    self.origin_degree[2] = origindegree[2]/3.14*180.0
    return all_close(joint_goal, current_joints, 0.01)

 ###====================== place on table ======================###    
  def pose4(self):
    group = self.group
    group.set_max_acceleration_scaling_factor(0.1)
    group.set_max_velocity_scaling_factor(0.1)
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = -295.84*(3.14/180)
    joint_goal[1] = -33.35*(3.14/180)
    joint_goal[2] = 78.18*(3.14/180)
    joint_goal[3] = -134.82*(3.14/180)
    joint_goal[4] = -89.56*(3.14/180)
    joint_goal[5] =  64.4*(3.14/180)  
    group.go(joint_goal, wait=True)
    group.stop()
    group.clear_pose_targets()
    current_joints = group.get_current_joint_values()
    origin_orientation =  group.get_current_pose().pose.orientation
    origindegree =  euler_from_quaternion([origin_orientation.x, origin_orientation.y, origin_orientation.z, origin_orientation.w]) 

    
    self.origin_degree[0] = origindegree[0]/3.14*180.0
    self.origin_degree[1] = origindegree[1]/3.14*180.0
    self.origin_degree[2] = origindegree[2]/3.14*180.0
    return all_close(joint_goal, current_joints, 0.01)

  ###====================== home ======================###
  def pose5(self):
    group = self.group
    group.set_max_acceleration_scaling_factor(0.1)
    group.set_max_velocity_scaling_factor(0.1)
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = -pi*3/2
    joint_goal[1] = -pi/2
    joint_goal[2] =  pi/2
    joint_goal[3] = -pi/2
    joint_goal[4] = -pi/2
    joint_goal[5] =  pi/2    
    group.go(joint_goal, wait=True)
    group.stop()
    group.clear_pose_targets()
    current_joints = group.get_current_joint_values()
    origin_orientation =  group.get_current_pose().pose.orientation
    origindegree =  euler_from_quaternion([origin_orientation.x, origin_orientation.y, origin_orientation.z, origin_orientation.w]) 

    
    self.origin_degree[0] = origindegree[0]/3.14*180.0
    self.origin_degree[1] = origindegree[1]/3.14*180.0
    self.origin_degree[2] = origindegree[2]/3.14*180.0
    return all_close(joint_goal, current_joints, 0.01)

  def pass_info_to_server(self):
      rospy.wait_for_service('obj_location')
      try:
          #create a server object
          val = rospy.ServiceProxy('obj_location', obj_coordinate)
          #val(arg) -> send a req to server
          resp = val('req communication')
          # resp = obj_coordinateResponse(val)
          return resp.obj_x, resp.obj_y

      except rospy.ServiceException as e:
          print ('error when send commad !')
          return 0, 0


def main():
  try:
    print("============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ...")
    tutorial = MoveGroupTutorial()
    #rospy.Subscriber("object_coordinate", Coordinate_list, callback)

    print("Press any key to start strategy!")
    raw_input()
    print('================================')
    print("pose1")
    tutorial.test_communication()


    
    print("============ Python tutorial demo complete!")
    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  rospy.init_node('Strategy')
  main()
