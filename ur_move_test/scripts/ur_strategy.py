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
from ur_move_test.srv import gripper_cmd 
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion

pos_x = 0
pos_y = 0
pos_z = 0


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
    super(MoveGroupTutorial, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    #rospy.init_node('move_group_tutorial_ur5', anonymous=True)
 
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "arm" 
    group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    reference_frame = '/base_link'
    group.set_pose_reference_frame(reference_frame)
    ee_link = group.get_end_effector_link()
    
    group.set_end_effector_link(ee_link)

    group.set_planner_id("CSAGO_RRT")
    # group.set_goal_position_tolerance(0.1)
    # group.set_goal_orientation_tolerance(0.1)
    group.set_max_acceleration_scaling_factor(0.1)
    group.set_max_velocity_scaling_factor(0.1)


    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.reference_frame = reference_frame
    self.end_effector_link = ee_link

    self.origin_degree = [0, 0, 0]
    self.target_angle = 0
    self.target_x = 0
    self.target_y = 0

  ###====================== home ======================###
  def pose(self): 
    group = self.group
    group.set_max_acceleration_scaling_factor(0.1)
    group.set_max_velocity_scaling_factor(0.1)
    joint_goal = group.get_current_joint_values()
    joint_goal[0] =  pi/2
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

    origin =  group.get_current_pose(self.end_effector_link).pose
    #print(origin)
    origindegree =  euler_from_quaternion([origin_orientation.x, origin_orientation.y, origin_orientation.z, origin_orientation.w]) 
    #print(origindegree)
    
    self.origin_degree[0] = origindegree[0]/3.14*180.0
    self.origin_degree[1] = origindegree[1]/3.14*180.0
    self.origin_degree[2] = origindegree[2]/3.14*180.0
    return all_close(joint_goal, current_joints, 0.01)

  ###====================== obj_location(up) ======================###
  def pose1(self):
    global pos_x, pos_y, pos_z
    obj_x = 209
    obj_y = 173

    #obj_x, obj_y = self.pass_info_to_server()
    
    print("x: ",obj_x,"y: ",obj_y)

    group = self.group
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = ((234.16 - obj_x)*4.16 -108.8)/1000 #-0.108
    pose_goal.position.y = (487 -70 + (obj_y*2.27))/1000 #0.630
    pose_goal.position.z = 0.146  #0.106  #0.0823

    pose_goal.position.x = pose_goal.position.x + (0.015)
    pose_goal.position.y = pose_goal.position.y + 0.148
    pose_goal.position.z = pose_goal.position.z + 1.03

    # give next move a ref
    pos_x = pose_goal.position.x
    pos_y = pose_goal.position.y
    pos_z = pose_goal.position.z

   
    quaternion = quaternion_from_euler(np.radians(-180),np.radians(0), np.radians(-90))   #roll_angle, pitch_angle, yaw_angle  


    pose_goal.orientation.x = quaternion[0]
    #print(pose_goal.orientation.x)
    pose_goal.orientation.y = quaternion[1]
    #print(pose_goal.orientation.y)
    pose_goal.orientation.z = quaternion[2]
    #print(pose_goal.orientation.z)
    pose_goal.orientation.w = quaternion[3]
    #print(pose_goal.orientation.w)


    group.set_pose_target(pose_goal, self.end_effector_link)

    plan = group.go(wait=True)
    origin =  group.get_current_pose()
    print(origin)
    group.stop()
    group.clear_pose_targets()


  ###====================== obj_location(down) ======================###
  def pose1_2(self):
    global pos_x, pos_y, pos_z

    group = self.group
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = pos_x
    pose_goal.position.y = pos_y
    pose_goal.position.z = pos_z - 0.2

    
    quaternion = quaternion_from_euler(np.radians(-180),np.radians(0), np.radians(-90))    


    pose_goal.orientation.x = quaternion[0]
    #print(pose_goal.orientation.x)
    pose_goal.orientation.y = quaternion[1]
    #print(pose_goal.orientation.y)
    pose_goal.orientation.z = quaternion[2]
    #print(pose_goal.orientation.z)
    pose_goal.orientation.w = quaternion[3]
    #print(pose_goal.orientation.w)


    group.set_pose_target(pose_goal, self.end_effector_link)

    plan = group.go(wait=True)
    origin =  group.get_current_pose()
    #print(origin)
    group.stop()
    group.clear_pose_targets()


  ###====================== obj_location(up) ======================###
  def pose1_3(self):
    global pos_x, pos_y, pos_z

    group = self.group
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = pos_x
    pose_goal.position.y = pos_y
    pose_goal.position.z = pos_z

    
    quaternion = quaternion_from_euler(np.radians(-180),np.radians(0), np.radians(-90))    


    pose_goal.orientation.x = quaternion[0]
    #print(pose_goal.orientation.x)
    pose_goal.orientation.y = quaternion[1]
    #print(pose_goal.orientation.y)
    pose_goal.orientation.z = quaternion[2]
    #print(pose_goal.orientation.z)
    pose_goal.orientation.w = quaternion[3]
    #print(pose_goal.orientation.w)


    group.set_pose_target(pose_goal, self.end_effector_link)

    plan = group.go(wait=True)
    origin =  group.get_current_pose()
    #print(origin)
    group.stop()
    group.clear_pose_targets()


  ###====================== upper cabin(out) ======================###
  def pose2_1(self):
    group = self.group
    group.set_max_acceleration_scaling_factor(0.1)
    group.set_max_velocity_scaling_factor(0.1)
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 91.48*(3.14/180)
    joint_goal[1] = -74.9*(3.14/180)
    joint_goal[2] = 78.99*(3.14/180)
    joint_goal[3] = -17.38*(3.14/180)
    joint_goal[4] = 1.47*(3.14/180)
    joint_goal[5] = -76.71*(3.14/180)
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


  ###====================== upper cabin(mid) ======================###
  def pose2_2(self):
    group = self.group
    group.set_max_acceleration_scaling_factor(0.1)
    group.set_max_velocity_scaling_factor(0.1)
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 97.91*(3.14/180)
    joint_goal[1] = -67.72*(3.14/180)
    joint_goal[2] = 81.09*(3.14/180)
    joint_goal[3] = -15.83*(3.14/180)
    joint_goal[4] = 7.9*(3.14/180)
    joint_goal[5] = -87.53*(3.14/180)
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


  ###====================== upper cabin(in) ======================###
  def pose2_3(self):
    group = self.group
    group.set_max_acceleration_scaling_factor(0.1)
    group.set_max_velocity_scaling_factor(0.1)
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 101.55*(3.14/180)
    joint_goal[1] = -64.94*(3.14/180)
    joint_goal[2] = 79.3*(3.14/180)
    joint_goal[3] = -16.04*(3.14/180)
    joint_goal[4] = 11.56*(3.14/180)
    joint_goal[5] = -88.3*(3.14/180)
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


 ###====================== lower cabin(out) ======================###    
  def pose3_1(self):
    group = self.group
    group.set_max_acceleration_scaling_factor(0.1)
    group.set_max_velocity_scaling_factor(0.1)
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 94.82*(3.14/180)
    joint_goal[1] = -33.73*(3.14/180)
    joint_goal[2] = 97.47*(3.14/180)
    joint_goal[3] = -66.2*(3.14/180)
    joint_goal[4] = 5.11*(3.14/180)
    joint_goal[5] = -87.02*(3.14/180)  
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


 ###====================== lower cabin(mid) ======================###    
  def pose3_2(self):
    group = self.group
    group.set_max_acceleration_scaling_factor(0.1)
    group.set_max_velocity_scaling_factor(0.1)
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 103.65*(3.14/180)
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


 ###====================== lower cabin(in) ======================###    
  def pose3_3(self):
    group = self.group
    group.set_max_acceleration_scaling_factor(0.1)
    group.set_max_velocity_scaling_factor(0.1)
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 102.27*(3.14/180)
    joint_goal[1] = -32.02*(3.14/180)
    joint_goal[2] = 92.41*(3.14/180)
    joint_goal[3] = -61.46*(3.14/180)
    joint_goal[4] = 12.55*(3.14/180)
    joint_goal[5] = -88.4*(3.14/180)  
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
    # group = self.group
    # group.set_max_acceleration_scaling_factor(0.1)
    # group.set_max_velocity_scaling_factor(0.1)
    # joint_goal = group.get_current_joint_values()
    # joint_goal[0] = -295.84*(3.14/180)
    # joint_goal[1] = -33.35*(3.14/180)
    # joint_goal[2] = 78.18*(3.14/180)
    # joint_goal[3] = -134.82*(3.14/180)
    # joint_goal[4] = -89.56*(3.14/180)
    # joint_goal[5] =  64.4*(3.14/180)  
    # group.go(joint_goal, wait=True)
    # group.stop()
    # group.clear_pose_targets()
    # current_joints = group.get_current_joint_values()
    # origin_orientation =  group.get_current_pose().pose.orientation
    # origindegree =  euler_from_quaternion([origin_orientation.x, origin_orientation.y, origin_orientation.z, origin_orientation.w]) 

    
    # self.origin_degree[0] = origindegree[0]/3.14*180.0
    # self.origin_degree[1] = origindegree[1]/3.14*180.0
    # self.origin_degree[2] = origindegree[2]/3.14*180.0
    # return all_close(joint_goal, current_joints, 0.01)

    global pos_x, pos_y, pos_z

    group = self.group
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = pos_x
    pose_goal.position.y = pos_y
    pose_goal.position.z = pos_z

    
    quaternion = quaternion_from_euler(np.radians(-180),np.radians(0), np.radians(-90))    


    pose_goal.orientation.x = quaternion[0]
    print(pose_goal.orientation.x)
    pose_goal.orientation.y = quaternion[1]
    print(pose_goal.orientation.y)
    pose_goal.orientation.z = quaternion[2]
    print(pose_goal.orientation.z)
    pose_goal.orientation.w = quaternion[3]
    print(pose_goal.orientation.w)


    group.set_pose_target(pose_goal, self.end_effector_link)

    plan = group.go(wait=True)
    origin =  group.get_current_pose()
    print(origin)
    group.stop()
    group.clear_pose_targets()

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


  ###====================== gripper_control ======================###
  def gripper_control(self,cmd):
    resp = self.pass_gripper_cmd_to_server(cmd)
    print(resp)



  def pass_info_to_server(self):
      rospy.wait_for_service('obj_location')
      try:
          #create a server object
          val = rospy.ServiceProxy('obj_location', obj_coordinate)
          #val(arg) -> send a req to server
          resp = val("req coordinate")
          return resp.obj_x, resp.obj_y

      except rospy.ServiceException as e:
          print ('error when send image commad !')
          return 0, 0


  def pass_gripper_cmd_to_server(self,cmd):
      rospy.wait_for_service('gripper_command')
      try:
          #create a server object
          val = rospy.ServiceProxy('gripper_command', gripper_cmd)
          #val(arg) -> send a req to server
          resp = val(cmd)  # 0 = close, 1 = open
          return resp.resp

      except rospy.ServiceException as e:
          print ('error when send gripper commad !')
          return 'fail'


def main():
  try:
    print("============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ...")
    tutorial = MoveGroupTutorial()
    #rospy.Subscriber("object_coordinate", Coordinate_list, callback)
    
    print("Press any key to start strategy!")
 

    #raw_input()
    print('================================')
    print("pose")
    tutorial.pose()

    #raw_input()
    print('================================')
    print("gripper_reset")
    tutorial.gripper_control('r')

    #raw_input()
    print('================================')
    print("gripper_active")
    tutorial.gripper_control('a')

    #raw_input()
    print('================================')
    print("pose1")
    tutorial.pose1()

    #raw_input()
    print('================================')
    print("pose1_down")
    tutorial.pose1_2()

    
    #raw_input()
    print('================================')
    print("gripper_close")
    tutorial.gripper_control('c')

    #raw_input()
    print('================================')
    print("pose1_up")
    tutorial.pose1_3()

    #raw_input()
    print('================================')
    print("pose2_out")
    tutorial.pose2_1()

    #raw_input()
    print('================================')
    print("pose2_in")
    tutorial.pose2_3()

    #raw_input()
    print('================================')
    print("gripper_open")
    tutorial.gripper_control('o')

    #raw_input()
    print('================================')
    print("pose2_out")
    tutorial.pose2_1()

    #raw_input()
    print('================================')
    print("pose3_out")
    tutorial.pose3_1()

    #raw_input()
    print('================================')
    print("pose3_in")
    tutorial.pose3_3()

    #raw_input()
    print('================================')
    print("gripper_close")
    tutorial.gripper_control('c')

    #raw_input()
    print('================================')
    print("pose3_out")
    tutorial.pose3_1()

    #raw_input()
    print('================================')
    print("pose4")
    tutorial.pose4()

    #raw_input()
    print('================================')
    print("pose1_down")
    tutorial.pose1_2()

    #raw_input()
    print('================================')
    print("gripper_close")
    tutorial.gripper_control('o')

    #raw_input()
    print('================================')
    print("pose4")
    tutorial.pose4()

    #raw_input()
    print('================================')
    print("pose")
    tutorial.pose()
    

    
    print("============ Python tutorial demo complete!")
    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  rospy.init_node('Strategy')
  main()
