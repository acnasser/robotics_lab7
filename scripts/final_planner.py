#!/usr/bin/env python3
import rospy
import math
import tf2_ros
from tf.transformations import *
from geometry_msgs.msg import Quaternion
import tf2_geometry_msgs
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from robot_vision_lectures.msg import SphereParams
from geometry_msgs.msg import PointStamped
from std_msgs.msg import UInt8

points_in_camera = []
first_point = []
counter = 0


def xyzCallback(data):
    global points_in_camera
    x = data.xc
    y = data.yc
    z = data.zc
    points_in_camera = [x, y, z]

def tpCallback(data):
    global first_point
    global counter
    while counter == 0:
        x = data.linear.x
        y = data.linear.y
        z = data.linear.z
        xa = data.angular.x
        ya = data.angular.y
        za = data.angular.z
        first_point = [x, y, z, xa, ya, za]
        counter += 1

def main():

    rospy.init_node('simple_planner', anonymous=True)
    plan_pub = rospy.Publisher('/plan', Plan, queue_size=10)

   
    rospy.Subscriber('sphere_params', SphereParams, xyzCallback)
    rospy.Subscriber('/ur5e/toolpose', Twist, tpCallback)
   
    loop_rate = rospy.Rate(10)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    while not rospy.is_shutdown():
        if len(points_in_camera) != 0:
            
            plan = Plan()
            
            plan_point1 = Twist()
            point_mode1 = UInt8()

            plan_point1.linear.x = first_point[0]
            plan_point1.linear.y = first_point[1] 
            plan_point1.linear.z = first_point[2]
            plan_point1.angular.x = first_point[3]
            plan_point1.angular.y = first_point[4]
            plan_point1.angular.z = first_point[5]
            point_mode1.data = 0
            
            plan.points.append(plan_point1)
            plan.modes.append(point_mode1)
            
	    #--------------------------------------------------------------------
           
            pt_in_cam = PointStamped()
            pt_in_cam.header.frame_id = 'camera_color_optical_frame'
            pt_in_cam.header.stamp = rospy.get_rostime()
            pt_in_cam.point.x = points_in_camera[0]
            pt_in_cam.point.y = points_in_camera[1]
            pt_in_cam.point.z = points_in_camera[2]
            points_in_base = tfBuffer.transform(pt_in_cam, 'base', rospy.Duration(1.0))
   			
   	    #--------------------------------------------------------------------
           
            plan_point_to_ball = Twist()
            point_mode_to_ball = UInt8()
            
            plan_point_to_ball.linear.x = points_in_base.point.x 
            plan_point_to_ball.linear.y = points_in_base.point.y
            plan_point_to_ball.linear.z = points_in_base.point.z + 0.01
            plan_point_to_ball.angular.x = first_point[3]
            plan_point_to_ball.angular.y = first_point[4]
            plan_point_to_ball.angular.z = first_point[5]
            point_mode_to_ball.data = 0
            
            plan.points.append(plan_point_to_ball)
            plan.modes.append(point_mode_to_ball)
            
            #--------------------------------------------------------------------
            
            plan_point_close_grip = Twist()
            point_mode_close_grip = UInt8()
            
            plan_point_close_grip.linear.x = points_in_base.point.x 
            plan_point_close_grip.linear.y = points_in_base.point.y
            plan_point_close_grip.linear.z = points_in_base.point.z + 0.01
            plan_point_close_grip.angular.x = first_point[3]
            plan_point_close_grip.angular.y = first_point[4]
            plan_point_close_grip.angular.z = first_point[5]
            point_mode_close_grip.data = 2
            
            plan.points.append(plan_point_close_grip)
            plan.modes.append(point_mode_close_grip)
            
            #---------------------------------------------------------------------
            
            plan_point_raise_gripper = Twist()
            point_mode_raise_gripper = UInt8()
            
            plan_point_raise_gripper.linear.x = points_in_base.point.x 
            plan_point_raise_gripper.linear.y = points_in_base.point.y
            plan_point_raise_gripper.linear.z = points_in_base.point.z + 0.05
            plan_point_raise_gripper.angular.x = first_point[3]
            plan_point_raise_gripper.angular.y = first_point[4]
            plan_point_raise_gripper.angular.z = first_point[5]
            point_mode_raise_gripper.data = 0
            
            plan.points.append(plan_point_raise_gripper)
            plan.modes.append(point_mode_raise_gripper)
            
            #--------------------------------------------------------------------
            
            plan_point_move_fwrd = Twist()
            point_mode_move_fwrd = UInt8()
            
            plan_point_move_fwrd.linear.x = points_in_base.point.x 
            plan_point_move_fwrd.linear.y = points_in_base.point.y - 0.05
            plan_point_move_fwrd.linear.z = points_in_base.point.z + 0.05
            plan_point_move_fwrd.angular.x = first_point[3]
            plan_point_move_fwrd.angular.y = first_point[4]
            plan_point_move_fwrd.angular.z = first_point[5]
            point_mode_move_fwrd.data = 0
            
            plan.points.append(plan_point_move_fwrd)
            plan.modes.append(point_mode_move_fwrd)
            
            #--------------------------------------------------------------------
            
            plan_point_move_down = Twist()
            point_mode_move_down = UInt8()
            
            plan_point_move_down.linear.x = points_in_base.point.x 
            plan_point_move_down.linear.y = points_in_base.point.y - 0.05
            plan_point_move_down.linear.z = points_in_base.point.z + 0.01
            plan_point_move_down.angular.x = first_point[3]
            plan_point_move_down.angular.y = first_point[4]
            plan_point_move_down.angular.z = first_point[5]
            point_mode_move_down.data = 0
            
            plan.points.append(plan_point_move_down)
            plan.modes.append(point_mode_move_down)
            
            #--------------------------------------------------------------------
            
            plan_point_drop_ball = Twist()
            point_mode_drop_ball = UInt8()
            
            plan_point_drop_ball.linear.x = points_in_base.point.x 
            plan_point_drop_ball.linear.y = points_in_base.point.y - 0.05
            plan_point_drop_ball.linear.z = points_in_base.point.z + 0.01
            plan_point_drop_ball.angular.x = first_point[3]
            plan_point_drop_ball.angular.y = first_point[4]
            plan_point_drop_ball.angular.z = first_point[5]
            point_mode_drop_ball.data = 1
            
            plan.points.append(plan_point_drop_ball)
            plan.modes.append(point_mode_drop_ball)
            
            #--------------------------------------------------------------------
            
            plan_point_move_up = Twist()
            point_mode_move_up = UInt8()
            
            plan_point_move_up.linear.x = points_in_base.point.x 
            plan_point_move_up.linear.y = points_in_base.point.y - 0.05
            plan_point_move_up.linear.z = points_in_base.point.z + 0.05
            plan_point_move_up.angular.x = first_point[3]
            plan_point_move_up.angular.y = first_point[4]
            plan_point_move_up.angular.z = first_point[5]
            point_mode_move_up.data = 0
            
            plan.points.append(plan_point_move_up)
            plan.modes.append(point_mode_move_up)
            

          
            #--------------------------------------------------------------------
			
            
            move = input("Would you like to move the robot? y/n")
            
            if move == "y":
            	plan_pub.publish(plan)
            else:
            	print(plan)
            
            
            #---------------------------------------------------------------------
		
           
            loop_rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
