#!/usr/bin/env python

# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
# Author: Mouhyemen Khan

# import modules
import rospy, tf, time, math, progressbar, argparse
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

def initialize_transforms():
  start = time.time()

  # Creating global variables
  global T0_3, T0_1, T1_2, T2_3, T0_EE
  global q1, q2, q3, q4, q5, q6

  # Define DH param symbols
  d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
  a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
  alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

  # Joint angle symbols
  q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

  # Modified DH params
  theta1, theta2, theta3 = q1, q2 - pi/2, q3
  theta4, theta5, theta6 = q4, q5, q6
  theta7 = q7

  s = {alpha0:     0, a0:   0   , d1: 0.75,
       alpha1: -pi/2, a1: 0.35  , d2: 0,
       alpha2:     0, a2: 1.25  , d3: 0,
       alpha3: -pi/2, a3: -0.054, d4: 1.5,
       alpha4:  pi/2, a4:   0   , d5: 0,
       alpha5: -pi/2, a5:   0   , d6: 0,
       alpha6:     0, a6:   0   , d7: 0.303, q7: 0}


  # Define Modified DH Transformation matrix
  T0_1  = H_transform(alpha0, a0, d1, theta1)
  T1_2  = H_transform(alpha1, a1, d2, theta2)
  T2_3  = H_transform(alpha2, a2, d3, theta3)
  T3_4  = H_transform(alpha3, a3, d4, theta4)
  T4_5  = H_transform(alpha4, a4, d5, theta5)
  T5_6  = H_transform(alpha5, a5, d6, theta6)
  T6_EE = H_transform(alpha6, a6, d7, theta7)

  # Create individual transformation matrices
  T0_1  = T0_1.subs(s)
  T1_2  = T1_2.subs(s)
  T2_3  = T2_3.subs(s)
  T3_4  = T3_4.subs(s)
  T4_5  = T4_5.subs(s)
  T5_6  = T5_6.subs(s)
  T6_EE = T6_EE.subs(s)
  T0_3  = T0_1*T1_2*T2_3

  T0_EE = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_EE

  print "[INITIALIZATION TIME] Time %04.4f seconds." % (time.time() - start)

# Calculating homogenous transformation with DH parameters
def H_transform(alpha, a, d, theta):
  cT, sT = cos(theta), sin(theta)
  cA, sA = cos(alpha), sin(alpha)

  A = Matrix([
    [ cT  ,   -sT,    0,    a   ],
    [sT*cA, cT*cA,  -sA,  -d*sA ],
    [sT*sA, cT*sA,   cA,   d*cA ],
    [  0  ,     0,    0,     1  ] 
    ])
  return A

# extrinsic rotation matrix calculation
def r_zyx(roll, pitch, yaw):
    R_x = Matrix([
      [1,         0,          0],
      [0, cos(roll), -sin(roll)],
      [0, sin(roll),  cos(roll)]
      ])

    R_y = Matrix([
      [ cos(pitch), 0, sin(pitch)],
      [          0, 1,          0],
      [-sin(pitch), 0, cos(pitch)]
      ])

    R_z = Matrix([
      [cos(yaw),-sin(yaw), 0],
      [sin(yaw), cos(yaw), 0],
      [       0,        0, 1]
      ])

    return R_z*R_y*R_x

def hypotenuse(sideA, sideB):
  return sqrt(sideA*sideA + sideB*sideB)

def cosine_angle(a, b, c):
  # for 3 sides of a triangle, give angle b/w a & b
  cos_angle = (a*a + b*b - c*c)/ (2*a*b)
  return acos(cos_angle)

def handle_calculate_IK(req):
    global T0_3, T0_1, T1_2, T2_3, T0_EE
    global q1, q2, q3, q4, q5, q6, error

    rospy.loginfo("[INFO] Received %s EE-poses from plan." % len(req.poses))

    # set up the progress bar
    widgets = [
    "Calculating IK:", progressbar.Percentage(), " ", 
    progressbar.Bar(), " ", progressbar.Timer()]
    pbar = progressbar.ProgressBar( maxval=len(req.poses), widgets=widgets).start()
    
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []

        # Constant link-length(s) and link-offset(s)
        a1, a2, a3 = 0.35, 1.25, -0.054
        d1, d4, dG = 0.75, 1.50, 0.303

        # iterate over every pose and calc the IK for each pose
        for x in xrange(0, len(req.poses)):
          kinematics_start_time = time.time()
          pbar.update(x)

          # create an instance of trajectory point
          joint_trajectory_point = JointTrajectoryPoint()

          # Extract end-effector position and orientation from request
          # px,py,pz          = end-effector position
          # roll, pitch, yaw  = end-effector orientation
          px = req.poses[x].position.x
          py = req.poses[x].position.y
          pz = req.poses[x].position.z

          (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
              [ req.poses[x].orientation.x, 
                req.poses[x].orientation.y,
                req.poses[x].orientation.z, 
                req.poses[x].orientation.w  ])

          # Re-initialize the transform from 3rd frame to 0th frame
          T0_3    = T0_1*T1_2*T2_3

          # Create rotation transform from end-effector as seen from ground frame
          R0_EE   = r_zyx(roll, pitch, yaw)

          # Align EE from URDF to DH convention by multiplying with correction-matrix
          R_corr  = r_zyx(0, -pi/2, pi)
          R0_EE   = R0_EE*R_corr
          
          # Find wrist-center (WC) location
          wx = px - dG*R0_EE[0,2]
          wy = py - dG*R0_EE[1,2]
          wz = pz - dG*R0_EE[2,2]

          # Finding theta1
          theta1 = atan2(wy, wx)    # project WC's vector on x-y plane
          theta1 = theta1.evalf()

          # Finding cartesian distances b/w joints 2, 3, and WC
          r_wc = hypotenuse(wx, wy)    # vector proj on x-y plane of WC
          a    = a2
          b    = hypotenuse(a3, d4)
          c    = hypotenuse(r_wc-a1, wz-d1)

          # Finding theta2 = 90 - Beta - Delta
          beta   = cosine_angle(a, c, b)       # angle beta b/w a & c
          delta  = atan2(wz - d1, r_wc - a1)
          theta2 = pi/2 - beta - delta
          theta2 = theta2.evalf()
          
          # Finding theta3 = -(gamma - alpha)
          gamma  = cosine_angle(a, b, c)       # angle gamma b/w a & b
          alpha  = atan2(d4, a3)
          theta3 = -(gamma - alpha)
          theta3 = theta3.evalf()

          # Evaluate 3rd frame rel. to 0th frame using theta 1, 2, 3
          T0_3 = T0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

          # Rotational transform from 6th to 3rd frame
          R3_0 = T0_3[:3, :3].transpose()   # take inverse (or transpose)
          R3_6 = R3_0 * R0_EE
          
          # Finding rotational components from 6th to 3rd frame
          r33, r13 = R3_6[2,2], R3_6[0,2]
          r23      = R3_6[1,2]
          r22, r21 = R3_6[1,1], R3_6[1,0]

          # Finding values of theta4, 5, and 6
          theta4 = atan2(r33, -r13)
          theta4 = theta4.evalf()

          theta5 = atan2(sqrt(1-r23*r23), r23)
          theta5 = theta5.evalf()

          theta6 = atan2(-r22, r21)
          theta6 = theta6.evalf()

          # Print all IK theta values
          # print "[Calculating IK {}] Angles 1 to 6: {:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}"\
          # .format(x, theta1, theta2, theta3, theta4, theta5, theta6)

          # Calc FK EE error
          if error is 1:
              FK = T0_EE.evalf(subs={q1: theta1, q2: theta2, q3: theta3, \
              q4: theta4, q5: theta5, q6: theta6 })
              ex = FK[0,3]
              ey = FK[1,3]
              ez = FK[2,3]
              ee_x_e = abs(ex-px)
              ee_y_e = abs(ey-py)
              ee_z_e = abs(ez-pz)
              ee_offset = math.sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
              print ("\nEnd effector x-error: %04.5f" % ee_x_e)
              print ("End effector y-error: %04.5f" % ee_y_e)
              print ("End effector z-error: %04.5f" % ee_z_e)
              print ("Overall end effector error: %04.5f units \n" % ee_offset)

              print ("[TOTAL RUN TIME: IK & FK] %04.4f seconds" % (time.time()-kinematics_start_time))
        pbar.finish()

        # Populate response for the IK request        
        joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
        joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("Length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')

    # Initialize all DH parameters and relevant homogeneous transforms
    initialize_transforms()

    # Request service of type CalculateIK
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request."
    rospy.spin()

if __name__ == "__main__":
    global error

    ap = argparse.ArgumentParser()
    ap.add_argument("-e", "--error", type=int, default=-1)
    args    = vars(ap.parse_args())
    error   = args["error"]
    
    IK_server()