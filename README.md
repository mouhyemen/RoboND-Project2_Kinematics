
# Project 2: Pick & Place Project
---


## 1. Project Summary
The goal of the project is to calculate the joint angles given the end-effector's pose for a 6 degree-of-freedom Kuka Arm 210 by applying principles of kinematics.

<p align="center">
  <img src="/images/kuka_arm.png">
</p>
	
### 1.1 Objectives:
* Calculate joint angles - θ1, θ2, θ3, θ4, θ5, θ6
* Grasp the object from the shelf
* Drop the object in the bin

### 1.2 Outcomes:
* Calculated all joint angles (with optimized IK)
* Grasped and placed 6 objects in the bin
* YouTube link:

<p align="center">
  <img src="/images/pick_place.png">
</p>

## 2. Forward Kinematics Analysis
We use forward kinematics to find the end-effector's location in a frame of reference provided we know the joint angles.

### 2.1 Denavit-Hartenberg Derivation
To find the joint angles for a robotic arm manipulator, we use Denavit-Hartenberg convention to derive a set of necessary parameters.

The set of parameters in a DH table are:

Name | Symbol | Definition
--- | :---: | --- | 
Joint angle | θ(i) 	| Angle between x(i-1) and x(i) about z(i)
Link offset | d(i) 	| Distance from x(i-1) to x(i) along z(i)
Link length | a(i-1) 	| Distance from z(i-1) to z(i) along x(i-1)
Twist angle | α(i-1)	| Angle between z(i-1) and z(i) about x(i-1)

<p align="center">
  <img src="/images/gripper_frame.png">
</p>

The set of derived DH parameters are shown below.

Links 	| θ(i) 	| d(i) 	| a(i-1) | α(i-1) 
--- 	| --- 		| --- 	| --- 	 | ---
0->1 	| q1* 		| d1	| 0 	|  0
1->2 	|q2* - 90	| 0 	| a1 	| -90
2->3 	| q3* 		| 0 	| a2 	|  0
3->4 	| q4*		| d4 	| a3 	| -90
4->5 	| q5* 		| 0 	| 0 	|  90
5->6 	| q7* 		| 0 	| 0 	| -90
6->EE 	| qG* 		| dG 	| 0 	| 0

Every joint in the Kuka arm are revolute joints and determine the angular rotation for the *i-th joint* - hence marked by qi*. Between `Joint 2` and `Joint 3`, there is an offset of 90 degrees which needs to be accounted for.

The values for the link offsets and link lengths are:
`d1 = 0.75      a1 = 0.35`
`d4 = 1.50      a2 = 1.25`
`dG = 0.303     a3 = -0.054`

### 2.2 Homogeneous Transformation Matrix
The homogeneous transform is a 4x4 matrix that contains information of the orientation and position of a frame in another reference frame. 

<p align="center">
  <img height="150" src="/images/transform.png">
</p>

The transform for each individual joint is shown below.

<p align="center">
  <img height="300" src="/images/joint_transforms.png">
</p>
	
The equation for calculating a homogeneous transform between two reference frames and its resultant output is shown below

<p align="center">
  <img src="/images/h_transform.png">
</p>

The following code is used for generating the homogeneous transform for a set of DH parameters.

```python
# Calculating homogenous transformation with a set of DH parameters
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
```

For calculating the homogeneous transform between the `gripper_link` and `base_ink`, an extrinsic body fixed rotations are performed. The sequence of operations involve performing a roll along x-axis, pitch along y-axis, and yaw along z-axis. For extrinsic rotations, we pre-multiply the sequence of operations like this:

`R0_EE = Rot(Z, yaw) * Rot(Y, pitch) * Rot(X, roll)`

To align the URDF model of the robot with the DH parameters, we apply a transformation to the gripper by rotating first along the z-axis by 180, and then rotating about the y-axis by -90 degrees.

`R_corr = Rot(Z, 180) * Rot(Y, -90)`

Hence, `R0G = R0_EE * R_corr`.

The following code is used for performing extrinsic rotations linking the `gripper_link` and `base_link` as well as performing a correctional transform to the rotation matrix.

```python
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
# Create rotation transform from end-effector as seen from ground frame
R0_EE   = r_zyx(roll, pitch, yaw)

# Align EE from URDF to DH convention by multiplying with correction-matrix
R_corr  = r_zyx(0, -pi/2, pi)
R0_EE   = R0_EE*R_corr
```

## 3. Inverse Kinematics Analysis
We use inverse kinematics to find the joint angles of a robotic arm in a frame of reference given the pose of the end-effector.

The Kuka Arm has a total of 6 joints where every joint is revolute. `Joints 4, 5, 6` form the spherical wrist of the arm with `Joint 5` being the wrist center. For performing inverse kinematics, we decouple the system into 2 parts. 

The first part consists of `Joints 1, 2, 3` which are responsible for determining the position of the wrist center also called *Inverse Position*. 

The second part consists of `Joints 4, 5, 6` which are responsible for determining the orientation of the wrist center also called *Inverse Orientation*.

### 3.1 Inverse Position
We first find the wrist center's position which is marked by the red vector in the diagram below. The green vector represents the end-effector's position from the ground frame relative to the ground frame. The black vector represents the end-effector's position in the wrist-center's frame relative to the ground frame. 

<p align="center">
  <img src="/images/wc_figure.png">
</p>
<p align="center">
  <img height="150" src="/images/wrist_center.png">
</p>

By doing a simple vector subtraction, we can find the wrist-center's location in the ground frame relative to the ground frame. We use the following equation to find the wrist center's position. The corresponding vector's mathematical representation is color coded.

#### 3.1.1 - Finding cartesian distances between joints
Before finding all the angles, first let us find all the cartesian distances between `Joint 2`, `Joint 3`, and `Wrist Center`. We are interested in `a, b, c`. Parameters used for deriving a particular side has been color-coded.
`a = a2` (just the link length between joints 2 and 3)

`b = sqrt(a3 * a3 + d4 * d4)` (color coded in green)

`c = sqrt(cx * cx + cz * cz)` (color coded in red)

where `cx = r_wc - a1` and `cz = wz - d1` 

and `r_wc = sqrt(wx * wx + wy * wy)` (color coded in blue)

<p align="center">
  <src="/images/distances.png">
</p>

#### 3.1.2 - Finding angles θ1, θ2, θ3
* θ1: For finding θ1, we project the vector going from the `base_link` to the end-effector (or `gripper_link`) onto the `XY-plane` and apply an inverse tangent operation. The following diagram shows how θ1 is derived where `θ1 = atan2(wy, wx)`.

<p align="center">
  <src="/images/theta1.png">
</p>

* θ2: For finding θ2, we use law of cosines for finding angle β (color coded in red) and inverse tangent function for finding angle δ (color coded in blue). The following diagram shows how θ2 is derived where `θ2 = 90 - β - δ`.

	`cosβ = (a*a + c*c - b*b) / (2 * a * c)`

	`sinβ = sqrt(1 - cosβ*cosβ)`

	`β = atan2(sinβ, cosβ)` (color coded in red)

	`δ = atan2(cz, cx)` (color coded in blue)
	
<p align="center">
  <src="/images/theta2.png">
</p>

* θ3: For finding θ3, we use law of cosines for finding angle γ (color coded in red) and inverse tangent function for finding angle	α (color coded in blue). The following diagram shows how θ3 is derived where `θ3 = - (γ - α)`.

	`cosγ = (a*a + c*c - b*b) / (2 * a * c)`

	`sinγ = sqrt(1 - cosγ*cosγ)`

	`γ = atan2(sinγ, cosγ)` (color coded in red)

	`α = atan2(d4, a3)` (color coded in blue)
	
<p align="center">
  <img src="/images/theta3.png">
</p>
	
### 3.2 Inverse Orientation - Finding angles θ4, θ5, θ6
For the inverse orientation problem, we will decompose the rotation transform from the `gripper_link` to the `base_link` as such:
`R0G = R03 * R36 * R6G` 
`R03.inverse * R0G = R03.inverse * R03 * R36 * I` (since the 6th frame and gripper frame have same orientation)
`R36 = R03' * R0G` (since rotation matrix' inverse is its transpose)

We know `R0G` from the extrinsic body fixed rotations calculated earlier.

`R0_EE = Rot(Z, yaw) * Rot(Y, pitch) * Rot(X, roll)`

`R_corr = Rot(Z, 180) * Rot(Y, -90)`

`R0G = R0_EE * R_corr`

Hence, `R36 = R03' * R0G` where the matrix R36 is shown below.

<p align="center">
  <img src="/images/inverse_orient.png">
</p>


* θ4: For finding θ4, we look at elements r13 and r33. 
	`θ4 = atan2(r33, -r13)`

* θ5: For finding θ5, we look at elements r23. 
	`θ5 = atan2(sqrt(1 - r23*r23), r23)`

* θ6: For finding θ6, we look at elements r21 and r22. 
	`θ6 = atan2(-r22, r21)`

