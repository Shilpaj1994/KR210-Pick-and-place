# Project: Kinematics Pick & Place

## Writeup

This is a writeup for the Udacity's Robotics Nanodegree's Pick and Place Project. This directory is a ROS package and instructions for setup are available in `README` file

---

**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # "Image References"

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
Rubric consists of 3 points:

- Writeup

- Kinematics Analysis

- Project Implementation

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

---

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

![](https://github.com/Shilpaj1994/KR210-Pick-and-place/blob/master/misc_images/misc1.png?raw=true)

- Above shown is a Kuka KR210 robot

- The `kr210.urdf.xacro` file is located in directory `kuka_arm/urdf/`

- Universal Robotic Description Format (URDF) file contains the detail description of the links and joints

- Following is an example of `urdf` file written in `xml` which includes the description of links

  ![](https://github.com/Shilpaj1994/KR210-Pick-and-place/blob/master/misc_images/urdf.JPG?raw=true)

- A robot consists of various joints and links. All the links are connected in a particular pattern to cover the workspace around the robot.

- The first link is known as `base_link` while the last link, in most cases, known as `end-effector`

- **Forward kinematics** refers to the use of the kinematic equations of a robot to compute the position of the end-effector from specified values for the joint parameters

- Consider a 2-link robot, the position of link-2 with respect to link-1 can be completely described by using 6-parameters (3 Rotations and 3 Translations)

- If number of links are increased, as in the case of Kuka-KR210 (7-links), it becomes very complex to describe the position of end-effectors with respect to base_link

- To simplify this problem, D-H parameters are used.

- With D-H parameter, we can completely describe the relation of 2-links using only 4-parameters.

- Following are the D-H Parameters:

  - **Link Length:** a(i-1) = Zi-1 - Zi along the X(i-1) axis

  - **Link Offset:** d(i) = X(i-1) - X(i) along Z(i) axis

  - **Link Twist: ** alpha(i-1) = angle from Z(i-1) to Z(i) measured about Xi-1 using right hand rule

  - **Joint Angle:** angle from X(i-1) to X(i) measured about Zi using right hand rule. all joint angles will be zero at initial Robot state in KR210 except joint 2 which has a -90 degree constant offset between X(1) and X(2)

    where i=link number

- Following video shows the conventions for D-H parameters and how to calculate D-H parameters for a robot.

<iframe width="900" height="506" src="https://www.youtube.com/embed/rA9tm0gTln8" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

- Following is diagram representing link and joints of above shown robot

![](https://github.com/Shilpaj1994/KR210-Pick-and-place/blob/master/misc_images/DH.png?raw=true)

- Following is a procedure to find out DH parameters

  ```latex
  1. Label all joints from {1, 2, … , *n*}.
  
  2. Label all links from {0, 1, …, n} starting with the fixed base link as 0.
  
  3. Draw lines through all joints, defining the joint axes.
  
  4. Assign the Z-axis of each frame to point along its joint axis.
  
  5. Identify the common normal between each frame \hat{Z}_{i-1}*Z*^*i*−1 and frame \hat{Z}_{i}*Z*^*i* .
  
  6. The endpoints of "intermediate links" (i.e., not the base link or the end effector) are associated with two joint axes, {i} and {i+1}. For *i* from 1 to *n-1*, assign the \hat{X}_{i}*X*^*i* to be …
  
     ​	For skew axes, along the normal between \hat{Z}_{i}*Z*^*i* and \hat{Z}_{i+1}*Z*^*i*+1 and pointing from {*i*} to {*i+1*}.
  
     ​	For intersecting axes, normal to the plane containing \hat{Z}_{i}*Z*^*i* and \hat{Z}_{i+1}*Z*^*i*+1.
  
     ​	For parallel or coincident axes, the assignment is arbitrary; look for ways to make other DH parameters equal to zero.
  
  7. For the base link, always choose frame {0} to be coincident with frame {1} when the first joint variable ({\theta}_{1}*θ*1 or {d}_{1}*d*1) is equal to zero. This will guarantee that {\alpha}_{0}*α*0 = {a}_{0}*a*0 = 0, and, if joint 1 is a revolute, {d}_{1}*d*1 = 0. If joint 1 is prismatic, then {\theta}_{1}*θ*1= 0.
  
  8. For the end effector frame, if joint *n* is revolute, choose {X}_{n}*X**n* to be in the direction of {X}_{n-1}*X**n*−1 when {\theta}_{n}*θ**n* = 0 and the origin of frame {*n*} such that {d}_{n}*d**n* = 0. 
  ```

- Following is the D-H parameter table for the above robot. The link lengths can be measured in `gazebo` as well as can be found out in `urdf` files.

  | Links |  i   | alpha(i-1) | a(i-1) | d(i)  | theta(i) |      |
  | :---: | :--: | :--------: | :----: | :---: | :------: | ---- |
  | 0->1  |  1   |     0      |   0    | 0.75  |    q1    |      |
  | 1->2  |  2   |    -90     |  0.35  |   0   |  -90+q2  |      |
  | 2->3  |  3   |     0      |        | 1.25  |    q3    |      |
  | 3->4  |  4   |    -90     | -0.05  |  1.5  |    q4    |      |
  | 4->5  |  5   |     90     |   0    |   0   |    q5    |      |
  | 5->6  |  6   |    -90     |   0    |   0   |    q6    |      |
  | 6->7  |  7   |     0      |   0    | 0.303 |    q7    |      |

- Code

```python
import numpy as np
from numpy import array
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2, pprint
from sympy.matrices import Matrix

DH_Table = {alpha0:      0, a0:      0, d1:  0.75, q1:        q1,
            alpha1: -pi/2., a1:   0.35, d2:     0, q2: -pi/2.+q2,
            alpha2:      0, a2:   1.25, d3:     0, q3:        q3,
            alpha3: -pi/2., a3: -0.054, d4:   1.5, q4:        q4,
            alpha4:  pi/2., a4:      0, d5:     0, q5:        q5,
            alpha5: -pi/2., a5:      0, d6:     0, q6:        q6,
            alpha6:      0, a6:      0, d7: 0.303, q7:         0}
```

---

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

- Individual Transformation Matrices of two consecutive links is given by:

  ![](misc_images/tt.png)

- Code for the following

  ```python
  # Function to return homogeneous transform matrix
  def TF_Mat(alpha, a, d, q):
      TF = Matrix([[            cos(q),           -sin(q),           0,             a],
                   [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                   [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                   [                 0,                 0,           0,             1]])
      return TF
  ```

- Transformation Matrices for all the links:

```python
T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)
```

- Generalized Homogenous Transform between base_link and end-effector is calculated by performing intrinsic transformations by pre-multiplying all individual joints transformation matrices

```python
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
```

---

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

- **Inverse Kinematics** is a process of finding the joint parameters using the position of end-effectors
- The last three joints in KUKA KR210 robot (Joint_4, Joint_5, and Joint_6) are revolute and their joint axes intersect at a single point (Joint_5), hence we have a case of spherical wrist with joint_5 being the common intersection point; the wrist center (**WC**). This allows us to kinematically decouple the IK problem into **Inverse Position** and **Inverse Orientation** problems.
- Using Inverse Position, we find 3 wrist joint angles from the pose orientation.
- Using Inverse Orientation, we can find 3 joint angles from the base

### Inverse Position Kinematics

- We have to find out center of wrist, given the end-effector coordinates
- Before that we need to account for a rotation discrepancy between DH parameters and Gazebo (URDF)

![](https://github.com/Shilpaj1994/KR210-Pick-and-place/blob/master/misc_images/corr.png?raw=true)

- The correction is done by rotating z-axis by 180 degrees and then y-axis by -90 degrees

```python
R_z = Matrix([  [cos(np.pi), -sin(np.pi), 0, 0],
		[sin(np.pi),  cos(np.pi), 0, 0],
		[         0,           0, 1, 0],
		[         0,           0, 0, 1]])

R_y = Matrix([  [ cos(-np.pi/2),  0, sin(-np.pi/2), 0],
		[             0,  1,             0, 0],
		[-sin(-np.pi/2),  0, cos(-np.pi/2), 0],
		[             0,  0,             0, 1]])
R_corr = simplify(R_z * R_y)
```

- Finally, translation on the opposite direction of the gripper link (that lays on the Z axis) to find the wrist center.

![](https://github.com/Shilpaj1994/KR210-Pick-and-place/blob/master/misc_images/1.png?raw=true)

-  Following equation is used to find wrist center

![](https://github.com/Shilpaj1994/KR210-Pick-and-place/blob/master/misc_images/2.png?raw=true)

- From DH table, we can find out the wrist offset is 0.303m

```python
EE = Matrix([[px],
            [py],
            [pz]])

ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

# Calculate Wrest Center
WC = EE - (0.303) * ROT_EE[:,2]
```

- Once the wrist center (WC) is known we can calculate the first joint angle with a simple arctangent:

![](https://github.com/Shilpaj1994/KR210-Pick-and-place/blob/master/misc_images/3.png?raw=true)
  ![](https://github.com/Shilpaj1994/KR210-Pick-and-place/blob/master/misc_images/4.gif?raw=true)

```python
theta1 = atan2(WC[1],WC[0])
```

- For joints 2 and 3, using cosine rule:

![](https://github.com/Shilpaj1994/KR210-Pick-and-place/blob/master/misc_images/misc3.png?raw=true)

```python
side_a = 1.501
side_b = sqrt(pow((sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
side_c = 1.25

#Cosine Laws SSS to find all inner angles of the triangle
a = acos((-0.6875 + side_b*side_b) / (2.5*side_b))
b = acos(( 3.8125 - side_b*side_b) / (3.75))
c = acos(( 0.6875 + side_b*side_b) / (3.0*side_b))

#Find theta2 and theta3
theta2 = pi/2 - a - atan2(WC[2]-0.75, sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35)
theta3 = pi/2 - (b+0.036) # 0.036 accounts for sag in link4 of -0.054m
```

### Inverse Orientation Kinematics

![](https://github.com/Shilpaj1994/KR210-Pick-and-place/blob/master/misc_images/5.png?raw=true)

- This is used to find the parameters for first 3 joints starting from the base
- Using above equation, we get the other 3 parameters

```python
#Extract rotation matrix R0_3 from transformation matrix T0_3 then substitute angles q1-3
R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3:theta3})

# Get rotation matrix R3_6 from (transpose of R0_3 * R_EE)
R3_6 = R0_3.transpose() * ROT_EE

# Euler angles from rotation matrix
theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])

# select best solution based on theta5
if (theta5 >= np.pi) :
    theta4 = atan2(-R3_6[2,2], R3_6[0,2])
    theta6 = atan2(R3_6[1,1],-R3_6[1,0])
else:
    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    theta6 = atan2(-R3_6[1,1],R3_6[1,0])
```



---

### Project Implementation

- Video shows pick and place task performed by the robot.

<iframe width="900" height="506" src="https://www.youtube.com/embed/BCYN8gNxg3E" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

