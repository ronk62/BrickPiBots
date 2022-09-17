
'''
An attempt at localization based on  Angela Sodemann's  youtube series...

"Robotics 1 U1 (Kinematics) S5 (Homogeneous Transformation Matrix) P1 (HTM from Rotation Matrix)"

The main difference in the analysis is that Kinematics assumes a linkage between the
frames, while localization assumes tags placed in the base frame at a location and
rotation that is not rigidly linked to the base-frame. Likewise, the cam/bot is not rigidly
linked to the tags.

The result is that...
1) the relative tag rotation angles are not used for displacement calculations
2) the rotations and x,y displacements for tags are measured and entered as known data
3) the cam/bot matrix data comes from the processed vid capture and apriltag lib


Notations and code outline
--------------------------

H0 can represent the base frame itself with H1 representing the Apriltag...
...or...H0 can represent the Apriltag with H1 representing the the cam/bot

H0 in this bit of code is at the origin, aligned with the base frame

H0 matrix is constructed from intuition due to initial condition simplicity

H0_1 is built from R0_1 and d0_1 components

R0_1 and d0_1 are composed from angles, displacement values, and formulas

H0_1 is initially composed from R0_1 and d0_1 components using np.concatinate...
...then computed as the dot.product of H0 and H0_1


ref. 

https://www.youtube.com/watch?v=fXewWpehAWw&list=PLT_0lwItn0sDBE98BsbaZezflB96ws12b&index=17

https://www.andre-gaschler.com/rotationconverter/

'''

import numpy as np
import matplotlib.pyplot as plt
import math


R0_1 = np.array([], dtype=np.int32)
d0_1 = np.array([], dtype=np.int32)

H0 = np.array([], dtype=np.int32)
H1 = np.array([], dtype=np.int32)

H0_1 = np.array([], dtype=np.int32)

# H0 at origin
H0 = [[1,0,0,0],
[0,1,0,0],
[0,0,1,0],
[0,0,0,1]]

# # H0 NOT at origin
# H0 = [[1,0,0,0],
# [0,1,0,400],
# [0,0,1,0],
# [0,0,0,1]]


# H1 also NOT at origin, offset from H0
H1 = [[1,0,0,5],
[0,1,0,5],
[0,0,1,0],
[0,0,0,1]]

# Assign Eurler rotation angles
theta1Deg=90  # rotation angle between H0 and H1

# convert angles from deg to radians
theta1Rad=math.radians(theta1Deg)

# Assign displacement values
ax1=5         # x displacement between H0 and H1
ay1=5         # y displacement between H0 and H1


### Define the rotations
# R0_1
R0_1 = [[np.cos(theta1Rad),-np.sin(theta1Rad),0],
[np.sin(theta1Rad),np.cos(theta1Rad),0],
[0,0,1]]


# Apply the displacement translations
d0_1 = [[ax1],[ay1],[0]]

# setup vars to graph the unit vectors
# V0 location and rotation
x0 = H0[0][3]
y0 = H0[1][3]
vxx0 = [x0,x0 + H0[0][0]]
vxy0 = [y0,y0 + H0[1][0]]
vyx0 = [x0,x0 + H0[0][1]]
vyy0 = [y0,y0 + H0[1][1]]

print()
print("the x,y position of H0 is  ", x0,y0)
print()


H0_1 = np.concatenate((R0_1, d0_1), 1)
H0_1 = np.concatenate((H0_1, [[0,0,0,1]]), 0)

H0_1 = np.dot(H0, H0_1)

print()
print("H0_1 is  ")
print(np.matrix(H0_1))
print()

# extract the values from H0_1 for graphing
x1 = H0_1[0][3]
y1 = H0_1[1][3]
vxx1 = [x1,x1 + H0_1[0][0]]
vxy1 = [y1,y1 + H0_1[1][0]]
vyx1 = [x1,x1 + H0_1[0][1]]
vyy1 = [y1,y1 + H0_1[1][1]]

print()
print("the x,y position of H0_1 is  ", x1,y1)
print()



## graph the unit vectors
fig, ax = plt.subplots()

plt.scatter(x0,y0, label='v0 origin', color='r', s=25, marker="o")
line1, = ax.plot(vxx0,vxy0, label='v0 x', lw=0.4, color='r', marker=">")
line2, = ax.plot(vyx0,vyy0, label='v0 y', lw=0.4, color='b', marker="^")

plt.scatter(x1,y1, label='v1 origin', color='y', s=25, marker="o")
line3, = ax.plot(vxx1,vxy1, label='v1 x', lw=0.4, color='y', marker="None")
line4, = ax.plot(vyx1,vyy1, label='v1 y', lw=0.4, color='c', marker="None")

# plt.scatter(x2,y2, label='v2 origin', color='m', s=25, marker="o")
# line5, = ax.plot(vxx2,vxy2, label='v2 x', lw=0.4, color='m', marker="None")
# line6, = ax.plot(vyx2,vyy2, label='v2 y', lw=0.4, color='g', marker="None")

# plt.scatter(x3,y3, label='v3 origin', color='darkorange', s=25, marker="o")
# line7, = ax.plot(vxx3,vxy3, label='v3 x', lw=0.4, color='darkorange', marker="None")
# line8, = ax.plot(vyx3,vyy3, label='v3 y', lw=0.4, color='royalblue', marker="None")

plt.axis('equal')
plt.xlabel('x-position')
plt.ylabel('y-position')
plt.title('unit vectors')
plt.legend()
plt.show()

print()

