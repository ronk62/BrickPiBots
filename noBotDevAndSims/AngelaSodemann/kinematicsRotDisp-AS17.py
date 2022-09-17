
'''
Somewhat munged localization based on  Angela Sodemann's  youtube series...

"Robotics 1 U1 (Kinematics) S5 (Homogeneous Transformation Matrix) P1 (HTM from Rotation Matrix)"


Notations and code outline
--------------------------

H0 and R0 matrices are constructed from intuition due to initial condition simplicity

Hn_m built from Rn_m and dn_m components

R0_x and d0_x are composed from angles, displacement values, and formulas

H0 = base frame = x0, y0  (ref location, displasement can be offset from origin)
H0_1 is initially composed from R0_1 and d0_1 components, then...
    ...(as a dot.product of H0 and H0_1) H0_1 = x,y location of tag1 = displacement only 
H1_2 = x,y location with rotation of tag1 (displacement of 0 from a2)
H0_2 = dot.product of H0_1 and H1_2 and is the unit vector of the tag1 in base frame

...next section theoretical and tbd...
H2_3 is initially composed from R2_3 and d2_3 components, then...
    ...(as a dot.product of H0_2 and H2_3) H2_3 = x,y location of Bob/Cam...?...displacement only 
H0_3 = dot.product of H0_2 and H2_3


ref. 

https://www.youtube.com/watch?v=fXewWpehAWw&list=PLT_0lwItn0sDBE98BsbaZezflB96ws12b&index=17

https://www.andre-gaschler.com/rotationconverter/

'''



import numpy as np
import matplotlib.pyplot as plt
import math


R0_1 = np.array([], dtype=np.int32)
d0_1 = np.array([], dtype=np.int32)

R1_2 = np.array([], dtype=np.int32)
d1_2 = np.array([], dtype=np.int32)

R0_2 = np.array([], dtype=np.int32)

H0 = np.array([], dtype=np.int32)
H1 = np.array([], dtype=np.int32)
H2 = np.array([], dtype=np.int32)

H0_1 = np.array([], dtype=np.int32)
H1_2 = np.array([], dtype=np.int32)

# R = False

# # H0 at origin
# H0 = [[1,0,0,0],
# [0,1,0,0],
# [0,0,1,0],
# [0,0,0,1]]

# H0 NOT at origin
H0 = [[1,0,0,0],
[0,1,0,400],
[0,0,1,0],
[0,0,0,1]]

# R0
R0 = [[1,0,0],
[0,1,0],
[0,0,1]]

# # H1 also NOT at origin, offset from H0
# H1 = [[1,0,0,5],
# [0,1,0,5],
# [0,0,1,0],
# [0,0,0,1]]

# Assign Eurler rotation angles
theta1Deg=-135  # rotation angle between H0 and simulated Apriltag
theta2Deg=45
theta3Deg=90

# convert angles from deg to radians
theta1Rad=math.radians(theta1Deg)
theta2Rad=math.radians(theta2Deg)
theta3Rad=math.radians(theta3Deg)

# Assign displacement values
a1=7.07         # displacement between H0 and simulated Apriltag
a2=0            # dummy displacement used to only rotate the simulated Apriltag
a3=1         # displacement between simulated Apriltag and simulated Bot/Cam

### Define the rotations

# R0_1
R0_1 = [[np.cos(theta1Rad),-np.sin(theta1Rad),0],
[np.sin(theta1Rad),np.cos(theta1Rad),0],
[0,0,1]]

# R1_2
R1_2 = [[np.cos(theta2Rad),-np.sin(theta2Rad),0],
[np.sin(theta2Rad),np.cos(theta2Rad),0],
[0,0,1]]

# R2_3
R2_3 = [[np.cos(theta3Rad),-np.sin(theta3Rad),0],
[np.sin(theta3Rad),np.cos(theta3Rad),0],
[0,0,1]]

# Apply the displacement translations
d0_1 = [[a1*np.cos(theta1Rad)],[a1*np.sin(theta1Rad)],[0]]
d1_2 = [[a2*np.cos(theta2Rad)],[a2*np.sin(theta2Rad)],[0]]
d2_3 = [[a3*np.cos(theta3Rad)],[a3*np.sin(theta3Rad)],[0]]

# setup vars to graph the unit vectors

# V0 location and rotation
x0 = H0[0][3]
y0 = H0[1][3]
vxx0 = [x0,x0 + R0[0][0]]
vxy0 = [y0,y0 + R0[1][0]]
vyx0 = [x0,x0 + R0[0][1]]
vyy0 = [y0,y0 + R0[1][1]]

print()
print("the x,y position of H0 is  ", x0,y0)
print()

'''
# x1,y1 = R0_1[0][3], R0_1[1][3]
x1 = d0_1[0]
y1 = d0_1[1]
vxx1 = [x1,x1 + R0_1[0][0]]
vxy1 = [y1,y1 + R0_1[1][0]]
vyx1 = [x1,x1 + R0_1[0][1]]
vyy1 = [y1,y1 + R0_1[1][1]]

print()
print("the x,y position of R0_1 is  ", x1,y1)
print()
'''

'''
# x2,y2 = R1_2[0][3], R1_2[1][3]
x2 = x1
y2 = y1
vxx2 = [x2,x2 + R1_2[0][0]]
vxy2 = [y2,y2 + R1_2[1][0]]
vyx2 = [x2,x2 + R1_2[0][1]]
vyy2 = [y2,y2 + R1_2[1][1]]

print()
print("the x,y position of R1_2 is  ", x2,y2)
print()
'''

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

H1_2 = np.concatenate((R1_2, d1_2), 1)
H1_2 = np.concatenate((H1_2, [[0,0,0,1]]), 0)

print()
print("H1_2 is  ")
print(np.matrix(H1_2))
print()

H2_3 = np.concatenate((R2_3, d2_3), 1)
H2_3 = np.concatenate((H2_3, [[0,0,0,1]]), 0)

print()
print("H2_3 is  ")
print(np.matrix(H2_3))
print()


H0_2 = np.dot(H0_1,H1_2)

print()
print("H0_2 is  ")
print(np.matrix(H0_2))
print()

# extract the values from H0_2 for graphing
x2 = H0_2[0][3]
y2 = H0_2[1][3]
vxx2 = [x2,x2 + H0_2[0][0]]
vxy2 = [y2,y2 + H0_2[1][0]]
vyx2 = [x2,x2 + H0_2[0][1]]
vyy2 = [y2,y2 + H0_2[1][1]]

print()
print("the x,y position of H0_2 is  ", x2,y2)
print()

H0_3 = np.dot(H0_2,H2_3)

print()
print("H0_3 is  ")
print(np.matrix(H0_3))
print()

# extract the values from H0_3 for graphing
x3 = H0_3[0][3]
y3 = H0_3[1][3]
vxx3 = [x3,x3 + H0_3[0][0]]
vxy3 = [y3,y3 + H0_3[1][0]]
vyx3 = [x3,x3 + H0_3[0][1]]
vyy3 = [y3,y3 + H0_3[1][1]]

print()
print("the x,y position of H0_3 is  ", x3,y3)
print()


## graph the unit vectors
fig, ax = plt.subplots()

plt.scatter(x0,y0, label='v0 origin', color='r', s=25, marker="o")
line1, = ax.plot(vxx0,vxy0, label='v0 x', lw=0.4, color='r', marker=">")
line2, = ax.plot(vyx0,vyy0, label='v0 y', lw=0.4, color='b', marker="^")

plt.scatter(x1,y1, label='v1 origin', color='y', s=25, marker="o")
line3, = ax.plot(vxx1,vxy1, label='v1 x', lw=0.4, color='y', marker="None")
line4, = ax.plot(vyx1,vyy1, label='v1 y', lw=0.4, color='c', marker="None")

plt.scatter(x2,y2, label='v2 origin', color='m', s=25, marker="o")
line5, = ax.plot(vxx2,vxy2, label='v2 x', lw=0.4, color='m', marker="None")
line6, = ax.plot(vyx2,vyy2, label='v2 y', lw=0.4, color='g', marker="None")

plt.scatter(x3,y3, label='v3 origin', color='darkorange', s=25, marker="o")
line7, = ax.plot(vxx3,vxy3, label='v3 x', lw=0.4, color='darkorange', marker="None")
line8, = ax.plot(vyx3,vyy3, label='v3 y', lw=0.4, color='royalblue', marker="None")

plt.axis('equal')
plt.xlabel('x-position')
plt.ylabel('y-position')
plt.title('unit vectors')
plt.legend()
plt.show()

print()

