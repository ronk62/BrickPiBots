
### Change Notes:
#
# 12/18/2021    - version 'b' ...experimenting with more customized point clouds
#                (custom model and scene point clouds)
#                changed vars: cloud to dstPC and rotated_cloud to srcPC
#                dstPC = cloud = scene
#                srcPC = rotated_cloud = model
#

'''

from https://github.com/opencv/opencv_contrib/blob/master/modules/surface_matching/samples/ppf_icp.py

'''

import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from numpy.core.fromnumeric import size


# def rotation(theta):
#     tx, ty, tz = theta

#     Rx = np.array([[1, 0, 0], [0, np.cos(tx), -np.sin(tx)], [0, np.sin(tx), np.cos(tx)]])
#     Ry = np.array([[np.cos(ty), 0, -np.sin(ty)], [0, 1, 0], [np.sin(ty), 0, np.cos(ty)]])
#     Rz = np.array([[np.cos(tz), -np.sin(tz), 0], [np.sin(tz), np.cos(tz), 0], [0, 0, 1]])

#     return np.dot(Rx, np.dot(Ry, Rz))

# cloud, rotated_cloud = [None]*3, [None]*3
srcPC = np.array([[]], dtype=np.float32)
dstPC = np.array([[]], dtype=np.float32)
retval, residual, pose = [None]*3, [None]*3, [None]*3
# noise = np.random.normal(0.0, 0.1, height * width * 3).reshape((-1, 3))
# noise2 = np.random.normal(0.0, 1.0, height * width)

print("")

## create ipc object
icp = cv.ppf_match_3d_ICP(100)


###### create src and dst point clouds ######

# brute force method, only 30 points

# for i in range(11):
#     srcPC = np.append(srcPC, [i, 0, 0])     # draw horizontal base of triangle
#     srcPC = np.append(srcPC, [i, i, 0])     # draw hypotenuse of triangle
#     srcPC = np.append(srcPC, [10, i, 0])    # draw verticle side of triangle

# for i in range(11):
#     dstPC = np.append(dstPC, [i+5, 3, 0])   # draw horizontal base of triangle
#     dstPC = np.append(dstPC, [i+5, i+3, 0]) # draw hypotenuse of triangle
#     dstPC = np.append(dstPC, [15, i+3, 0])  # draw verticle side of triangle


# brute force method, 300 points

for i in range(101):
    srcPC = np.append(srcPC, [i, 0, 0])     # draw horizontal base of triangle
    srcPC = np.append(srcPC, [i, i, 0])     # draw hypotenuse of triangle
    srcPC = np.append(srcPC, [100, i, 0])    # draw verticle side of triangle

for i in range(101):
    dstPC = np.append(dstPC, [i, 50, 0])   # draw horizontal base of triangle
    dstPC = np.append(dstPC, [i, i+50, 0]) # draw hypotenuse of triangle
    dstPC = np.append(dstPC, [100, i+50, 0])  # draw verticle side of triangle

srcPC = np.reshape(srcPC, (-1,3))
dstPC = np.reshape(dstPC, (-1,3))


## convert to float32
srcPC = np.float32(srcPC)
dstPC = np.float32(dstPC)


###### apply icp ######
retval, residual, pose = icp.registerModelToScene(srcPC, dstPC)

print("retval ", retval)
print("residual ", residual)
print("pose ", pose)

# while residual > 0.05:
#     retval, residual, pose = icp.registerModelToScene(srcPC, dstPC)
#     print("retval ", retval)
#     print("residual ", residual)
#     print("pose ", pose)    


################## extract values for plot ################## 

# x,y translation values
regXt = pose[0][3]
regYt = pose[1][3]

# dstPC...
dstPCxt = np.array([], dtype=np.float)
dstPCyt = np.array([], dtype=np.float)

for i in range(len(dstPC)):
    x = dstPC[i][0]
    y = dstPC[i][1]
    dstPCxt = np.append(dstPCxt,x)
    dstPCyt = np.append(dstPCyt,y)

# srcPC...
srcPCxt = np.array([], dtype=np.float)
srcPCyt = np.array([], dtype=np.float)

for i in range(len(srcPC)):
    x = srcPC[i][0]
    y = srcPC[i][1]
    srcPCxt = np.append(srcPCxt,x)
    srcPCyt = np.append(srcPCyt,y)


################### Create plots ################## 

# raw point cloud data
plt.figure(1)

plt.scatter(srcPCxt,srcPCyt, label='src/model point cloud', color='r', s=25, marker="o")
plt.scatter(dstPCxt,dstPCyt, label='dst/scene point cloud', color='b', s=25, marker="o")
plt.scatter(0,0, label='model x,y start location', color='k', s=25, marker="o")
plt.scatter(regXt,regYt, label='model x,y reg/end location', color='g', s=25, marker="o")
plt.axis('equal')
plt.xlabel('x-position')
plt.ylabel('y-position')
plt.title('ppf icp V20211218 - raw point cloud')
plt.legend()
plt.show()

print("")
