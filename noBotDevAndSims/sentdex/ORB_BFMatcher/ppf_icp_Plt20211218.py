'''

from https://github.com/opencv/opencv_contrib/blob/master/modules/surface_matching/samples/ppf_icp.py

'''


import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from numpy.core.fromnumeric import size


def rotation(theta):
    tx, ty, tz = theta

    Rx = np.array([[1, 0, 0], [0, np.cos(tx), -np.sin(tx)], [0, np.sin(tx), np.cos(tx)]])
    Ry = np.array([[np.cos(ty), 0, -np.sin(ty)], [0, 1, 0], [np.sin(ty), 0, np.cos(ty)]])
    Rz = np.array([[np.cos(tz), -np.sin(tz), 0], [np.sin(tz), np.cos(tz), 0], [0, 0, 1]])

    return np.dot(Rx, np.dot(Ry, Rz))

width = 20
height = 10
max_deg = np.pi / 12

cloud, rotated_cloud = [None]*3, [None]*3
retval, residual, pose = [None]*3, [None]*3, [None]*3
noise = np.random.normal(0.0, 0.1, height * width * 3).reshape((-1, 3))
noise2 = np.random.normal(0.0, 1.0, height * width)

x, y = np.meshgrid(
    range(-width//2, width//2),
    range(-height//2, height//2),
    sparse=False, indexing='xy'
)
z = np.zeros((height, width))

cloud[0] = np.dstack((x, y, z)).reshape((-1, 3)).astype(np.float32)
cloud[1] = noise.astype(np.float32) + cloud[0]
cloud[2] = cloud[1]
cloud[2][:, 2] += noise2.astype(np.float32)

R = rotation([
    0, #np.random.uniform(-max_deg, max_deg),
    np.random.uniform(-max_deg, max_deg),
    0, #np.random.uniform(-max_deg, max_deg)
])
t = np.zeros((3, 1))
Rt = np.vstack((
    np.hstack((R, t)),
    np.array([0, 0, 0, 1])
)).astype(np.float32)

icp = cv.ppf_match_3d_ICP(100)

I = np.eye(4)
print("Unaligned error:\t%.6f" % np.linalg.norm(I - Rt))
for i in range(3):
    rotated_cloud[i] = np.matmul(Rt[0:3,0:3], cloud[i].T).T + Rt[:3,3].T
    retval[i], residual[i], pose[i] = icp.registerModelToScene(rotated_cloud[i], cloud[i])
    print("i = ", i)
    print("ICP error:\t\t%.6f" % np.linalg.norm(I - np.matmul(pose[0], Rt)))
    print("retval[i] ", retval[i])
    print("residual[i] ", residual[i])
    print("pose[i] ", pose[i])

cloud0 = cloud[0]
cloud1 = cloud[1]
cloud2 = cloud[2]
print("cloud0[0:1] ", cloud0[0:1])
print("len(cloud0) ", len(cloud0))

rotated_cloud0 = rotated_cloud[0]
rotated_cloud1 = rotated_cloud[1]
rotated_cloud2 = rotated_cloud[2]

pose0 = pose[0]
pose1 = pose[1]
finalPose = pose[2]

print("")

################## extract values for plot ################## 

# cloud0...
c0Xt = np.array([], dtype=np.float)
c0Yt = np.array([], dtype=np.float)
# c0Zt = np.array([], dtype=np.float)

for i in range(len(cloud0)):
    x = cloud0[i][0]
    y = cloud0[i][1]
    c0Xt = np.append(c0Xt,x)
    c0Yt = np.append(c0Yt,y)

# cloud1...
c1Xt = np.array([], dtype=np.float)
c1Yt = np.array([], dtype=np.float)
# c1Zt = np.array([], dtype=np.float)

for i in range(len(cloud1)):
    x = cloud1[i][0]
    y = cloud1[i][1]
    c1Xt = np.append(c1Xt,x)
    c1Yt = np.append(c1Yt,y)

# cloud2...
c2Xt = np.array([], dtype=np.float)
c2Yt = np.array([], dtype=np.float)
# c2Zt = np.array([], dtype=np.float)

for i in range(len(cloud2)):
    x = cloud2[i][0]
    y = cloud2[i][1]
    c2Xt = np.append(c2Xt,x)
    c2Yt = np.append(c2Yt,y)


# rotated_cloud0...
rc0Xt = np.array([], dtype=np.float)
rc0Yt = np.array([], dtype=np.float)
# c0Zt = np.array([], dtype=np.float)

for i in range(len(rotated_cloud0)):
    x = rotated_cloud0[i][0]
    y = rotated_cloud0[i][1]
    rc0Xt = np.append(rc0Xt,x)
    rc0Yt = np.append(rc0Yt,y)


# rotated_cloud1...
rc1Xt = np.array([], dtype=np.float)
rc1Yt = np.array([], dtype=np.float)
# c1Zt = np.array([], dtype=np.float)

for i in range(len(rotated_cloud1)):
    x = rotated_cloud1[i][0]
    y = rotated_cloud1[i][1]
    rc1Xt = np.append(rc1Xt,x)
    rc1Yt = np.append(rc1Yt,y)


# rotated_cloud2...
rc2Xt = np.array([], dtype=np.float)
rc2Yt = np.array([], dtype=np.float)
# c2Zt = np.array([], dtype=np.float)

for i in range(len(rotated_cloud2)):
    x = rotated_cloud2[i][0]
    y = rotated_cloud2[i][1]
    rc2Xt = np.append(rc2Xt,x)
    rc2Yt = np.append(rc2Yt,y)

regXt = finalPose[0][3]
regYt = finalPose[1][3]


################## Create plots ################## 

# raw point cloud data
plt.figure(1)

plt.scatter(c0Xt,c0Yt, label='original point cloud', color='y', s=25, marker="o")
# plt.scatter(c1Xt,c1Yt, label='c1 point cloud', color='r', s=25, marker="o")
plt.scatter(c2Xt,c2Yt, label='c2 (noisy) point cloud', color='b', s=25, marker="o")
# plt.scatter(rc2Xt,rc2Yt, label='rotated c2 point cloud', color='g', s=25, marker="o")
plt.scatter(0,0, label='x,y model location', color='k', s=25, marker="o")
plt.axis('equal')
plt.xlabel('x-position')
plt.ylabel('y-position')
plt.title('ppf icp V20211218 - raw point cloud')
plt.legend()
# plt.show()

print("")

# processed point cloud data
plt.figure(2)

plt.scatter(c0Xt,c0Yt, label='original point cloud', color='y', s=25, marker="o")
plt.scatter(rc0Xt,rc0Yt, label='rotated c0 point cloud', color='r', s=25, marker="o")
plt.scatter(rc1Xt,rc1Yt, label='rotated c1 point cloud', color='b', s=25, marker="o")
plt.scatter(rc2Xt,rc2Yt, label='rotated c2 point cloud', color='g', s=25, marker="o")
plt.scatter(regXt,regYt, label='x,y model location', color='k', s=25, marker="o")
plt.axis('equal')
plt.xlabel('x-position')
plt.ylabel('y-position')
plt.title('ppf icp V20211218 - processed point cloud')
plt.legend()
plt.show()

print("")
