import numpy as np
import matplotlib.pyplot as plt
#import matplotlib.lines as lines


H0 = np.array([], dtype=np.int32)
H1 = np.array([], dtype=np.int32)
H1_R = np.array([], dtype=np.int32)
HR1 = np.array([], dtype=np.int32)

'''
ref. https://www.andre-gaschler.com/rotationconverter/
'''

R = False

# HR1 is a 90 deg cw rotation around cartesian std 'z' axis
HR1 = [[0,1,0,0],
[-1,0,0,0],
[0,0,1,0],
[0,0,1,1]]


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

# # H1 also NOT at origin
# H1 = [[1,0,0,0],
# [0,1,0,400],
# [0,0,1,0],
# [0,0,0,1]]

# # H1 also NOT at origin, offset from H0
# H1 = [[1,0,0,0],
# [0,1,0,395],
# [0,0,1,0],
# [0,0,0,1]]

# H1 also NOT at origin, offset from H0 and manually rotated 90 deg cw
H1 = [[0,1,0,0],
[-1,0,0,395],
[0,0,1,0],
[0,0,0,1]]

# print()
# print("the shape of H0 is  ", np.shape(H0))

if R == True:
    # Apply rotations
    H1_R=np.dot(H1,HR1)

    # for testing
    print()
    print("H1...",np.matrix(H1))
    print()
    print("H1_R...",np.matrix(H1_R))
    print()

    # for convenience, set H1 to R1_R
    H1=H1_R

x0,y0 = H0[0][3], H0[1][3]
vx0 = [x0,x0 + H0[0][0]]
vy0 = [y0,y0 + H0[1][1]]

x1,y1 = H1[0][3], H1[1][3]
vx1 = [x1,x1 + H1[0][0]]
vy1 = [y1,y1 + H1[1][1]]


print()
print("the x,y position of H0 is  ", x0,y0)
print()

print()
print("the x,y position of H1 is  ", x1,y1)
print()


## graph the unit vectors
fig, ax = plt.subplots()

plt.scatter(x0,y0, label='v0 origin', color='k', s=25, marker="o")
line1, = ax.plot(vx0,[y0,y0], label='v0 x', lw=0.4, color='r', marker=">")
line2, = ax.plot([x0,x0],vy0, label='v0 y', lw=0.4, color='b', marker="^")

plt.scatter(x1,y1, label='v1 origin', color='k', s=25, marker="o")
line3, = ax.plot(vx1,[y1,y1], label='v1 x', lw=0.4, color='y', marker=">")
line4, = ax.plot([x1,x1],vy1, label='v1 y', lw=0.4, color='c', marker="^")

plt.axis('equal')
plt.xlabel('x-position')
plt.ylabel('y-position')
plt.title('unit vectors')
plt.legend()
plt.show()

print()

'''
## graph the relative position data
plt.figure(1)

plt.scatter(0,0, label='robot location', color='r', s=25, marker="o")
plt.scatter(AprTagx1,AprTagy1, label='tag location', color='k', s=25, marker="o")
plt.axis('equal')
plt.xlabel('x-position')
plt.ylabel('y-position')
plt.title('Apriltag location in robot frame - relative position data V20210131')
plt.legend()
# plt.show()

'''