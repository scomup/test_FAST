#!/usr/bin/env python
# coding:utf-8

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import math
import re

def euler2mat(z=0, y=0, x=0):
    
    Ms = []
    if z:
        cosz = math.cos(z)
        sinz = math.sin(z)
        Ms.append(np.array(
                [[cosz, -sinz, 0],
                 [sinz, cosz, 0],
                 [0, 0, 1]]))
    if y:
        cosy = math.cos(y)
        siny = math.sin(y)
        Ms.append(np.array(
                [[cosy, 0, siny],
                 [0, 1, 0],
                 [-siny, 0, cosy]]))
    if x:
        cosx = math.cos(x)
        sinx = math.sin(x)
        Ms.append(np.array(
                [[1, 0, 0],
                 [0, cosx, -sinx],
                 [0, sinx, cosx]]))
    if Ms:
        return reduce(np.dot, Ms[::-1])
    return np.eye(3)


x = []
y = []
z = []
print 'in'
cam = []
f = open('mapPoint.txt', 'r')
for line in f:
    itemList = filter(lambda w: len(w) > 0, re.split(r'\s|:|\,', line))
    if(itemList[0] == 'camera'):
        c = [float(itemList[1]), float(itemList[2]), float(itemList[3]), float(itemList[4]), float(itemList[5]), float(itemList[6])]
        cam.append(c)
    else:
        x.append(float(itemList[1]))
        y.append(float(itemList[2]))
        z.append(float(itemList[3]))

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(0, 0, 0, color='b', marker='^')

first_flag = 1
for c in cam:
    R = euler2mat(c[3], c[4], c[5])
    v = np.array([[0],[0],[1]])
    v = np.dot(R,v)
    if first_flag:  
        first_flag = 0
        ax.quiver(c[0], c[1], c[2], v[0][0], v[1][0], v[2][0], color='g' ,length=0.2, normalize=True)
    else:
        ax.quiver(c[0], c[1], c[2], v[0][0], v[1][0], v[2][0], color='y' ,length=0.2, normalize=True)
    
ax.scatter(x, y, z, color='r', marker='o')
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

ax.set_xlim(-3.0, 3.0)
ax.set_ylim(-3.0, 3.0)
ax.set_zlim(-3.0, 3.0)
plt.show()
