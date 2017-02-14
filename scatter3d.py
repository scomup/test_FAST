#!/usr/bin/env python
# coding:utf-8

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

x = []
y = []
z = []

f = open('mapPoint.txt', 'r')
for line in f:
    itemList = line.split(',')
    x.append(float(itemList[0]))
    y.append(float(itemList[1]))
    z.append(float(itemList[2]))

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

print x
print y
print z
ax.scatter(x, y, z, c='r', marker='o')

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

ax.set_xlim(-3.0, 3.0)
ax.set_ylim(-3.0, 3.0)
ax.set_zlim(0.0, 3.0)
plt.show()
