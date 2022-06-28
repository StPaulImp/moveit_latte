#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import math
import numpy as np


#
def rotmat2euler(mat):
    angle_y = -math.asin(mat[2, 0])
    angle_x = math.atan2(mat[2, 1], mat[2, 2])
    angle_z = math.atan2(mat[1, 0]/math.cos(angle_y), mat[0, 0]/math.cos(angle_y))
    return np.array([angle_x, angle_y, angle_z])


def rotmat2quaternion(mat):
    pass


#
def rotmat2rodrigues(mat):
    theta = math.acos((mat.trace()-1)/2)
    r = (mat - mat.T)/(2*math.sin(theta))
    rx = r[2, 1]
    ry = r[0, 2]
    rz = r[1, 0]
    return np.array([rx, ry, rz]) * theta


def rodrigues2rotmat(vec):
    angle = np.sqrt(np.sum(vec**2))
    if angle == 0:
        return np.eye(3)
    axis = vec / angle
    mat = math.cos(angle) * np.eye(3) + (1 - math.cos(angle)) * axis.reshape(-1, 1) * axis + \
          math.sin(angle) * np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return mat


def rodirgues2euler(vec):
    mat = rodrigues2rotmat(vec)
    euler = rotmat2euler(mat)
    return euler


def euler2rodrigues(vec):
    mat = euler2rotmat(vec)
    rod = rotmat2rodrigues(mat)
    return rod


def euler2rotmat(vec):
    mat = np.zeros((3, 3))
    x, y, z = vec[0], vec[1], vec[2]
    mat[0, 0] = math.cos(y) * math.cos(z)
    mat[0, 1] = math.sin(x) * math.sin(y) * math.cos(z) - math.cos(x) * math.sin(z)
    mat[0, 2] = math.cos(x) * math.sin(y) * math.cos(z) + math.sin(x) * math.sin(z)

    mat[1, 0] = math.cos(y) * math.sin(z)
    mat[1, 1] = math.sin(x) * math.sin(y) * math.sin(z) + math.cos(x) * math.cos(z)
    mat[1, 2] = math.cos(x) * math.sin(y) * math.sin(z) - math.sin(x) * math.cos(z)

    mat[2, 0] = -math.sin(y)
    mat[2, 1] = math.sin(x) * math.cos(y)
    mat[2, 2] = math.cos(x) * math.cos(y)

    return mat


if __name__ == '__main__':
    a = np.array([1.0, 2.0, 3.0])
    b = rodrigues2rotmat(a)
    print(b)
    c = rotmat2rodrigues(b)
    print(c)
