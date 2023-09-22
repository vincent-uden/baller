import numpy as np
import math

from baller.utils.hubert.constants import L2, L3, L4, L5, L6, L7, L8, L9
from baller.utils.math.matrix import homogenous_transformation_matrix as htm

DEG90 = np.pi / 2


def joint1pos(j1: float) -> list[float]:
    frame0to1 = htm(rz=j1) @ htm(dx=L6, dy=-L4, dz=L2 + L3) @ htm(rx=DEG90)
    P4 = np.array([0, 0, 0, 1])
    P = frame0to1 @ P4
    return [P[0], P[1], P[2]]

def joint2pos(j1: float, j2: float) -> list[float]:
    frame0to1 = htm(rz=j1) @ htm(dx=L6, dy=-L4, dz=L2 + L3) @ htm(rx=DEG90)
    frame1to2 = htm(rz=j2) @ htm(dx=L7, dy=-L8, dz=-L5)
    frame0to2 = frame0to1 @ frame1to2
    P4 = np.array([0, 0, 0, 1])
    P = frame0to2 @ P4
    return [P[0], P[1], P[2]]

def joint3pos(j1: float, j2: float, j3: float) -> list[float]:
    x = L4*math.sin(j1) - L5*math.sin(j1) + L6*math.cos(j1) + L7*math.cos(j1)*math.cos(j2) + L8*math.sin(j2)*math.cos(j1) + L9*math.sin(j2 + j3)*math.cos(j1)
    y = -L4*math.cos(j1) + L5*math.cos(j1) + L6*math.sin(j1) + L7*math.sin(j1)*math.cos(j2) + L8*math.sin(j1)*math.sin(j2) + L9*math.sin(j1)*math.sin(j2 + j3)
    z = L2 + L3 + L7*math.sin(j2) - L8*math.cos(j2) - L9*math.cos(j2 + j3)
    return [x, y, z]

def launcher_pos(j1: float, j2: float, j3: float) -> list[float]:
    x = L4*math.sin(j1) - L5*math.sin(j1) + L6*math.cos(j1) + L7*math.cos(j1)*math.cos(j2) + L8*math.sin(j2)*math.cos(j1) + L9*math.sin(j2 + j3)*math.cos(j1)
    y = -L4*math.cos(j1) + L5*math.cos(j1) + L6*math.sin(j1) + L7*math.sin(j1)*math.cos(j2) + L8*math.sin(j1)*math.sin(j2) + L9*math.sin(j1)*math.sin(j2 + j3)
    z = L2 + L3 + L7*math.sin(j2) - L8*math.cos(j2) - L9*math.cos(j2 + j3)
    return [x, y, z]
