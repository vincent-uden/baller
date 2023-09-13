import numpy as np
from .constants import L2, L3, L4, L5, L6, L7, L8, L9
from ..math.matrix import homogenous_transformation_matrix as htm

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
    frame0to1 = htm(rz=j1) @ htm(dx=L6, dy=-L4, dz=L2 + L3) @ htm(rx=DEG90)
    frame1to2 = htm(rz=j2) @ htm(dx=L7, dy=-L8, dz=-L5)
    frame2to3 = htm(rz=j3) @ htm(dy=-L9)

    # Complete transformation
    frame0to3 = frame0to1 @ frame1to2 @ frame2to3

    P4 = np.array([0, 0, 0, 1])
    P = frame0to3 @ P4
    return [P[0], P[1], P[2]]
