import numpy as np
import matplotlib.pyplot as plt

from baller.utils.hubert.constants import L2, L3, L6, L8, L9
from baller.utils.hubert.forward_kinematics import joint1pos, joint2pos, joint3pos

X_MIN = -L6 - L8 - L9
X_MAX = L6 + L8 + L9
Y_MIN = -L6 - L8 - L9
Y_MAX = L6 + L8 + L9
Z_MIN = 0
Z_MAX = L2 + L3 + L8 + L9


class Hubert3DModel:

    def __init__(self, j1: float = 0, j2: float = 0, j3: float = 0):
        """
        Joint angles are given in degrees
        """
        # Create a figure and 3D axis
        self.fig = plt.figure(figsize=(8, 6))
        self.ax = self.fig.add_subplot(111, projection='3d')

        # Set axis labels
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_zlabel('Z [m]')

        # Set axis limits
        self.ax.set_xlim(X_MIN, X_MAX)
        self.ax.set_ylim(Y_MIN, Y_MAX)
        self.ax.set_zlim(Z_MIN, Z_MAX)

        self.j1 = j1
        self.j2 = j2
        self.j3 = j3

        # Get the inital arm position
        x, y, z = self.arm_pos(j1, j2, j3)

        # Plot the initial lines with three segments
        self.line = self.ax.plot(x, y, z, color='b', linewidth=2)

    def arm_pos(self, j1: float = 0, j2: float = 0, j3: float = 0):
        """
        Return the x, y and z positoin of all joints in three lists
        """
        self.j1 = np.deg2rad(j1)
        self.j2 = np.deg2rad(j2)
        self.j3 = np.deg2rad(j3)

        joint1 = joint1pos(self.j1)
        joint2 = joint2pos(self.j1, self.j2)
        joint3 = joint3pos(self.j1, self.j2, self.j3)

        # Extract coordinates for the segments
        x0, y0, z0 = [0, 0, 0]
        x1, y1, z1 = [0, 0, L2 + L3]
        x2, y2, z2 = joint1
        x3, y3, z3 = joint2
        x4, y4, z4 = joint3

        return [x0, x1, x2, x3, x4], [y0, y1, y2, y3, y4], [z0, z1, z2, z3, z4]

    def move_arm(self, j1: float, j2: float, j3: float):
        # Get the arm position
        x, y, z = self.arm_pos(j1, j2, j3)

        self.line[0].set_xdata(x)
        self.line[0].set_ydata(y)
        self.line[0].set_3d_properties(z)

        self.fig.canvas.draw_idle()


robot_arm = Hubert3DModel()
plt.show()