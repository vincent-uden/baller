import numpy as np
import matplotlib.pyplot as plt
from typing import Optional

from baller.utils.hubert.constants import L2, L3, L6, L8, L9
from baller.utils.hubert.forward_kinematics import joint1pos, joint2pos, joint3pos
from baller.trajectory_solver.trajectory_solver import trajectory_solver_from_launcher_pos, launcher_pitch

X_MIN = -L6 - L8 - L9
X_MAX = L6 + L8 + L9
Y_MIN = -L6 - L8 - L9
Y_MAX = L6 + L8 + L9
Z_MIN = 0
Z_MAX = L2 + L3 + L8 + L9


class Hubert3DModel:

    def __init__(self, j1: float = 0, j2: float = 0, j3: float = 0, ax = None, fig = None, color: str = 'b'):
        """
        Joint angles are given in degrees
        """
        # Create a figure and 3D axis
        self.fig = fig or plt.figure(figsize=(8, 6))
        self.ax = ax or self.fig.add_subplot(111, projection='3d')

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
        self.line = self.ax.plot(x, y, z, color=color, linewidth=2)

    def arm_pos(self, j1: Optional[float], j2: Optional[float], j3: Optional[float]):
        """
        Return the x, y and z positoin of all joints in three lists
        """
        if j1 is not None:
            self.j1 = np.deg2rad(j1)
        if j2 is not None:
            self.j2 = np.deg2rad(j2)
        if j3 is not None:
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

    def move_arm(self, joint_angles: list[float]):
        # Get the arm position
        x, y, z = self.arm_pos(*joint_angles[:3])

        self.line[0].set_xdata(x)
        self.line[0].set_ydata(y)
        self.line[0].set_3d_properties(z)

        self.fig.canvas.draw_idle()


class Launcher3DModel:

    def __init__(self, hubert: Hubert3DModel, target_plane: float, ax, fig) -> None:
        self.hubert = hubert
        self.target_plane = target_plane

        # Create a figure and 3D axis
        self.fig = fig
        self.ax = ax

        x, y, z = self.parabola()

        self.line = self.ax.plot(x, y, z, color='g', linestyle='--')

    def parabola(self) -> tuple[list[float], list[float], list[float]]:
        hand_x, hand_y, hand_z = joint3pos(self.hubert.j1, self.hubert.j2, self.hubert.j3)

        pitch = launcher_pitch(self.hubert.j2, self.hubert.j3)
        yaw = self.hubert.j1

        xs = np.linspace(hand_x, self.target_plane)
        ys = []
        zs = []
        for x in xs:
            _, y, z = trajectory_solver_from_launcher_pos(hand_x, hand_y, hand_z, pitch=pitch, yaw=yaw, target_plane=self.target_plane)
            ys.append(y)
            zs.append(z)
        
        return xs.tolist(), ys, zs
    
    def move_launcher(self):
        # Get the arm position
        x, y, z = self.parabola()

        self.line[0].set_xdata(x)
        self.line[0].set_ydata(y)
        self.line[0].set_3d_properties(z)

        self.fig.canvas.draw_idle()


if __name__ == '__main__':

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    robot_arm = Hubert3DModel(ax=ax, fig=fig)
    extra_robot_arm = Hubert3DModel(ax=ax, fig=fig, j1=90, color='r')
    launcher = Launcher3DModel(robot_arm, 1.0, ax, fig)
    plt.show()