import numpy as np
import matplotlib.pyplot as plt
from typing import Optional, Literal

from baller.utils.hubert.constants import L2, L3, L6, L8, L9
from baller.utils.hubert.forward_kinematics import joint1pos, joint2pos, joint3pos
from baller.trajectory_solver.trajectory_solver import trajectory_solver_from_launcher_pos, launcher_pitch

X_MIN = -L6 - L8 - L9
X_MAX = L6 + L8 + L9
Y_MIN = -L6 - L8 - L9
Y_MAX = L6 + L8 + L9
Z_MIN = 0
Z_MAX = L2 + L3 + L8 + L9


class Model3D:

    def __init__(self, ax = None, fig = None, color: str = 'b', linestyle: str = '-') -> None:
        # Create a figure and 3D axis
        self.fig = fig or plt.figure(figsize=(8, 6))
        self.ax = ax or self.fig.add_subplot(111, projection='3d')

        self.color = color
        self.linestyle = linestyle

        self.visable = True

        self.artist = None

    def update_canvas(self):
        self.fig.canvas.draw_idle()

    def toggle_visability(self):
        self.visable = not self.visable
        if self.artist is not None:
            self.artist[0].set_visible(self.visable)
            self.update_canvas()


class Hubert3DModel(Model3D):

    def __init__(self, j1: float = 0, j2: float = 0, j3: float = 0, ax = None, fig = None, color: str = 'b', linestyle: str = '-'):
        """
        Joint angles are given in degrees
        """
        super().__init__(ax=ax, fig=fig, color=color, linestyle=linestyle)

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
        x, y, z = self.arm_pos()

        # Plot the initial lines with three segments
        self.artist = self.ax.plot(x, y, z, color=self.color, linewidth=2, linestyle=self.linestyle)

    def get_pose(self, units: Literal['rad', 'deg'] = 'rad') -> tuple[float, float, float]:
        if units == 'deg':
            j1 = np.rad2deg(self.j1)
            j2 = np.rad2deg(self.j2)
            j3 = np.rad2deg(self.j3)
        else:
            j1 = self.j1
            j2 = self.j2
            j3 = self.j3

        return j1, j2, j3


    def arm_pos(self):
        """
        Return the x, y and z positoin of all joints in three lists
        """
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

    def move_arm(self, j1: float, j2: float, j3: float, units: Literal['rad', 'deg'] = 'rad', **_):
        # Get the arm position

        if units == 'deg':
            self.j1 = np.deg2rad(j1)
            self.j2 = np.deg2rad(j2)
            self.j3 = np.deg2rad(j3)
        else:
            self.j1 = j1
            self.j2 = j2
            self.j3 = j3

        x, y, z = self.arm_pos()

        self.artist[0].set_xdata(x)
        self.artist[0].set_ydata(y)
        self.artist[0].set_3d_properties(z)

        self.update_canvas()


class Launcher3DModel(Model3D):

    def __init__(self, hubert: Hubert3DModel, target_plane: float, ax, fig, color: str = 'g', linestyle: str = '--') -> None:
        
        super().__init__(ax=ax, fig=fig, color=color, linestyle=linestyle)

        self.hubert = hubert
        self.target_plane = target_plane

        x, y, z = self.parabola()

        self.artist = self.ax.plot(x, y, z, color=self.color, linestyle=self.linestyle)

    def parabola(self) -> tuple[list[float], list[float], list[float]]:
        hand_x, hand_y, hand_z = joint3pos(self.hubert.j1, self.hubert.j2, self.hubert.j3)

        pitch = launcher_pitch(self.hubert.j2, self.hubert.j3)
        yaw = self.hubert.j1

        xs = np.linspace(hand_x, self.target_plane)
        ys = [hand_y]
        zs = [hand_z]
        for x in xs[1:]:
            _, y, z = trajectory_solver_from_launcher_pos(hand_x, hand_y, hand_z, pitch=pitch, yaw=yaw, target_plane=x)
            ys.append(y)
            zs.append(z)
        
        return xs.tolist(), ys, zs
    
    def move_launcher(self, target_plane: Optional[float] = None):
        # Get the arm position'
        if target_plane is not None:
            self.target_plane = target_plane

        x, y, z = self.parabola()

        self.artist[0].set_xdata(x)
        self.artist[0].set_ydata(y)
        self.artist[0].set_3d_properties(z)

        self.update_canvas()


if __name__ == '__main__':

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    robot_arm = Hubert3DModel(ax=ax, fig=fig)
    extra_robot_arm = Hubert3DModel(ax=ax, fig=fig, j1=90, color='r')
    launcher = Launcher3DModel(robot_arm, 1.0, ax, fig)
    launcher2 = Launcher3DModel(extra_robot_arm, 1.0, ax, fig, color='orange')
    plt.show()