import numpy as np
import matplotlib.pyplot as plt
from typing import Optional, Literal
import time

from baller.utils.hubert.constants import L2, L3, L6, L8, L9
from baller.utils.hubert.forward_kinematics import joint1pos, joint2pos, joint3pos
from baller.trajectory_solver.trajectory_solver import trajectory_solver_from_launcher_pos, launcher_pitch
from baller.model.hubert import HubertModel

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
        self.ax.set_aspect('equal')

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

    def update_color(self, color: str):
        self.color = color
        if self.artist is not None:
            self.artist[0].set_color(self.color)


class Hubert3DModel(Model3D, HubertModel):

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
        self.ax.set_aspect('equal')

        self.joints = {
            'j1': j1,
            'j2': j2,
            'j3': j3,
            'j4': 0,
            'j5': 0,
        }

        # Get the inital arm position
        x, y, z = self._arm_pos()

        # Plot the initial lines with three segments
        self.artist = self.ax.plot(x, y, z, color=self.color, linewidth=2, linestyle=self.linestyle)

    def get_pose(self, units: Literal['rad', 'deg'] = 'rad') -> dict[str, float]:
        if units == 'deg':
            joints = {j: float(np.rad2deg(v)) for j, v in self.joints.items()}
        else:
            joints = {j: float(v) for j, v in self.joints.items()}

        return joints


    def _arm_pos(self) -> tuple[list[float], list[float], list[float]]:
        """
        Return the x, y and z positoin of all joints in three lists
        """
        joint1 = joint1pos(**self.joints)
        joint2 = joint2pos(**self.joints)
        joint3 = joint3pos(**self.joints)

        # Extract coordinates for the segments
        x0, y0, z0 = [0, 0, 0]
        x1, y1, z1 = [0, 0, L2 + L3]
        x2, y2, z2 = joint1
        x3, y3, z3 = joint2
        x4, y4, z4 = joint3

        return [x0, x1, x2, x3, x4], [y0, y1, y2, y3, y4], [z0, z1, z2, z3, z4]

    def set_pose(self, units: Literal['rad', 'deg'] = 'rad', **joints: float):
        # Get the arm position

        for j, v in joints.items():
            self.joints[j] = v if units == 'rad' else np.deg2rad(v)

        x, y, z = self._arm_pos()

        self.artist[0].set_xdata(x)
        self.artist[0].set_ydata(y)
        self.artist[0].set_3d_properties(z)

        self.update_canvas()

    def wait_unitl_idle(self) -> None:
        time.sleep(1)


class Launcher3DModel(Model3D):

    def __init__(self, hubert: Hubert3DModel, target_plane: float, ax, fig, color: str = 'g', linestyle: str = '--') -> None:
        
        super().__init__(ax=ax, fig=fig, color=color, linestyle=linestyle)

        self.hubert = hubert
        self.target_plane = target_plane

        x, y, z = self.parabola()

        self.artist = self.ax.plot(x, y, z, color=self.color, linestyle=self.linestyle)

    def parabola(self) -> tuple[list[float], list[float], list[float]]:
        hand_x, hand_y, hand_z = joint3pos(**self.hubert.joints)

        pitch = launcher_pitch(self.hubert.joints['j2'], self.hubert.joints['j3'])
        yaw = self.hubert.joints['j1']

        _xs = np.linspace(hand_x, self.target_plane)
        xs = [_xs[0]]
        ys = [hand_y]
        zs = [hand_z]
        for x in _xs[1:]:
            try:
                _, y, z = trajectory_solver_from_launcher_pos(hand_x, hand_y, hand_z, pitch=pitch, yaw=yaw, target_plane=x)
            except AssertionError:
                break
            xs.append(x)
            ys.append(y)
            zs.append(z)
        
        return xs, ys, zs
    
    def move_launcher(self, target_plane: Optional[float] = None):
        # Get the arm position'
        if target_plane is not None:
            self.target_plane = target_plane

        x, y, z = self.parabola()

        self.artist[0].set_xdata(x)
        self.artist[0].set_ydata(y)
        self.artist[0].set_3d_properties(z)

        self.update_canvas()


class Target3DModel(Model3D):

    def __init__(self, x: float, y: float, z: float, ax, fig, color: str = 'r', marker: str = 'o', markersize: int = 10) -> None:
        super().__init__(ax, fig, color, "None")
        self.marker = marker
        self.markersize = markersize

        self.x = x
        self.y = y
        self.z = z

        # Increase the the plot width
        self.ax.set_xlim(right=1.0)
        self.ax.set_aspect('equal')

        self.artist = self.ax.plot(
            [self.x], [self.y], [self.z],
            marker=self.marker,
            color=self.color,
            linestyle=self.linestyle,
            markersize=self.markersize
        )

    def move_target(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z

        self.artist[0].set_xdata([self.x])
        self.artist[0].set_ydata([self.y])
        self.artist[0].set_3d_properties([self.z])


if __name__ == '__main__':

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    robot_arm = Hubert3DModel(ax=ax, fig=fig)
    extra_robot_arm = Hubert3DModel(ax=ax, fig=fig, j1=90, color='r')
    launcher = Launcher3DModel(robot_arm, 1.0, ax, fig)
    launcher2 = Launcher3DModel(extra_robot_arm, 1.0, ax, fig, color='orange')
    plt.show()