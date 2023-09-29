import matplotlib.pyplot as plt
from argparse import ArgumentParser, Namespace, Action
import sys

from baller.communication.hubert import Servo, Hubert
from baller.model.slider import SliderWindow
from baller.model.model import Hubert3DModel, Launcher3DModel, Target3DModel
from baller.inverse_kinematics.ik import target_pos_to_joint_angles, LAUNCH_PLANE_OFFSET
import baller.trajectory_solver.trajectory_solver as ts


hubert_com = None       # Handles communication with Hubert
hubert_model = None     # A 3D model of Hubert
hubert_pose = None      # A pose estimator of Huberts actual pose

launcher = None         # A model for the trajectory of a projectile from the launcher
target = None           # A target to hit

sw = None               # Window for sliders


def ik_callback(x: float, y: float, z: float, v0: float, **_) -> None:
    """
    Perform inverse kinematics
    """
    assert hubert_model is not None
    assert launcher is not None
    assert target is not None

    ts.V0 = v0

    target.move_target(x, y, z)
    _j1, _j2, _j3, *_ = hubert_model.get_pose(units='rad')
    j1, j2, j3, dist = target_pos_to_joint_angles(x, y, z, j1=_j1, j2=_j2, j3=_j3)

    if dist > 0.01:
        # Miss with more than 1 cm
        launcher.update_color('r')
    else:
        # Hit
        launcher.update_color('g')

    hubert_model.move_arm(j1, j2, j3, units='rad')
    launcher.move_launcher(target_plane=x)

    if hubert_com is not None:
        j1_deg, j2_deg, j3_deg = hubert_model.get_pose(units='deg')
        hubert_com.set_position(j1=j1_deg, j2=j2_deg, j3=j3_deg)


def get_pos_callback() -> None:
    assert hubert_com is not None
    assert hubert_pose is not None

    joints = hubert_com.get_position()
    hubert_pose.move_arm(j1=joints[0], j2=joints[1], j3=joints[2], units='deg')


class NotImplementedAction(Action):
    def __call__(self, parser, namespace, values, option_string=None):
        msg = 'Argument "{}" is under development and has not yet been implemented.'.format(option_string)
        sys.exit(msg)


def parse_args() -> Namespace:
    parser = ArgumentParser("baller", description="Interact with hubert")
    parser.add_argument('-p', '--port', help="USB port that Hubert is connected to")
    parser.add_argument('-b', '--baudrate', default=57600, type=int, help="Baudrate of the serial communication")
    parser.add_argument('--conf', action=NotImplementedAction, help="Read connection details from configuration file. Not implemented yet")
    parser.add_argument('-v', '--visual-mode', action='store_true', help="Open a window that displays Huberts real time position (only takes effect if Hubert is connected)")
    parser.add_argument('-i', '--interactive', action='store_true', help="Start an interactive session")
    parser.add_argument('-t', '--target-plane', type=float, help="Add a target plane at x = [target-plane]. Also adds sliders for the target position. (Overrides interactive)")

    return parser.parse_args()


servos = [
    Servo([-45, 0, 90], [2150, 1620, 690]),
    Servo([0, 90], [2250, 1350]),
    Servo([-90, 0, 90], [600, 1400, 2400]),
    Servo([-90, 0, 90], [2400, 1500, 600]),
    Servo([0, 90], [2100, 1200]),
]


def main():
    args = parse_args()

    global hubert_com, hubert_model, hubert_pose, launcher, sw, target

    if args.conf is not None:
        # Assing variables from configuration file
        raise NotImplementedError("This argument has not yet been implemented")
    
    if args.port is not None:
        # Connect to Hubert
        hubert_com = Hubert(args.port, baudrate=args.baudrate, servos=servos, timeout=0.1)
        hubert_com.connect()

    if args.target_plane is not None:
        # Target position
        tx = args.target_plane
        ty = -LAUNCH_PLANE_OFFSET
        tz = 0.2

        hubert_model = Hubert3DModel()

        launcher = Launcher3DModel(hubert_model, tx, hubert_model.ax, hubert_model.fig)
        target = Target3DModel(tx, ty, tz, ax=hubert_model.ax, fig=hubert_model.fig)

        sw = SliderWindow()
        sw.add_slider("x", 0.3, 2.0, tx)
        sw.add_slider("y", -0.5, 0.5, ty)
        sw.add_slider("z", 0.0, 1.0, tz)
        sw.add_slider("v0", 1.0, 10.0, ts.V0)

        sw.add_slider_callback(ik_callback)

        if hubert_com is not None:
            sw.add_button("launch", [lambda x: hubert_com.launch()])

        # Call the callback so that it draws correctly the first time
        ik_callback(tx, ty, tz, ts.V0)
    
    elif args.interactive:
        hubert_model = Hubert3DModel()

        sw = SliderWindow()
        for i in range(len(servos)):
            sw.add_slider(f'j{i+1}', *servos[i].servo_range(), 0)

        sw.add_slider_callback(lambda **js: hubert_model.move_arm(**js, units='deg'))
        
        if hubert_com is not None:
            sw.add_slider_callback(hubert_com.set_position)
            sw.add_button("launch", [lambda x: hubert_com.launch()])

    if args.visual_mode and hubert_com is not None:
        fig = None if hubert_model is None else hubert_model.fig
        ax = None if hubert_model is None else hubert_model.ax
        hubert_pose = Hubert3DModel(ax=ax, fig=fig, color='orange', linestyle='--')

        timer = fig.canvas.new_timer(interval=50)
        timer.add_callback(get_pos_callback)
        timer.start()
    
    if sw is not None:
        sw.draw()
    
    figs = figs = list(map(plt.figure, plt.get_fignums()))
    if len(figs) > 0:
        plt.show()


if __name__ == '__main__':
    main()