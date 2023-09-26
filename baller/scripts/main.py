import matplotlib.pyplot as plt
from argparse import ArgumentParser, Namespace, Action
import sys

from baller.communication.hubert import Servo, Hubert
from baller.model.slider import SliderWindow
from baller.model.model import Hubert3DModel, Launcher3DModel
from baller.inverse_kinematics.ik import target_pos_to_joint_angles


hubert_com = None       # Handles communication with Hubert
hubert_model = None     # A 3D model of Hubert
hubert_pose = None      # A pose estimator of Huberts actual pose

launcher = None         # A model for the trajectory of a projectile from the launcher

sw = None               # Window for sliders


def ik_callback(x: float, y: float, z: float, **_) -> None:
    """
    Perform inverse kinematics
    """
    assert hubert_model is not None
    assert launcher is not None

    _j1, _j2, _j3, *_ = hubert_model.get_pose(units='rad')
    j1, j2, j3 = target_pos_to_joint_angles(x, y, z, j1=_j1, j2=_j2, j3=_j3)

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
    parser.add_argument('-v', '--visual-mode', action='store_true', help="Open a window that displays Huberts real time position")
    parser.add_argument('-i', '--interactive', action='store_true', help="Start an interactive session")
    parser.add_argument('-t', '--target-plane', type=float, help="Add a target plane at x = [target-plane]. Also adds sliders for the target position. (Overrides interactive)")

    return parser.parse_args()


servos = [
    Servo([-90, 0, 90], [700, 1600, 2070]),
    Servo([0, 180], [1350, 2200]),
    Servo([-90, 90], [1410, 2400]),
    Servo([-90, 90], [600, 1500]),
    Servo([-90, 90], [1170, 2100]),
]


def main():
    args = parse_args()

    global hubert_com, hubert_model, hubert_pose, launcher, sw

    if args.conf is not None:
        # Assing variables from configuration file
        raise NotImplementedError("This argument has not yet been implemented")
    
    if args.port is not None:
        # Connect to Hubert
        hubert_com = Hubert(args.port, baudrate=args.baudrate, servos=servos, timeout=0.1)
        hubert_com.connect()

    if args.target_plane is not None:
        hubert_model = Hubert3DModel()

        launcher = Launcher3DModel(hubert_model, args.target_plane, hubert_model.ax, hubert_model.fig)

        sw = SliderWindow()
        sw.add_slider("x", 0.3, 2.0, args.target_plane)
        sw.add_slider("y", -0.5, 0.5, 0.0)
        sw.add_slider("z", 0.0, 1.0, 0.5)

        sw.add_slider_callback(ik_callback)
    
    elif args.interactive:
        hubert_model = Hubert3DModel()

        sw = SliderWindow()
        for i in range(len(servos)):
            sw.add_slider(f'j{i+1}', *servos[i].servo_range(), 0)

        sw.add_slider_callback(lambda **js: hubert_model.move_arm(**js, units='deg'))
        
        if hubert_com is not None:
            sw.add_slider_callback(hubert_com.set_position)

    if args.visual_mode and hubert_com is not None:
        fig = None if hubert_model is None else hubert_model.fig
        ax = None if hubert_model is None else hubert_model.ax
        hubert_pose = Hubert3DModel(ax=ax, fig=fig, color='orange', linestyle='--')

        timer = fig.canvas.new_timer(interval=1000)
        timer.add_callback(get_pos_callback)
        timer.start()
    
    if sw is not None:
        sw.draw()
    
    figs = figs = list(map(plt.figure, plt.get_fignums()))
    if len(figs) > 0:
        plt.show()


if __name__ == '__main__':
    main()