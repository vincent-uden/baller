import matplotlib.pyplot as plt
from argparse import ArgumentParser, Namespace, Action
import sys
from typing import Optional

from baller.communication.hubert import Servo, Hubert
from baller.model.slider import SliderWindow
from baller.model.model import Hubert3DModel, Launcher3DModel, Target3DModel
from baller.inverse_kinematics.ik import target_pos_to_joint_angles, LAUNCH_PLANE_OFFSET
import baller.trajectory_solver.trajectory_solver as ts
from baller.model.pose_model import StaticPose


hubert_com: Optional[Hubert] = None                 # Handles communication with Hubert
hubert_model: Optional[Hubert3DModel] = None        # A 3D model of Hubert
hubert_pose: Optional[Hubert3DModel] = None         # A pose estimator of Huberts actual pose

launcher: Optional[Launcher3DModel] = None          # A model for the trajectory of a projectile from the launcher
target: Optional[Target3DModel] = None              # A target to hit
pose_recorder: Optional[StaticPose] = None          # Record a pose

sw: Optional[SliderWindow] = None                   # Window for sliders


def ik_callback(x: float, y: float, z: float, v0: float, **_) -> None:
    """
    Perform inverse kinematics
    """
    assert hubert_model is not None
    assert launcher is not None
    assert target is not None

    ts.V0 = v0

    target.move_target(x, y, z)
    _joints = hubert_model.get_pose(units='rad')
    j1, j2, j3, dist = target_pos_to_joint_angles(x, y, z, j1=_joints['j1'], j2=_joints['j2'], j3=_joints['j3'])

    if dist > 0.01:
        # Miss with more than 1 cm
        launcher.update_color('r')
    else:
        # Hit
        launcher.update_color('g')

    hubert_model.set_pose(j1=j1, j2=j2, j3=j3, units='rad')
    launcher.move_launcher(target_plane=x)

    if hubert_com is not None:
        joints = hubert_model.get_pose(units='deg')
        hubert_com.set_pose(**joints, units='deg')


def get_pos_callback() -> None:
    assert hubert_com is not None
    assert hubert_pose is not None

    joints = hubert_com.get_pose()
    hubert_pose.set_pose(**joints, units='deg')


def setup_interactive(args: Namespace):
    global hubert_model, hubert_com, sw

    hubert_model = Hubert3DModel()

    sw = SliderWindow()
    for i in range(len(servos)):
        sw.add_slider(f'j{i+1}', *servos[i].servo_range(units='deg'), 0)

    sw.add_slider_callback(lambda **js: hubert_model.set_pose(**js, units='deg'))
    
    if hubert_com is not None:
        sw.add_slider_callback(lambda **joints: hubert_com.set_pose(**joints, units='deg'))
        sw.add_button("launch", [lambda x: hubert_com.launch()])


def setup_target(args: Namespace):
    global hubert_model, hubert_com, sw, launcher, target
    # Target position
    tx = args.x
    ty = args.y
    tz = args.z
    ts.V0 = args.v0

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


def record_pose(posename: str):
    pose_recorder.record(posename)
    pose_recorder.save_pose_dict()


def setup_record(args: Namespace):
    global hubert_model, hubert_com, sw, pose_recorder

    print(args)
    
    hubert_model = Hubert3DModel()
    pose_recorder = StaticPose(hubert_model, posefile=args.outfile)

    sw = SliderWindow()
    for i in range(len(servos)):
        sw.add_slider(f'j{i+1}', *servos[i].servo_range(units='deg'), 0)

    sw.add_slider_callback(lambda **js: hubert_model.set_pose(**js, units='deg'))

    sw.add_button("record pose", [lambda x: record_pose(args.pose)])
    
    if hubert_com is not None:
        sw.add_slider_callback(lambda **joints: hubert_com.set_pose(**joints, units='deg'))


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
    
    subparsers = parser.add_subparsers(title="subcommands", required=True)

    interactive_parser = subparsers.add_parser("interactive", aliases=["i"], help="Create an interactive session were you can control each joint in Hubert")
    interactive_parser.set_defaults(func=setup_interactive)

    target_parser = subparsers.add_parser("target", aliases=["t"], help="Control the placement of a target and move Hubert to aim for it")
    target_parser.set_defaults(func=setup_target)
    target_parser.add_argument('--v0', type=float, default=ts.V0, help="The velocity of the projectile")
    target_parser.add_argument('x', type=float, nargs='?', default=0.4, help="The x-coordinate of the target")
    target_parser.add_argument('y', type=float, nargs='?', default=-LAUNCH_PLANE_OFFSET, help="The y-coordinate of the target")
    target_parser.add_argument('z', type=float, nargs='?', default=0.2, help="The z-coordinate of the target")

    record_parser = subparsers.add_parser("record", aliases=["r", "rec"], help="Record a fixed motion for Hubert")
    record_parser.set_defaults(func=setup_record)
    record_parser.add_argument('pose', help="The name of the motion")
    record_parser.add_argument('-o', '--outfile', default="./pose.yml", help="The output file to save the motion to. If it exists it will append to it")

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

    # Run the correct subcommand
    args.func(args)

    if args.visual_mode:
        if hubert_com is None:
            raise ValueError("Can only use visual mode when connected to Hubert")
        
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