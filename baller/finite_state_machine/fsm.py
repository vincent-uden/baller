import cv2
from dataclasses import dataclass
from enum import Enum, IntEnum, auto

from baller.communication.hubert import Hubert
from baller.image_analysis.image_analysis import get_target_position
from baller.image_analysis.pixel_coordinates_to_spatial import pixel_to_spatial
from baller.inverse_kinematics.ik import target_pos_to_joint_angles


@dataclass
class Target:
    x: float
    y: float
    z: float


class VerbosityLevel(IntEnum):
    Error = 0
    Info = auto()
    Debug = auto()


class InteractivityLevel(IntEnum):
    Autonomous = 0
    Assisted = auto()
    Manual = auto()


class OperationState(Enum):
    IDLE = auto()
    CHECK_MAGAZINE = auto()
    RELOADING = auto()
    TARGETING = auto()
    SHOOTING = auto()


class QuitMainLoop(Exception):
    pass


class FSM:

    def __init__(self, hubert: Hubert, target_plane: float, interactive: int = 0, verbose: int = 0) -> None:
        self.camera = cv2.VideoCapture(0)
        
        self.hubert = hubert
        self.target_plane = target_plane

        self.targets: list[Target] = []

        self.interactive = interactive
        self.verbose = verbose

        # Variables
        self.magazine_count = 0

        self.state = OperationState.IDLE

    def targeting(self):
        # Reset the pose
        self.hubert.set_pose(j1=0, j2=0, j3=0)
        self.hubert.wait_unitl_idle()

        ret, frame = self.camera.read()
        if not ret:
            raise RuntimeError("Could not read frame")

        ys, zs = get_target_position(frame)
        self.targets = []

        for py, pz in zip(ys, zs):
            _, y, z = pixel_to_spatial(py, pz)
            self.targets.append(Target(self.target_plane, y, z))

            cv2.circle(frame, (int(py), int(pz)), 5, (255, 0, 0))

        if self.verbose <= VerbosityLevel.Info:
            # If we are above the lowest verbosity level
            cv2.imshow("frame", frame)
            cv2.waitKey(100)
        
        self._print(
            f"Found targets at: {self.targets}",
            VerbosityLevel.Debug,
        )

    def run(self):
        """
        Run the main loop
        """
        try:
            self._loop()
        except (KeyboardInterrupt, QuitMainLoop):
            self._print("Quiting main loop", verbosity_level=VerbosityLevel.Info)
            if self._wait_for_interaction("Continue running? yes/no").lower() == "yes":
                self.run()
            else:
                print("Quiting Hubert")

    def _loop(self):
        while True:
            if self.state == OperationState.IDLE:
                self.idle()
                self.state = OperationState.CHECK_MAGAZINE

            if self.state == OperationState.CHECK_MAGAZINE:
                self.check_magazine()
                if self.magazine_count > 0:
                    self.state = OperationState.TARGETING
                else:
                    self.state = OperationState.RELOADING
            
            if self.state == OperationState.RELOADING:
                self.reloading()
                self.state = OperationState.CHECK_MAGAZINE

            if self.state == OperationState.TARGETING:
                if len(self.targets) == 0:
                    self.targeting()
                self.state = OperationState.SHOOTING

            if self.state == OperationState.SHOOTING:
                if self.magazine_count == 0:
                    self.state = OperationState.CHECK_MAGAZINE
                    continue
                
                if len(self.targets) == 0:
                    self.state = OperationState.TARGETING
                    continue

                self.shoot()

    def idle(self):
        """
        Wait for a startup signal
        """
        self._wait_for_interaction(
            "Press enter to start Hubert",
            interactivity_level=InteractivityLevel.Autonomous,
        )
    
    def reloading(self):
        nb_shoots = self._wait_for_interaction("Reload and input number of shoots", interactivity_level=InteractivityLevel.Autonomous)
        try:
            self.magazine_count = int(nb_shoots)
        except ValueError:
            print(f"{nb_shoots} is not a valid interger. Please try again")
            self.reloading()

    def check_magazine(self):
        """
        Check the magazine
        """
        pass

    def shoot(self):
        """
        Shoot one shot
        """
        self.magazine_count -= 1

        target = self.targets.pop()

        self._print(
            f"Going for target at: {target}",
            VerbosityLevel.Debug,
        )
        pose = self.hubert.get_pose(units='rad')
        new_pose = target_pos_to_joint_angles(target.x, target.y, target.z, j1=pose['j1'], j2=pose['j2'], j3=pose['j3'])

        self._print(
            f"Setting pose to: j1 = {new_pose[0]}, j2 = {new_pose[1]}, j3 = {new_pose[2]}",
            verbosity_level=VerbosityLevel.Debug,
        )

        self._wait_for_interaction(interactivity_level=InteractivityLevel.Manual)

        self.hubert.set_pose(j1=new_pose[0], j2=new_pose[1], j3=new_pose[2], units='rad')
        self.hubert.wait_unitl_idle()
        
        self._wait_for_interaction(
            "Ready to launch",
            interactivity_level=InteractivityLevel.Assisted,
        )

        self.hubert.launch()
        self.hubert.wait_unitl_idle()

    def _print(self, msg: str, verbosity_level: VerbosityLevel = VerbosityLevel.Error):
        """
        Print msg if the verbosity level is less is high enough
        """
        if self.verbose <= verbosity_level:
            print(msg)

    def _wait_for_interaction(self, msg: str = "Next", interactivity_level: InteractivityLevel = InteractivityLevel.Autonomous, default: str = "") -> str:
        """
        Wait for human interaction if interactivity level is high enough
        """
        if self.interactive <= interactivity_level:
            ans = input(msg + ": ")
            if ans == 'q':
                raise QuitMainLoop
            return ans
            
        return default



if __name__ == '__main__':
    v = cv2.VideoCapture(0)

    while True:

        ret, frame = v.read()

        print(frame.shape)
        cv2.imshow("frame", frame)

        cv2.imwrite("hi.png", frame)

        if cv2.waitKey(10) & 0xff == ord('q'):
            break

        break