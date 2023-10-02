import cv2
from dataclasses import dataclass

from baller.communication.hubert import Hubert
from baller.image_analysis.image_analysis import get_target_position
from baller.image_analysis.pixel_coordinates_to_spatial import pixel_to_spatial
from baller.inverse_kinematics.ik import target_pos_to_joint_angles


@dataclass
class Target:
    x: float
    y: float
    z: float


class FSM:

    def __init__(self, hubert: Hubert, target_plane: float) -> None:
        self.camera = cv2.VideoCapture(0)
        
        self.hubert = hubert
        self.target_plane = target_plane

        self.targets: list[Target] = []


    def targeting(self):
        ret, frame = self.camera.read()
        if not ret:
            raise RuntimeError("Could not read frame")

        ys, zs = get_target_position(frame)
        self.targets = []

        for py, pz in zip(ys, zs):
            _, y, z = pixel_to_spatial(py, pz)
            self.targets.append(Target(self.target_plane, y, z))

            cv2.circle(frame, (int(py), int(pz)), 5, (255, 0, 0))

        cv2.imshow("frame", frame)
        cv2.waitKey(100)

    def shooting(self):
        for t in self.targets:
            pose = self.hubert.get_pose(units='rad')
            print(t.x, t.y, t.z)
            new_pose = target_pos_to_joint_angles(t.x, t.y, t.z, j1=pose['j1'], j2=pose['j2'], j3=pose['j3'])
            self.hubert.set_pose(j1=new_pose[0], j2=new_pose[1], j3=new_pose[2], units='rad')
            self.hubert.wait_unitl_idle()
            self.hubert.launch()
            self.hubert.wait_unitl_idle()

    def run(self):
        while True:
            self.hubert.set_pose(j1=0, j2=0, j3=0)
            self.hubert.wait_unitl_idle()

            print("targeting")
            self.targeting()
            print(f"Shooting: {', '.join(f'({t.y}, {t.z})' for t in self.targets)}")
            self.shooting()



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