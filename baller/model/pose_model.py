import os
import yaml
from typing import Optional

from baller.model.hubert import HubertModel

class StaticPose:

    def __init__(self, hubert: HubertModel, posefile: str = "") -> None:
        self.posedict: dict[str, list] = {}
        self.posefile = posefile

        self.hubert = hubert

        self.load_pose_file(posefile)

    def load_pose_file(self, posefile: Optional[str] = None) -> None:
        if posefile is not None:
            self.posefile = posefile
        
        if not os.path.exists(self.posefile):
            return
        
        with open(self.posefile, 'r') as f:
            self.posedict = yaml.safe_load(f)

    def take_pose(self, posename: str) -> None:
        assert self.hubert is not None, "Can not take a pose when disconnected from Hubert"

        if posename not in self.posedict:
            raise KeyError(f"{posename} is not a recogniced pose")
        
        for pose in self.posedict[posename]:
            self.hubert.set_pose(**pose)
            self.hubert.wait_unitl_idle()

    def save_pose_dict(self, posefile: Optional[str] = None) -> None:
        if posefile is not None:
            self.posefile = posefile

        with open(self.posefile, 'w') as f:
            yaml.dump(self.posedict, f)

    def record(self, posename: str, **pose: float) -> None:
        if len(pose) == 0:
            pose = self.hubert.get_pose(units='deg')

        if posename not in self.posedict:
            self.posedict[posename] = [pose]
        else:
            self.posedict[posename].append(pose)
