import matplotlib.pyplot as plt

from baller.communication.slider import SliderWindow
from baller.communication.model import Hubert3DModel

model = Hubert3DModel()
sw = SliderWindow()
sw.add_slider('j1', 0, 180, 0)
sw.add_slider('j2', 0, 180, 0)
sw.add_slider('j3', 0, 90, 0)
sw.add_callback(model.move_arm)
sw.draw()

plt.show()