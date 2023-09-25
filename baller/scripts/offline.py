import matplotlib.pyplot as plt

from baller.model.slider import SliderWindow
from baller.model.model import Hubert3DModel, Launcher3DModel


model = Hubert3DModel()
launcher = Launcher3DModel(model, 1.0, model.ax, model.fig)
sw = SliderWindow()

sw.add_slider('j1', -90, 90, 0)
sw.add_slider('j2', 0, 180, 0)
sw.add_slider('j3', 0, 180, 0)
sw.add_slider('j4', 0, 180, 0)
sw.add_slider('j5', 0, 180, 0)
sw.add_button("Toggle launcher visability", callbacks=[lambda event: launcher.toggle_visability()])

sw.add_slider_callback(model.move_arm)
sw.add_slider_callback(lambda js: launcher.move_launcher())
sw.draw()

plt.show()
