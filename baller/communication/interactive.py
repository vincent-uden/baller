import matplotlib.pyplot as plt

from baller.communication.slider import SliderWindow
from baller.communication.model import Hubert3DModel
from hubert import Servo, Hubert
import time


model = Hubert3DModel()
sw = SliderWindow()
servos = [
    Servo([0, 180], [560, 2330]),
    Servo([0, 180], [750, 2200]),
    Servo([0, 180], [550, 2400]),
    Servo([0, 180], [550, 2340]),
    Servo([0, 180], [950, 2400]),
]
hubert = Hubert("COM3", baudrate=57600, servos=servos)
hubert.connect()

sw.add_slider('j1', 0, 180, 0)
sw.add_slider('j2', 0, 180, 0)
sw.add_slider('j3', 0, 180, 0)
sw.add_slider('j4', 0, 180, 0)
sw.add_slider('j5', 0, 180, 0)
sw.add_callback(model.move_arm)
sw.add_callback(hubert.set_position)
sw.draw()

plt.show(block=False)

# Get a list of all figures
figs = list(map(plt.figure, plt.get_fignums()))
prev_time = time.time()
interval = 1
try:
    while True:
        for fig in figs:
            fig.canvas.flush_events()

        # curr_time = time.time()
        # if curr_time - prev_time > interval:
        #     prev_time = curr_time
        #     print(hubert.get_status())
        #     print(hubert.get_position())
except KeyboardInterrupt:
    pass
