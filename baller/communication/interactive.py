import matplotlib.pyplot as plt

from baller.communication.slider import SliderWindow
from baller.communication.model import Hubert3DModel
from hubert import Servo, Hubert
import time


model = Hubert3DModel()
sw = SliderWindow()
# servos = [
#     Servo([0, 180], [560, 2330]),
#     Servo([0, 180], [750, 2200]),
#     Servo([0, 180], [550, 2400]),
#     Servo([0, 180], [550, 2340]),
#     Servo([0, 180], [950, 2400]),
# ]
servos = [
    Servo([-90, 0, 45], [700, 1600, 2070]),
    Servo([-90, 0], [1350, 2200]),
    Servo([0, 90], [1410, 2400]),
    Servo([-90, 0], [600, 1500]),
    Servo([-90, 0], [1170, 2100]),
]
hubert = Hubert("/dev/cu.usbmodem14101", baudrate=57600, servos=servos, timeout=0.1)
hubert.connect()

# sw.add_slider('j1', 0, 180, 0)
# sw.add_slider('j2', 0, 180, 0)
# sw.add_slider('j3', 0, 180, 0)
# sw.add_slider('j4', 0, 180, 0)
# sw.add_slider('j5', 0, 180, 0)

for i in range(len(servos)):
    sw.add_slider(f'j{i+1}', *servos[i].servo_range(), 0)

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

        curr_time = time.time()
        if curr_time - prev_time > interval:
            prev_time = curr_time
            print(hubert.get_status())
            print(hubert.get_position())
except KeyboardInterrupt:
    pass
