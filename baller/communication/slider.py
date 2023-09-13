import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.widgets import Slider as pltSlider
from typing import Optional, Callable


SliderRect = tuple[float, float, float, float]


class Slider:

    def __init__(self, name: str, rmin: float, rmax: float, init_value: float = 0) -> None:
        self.name = name
        self.rmin = rmin
        self.rmax = rmax
        self.init_value = init_value

        self.slider_rect: Optional[SliderRect] = None
        self.slider_ax = None
        self.slider: Optional[pltSlider] = None
        
    def create_slider(self, fig: Figure, rect: SliderRect):
        """
        Add a slider to a figure
        """
        self.slider_ax = fig.add_axes(rect)
        self.slider = pltSlider(self.slider_ax, self.name, self.rmin, self.rmax, valinit=self.init_value)

    def add_callback(self, func: Callable):
        if self.slider is None:
            raise RuntimeError("You must create the slider before adding a callback")
        self.slider.on_changed(func)

    @property
    def val(self) -> float:
        if self.slider is None:
            return 0
        return self.slider.val


class SliderWindow:

    def __init__(self) -> None:
        self.fig = plt.figure()

        self.sliders: list[Slider] = []

        self.is_drawn = False

        self.callbacks: list[Callable] = []

    def add_slider(self, name: str, rmin: float, rmax: float, init_value: float = 0):
        """
        Add a slider to the slider window
        Must be called before draw, oterwise a RuntimeError will be raised
        """
        if self.is_drawn:
            raise RuntimeError("Can not add sliders after the window has been drawn")
        
        self.sliders.append(
            Slider(name=name, rmin=rmin, rmax=rmax, init_value=init_value)
        )

    def draw(self):
        """
        Draw the windown with the sliders
        """
        n_sliders = len(self.sliders)
        slider_h = 0.8 / n_sliders
        for i, slider in enumerate(self.sliders):
            slider_y = (n_sliders - i - 1) * slider_h + 0.1
            slider_rect = (0.1, slider_y, 0.8, slider_h)
            slider.create_slider(self.fig, slider_rect)
            slider.add_callback(self._callback)

        self.is_drawn = True

    def add_callback(self, func: Callable):
        """
        Add a callback
        Must be called after draw, oterwise a RuntimeError will be raised
        """
        self.callbacks.append(func)

    def _callback(self, val):
        slider_vals = {s.name: s.val for s in self.sliders}
        for callback in self.callbacks:
            callback(**slider_vals)

    
if __name__ == '__main__':
    sw = SliderWindow()
    sw.add_slider("j1", 0, 180, 90)
    sw.add_slider("j2", 0, 180, 90)
    sw.add_slider("j3", 0, 180, 90)
    sw.draw()

    def callback(j1, j2, j3):
        print(f"J1 = {j1}, J2 = {j2}, J3 = {j3}")

    sw.add_callback(callback)
    plt.show()