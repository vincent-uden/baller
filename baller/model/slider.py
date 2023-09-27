import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.widgets import Slider as pltSlider, Button as pltButton, Widget as pltWidget
from typing import Optional, Callable
from abc import ABC, abstractmethod


WidgetRect = tuple[float, float, float, float]


class Widget(ABC):

    def __init__(self, name: str, fig: Figure) -> None:
        self.name = name
        self.fig = fig
        self.widget_ax: Optional[plt.Axes] = None

    def create_widget(self, rect: WidgetRect):
        """
        Add an axes for the widget
        """
        self.widget_ax = self.fig.add_axes(rect)

    @abstractmethod
    def add_callback(self, func: Callable):
        pass


class Slider(Widget):

    def __init__(self, fig, name: str, rmin: float, rmax: float, init_value: float = 0) -> None:
        super().__init__(name, fig)

        self.rmin = rmin
        self.rmax = rmax
        self.init_value = init_value

        self.slider: Optional[pltSlider] = None
        
    def create_widget(self, rect: WidgetRect):
        """
        Add a slider to a figure
        """
        super().create_widget(rect)
        assert self.widget_ax is not None, "Failed to create widget"
        self.slider = pltSlider(self.widget_ax, self.name, self.rmin, self.rmax, valinit=self.init_value)

    def add_callback(self, func: Callable):
        if self.slider is None:
            raise RuntimeError("You must create the slider before adding a callback")
        self.slider.on_changed(func)

    @property
    def val(self) -> float:
        if self.slider is None:
            return 0
        return self.slider.val
    

class Button(Widget):

    def __init__(self, name: str, fig: Figure) -> None:
        super().__init__(name, fig)
        self.button: Optional[pltButton] = None
        
    def create_widget(self, rect: WidgetRect):
        """
        Add a button to a figure
        """
        super().create_widget(rect)
        assert self.widget_ax is not None, "Failed to create widget"
        self.button = pltButton(self.widget_ax, self.name)

    def add_callback(self, func: Callable):
        if self.button is None:
            raise RuntimeError("You must create the slider before adding a callback")
        self.button.on_clicked(func)

class SliderWindow:

    def __init__(self) -> None:
        self.fig = plt.figure()

        self.widgets: list[tuple[Widget, list[Callable]]] = []

        self.is_drawn = False

        self.slider_callbacks: list[Callable] = []

    def add_slider(self, name: str, rmin: float, rmax: float, init_value: float = 0, callbacks: list[Callable] = list()):
        """
        Add a slider to the slider window
        Must be called before draw, oterwise a RuntimeError will be raised
        """
        if self.is_drawn:
            raise RuntimeError("Can not add sliders after the window has been drawn")
        
        callbacks.append(self._slider_callback)
        
        self.widgets.append(
            (
                Slider(fig=self.fig, name=name, rmin=rmin, rmax=rmax, init_value=init_value),
                callbacks,
            )
        )

    def add_button(self, label: str, callbacks: list[Callable] = list()):
        if self.is_drawn:
            raise RuntimeError("Can not add sliders after the window has been drawn")
        
        self.widgets.append(
            (
                Button(name=label, fig=self.fig),
                callbacks,
            )
        )

    def draw(self):
        """
        Draw the windown with the sliders
        """
        n_widgets = len(self.widgets)
        widget_h = 0.8 / n_widgets
        for i, (widget, callbacks) in enumerate(self.widgets):
            widget_y = (n_widgets - i - 1) * widget_h + 0.1
            widget_rect = (0.1, widget_y, 0.8, widget_h)
            widget.create_widget(widget_rect)
            for callback in callbacks:
                widget.add_callback(callback)

        self.is_drawn = True

    def add_slider_callback(self, func: Callable):
        """
        Add a callback
        Must be called after draw, oterwise a RuntimeError will be raised
        When calling the callback all slider values will be given as arguments by unpacking
        a dictionary with keys given by the slider name and values being the slider values.

        Ex:
            Given a slider window with 3 sliders "x", "y" and "z" the callsignature should be
            func(x: float, y: float, z: float, **_)
            The extra **_ is optional but recomended if you add more sliders for some reason
        """
        self.slider_callbacks.append(func)

    def _slider_callback(self, val):
        slider_vals = {s.name: s.val for s, _ in self.widgets if isinstance(s, Slider)}
        for callback in self.slider_callbacks:
            callback(**slider_vals)

    
if __name__ == '__main__':
    sw = SliderWindow()
    sw.add_slider("j1", 0, 180, 90)
    sw.add_slider("j2", 0, 180, 90)
    sw.add_slider("j3", 0, 180, 90)
    sw.draw()

    def callback(j1, j2, j3):
        print(f"J1 = {j1}, J2 = {j2}, J3 = {j3}")

    sw.add_slider_callback(callback)
    plt.show()