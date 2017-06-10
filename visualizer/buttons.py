from kivy.graphics.vertex_instructions import Rectangle, Triangle
from kivy.uix.widget import Widget


class PlayButton(Widget):
    def initialize(self):
        self.pauseb = [x for x in self.canvas.children if isinstance(x, Rectangle)]
        self.playb = [x for x in self.canvas.children if isinstance(x, Triangle)][0]
        self.canvas.remove(self.playb)
        return self

    def play(self):
        self.canvas.remove(self.playb)
        [self.canvas.add(x) for x in self.pauseb]

    def pause(self):
        self.canvas.add(self.playb)
        [self.canvas.remove(x) for x in self.pauseb]
