import pydot
from io import BytesIO
from PIL import Image
from python_qt_binding.QtGui import QPixmap, QImage
from python_qt_binding.QtWidgets import QGraphicsScene, QGraphicsPixmapItem

class LifecycleDrawing:
    def __init__(self):
        self.scene = QGraphicsScene()

    def draw_state_machine(self, current_state=None, transition_state=None):
        self.scene.clear()
        dot = self.create_dot_graph(current_state, transition_state)
        png_str = dot.create_png()
        image = Image.open(BytesIO(png_str))
        image_qt = self.pil2pixmap(image)
        pixmap_item = QGraphicsPixmapItem(image_qt)
        self.scene.addItem(pixmap_item)
        return self.scene

    def create_dot_graph(self, current_state=None, transition_state=None):
        dot = pydot.Dot(graph_type='digraph')

        # States with colors and shapes
        states = {
            'Unconfigured': ('green', 'box'),
            'Inactive': ('green', 'box'),
            'Active': ('green', 'box'),
            'Finalized': ('green', 'box'),
            'Configuring': ('yellow', 'box'),
            'CleaningUp': ('yellow', 'box'),
            'ShuttingDown': ('yellow', 'box'),
            'ErrorProcessing': ('red', 'box'),
            'Activating': ('yellow', 'ellipse'),  # Oval shape for "Activating"
            'Deactivating': ('yellow', 'box')
        }

        for state, (color, shape) in states.items():
            fillcolor = 'red' if state == current_state and transition_state == 'failed' else color
            node = pydot.Node(state, style='filled', fillcolor=fillcolor, shape=shape)
            dot.add_node(node)

        # Transitions
        transitions = [
            ('Unconfigured', 'Configuring'),
            ('Configuring', 'Inactive'),
            ('Inactive', 'Activating'),
            ('Activating', 'Active'),
            ('Active', 'Deactivating'),
            ('Deactivating', 'Inactive'),
            ('Inactive', 'CleaningUp'),
            ('CleaningUp', 'Unconfigured'),
            ('Inactive', 'ShuttingDown'),
            ('Active', 'ShuttingDown'),
            ('ShuttingDown', 'Finalized'),
            ('Configuring', 'ErrorProcessing'),
            ('Activating', 'ErrorProcessing'),
            ('Deactivating', 'ErrorProcessing'),
            ('CleaningUp', 'ErrorProcessing'),
            ('ShuttingDown', 'ErrorProcessing')
        ]

        for from_state, to_state in transitions:
            edge = pydot.Edge(from_state, to_state)
            dot.add_edge(edge)

        return dot

    def pil2pixmap(self, image):
        image = image.convert("RGBA")
        data = image.tobytes("raw", "RGBA")
        qimage = QImage(data, image.width, image.height, QImage.Format_ARGB32)
        pixmap = QPixmap.fromImage(qimage)
        return pixmap
