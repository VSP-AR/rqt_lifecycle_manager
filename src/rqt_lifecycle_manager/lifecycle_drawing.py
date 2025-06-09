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

        # Define colors with hexadecimal values
        default_color = '#FFFFFF'  # White
        transition_color = '#FFFF00'  # Yellow
        success_color = '#00FF00'  # Green
        failure_color = '#FF0000'  # Red
        # Primary states with light blue fill
        primary_states = {
            'unconfigured': '#ADD8E6',   # light blue
            'inactive': '#ADD8E6',
            'active': '#ADD8E6',
            'finalized': '#ADD8E6'
        }

        # Transition states with light yellow fill
        transition_states = {
            'Configuring': '#FFFFE0',    # light yellow
            'CleaningUp': '#FFFFE0',
            'ShuttingDown': '#FFFFE0',
            'Activating': '#FFFFE0',
            'Deactivating': '#FFFFE0',
            'ErrorProcessing': '#FFFFE0'
        }

        # Add primary states
        for state, color in primary_states.items():
            fillcolor = color
            if state == current_state:
                if transition_state == 'in-progress':
                    fillcolor = transition_color
                elif transition_state == 'success':
                    fillcolor = success_color
                elif transition_state == 'failed':
                    fillcolor = failure_color
            node = pydot.Node(state, style='filled', fillcolor=fillcolor, shape='box')
            dot.add_node(node)

        # Add transition states
        for state, color in transition_states.items():
            fillcolor = color
            if state == current_state:
                if transition_state == 'in-progress':
                    fillcolor = transition_color
                elif transition_state == 'success':
                    fillcolor = success_color
                elif transition_state == 'failed':
                    fillcolor = failure_color
            node = pydot.Node(state, style='filled', fillcolor=fillcolor, shape='ellipse')
            dot.add_node(node)

        # Transitions
        transitions = [
            ('unconfigured', 'Configuring'),
            ('Configuring', 'inactive'),
            ('inactive', 'Activating'),
            ('Activating', 'active'),
            ('active', 'Deactivating'),
            ('Deactivating', 'inactive'),
            ('inactive', 'CleaningUp'),
            ('CleaningUp', 'unconfigured'),
            ('unconfigured', 'ErrorProcessing'),
            ('ErrorProcessing', 'finalized'),
            ('inactive', 'ShuttingDown'),
            ('ShuttingDown', 'finalized'),
            ('active', 'ShuttingDown'),
            ('ShuttingDown', 'finalized'),
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
        data = image.tobytes("raw", "BGRA")
        qimage = QImage(data, image.width, image.height, QImage.Format_RGB32)
        pixmap = QPixmap.fromImage(qimage)
        return pixmap