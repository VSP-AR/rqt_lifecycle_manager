import pydot
from io import BytesIO
from PIL import Image
from python_qt_binding.QtGui import QPixmap, QImage
from python_qt_binding.QtWidgets import QGraphicsScene, QGraphicsPixmapItem

def draw_state_machine(self, current_state=None, transition_state=None):
    self.scene.clear()
    dot = self.create_dot_graph(current_state, transition_state)

    # DEBUG: Print the DOT source code
    print("DOT SOURCE:")
    print(dot.to_string())

    try:
        png_str = dot.create_png()
        image = Image.open(BytesIO(png_str))
        image_qt = self.pil2pixmap(image)
        pixmap_item = QGraphicsPixmapItem(image_qt)
        self.scene.addItem(pixmap_item)
    except Exception as e:
        print("Error rendering state machine image:", e)

    return self.scene
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
        # Set graph direction left-to-right
        dot = pydot.Dot(graph_type='digraph', rankdir='LR')

        # Define colors with hexadecimal values
        default_color = '#FFFFFF'     # White
        transition_color = '#FFFF00'  # Yellow
        success_color = '#00FF00'     # Green
        failure_color = '#FF0000'     # Red

        # Primary states: box shape
        primary_states = {
            'unconfigured': default_color,
            'inactive': default_color,
            'active': default_color,
            'finalized': default_color
        }
       

        # Transition states: ellipse shape
        transition_states = {
            'Configuring': default_color,
            'Activating': default_color,
            'Deactivating': default_color,
            'CleaningUp': default_color,
            'ShuttingDown': default_color,
            'ErrorProcessing': default_color
        }

        # Add primary state nodes
        for state, color in primary_states.items():
            fillcolor = color
            if state == current_state:
                if transition_state == 'in-progress':
                    fillcolor = transition_color
                elif transition_state == 'success':
                    fillcolor = success_color
                elif transition_state == 'failed':
                    fillcolor = failure_color
            node = pydot.Node(state, style='filled', fillcolor=fillcolor,
                              shape='box', fontname='Helvetica', fontsize='10')
            dot.add_node(node)

        # Add transition state nodes
        for state, color in transition_states.items():
            fillcolor = color
            if state == current_state:
                if transition_state == 'in-progress':
                    fillcolor = transition_color
                elif transition_state == 'success':
                    fillcolor = success_color
                elif transition_state == 'failed':
                    fillcolor = failure_color
            node = pydot.Node(state, style='filled', fillcolor=fillcolor,
                              shape='ellipse', fontname='Helvetica', fontsize='10')
            dot.add_node(node)

        # Labeled transitions matching the official lifecycle
        transitions = [
            ('unconfigured', 'Configuring', 'configure()'),
            ('Configuring', 'inactive', 'on_configure() success'),
            ('inactive', 'Activating', 'activate()'),
            ('Activating', 'active', 'on_activate() success'),
            ('active', 'Deactivating', 'deactivate()'),
            ('Deactivating', 'inactive', 'on_deactivate() success'),
            ('inactive', 'CleaningUp', 'cleanup()'),
            ('CleaningUp', 'unconfigured', 'on_cleanup() success'),
            ('inactive', 'ShuttingDown', 'shutdown()'),
            ('active', 'ShuttingDown', 'shutdown()'),
            ('ShuttingDown', 'finalized', 'on_shutdown() success'),
            ('Configuring', 'ErrorProcessing', 'on_configure() fail'),
            ('Activating', 'ErrorProcessing', 'on_activate() fail'),
            ('Deactivating', 'ErrorProcessing', 'on_deactivate() fail'),
            ('CleaningUp', 'ErrorProcessing', 'on_cleanup() fail'),
            ('ShuttingDown', 'ErrorProcessing', 'on_shutdown() fail'),
            ('unconfigured', 'ErrorProcessing', 'error()'),
            ('ErrorProcessing', 'finalized', 'on_error()')
        ]

        for from_state, to_state, label in transitions:
            edge = pydot.Edge(from_state, to_state, label=label,
                              fontname='Helvetica', fontsize='9')
            dot.add_edge(edge)

        return dot

    def pil2pixmap(self, image):
        image = image.convert("RGBA")
        data = image.tobytes("raw", "BGRA")
        qimage = QImage(data, image.width, image.height, QImage.Format_RGB32)
        pixmap = QPixmap.fromImage(qimage)
        return pixmap
