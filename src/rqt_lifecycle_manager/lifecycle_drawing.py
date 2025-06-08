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

        

        # Primary states with default colors and shapes (box)
        primary_states = {
            'unconfigured': '#add8e6',    
            'inactive': '#add8e6',      
            'active': '#add8e6',          
            'finalized': '#add8e6'        
        }

        # Transition states with default colors and shapes (ellipse)
        transition_states = {
           'configuring': '#FFFFE0',
            'activating': '#FFFFE0',
            'deactivating': '#FFFFE0',
            'cleaningup': '#FFFFE0',
            'shuttingdown': '#FFFFE0',
            'errorprocessing': '#FFFFE0'
        }
        
         # Highlighting colors
        transition_color = '#90ee90'
        success_color = '#90ee90'
        failure_color = '#90ee90'

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