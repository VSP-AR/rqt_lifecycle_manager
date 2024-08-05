import os
import pydot
from io import BytesIO
from PIL import Image
from ament_index_python.packages import get_package_share_directory
from ament_index_python import get_resource

from python_qt_binding import loadUi, QtGui
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QWidget, QGraphicsView, QGraphicsScene, QGraphicsPixmapItem, QListWidget, QListWidgetItem

from .interactive_graphics_view import ZoomableGraphicsView

import rclpy
from rclpy.node import Node

from rqt_gui_py.plugin import Plugin

class RosLifecycleManager(Plugin):

    def __init__(self, context):
        super(RosLifecycleManager, self).__init__(context)
        self.setObjectName('LifecycleManagerPlugin')

        self._node = rclpy.create_node('lifecycle_manager_plugin')
        self._logger = self._node.get_logger().get_child('rqt_lifecycle_manager.ros_graph.LifecycleManagerPlugin')

        self._widget = QWidget()

        _, package_path = get_resource('packages', 'rqt_lifecycle_manager')
        ui_file = os.path.join(package_path, 'share', 'rqt_lifecycle_manager', 'resource', 'RosLifecycleManager.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('LifecycleManagerPluginUi')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        context.add_widget(self._widget)

        # Replace QGraphicsView with ZoomableGraphicsView
        self.graphicsViewObj = ZoomableGraphicsView(self._widget.graphicsViewObj)
        self._widget.graphicsViewObj = self.graphicsViewObj

        self._widget.pushRefresh.clicked.connect(self._refresh_lc_node_list)
        self._widget.pushConfigureObj.clicked.connect(self._configure_lc_node)
        self._widget.pushActivateObj.clicked.connect(self._activate_lc_node)
        self._widget.pushDeactivateObj.clicked.connect(self._deactivate_lc_node)
        self._widget.pushShutdownObj.clicked.connect(self._shutdown_lc_node)
        self._widget.pushCleanObj.clicked.connect(self._cleanup_lc_node)

        self.node_list = []

        self.scene = self.graphicsViewObj._scene
        self.draw_state_machine()

    def draw_state_machine(self):
        self.scene.clear()
        dot = self.create_dot_graph()
        png_str = dot.create_png()
        image = Image.open(BytesIO(png_str))
        image_qt = self.pil2pixmap(image)
        pixmap_item = QGraphicsPixmapItem(image_qt)
        self.scene.addItem(pixmap_item)

    def create_dot_graph(self):
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
            node = pydot.Node(state, style='filled', fillcolor=color, shape=shape)
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
        qimage = QtGui.QImage(data, image.width, image.height, QtGui.QImage.Format_ARGB32)
        pixmap = QtGui.QPixmap.fromImage(qimage)
        return pixmap

    def _refresh_lc_node_list(self):
        # Dummy implementation for refreshing the node list
        self.node_list = ['node1', 'node2', 'node3']
        self._widget.lifecycleNodeList.clear()
        for node_name in self.node_list:
            item = QListWidgetItem(node_name)
            self._widget.lifecycleNodeList.addItem(item)
        
        # Redraw the state machine diagram
        self.draw_state_machine()

    def get_selected_node(self):
        selected_items = self._widget.lifecycleNodeList.selectedItems()
        if selected_items:
            return selected_items[0].text()
        return None

    def _configure_lc_node(self):
        node_name = self.get_selected_node()
        if node_name:
            self._node.get_logger().info(f'Configuring {node_name}')
            self.draw_state_machine()
            # Implement lifecycle transition to configure state here

    def _activate_lc_node(self):
        node_name = self.get_selected_node()
        if node_name:
            self._node.get_logger().info(f'Activating {node_name}')
            self.draw_state_machine()
            # Implement lifecycle transition to activate state here

    def _deactivate_lc_node(self):
        node_name = self.get_selected_node()
        if node_name:
            self._node.get_logger().info(f'Deactivating {node_name}')
            self.draw_state_machine()
            # Implement lifecycle transition to deactivate state here

    def _shutdown_lc_node(self):
        node_name = self.get_selected_node()
        if node_name:
            self._node.get_logger().info(f'Shutting down {node_name}')
            self.draw_state_machine()
            # Implement lifecycle transition to shutdown state here

    def _cleanup_lc_node(self):
        node_name = self.get_selected_node()
        if node_name:
            self._node.get_logger().info(f'Cleaning up {node_name}')
            self.draw_state_machine()
            # Implement lifecycle transition to cleanup state here

    def shutdown_plugin(self):
        self._node.destroy_node()
        rclpy.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
