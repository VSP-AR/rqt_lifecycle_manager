import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python import get_resource

from python_qt_binding import loadUi, QtGui
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QWidget, QListWidget, QListWidgetItem

from .interactive_graphics_view import ZoomableGraphicsView
from .lifecycle.node_manager import LifecycleNodeListNodeManager
from .lifecycle_drawing import LifecycleDrawing

import rclpy
from rclpy.node import Node

from rqt_gui_py.plugin import Plugin

class RosLifecycleManager(Plugin):

    def __init__(self, context):
        super(RosLifecycleManager, self).__init__(context)
        self.setObjectName('LifecycleManagerPlugin')

        self._node = rclpy.create_node('lifecycle_manager_plugin')
        self._logger = self._node.get_logger().get_child('rqt_lifecycle_manager')

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

        # Initialize the node manager
        self.node_manager = LifecycleNodeListNodeManager(self._node, self._logger)

        # Initialize the drawing manager
        self.drawing_manager = LifecycleDrawing()

        # Connect item selection change signal to the slot
        self._widget.lifecycleNodeList.itemSelectionChanged.connect(self._on_node_selection_changed)

    def draw_state_machine(self, current_state=None, transition_state=None):
        scene = self.drawing_manager.draw_state_machine(current_state, transition_state)
        self.graphicsViewObj.setScene(scene)

    def _refresh_lc_node_list(self):
        self.node_list = self.node_manager.list_lifecycle_nodes()
        self._widget.lifecycleNodeList.clear()
        for node_name in self.node_list:
            item = QListWidgetItem(node_name)
            self._widget.lifecycleNodeList.addItem(item)

    def get_selected_node(self):
        selected_items = self._widget.lifecycleNodeList.selectedItems()
        if selected_items:
            return selected_items[0].text()
        return None

    def _on_node_selection_changed(self):
        node_name = self.get_selected_node()
        if node_name:
            state = self.node_manager.get_lifecycle_state(node_name)
            self._node.get_logger().info(f'Selected node {node_name} is in state: {state.label}')
            self.draw_state_machine(current_state=state.label)

    def _configure_lc_node(self):
        node_name = self.get_selected_node()
        if node_name:
            self._node.get_logger().info(f'Configuring {node_name}')
            self.draw_state_machine('Configuring', 'in-progress')
            result = self.node_manager.set_lifecycle_state(node_name, transition_label='configure')
            transition_state = 'success' if result else 'failed'
            self.draw_state_machine('Configuring', transition_state)

    def _activate_lc_node(self):
        node_name = self.get_selected_node()
        if node_name:
            self._node.get_logger().info(f'Activating {node_name}')
            self.draw_state_machine('Activating', 'in-progress')
            result = self.node_manager.set_lifecycle_state(node_name, transition_label='activate')
            transition_state = 'success' if result else 'failed'
            self.draw_state_machine('Activating', transition_state)

    def _deactivate_lc_node(self):
        node_name = self.get_selected_node()
        if node_name:
            self._node.get_logger().info(f'Deactivating {node_name}')
            self.draw_state_machine('Deactivating', 'in-progress')
            result = self.node_manager.set_lifecycle_state(node_name, transition_label='deactivate')
            transition_state = 'success' if result else 'failed'
            self.draw_state_machine('Deactivating', transition_state)

    def _shutdown_lc_node(self):
        node_name = self.get_selected_node()
        if node_name:
            self._node.get_logger().info(f'Shutting down {node_name}')
            self.draw_state_machine('ShuttingDown', 'in-progress')
            result = self.node_manager.set_lifecycle_state(node_name, transition_label='shutdown')
            transition_state = 'success' if result else 'failed'
            self.draw_state_machine('ShuttingDown', transition_state)

    def _cleanup_lc_node(self):
        node_name = self.get_selected_node()
        if node_name:
            self._node.get_logger().info(f'Cleaning up {node_name}')
            self.draw_state_machine('CleaningUp', 'in-progress')
            result = self.node_manager.set_lifecycle_state(node_name, transition_label='cleanup')
            transition_state = 'success' if result else 'failed'
            self.draw_state_machine('CleaningUp', transition_state)

    def shutdown_plugin(self):
        self._node.destroy_node()
        rclpy.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
