import os
from ament_index_python import get_resource

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

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
        self._widget.graphicsView = self.graphicsViewObj

        # Add emoji text to buttons after UI load
        self._widget.pushRefresh.setText("🔄 Refresh")
        self._widget.pushShutdownObj.setText("⏻ Shutdown")

        # Connect button signals
        self._widget.pushRefresh.clicked.connect(self._refresh_lc_node_list)
        self._widget.pushConfigureObj.clicked.connect(self._configure_lc_node)
        self._widget.pushActivateObj.clicked.connect(self._activate_lc_node)
        self._widget.pushDeactivateObj.clicked.connect(self._deactivate_lc_node)
        self._widget.pushShutdownObj.clicked.connect(self._shutdown_lc_node)
        self._widget.pushCleanObj.clicked.connect(self._cleanup_lc_node)

        # Initially disable some buttons
        self._widget.pushConfigureObj.setEnabled(False)
        self._widget.pushActivateObj.setEnabled(False)
        self._widget.pushDeactivateObj.setEnabled(False)
        self._widget.pushCleanObj.setEnabled(False)

        self.node_list = []

        # Initialize node and drawing managers
        self.node_manager = LifecycleNodeListNodeManager(self._node, self._logger)
        self.drawing_manager = LifecycleDrawing()
        self.draw_state_machine()

        # Connect dropdown selection change signal
        self._widget.lifecycleNodeList.currentIndexChanged.connect(self._on_node_selection_changed)

    def draw_state_machine(self, current_state=None, transition_state=None):
        self._logger.info(f'Drawing state machine: current_state={current_state}, transition_state={transition_state}')
        scene = self.drawing_manager.draw_state_machine(current_state, transition_state)
        self.graphicsViewObj.setScene(scene)

    def _refresh_lc_node_list(self):
        self._logger.info('Refreshing lifecycle node list')
        self.node_list = self.node_manager.list_lifecycle_nodes()
        self._logger.info(f'Found lifecycle nodes: {self.node_list}')
        self._widget.lifecycleNodeList.clear()
        self._widget.lifecycleNodeList.addItems(self.node_list)

        # Reset button state
        self._widget.pushConfigureObj.setEnabled(False)
        self._widget.pushActivateObj.setEnabled(False)
        self._widget.pushDeactivateObj.setEnabled(False)
        self._widget.pushCleanObj.setEnabled(False)

    def get_selected_node(self):
        node_name = self._widget.lifecycleNodeList.currentText()
        if node_name:
            self._logger.info(f'Selected node: {node_name}')
            return node_name
        return None

    def _on_node_selection_changed(self, index):
        node_name = self.get_selected_node()
        if node_name:
            state = self.node_manager.get_lifecycle_state(node_name)
            if state is None:
                self._logger.warn(f'No lifecycle state found for node {node_name}')
                # Disable buttons as fallback
                self._widget.pushConfigureObj.setEnabled(False)
                self._widget.pushActivateObj.setEnabled(False)
                self._widget.pushDeactivateObj.setEnabled(False)
                self._widget.pushCleanObj.setEnabled(False)
                return

            self._logger.info(f'Selected node {node_name} is in state: {state.label}')
            self.draw_state_machine(current_state=state.label, transition_state='success')

            # Enable/disable buttons based on lifecycle state
            self._widget.pushConfigureObj.setEnabled(state.label == 'unconfigured')
            self._widget.pushActivateObj.setEnabled(state.label == 'inactive')
            self._widget.pushDeactivateObj.setEnabled(state.label == 'active')
            self._widget.pushCleanObj.setEnabled(state.label == 'inactive')

        else:
            # Disable buttons if no valid selection
            self._widget.pushConfigureObj.setEnabled(False)
            self._widget.pushActivateObj.setEnabled(False)
            self._widget.pushDeactivateObj.setEnabled(False)
            self._widget.pushCleanObj.setEnabled(False)

    def _configure_lc_node(self):
        node_name = self.get_selected_node()
        if node_name:
            self._node.get_logger().info(f'Configuring {node_name}')
            self.draw_state_machine('Configuring', 'in-progress')
            result = self.node_manager.set_lifecycle_state(node_name, transition_label='configure')
            state = self.node_manager.get_lifecycle_state(node_name)

            if result:
                self.draw_state_machine(state.label, 'success')
            else:
                self.draw_state_machine(state.label, 'failed')

    def _activate_lc_node(self):
        node_name = self.get_selected_node()
        if node_name:
            self._node.get_logger().info(f'Activating {node_name}')
            self.draw_state_machine('Activating', 'in-progress')
            result = self.node_manager.set_lifecycle_state(node_name, transition_label='activate')
            state = self.node_manager.get_lifecycle_state(node_name)

            if result:
                self.draw_state_machine(state.label, 'success')
            else:
                self.draw_state_machine(state.label, 'failed')

    def _deactivate_lc_node(self):
        node_name = self.get_selected_node()
        if node_name:
            self._node.get_logger().info(f'Deactivating {node_name}')
            self.draw_state_machine('Deactivating', 'in-progress')
            result = self.node_manager.set_lifecycle_state(node_name, transition_label='deactivate')
            state = self.node_manager.get_lifecycle_state(node_name)

            if result:
                self.draw_state_machine(state.label, 'success')
            else:
                self.draw_state_machine(state.label, 'failed')

    def _shutdown_lc_node(self):
        node_name = self.get_selected_node()
        if node_name:
            self._node.get_logger().info(f'Shutting down {node_name}')
            self.draw_state_machine('ShuttingDown', 'in-progress')
            result = self.node_manager.set_lifecycle_state(node_name, transition_label='shutdown')
            state = self.node_manager.get_lifecycle_state(node_name)

            if result:
                self.draw_state_machine(state.label, 'success')
            else:
                self.draw_state_machine(state.label, 'failed')

    def _cleanup_lc_node(self):
        node_name = self.get_selected_node()
        if node_name:
            self._node.get_logger().info(f'Cleaning up {node_name}')
            self.draw_state_machine('CleaningUp', 'in-progress')
            result = self.node_manager.set_lifecycle_state(node_name, transition_label='cleanup')
            state = self.node_manager.get_lifecycle_state(node_name)

            if result:
                self.draw_state_machine(state.label, 'success')
            else:
                self.draw_state_machine(state.label, 'failed')

    def shutdown_plugin(self):
        self._node.destroy_node()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
