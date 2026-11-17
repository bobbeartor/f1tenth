"""Minimal asynchronous parameter transport using standard ROS 2 services.

Some Humble installations do not ship rclpy.parameter_client. Keep the GUI
compatible without requiring a ROS upgrade or blocking its event loop.
"""
from rcl_interfaces.srv import GetParameters, ListParameters, SetParametersAtomically


class RemoteParameters:
    def __init__(self, node, remote_node):
        prefix = remote_node.rstrip('/')
        self._list = node.create_client(ListParameters, prefix + '/list_parameters')
        self._get = node.create_client(GetParameters, prefix + '/get_parameters')
        self._set = node.create_client(
            SetParametersAtomically, prefix + '/set_parameters_atomically')

    def services_are_ready(self):
        return all(client.service_is_ready() for client in (self._list, self._get, self._set))

    def list_parameters(self, prefixes=None, depth=0):
        request = ListParameters.Request()
        request.prefixes = list(prefixes or [])
        request.depth = depth
        return self._list.call_async(request)

    def get_parameters(self, names):
        request = GetParameters.Request()
        request.names = list(names)
        return self._get.call_async(request)

    def set_parameters_atomically(self, parameters):
        request = SetParametersAtomically.Request()
        request.parameters = [parameter.to_parameter_msg() for parameter in parameters]
        return self._set.call_async(request)
