"""
pickup_site_node.py
-------------------
WHAT IT DOES: Acts as the "pickup site" server mentioned in the project spec.

The project says: "a node that acts as the pickup site"
This node:
  1. Advertises a service: /confirm_pickup
  2. When called with a zone_id, it confirms the object at that zone
  3. Returns the object name and whether pickup is approved

ANALOGY: Like a warehouse clerk who checks the manifest and says
         "Yes, package #3 is here, you can take it."
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String


# Object manifest: which object is at each zone
# In a real system, this might come from a database or sensor
ZONE_OBJECTS = {
    'zone_1': {'name': 'Red Box',    'marker_id': 0, 'weight': 1.2},
    'zone_2': {'name': 'Blue Cube',  'marker_id': 1, 'weight': 0.8},
    'zone_3': {'name': 'Green Cylinder', 'marker_id': 2, 'weight': 1.5},
}


class PickupSiteNode(Node):
    def __init__(self):
        super().__init__('mc_pickup_site')

        # Service: /confirm_pickup — robot calls this to confirm object identity
        self._confirm_srv = self.create_service(
            Trigger,
            '/confirm_pickup',
            self._confirm_pickup_callback
        )

        # Publisher: broadcasts which object is currently "active" at pickup
        self._object_pub = self.create_publisher(
            String, '/pickup_site/current_object', 10
        )

        # Track current zone (set by main_controller)
        self.current_zone_id = None

        # Timer to periodically broadcast current object info
        self.create_timer(1.0, self._broadcast_current_object)

        self.get_logger().info(
            'PickupSiteNode ready. '
            'Service: /confirm_pickup | '
            f'Known zones: {list(ZONE_OBJECTS.keys())}'
        )

    def set_active_zone(self, zone_id):
        """Called by main_controller to set which zone is being visited."""
        self.current_zone_id = zone_id
        obj = ZONE_OBJECTS.get(zone_id, {})
        self.get_logger().info(
            f'Active zone set to: {zone_id} '
            f'(object: {obj.get("name", "Unknown")})'
        )

    def _confirm_pickup_callback(self, request, response):
        """
        Service callback: confirms the object at the current zone.

        Returns:
            success=True if zone is known and object confirmed
            message=Object name and details
        """
        if self.current_zone_id is None:
            response.success = False
            response.message = 'No active zone set. Call set_active_zone first.'
            return response

        obj = ZONE_OBJECTS.get(self.current_zone_id)
        if obj is None:
            response.success = False
            response.message = f'Unknown zone: {self.current_zone_id}'
            return response

        response.success = True
        response.message = (
            f'Object confirmed: {obj["name"]} | '
            f'Marker ID: {obj["marker_id"]} | '
            f'Weight: {obj["weight"]}kg'
        )
        self.get_logger().info(f'✅ Pickup confirmed: {response.message}')
        return response

    def _broadcast_current_object(self):
        """Periodically publish what object is at the current zone."""
        if self.current_zone_id:
            obj = ZONE_OBJECTS.get(self.current_zone_id, {})
            msg = String()
            msg.data = f"{self.current_zone_id}: {obj.get('name', 'Unknown')}"
            self._object_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PickupSiteNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
