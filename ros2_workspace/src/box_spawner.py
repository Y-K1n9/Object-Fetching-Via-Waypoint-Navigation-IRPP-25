import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import random

class ObstacleSpawner(Node):
    def __init__(self):
        super().__init__('box_spawner_node')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('Waiting for Service....')
        self.count = 0
        self.occupied_locations = []
        self.timer = self.create_timer(5.0, self.spawn_box)
        self.spawn_box()

    
    def spawn_box(self):
        if self.count >=10:
            self.timer.cancel()
            return
        # this grabs a fresh request.
        request = SpawnEntity.Request()
        # name for box: id is no. of boxes spawned till this box
        request.name = f"box_no._{self.count}"

        found = False
        final_x = 0.0
        final_y = 0.0
        while not found:
            x = float(random.uniform(-8.0, 8.0))
            y = float(random.uniform(-8.0, 8.0))
            overlap = any(abs(x-ux)<0.3 and abs(y-uy)<0.3 for ux, uy in (self.occupied_locations))
            if not overlap: 
                found = True
                self.occupied_locations.append((x,y))
                final_x = x
                final_y = y
        request.xml = '''
            <sdf version = "1.6">
                <model name = "box">
                    <link name = "link">
                        <collision name = "c">
                            <geometry>
                                <box>
                                    <size>
                                        0.1 0.1 0.2
                                    </size>
                                </box>
                            </geometry>
                        </collision>
                        <visual name = "v">
                            <geometry>
                                <box>
                                    <size>
                                        0.1 0.1 0.2
                                    </size>
                                </box>
                            </geometry>
                        </visual>
                    </link>
                </model>
            </sdf>
        '''
        request.initial_pose.position.x = final_x
        request.initial_pose.position.y = final_y
        request.initial_pose.position.z = float(0.06)
        self.client.call_async(request)
        self.get_logger().info(f"Spawned: {request.name}")
        self.count+=1

def main():
    rclpy.init()
    node = ObstacleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__': main()