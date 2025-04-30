from launch_ros.actions import Node
from datetime import datetime

# Custom Imports
from launch_interface import LaunchInterface

class Metrics(LaunchInterface):

    def __init__(self, model_name, scene):
        self.__setModelName(model_name)
        self.__setScene(scene)
        super().__init__(name="Metrics")

    ########### SETTERS ###########
    def __setModelName(self, model_name):
        self.__model_name = model_name

    def __setScene(self, scene):
        self.__scene = scene

    ########### GETTERS ###########
    def getModelName(self):
        return self.__model_name
    
    def getScene(self):
        return self.__scene

    ########### METHODS ###########
    def createInterfaceNodes(self):
        hardware_metrics_node = Node(
            package='ros2_benchmark_logger',
            executable='hardware_metrics',
            parameters=[
                {
                    'use_sim_time': False
                }
            ]
        )

        self.addNode(hardware_metrics_node)

        if self.getScene() != "empty":

            topics = [
                {
                    "topic": f"/{self.getModelName()}/camera_sensor_broadcaster/cam_depth/depth/image_raw",
                    "type": "sensor_msgs/msg/Image"
                },
                {
                    "topic": f"/{self.getModelName()}/camera_sensor_broadcaster/cam_depth/points",
                    "type": "sensor_msgs/msg/PointCloud2"
                },
                {
                    "topic": f"/{self.getModelName()}/camera_sensor_broadcaster/cam_rgb/image_raw",
                    "type": "sensor_msgs/msg/Image"
                },
                {
                    "topic": f"/{self.getModelName()}/diff_drive_controller/cmd_vel_unstamped",
                    "type": "geometry_msgs/msg/Twist"
                },
                {
                    "topic": f"/{self.getModelName()}/diff_drive_controller/odom",
                    "type": "nav_msgs/msg/Odometry"
                },
                {
                    "topic": f"/{self.getModelName()}/imu_sensor_broadcaster/imu",
                    "type": "sensor_msgs/msg/Imu"
                },
                {
                    "topic": f"/{self.getModelName()}/joint_states",
                    "type": "sensor_msgs/msg/JointState"
                },
                {
                    "topic": f"/{self.getModelName()}/lidar_sensor_broadcaster/back_laser/scan",
                    "type": "sensor_msgs/msg/LaserScan"
                },    
                {
                    "topic": f"/{self.getModelName()}/lidar_sensor_broadcaster/front_laser/scan",
                    "type": "sensor_msgs/msg/LaserScan"
                },
                # {
                #     "topic": f"/{self.getModelName()}/robot_description",
                #     "type": "std_msgs/msg/String"
                # },
                {
                    "topic": f"/{self.getModelName()}/ultrasound_sensor_broadcaster/left_ultrasound/scan",
                    "type": "sensor_msgs/msg/Range"
                },
                {
                    "topic": f"/{self.getModelName()}/ultrasound_sensor_broadcaster/right_ultrasound/scan",
                    "type": "sensor_msgs/msg/Range"
                },
                {
                    "topic":"/clock",
                    "type": "rosgraph_msgs/msg/Clock"
                },
                {
                    "topic":"/tf",
                    "type": "tf2_msgs/msg/TFMessage"
                },    
                # {
                #     "topic":"/tf_static",
                #     "type": "tf2_msgs/msg/TFMessage"
                # }
            ]

            folder_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

            for topic in topics:
                ros2_metrics_node = Node(
                    package='ros2_benchmark_logger',
                    executable='ros2_metrics',
                    parameters=[
                        {
                            'folder_name': folder_name,
                            'topic_name': topic["topic"],
                            'topic_type': topic["type"],
                            'use_sim_time': False
                        }
                    ]
                )

                self.addNode(ros2_metrics_node)
