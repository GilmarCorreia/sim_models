# Default Imports
import os, re, platform, launch, yaml, time
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnShutdown

# Custom Imports
from launch_simulator import LaunchSimulator

class Gazebo(LaunchSimulator):

    def __init__(self, gazebo_version, model_name, robot_description = None, scene = "empty", enable_headless=False):
        self.__setGazeboVersion(gazebo_version)
        self.default_world = "empty.sdf"
        self.nodesAfterSimStart = []

        super().__init__(name=f"Gazebo_{self.getGazeboVersion()}",
                         model_name=model_name, 
                         robot_description=robot_description,
                         scene=scene,
                         enable_headless=enable_headless
                        )

    ########### SETTERS ###########
    def __setGazeboVersion(self, gazebo_version):
        self.__gazebo_version = gazebo_version

    ########### GETTERS ###########
    def getGazeboVersion(self):
        return self.__gazebo_version

    ########### METHODS ###########
    def createGazeboClassicNodes(self):            
        launch_arguments = {
            "verbose": "true"
        }

        if self.getScene() != "empty":
            launch_arguments["world"] = f"{get_package_share_directory(self.getPkgName())}/worlds/Gazebo/{self.getScene()}.world"

        if self.getEnableHeadless():
            gazebo_launch_file = "gzclient.launch.py"
        else:
            gazebo_launch_file = "gazebo.launch.py"

        node_gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(
                        FindPackageShare('gazebo_ros').find('gazebo_ros'), 'launch', gazebo_launch_file
                    )
                ]
            ),
            launch_arguments = launch_arguments.items()
        )
        self.addNode(node_gazebo)

        # Wait until gazebo has launched
        self.clock_checker = Node(
            package='sim_models',
            executable='clock_waiter',
            name='clock_waiter_gazebo',
            output='screen'
        )

        self.addNode(self.clock_checker)

        if (platform.system() == "Windows"):
            gazebo_spawn_command = f"ros2 run gazebo_ros spawn_entity.py -topic {self.getRobotDescriptionTopicName()} -entity {self.getModelName()}"

            # Launch CoppeliaSim
            node_gazebo_spawn = ExecuteProcess(
                name="urdf_spawner",
                cmd=[gazebo_spawn_command],
                shell=True,
                output='screen'
            )
        else: 
            node_gazebo_spawn = Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                shell=True,
                output='screen',
                arguments=[
                    "-topic", self.getRobotDescriptionTopicName(), 
                    "-entity", self.getModelName(),
                    #"-z", "0.2",
                ]
            )

        #self.addNode(node_gazebo_spawn)
        self.nodesAfterSimStart.append(node_gazebo_spawn)

    def createGazeboNodes(self):
        # Launching Gazebo
        node_gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(
                    FindPackageShare('ros_gz_sim').find('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py'
                )]
            ),
            launch_arguments={'gz_args': f'-r {self.default_world}'}.items()
        )

        self.addNode(node_gazebo)
            
        # Spawning model
        node_gazebo_spawn = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', self.getRobotDescriptionTopicName(),
                '-name', self.getModelName(),
                '-allow_renaming', 'true'
            ],
            output='screen'
        )

        # self.addNode(node_gazebo_spawn)

    def loadingROS2_Control(self):
        # Regular expression pattern to find content between <parameters> and </parameters>, getting the path of the *.yaml file using regex
        pattern = r"<parameters>(.*?)</parameters>"

        # Search for the pattern
        match = re.search(pattern, self.getRobotDescription(), re.DOTALL)

        # Check if a match is found
        if match:
            # Extract the content between <parameters> and </parameters>, path of the *.yaml file
            controllers_yaml_file_path = match.group(1)

            # Open and Read the YAML file from the path
            try:
                with open(controllers_yaml_file_path, 'r') as file:
                    yaml_content = yaml.safe_load(file)

                    # Reading all configured controllers inside /**:controller_manager:ros__parameters from *.yaml file
                    for controller_name, values in yaml_content["/**"]["controller_manager"]["ros__parameters"].items():
                        if(controller_name != "update_rate" and controller_name != "use_sim_time"):

                            token = "filepath"
                            
                            # Spawning controller/broadcaster
                            if(token in values):
                                filepath_value = values[token]

                                package_name = filepath_value[len("package://"):].split('/',1)

                                # Resolve the package path using ament_index_python
                                if (platform.system() == "Windows"):
                                    package_path = get_package_share_directory(package_name[0]) + "\\" + package_name[1]
                                else:
                                    package_path = get_package_share_directory(package_name[0]) + "/" + package_name[1]
                                
                                node_ros2_control = Node(
                                    package='controller_manager',
                                    executable='spawner',
                                    #namespace=model_name,
                                    arguments=[controller_name, "-p", package_path, "-c", '/'+self.getModelName()+"/controller_manager"],
                                    output='screen',
                                )
                            else:
                                node_ros2_control = Node(
                                    package='controller_manager',
                                    executable='spawner',
                                    #namespace=model_name,
                                    arguments=[controller_name, "-c", '/'+self.getModelName()+"/controller_manager"],
                                    output='screen'
                                )

                            #self.addNode(node_ros2_control)
                            self.nodesAfterSimStart.append(node_ros2_control)

            except FileNotFoundError:
                print(f"File not found: {controllers_yaml_file_path}")
            except yaml.YAMLError as exc:
                print("Error parsing YAML content:", exc)

    def createInterfaceNodes(self):
        # Launch Gazebo
        if(self.getGazeboVersion()=="classic"):
            self.createGazeboClassicNodes()
        elif(self.getGazeboVersion()=="latest"):
            self.createGazeboNodes()
	
        # ===============================================================
        # ==== Loading controllers and broadcaster from ROS2 Control ====
        # ===============================================================
        self.loadingROS2_Control()

        # Event to launch nodes after the clock_checker process exits
        clock_event_handler = launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=self.clock_checker,
                on_exit=self.nodesAfterSimStart,
            )
        )
        self.addNode(clock_event_handler)
