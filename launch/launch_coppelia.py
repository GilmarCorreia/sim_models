# Default Imports
import os, re, platform
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import launch

# Custom Imports
from launch_simulator import LaunchSimulator

class CoppeliaSim(LaunchSimulator):

    def __init__(self, model_name, robot_description=None, scene = "empty", enable_headless=False):
        super().__init__(name="CoppeliaSim", 
                         model_name=model_name, 
                         robot_description=robot_description,
                         scene=scene,
                         enable_headless=enable_headless
                        )

    ########### METHODS ###########
    
    def createInterfaceNodes(self):
        
        # Command to launch CoppeliaSim
        coppelia_sim_command = f"\"{os.environ['COPPELIASIM_ROOT_DIR']}"
        #model_filename = re.search(r'<coppelia_file\s+filename="([^"]+)"\s*/>', self.getRobotDescription()).group(1)

        if (platform.system() == "Windows"):
            coppelia_sim_command += "\coppeliaSim.exe\" "
        else:
            coppelia_sim_command += "/coppeliaSim.sh\" "

        if self.getTotalTime() > 0:    
            coppelia_sim_command += f"-s{self.getTotalTime()*1000} -q "

        # if self.getScene() != "empty":
        #     if self.getScene() == "default":
        #         coppelia_sim_command += f"-f\"{model_filename}\" "
        #     else:
        if (platform.system() == "Windows"):
            scene_path = f"{get_package_share_directory(self.getPkgName())}\\worlds\\CoppeliaSim\\{self.getScene()}.ttt"
        else:
            scene_path = f"{get_package_share_directory(self.getPkgName())}/worlds/CoppeliaSim/{self.getScene()}.ttt"
        coppelia_sim_command += f"-f\"{scene_path}\" "

        if self.getEnableLogger():
            if (platform.system() == "Windows"):
                plugins_path = f"{get_package_share_directory(self.getPkgName())}\\launch\\coppelia_plugins.lua"
                coppelia_sim_command += f"-c\"require('simLogger'); require('simROSManager')\""
            else:
                plugins_path = f"{get_package_share_directory(self.getPkgName())}/launch/coppelia_plugins.lua"
                coppelia_sim_command += f"-c\'dofile(\"{plugins_path}\")\' "
        
        if self.getEnableHeadless():
            coppelia_sim_command += "-H "
        
        # Launch CoppeliaSim
        launch_coppelia_sim = ExecuteProcess(
            name="coppelia_sim_launcher",
            cmd=[coppelia_sim_command],
            shell=True,
            output='screen'
        )

        self.addNode(launch_coppelia_sim)

        if self.getTotalTime() > 0: 
            if (platform.system() == "Windows"):
                exit_event_handler = RegisterEventHandler(
                    OnProcessExit(
                        target_action=launch_coppelia_sim,
                        on_exit=[
                            launch.actions.LogInfo(msg="Close the Process with Ctrl+C.")
                        ]
                    )
                )
            else:
                exit_event_handler = RegisterEventHandler(
                    OnProcessExit(
                        target_action=launch_coppelia_sim,
                        on_exit=[
                            launch.actions.LogInfo(msg="Close the Process with Ctrl+C."),
                            launch.actions.Shutdown()
                        ]
                    )
                )

            self.addNode(exit_event_handler)
            
            # Verify if clock is active
            clock_checker = Node(
                package='sim_models',
                executable='clock_waiter',
                name='clock_waiter_coppeliasim',
                output='screen'
            )

            self.addNode(clock_checker)

            shutdown_timer = TimerAction(
                period=float(self.getTotalTime()),  # Tempo em segundos (1 segundo)
                actions=[
                    launch.actions.LogInfo(msg="Coppelia closed. Shutting down other nodes..."),
                    Node(
                        package='sim_ros_manager',
                        executable='stop_sim',
                        output='screen'
                    )
                ]
            )
            #self.addNode(shutdown_timer)

            # Event to launch nodes after the clock_checker process exits
            clock_event_handler = launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=clock_checker,
                    on_exit=[shutdown_timer],
                )
            )
            self.addNode(clock_event_handler)