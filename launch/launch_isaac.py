# Default Imports
import omni.usd
import omni.kit.app
import omni.timeline

import os, re, platform
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
import launch

# Custom Imports
from launch_simulator import LaunchSimulator

class IsaacSim(LaunchSimulator):

    def __init__(self, model_name, robot_description=None, scene = "empty", enable_headless=False):
        super().__init__(name="IsaacSim", 
                         model_name=model_name, 
                         robot_description=robot_description,
                         scene=scene,
                         enable_headless=enable_headless
                        )

    ########### METHODS ###########
    def createInterfaceNodes(self):
        
        # # Command to launch CoppeliaSim
        # coppelia_sim_command = f"\"{os.environ['COPPELIASIM_ROOT_DIR']}"
        # model_filename = re.search(r'<coppelia_file\s+filename="([^"]+)"\s*/>', self.getRobotDescription()).group(1)

        # if (platform.system() == "Windows"):
        #     coppelia_sim_command += "\coppeliaSim.exe\" "
        # else:
        #     coppelia_sim_command += "/coppeliaSim.sh\" "

        # if self.getTotalTime() > 0:    
        #     coppelia_sim_command += f"-s{self.getTotalTime()*1000} -q "

        # Loading Scene in Isaac Sim
        scene_path = f"{get_package_share_directory(self.getPkgName())}/worlds/IsaacSim/{self.getScene()}.usd"
        omni.usd.get_context().open_stage(scene_path)

        #     if self.getScene() == "default":
        #         coppelia_sim_command += f"-f\"{model_filename}\" "
        #     else:
        #         scene_path = f"{get_package_share_directory(self.getPkgName())}/worlds/CoppeliaSim/{self.getScene()}.ttt"
        #         coppelia_sim_command += f"-f\"{scene_path}\""

        # if self.getEnableLogger():
        #     omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("omni.isaac.logger", True)
        
        if self.getEnableHeadless():
            pass
        
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        # # Launch CoppeliaSim
        # launch_coppelia_sim = ExecuteProcess(
        #     name="coppelia_sim_launcher",
        #     cmd=[coppelia_sim_command],
        #     shell=True,
        #     output='screen'
        # )

        # self.addNode(launch_coppelia_sim)

        # exit_event_handler = RegisterEventHandler(
        #     OnProcessExit(
        #         target_action=launch_coppelia_sim,
        #         on_exit=[
        #             launch.actions.Shutdown()
        #         ]
        #     )
        # )

        # self.addNode(exit_event_handler)
