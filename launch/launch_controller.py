import launch
from launch_ros.actions import Node

# Custom Imports
from launch_interface import LaunchInterface

class Controller(LaunchInterface):

    def __init__(self, model, model_name, controller_name):
        self.__setModel(model)
        self.__setModelName(model_name)
        self.__setControllerName(controller_name)
        super().__init__(name="Controller")

    ########### SETTERS ###########
    def __setModel(self, model):
        self.__model = model

    def __setModelName(self, model_name):
        self.__model_name = model_name

    def __setControllerName(self, controller_name):
        self.__controller_name = controller_name

    ########### GETTERS ###########
    def getModel(self):
        return self.__model

    def getModelName(self):
        return self.__model_name
    
    def getControllerName(self):
        return self.__controller_name

    ########### METHODS ###########
    def createInterfaceNodes(self):
        # Verify if clock is active
        clock_checker = Node(
            package='sim_models',
            executable='clock_waiter',
            name='clock_waiter_controller',
            output='screen'
        )

        self.addNode(clock_checker)

        # Controller node
        controller_node = Node(
            package='sim_controls',
            executable=self.getControllerName(),
            shell=True,
            output='screen',
            parameters=[
                {
                    'use_sim_time': False
                }
            ],
            arguments=[
                '--model', self.getModel(),
                '--model_name', self.getModelName(),
                '--linear_speed', '0.35'
            ]
        )

        # Event to launch nodes after the clock_checker process exits
        clock_event_handler = launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=clock_checker,
                on_exit=[controller_node],
            )
        )
        self.addNode(clock_event_handler)
