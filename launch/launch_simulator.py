from launch_interface import LaunchInterface

class LaunchSimulator(LaunchInterface):

    def __init__(self, name, model_name, robot_description = None, scene = "empty", enable_headless=False):
        self.__setModelName(model_name)
        self.__setRobotDescription(robot_description)
        self.__setScene(scene)
        self.__setEnableHeadless(enable_headless)

        super().__init__(name=name)

    ########### SETTERS ###########
    def __setModelName(self, model_name):
        self.__model_name = model_name

    def __setRobotDescription(self, robot_description):
        self.__robot_description = robot_description

    def __setScene(self, scene):
        self.__scene = scene

    def __setEnableHeadless(self, enable):
        self.__enableHeadless = enable

    ########### GETTERS ###########
    def getModelName(self):
        return self.__model_name

    def getRobotDescription(self):
        return self.__robot_description

    def getScene(self):
        return self.__scene
    
    def getEnableHeadless(self):
        return self.__enableHeadless