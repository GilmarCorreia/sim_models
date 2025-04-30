from launch_interface import LaunchInterface

class LaunchSimulator(LaunchInterface):

    def __init__(self, name, model_name, robot_description = None, scene = "empty", total_time=0, enable_logger=False, enable_headless=False):
        self.__setModelName(model_name)
        self.__setRobotDescription(robot_description)
        self.__setScene(scene)
        self.__setTotalTime(total_time)
        self.__setEnableLogger(enable_logger)
        self.__setEnableHeadless(enable_headless)

        super().__init__(name=name)

    ########### SETTERS ###########
    def __setModelName(self, model_name):
        self.__model_name = model_name

    def __setRobotDescription(self, robot_description):
        self.__robot_description = robot_description

    def __setScene(self, scene):
        self.__scene = scene

    def __setTotalTime(self, time):
        self.__totalTime = time
    
    def __setEnableLogger(self, enable):
        self.__enableLogger = enable

    def __setEnableHeadless(self, enable):
        self.__enableHeadless = enable

    ########### GETTERS ###########
    def getModelName(self):
        return self.__model_name

    def getRobotDescription(self):
        return self.__robot_description

    def getScene(self):
        return self.__scene
    
    def getTotalTime(self):
        return self.__totalTime
    
    def getEnableLogger(self):
        return self.__enableLogger
    
    def getEnableHeadless(self):
        return self.__enableHeadless