#!/bin/bash

# Function for custom model configuration
custom_model() {
    if [[ "$args" == "n" ]]; then
        read -p "What name do you want your model to have? (ex: r2d2): " model_name
        read -p "Do you want to use the default namespace development configuration? (y | n): " model_namespace_config
    fi

    model_params="model_name:=$model_name"
    
    if [[ "$model_namespace_config" == "n" ]]; then
        read -p "What namespace do you want your model to have? (ex: /test): " model_namespace
        model_params="$model_params model_namespace:=$model_namespace"
    fi

    scene_config
}

# Function for scene configuration
scene_config() {
    if [[ "$args" == "n" ]]; then
        read -p "Enter the scene to launch (empty | default | obstacle | static | dynamic | aisle): " scene
    fi

    if [[ "$scene" == "empty" || "$scene" == "default" || "$scene" == "obstacle" || "$scene" == "static" || "$scene" == "dynamic" || "$scene" == "aisle" ]]; then
        simulator_config
    else
        exit 0
    fi
}

# Function for simulator configuration
simulator_config() {
    if [[ "$args" == "n" ]]; then
        read -p "Enter the simulator to launch (coppelia | gazebo | isaac): " simulator
        read -p "Do you want to launch rviz2 and joint_state_publisher_gui? (y | n): " rviz_config
    fi

    simulator_params=""
    if [[ "$rviz_config" == "y" ]]; then
        simulator_params="launch_rviz2:=true"
    fi
    
    #if [[ "$simulator" == "gazebo" ]]; then
    #    conda activate gazebo
    #fi

    simulator_params="$simulator_params launch_$simulator:=true"
    launch_simulator
}

# Function to launch the simulator
launch_simulator() {
    if [[ "$args" == "n" ]]; then
        read -p "Do you want to launch the simulator in headless mode? (y | n): " headless
        read -p "Do you want to launch the simulator with logger enabled? (y | n): " logger
        read -p "Enter the time in seconds after which the simulation should stop (0 indicates that the simulation won't stop until you close it): " time
        read -p "Do you want to run a controller on the robot? (y | n): " control
    fi

    flags_params="total_time:=$time"
    if [[ "$headless" == "y" ]]; then
        flags_params="$flags_params enable_headless:=true"
    fi
    if [[ "$logger" == "y" ]]; then
        flags_params="$flags_params enable_logger:=true"
    fi
    if [[ "$control" == "y" ]]; then
        if [[ "$args" == "n" ]]; then
            read -p "Enter the controller to launch (pure_pursuit_control): " controller
            if [[ "$controller" != "pure_pursuit_control" ]]; then
                exit 0
            fi
        fi

        if [[ "$controller" != "" ]]; then
            flags_params="$flags_params controller_name:=$controller"
        fi
    fi
    
    folder_name=$SEER_WS_DIR/logs/$(date +"%Y-%m-%d_%H-%M-%S")_${model}_${scene}_${simulator}_${time}
    mkdir -p "$folder_name"
    mkdir -p "$folder_name/$simulator"
    mkdir -p "$folder_name/hardware"
    mkdir -p "$folder_name/ros"

    sudo chmod -R 755 "$folder_name"
    #sudo chown -R "$USER" "$folder_name"

    (
        echo {
        echo "   \"model\": \"$model\","
        echo "   \"scenario\": \"$scene\","
        echo "   \"simulator\": \"$simulator\"," 
        if [[ "$headless" == "y" ]]; then 
            (echo "   \"headless\": \"true\",") 
        else 
            (echo "   \"headless\": \"false\",") 
        fi
        if [[ "$rviz_config" == "y" ]]; then 
            (echo "   \"rviz2\": \"true\",")
        else
            (echo "   \"rviz2\": \"false\",")
        fi
        if [[ "$control" == "y" ]]; then 
            echo "   \"time\": \"$time\","
            echo "   \"controller\": \"$controller\""
        else
            echo "   \"time\": \"$time\""
        fi
        echo }
    ) > $folder_name/config.json

    export SEER_CONFIG_LOGS=TRUE
    # Launch the ROS simulation
    ros2 launch senai_models model_rsp.launch.py model:=$model $model_params scene:=$scene $simulator_params $flags_params
}

# Source ROS initialization (replace with your correct ROS init script)T
source ~/.bashrc

# Prompt the user for the model to launch
if [[ $# -eq 0 ]]; then
    echo "No arguments supplied"
    read -p "Enter the model to launch (mir100): " model
    args=n
else
    args=y
    model=$1
    model_config=y
    scene=$2
    simulator=$3
    rviz_config=n
    headless=n
    logger=y
    control=y
    time=$4
    controller=$5
fi

if [[ "$model" != "mir100" ]]; then
    exit 0
fi

# Prompt for model usage
if [[ "$args" == "n" ]]; then
    read -p "Do you want to use the default development configuration? (y | n): " model_config
fi

if [[ "$model_config" == "y" ]]; then
    scene_config
elif [[ "$model_config" == "n" ]]; then
    custom_model
else
    exit 0
fi

# Start the script by calling the initial scene config function
# scene_config
unset SEER_CONFIG_LOGS
exit 0