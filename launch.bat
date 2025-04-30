@echo off

goto :INIT

:INIT
    REM Prompt the user for the model to launch
    IF "%~1"=="" (
        set /p "model=Enter the model to launch (mir100): "
    ) ELSE (
        set "model=%1"
    )

    set "availableModels=mir100"
    set "found=false"
    for %%i in (%availableModels%) do (
        if "%%i"=="%model%" (
            set "found=true"
            goto :found_model
        )
    )

    :found_model
    if "%found%"=="false" (
        echo Model %model% does not exist.
        goto :EOF
    )

    REM Prompt for model usage
    IF "%~1"=="" (
        set /p "model_config=Do you want to use the default model parameters development configuration? (y | n): "
    ) ELSE (
        set "model_config=y"
    )

    if /i "%model_config%"=="y" goto :SCENE
    if /i "%model_config%"=="n" goto :MODEL

:MODEL
    set /p "model_name=What name do you want your model to have? (ex: r2d2): "
    set /p "model_namespace_config=Do you want to use the default namespace development configuration? (y | n): "

    if /i "%model_namespace_config%"=="n" set /p "model_namespace=What namespace do you want your model to have? (ex: /test): "

    set "model_params=model_name:=%model_name%"
    if /i "%model_namespace_config%"=="n" set "model_params=%model_params% model_namespace:=%model_namespace%"

    goto :SCENE

:SCENE
    IF "%~2"=="" (
        set /p "scene=Enter the scene to launch (empty | default | obstacle | static | dynamic | aisle): "
    ) ELSE (
        set "scene=%2"
    ) 

    set "availableScenes=empty default obstacle static dynamic aisle"
    set "found=false"
    for %%i in (%availableScenes%) do (
        if "%%i"=="%scene%" (
            set "found=true"
            goto :found_scene
        )
    )

    :found_scene
    if "%found%"=="false" (
        echo Scene %scene% does not exist
        goto :EOF
    )    

    goto :SIMULATOR

:SIMULATOR
    IF "%~3"=="" (
        set /p "simulator=Enter the simulator to launch (coppelia | gazebo | isaac): "
    ) ELSE (
        set "simulator=%3"
    )

    set "availableSimulators=coppelia gazebo isaac"
    set "found=false"
    for %%i in (%availableSimulators%) do (
        if "%%i"=="%simulator%" (
            set "found=true"
            goto :found_simulator
        )
    )

    :found_simulator
    if "%found%"=="false" (
        echo Simulator %simulator% does not exist
        goto :EOF
    )

    IF "%~1"=="" (
        set /p "rviz_config=Do you want to launch rviz2 and joint_state_publisher_gui? (y | n): "
    ) ELSE (
        set "rviz_config=n"
    )

    set "simulator_params="
    if /i "%rviz_config%"=="y" set "simulator_params=launch_rviz2:=true "
    
    if /i "%simulator%"=="gazebo" call %conda% activate gazebo
    if /i "%simulator%"=="isaac" call %conda% activate isaacsim
    set "simulator_params=%simulator_params%launch_%simulator%:=true"

    goto :CONFIGS

:CONFIGS 
    IF "%~1"=="" (
        set /p "headless=Do you want to launch the simulator in headless mode? (y | n): "
    ) ELSE (
        set "headless=n"
    )

    IF "%~1"=="" (
        set /p "logger=Do you want to launch the simulator with logger enabled? (y | n): "
    ) ELSE (
        set "logger=y"
    )

    IF "%~4"=="" (
        set /p "time=Enter the time in seconds after which the simulation should stop (0 indicates that the simulation won't stop until you close it): "
    ) ELSE (
        set "time=%4"
    )

    IF "%~1"=="" (
        set /p "controlflag=Do you want to run a controller on the robot? (y | n): "
    ) ELSE ( 
        IF "%~5"=="" (
            set "controlflag=n"
        ) ELSE (
            set "controlflag=y"
        ) 
    )

    set "flags_params=total_time:=%time%"

    if "%headless%"=="y" (
        set "flags_params=%flags_params% enable_headless:=true"
    )

    if "%logger%"=="y" (
        set "flags_params=%flags_params% enable_logger:=true"
    )

    if "%controlflag%"=="y" (
        goto :ROBOTCRTL
    ) ELSE (
        goto :LAUNCH
    )

:ROBOTCRTL
    IF "%~1"=="" (
        set /p "ctrl=Enter the controller to launch (pure_pursuit_control): "
    ) ELSE (
        set "ctrl=%5"
    )

    set "availableControllers=pure_pursuit_control"
    set "found=false"
    for %%i in (%availableControllers%) do (
        if "%%i"=="%ctrl%" (
            set "found=true"
            goto :found_controller
        )
    )

    :found_controller
    if "%found%"=="false" (
        echo Controller %ctrl% does not exist
        goto :EOF
    )

    set "flags_params=%flags_params% controller_name:=%ctrl%" 
    goto :LAUNCH

:LAUNCH
    for /f "tokens=2 delims==" %%I in ('wmic os get localdatetime /value') do set datetime=%%I
    set datefolder=%SEER_WS_DIR%\logs\%datetime:~0,4%-%datetime:~4,2%-%datetime:~6,2%_%datetime:~8,2%-%datetime:~10,2%-%datetime:~12,2%_%model%_%scene%_%simulator%_%time%
    mkdir %datefolder%
    mkdir %datefolder%\%simulator%
    mkdir %datefolder%\ros
    mkdir %datefolder%\hardware

    (
        echo {
        echo    "model": "%model%",
        echo    "scenario": "%scene%",
        echo    "simulator": "%simulator%", 
        if "%headless%"=="y" (
            echo    "headless": "true",
        ) ELSE (
            echo    "headless": "false",
        )
        if "%rviz_config%"=="y" (
            echo    "rviz2": "true",
        ) ELSE (
            echo    "rviz2": "false",
        )
        if "%control%"=="y" (
            echo    "time": "%time%",
            echo    "controller": "%controller%"
        ) ELSE (
            echo    "time": "%time%"
        )
        echo }
    ) > %datefolder%\config.json

    set SEER_CONFIG_LOGS=TRUE

    call %ROS_INIT_WS%

    call ros2 launch senai_models model_rsp.launch.py model:=%model% %model_params% scene:=%scene% %simulator_params% %flags_params%

    goto :EOF

:end
exit 0
