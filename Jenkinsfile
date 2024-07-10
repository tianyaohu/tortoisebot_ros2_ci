pipeline {
    agent {
        docker {
            image 'tianyaoh501679/jenkins_cicd_learning:tortoisebot_ros2_waypoint_test'
            args '-u root -v /home/user/ros2_ws/src/tortoisebot:/root/ros2_ws/src/tortoisebot -v /tmp/.X11-unix:/tmp/.X11-unix  -e DISPLAY=$DISPLAY ' 
        }
    }
    stages {
        stage('Prepare Environment') {
            steps {
                sh label: 'Setting Up Environment', script: '''
                echo "Sourcing ROS2 environment..."
                /bin/bash -c "source /opt/ros/galactic/setup.bash && cd /root/ros2_ws/ && colcon build && source /root/ros2_ws/install/setup.bash "
                '''
            }
        }
        stage('Start Gazebo Simulation') {
            steps {
                sh label: 'Launching Gazebo', script: '''
                /bin/bash -c "source /opt/ros/galactic/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 launch tortoisebot_bringup bringup.launch.py use_sim_time:=True &"
                '''
            }
        }
        stage('Start Waypoint Action Server') {
            steps {
                sh label: '', script: '''#!/bin/bash
                /bin/bash -c "source /opt/ros/galactic/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 run tortoisebot_waypoints tortoisebot_action_server &"
                '''
            }
        }
        stage('Running GTest') {
            steps {
                sh label: '', script: '''#!/bin/bash
                pwd
                /bin/bash -c "source /opt/ros/galactic/setup.bash && source /root/ros2_ws/install/setup.bash && cd /root/ros2_ws/ && colcon build --packages-select tortoisebot_waypoints &&  colcon test --packages-select tortoisebot_waypoints --event-handler=console_direct+"
                '''
            }
        }
        stage('View Test Result') {
            steps {
                sh label: '', script: '''#!/bin/bash
                /bin/bash -c "source /opt/ros/galactic/setup.bash && source /root/ros2_ws/install/setup.bash && cd /root/ros2_ws/ && colcon test-result --verbose"
                '''
            }
        }
    }
}
