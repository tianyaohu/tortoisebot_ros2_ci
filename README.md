# ROS2 TortoiseBot CI with Jenkins 🌌

## Overview 🌟
Welcome, This repository is part of Jenkins CI setup, containing pipeline script and docker image used. Any changes made to this repo will trigger a jenkins build for waypoint_test previously built.

## Installation 🛠️

### Clone Repo
In Terminal #1
```
#change it to your own workspace path if needed
cd ~/ros2_ws/src
git clone https://github.com/tianyaohu/tortoisebot_ros2_ci.git
```

## Run Jenkins with `start_jenkins.sh`
In Terminal #1
```
cd ./tortoisebot_ros2_ci/
#pull remote images from docker hub (5min~)
bash start_jenkins.sh
```
Expected Result:
![docker-compose up](media/start_jenkins.gif)

## Trigger Jenkins Build
Open a pull request or push to this repo, 🎉 jenkins build will be triggered automatically!

Demo:
![auto jenkins_build](media/demo.gif)