# RTABMap-ROS2-PX4
ROS 2 implementation of RTAB-Map SLAM. Simulating x500 drone using PX4 SITL. Mapping the environment, extracting 3D scan, and visualizing the output. All in a ready to go Docker image.

# How to Run

- Pull the docker image:

```docker pull monemati/rtabmap_px4_ros2_gazebo```

- Run the docker image:
```
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
xhost +local:docker

docker run --privileged -it --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_VISIBLE_DEVICES=all --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --env="XAUTHORITY=$XAUTH" --volume="$XAUTH:$XAUTH" --network=host --ipc=host --shm-size=2gb --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --name rtabmap_ros2  --entrypoint /bin/bash monemati/rtabmap_px4_ros2_gazebo
```

- Launch the whole thing with one command:

```tmuxinator rtabmap_ros2_gazebo```

- Control the drone:
```
# Focus on pygame window by clicking on it, then hit "r" on keyboard to arm the drone, and use WASD and Up-Down-Left-Right on the keyboard for flying, and use "l" for landing.
```  
# Demo



https://github.com/user-attachments/assets/95df77f0-6b0b-4da7-af41-93776e45e212



# Saving & Visualizing

- Launch the rtabmap db viewer:
```
rtabmap-databaseViewer ~/.ros/rtabmap.db
# Navigate to File >> Export 3d map
```
- Launch meshlab:
```
meshlab
# Navigate to File >> Import Mesh
```
 
# Customization

- Modifying the launch parameters:
```
nano /root/simulation_ws/src/simulation_launch/launch/full_simulation.launch.py 
cd /root/simulation_ws
colcon build
source install/setup.bash
```
- Modifying tmuxinator launching commands:

```nano /root/.config/tmuxinator/rtabmap_ros2_gazebo.yml```

