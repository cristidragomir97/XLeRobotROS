# syntax=docker/dockerfile:1
FROM ros:jazzy-ros-base
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-tornado python3-pil python3-psutil python3-colcon-common-extensions python3-venv python3-numpy python3-scipy python3-yaml \
    ros-jazzy-robot-state-publisher ros-jazzy-control-msgs ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rtabmap-ros ros-jazzy-navigation2 ros-jazzy-nav2-rviz-plugins \
    ros-jazzy-rviz2 ros-jazzy-teleop-twist-keyboard \
    libegl1 libegl-mesa0 libgl1-mesa-dri libglx-mesa0 libglfw3 \
    && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
    ros-jazzy-omni-wheel-drive-controller ros-jazzy-joint-trajectory-controller \
    ros-jazzy-xacro ros-jazzy-joy-teleop \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m venv --system-site-packages /opt/xlerobot-python \
    && /opt/xlerobot-python/bin/pip install --no-cache-dir numpy==1.26.4 mujoco==3.13.0 mjviser==0.0.14 python-fcl==0.7.0.11
ENV PATH=/opt/xlerobot-python/bin:$PATH
# ament_python scripts use the ROS interpreter; expose the isolated simulator modules to it.
ENV PYTHONPATH=/opt/xlerobot-python/lib/python3.12/site-packages
COPY dependencies/lekiwi_ros2/lekiwi_control /opt/xlerobot_ws/dependencies/lekiwi_control
COPY dependencies/lekiwi_ros2/lekiwi_description /opt/xlerobot_ws/dependencies/lekiwi_description
COPY dependencies/lekiwi_ros2/lekiwi_mujoco /opt/xlerobot_ws/dependencies/lekiwi_mujoco
COPY dependencies/lekiwi_ros2/modules/sts_hardware_interface /opt/xlerobot_ws/dependencies/sts_hardware_interface
COPY dependencies/mod101/src/mod101_description /opt/xlerobot_ws/dependencies/mod101_description
COPY dependencies/mod101/src/mod101_tool_parallel /opt/xlerobot_ws/dependencies/mod101_tool_parallel
COPY dependencies/mod101/src/mod101_tool_pggripper /opt/xlerobot_ws/dependencies/mod101_tool_pggripper
COPY dependencies/mod101/src/mod101_tool_pincopen /opt/xlerobot_ws/dependencies/mod101_tool_pincopen
COPY dependencies/mod101/src/mod101_tool_jaws /opt/xlerobot_ws/dependencies/mod101_tool_jaws
COPY dependencies/mod101/src/mod101_tool_none /opt/xlerobot_ws/dependencies/mod101_tool_none
COPY dependencies/mod101/src/mod101_tool_camera /opt/xlerobot_ws/dependencies/mod101_tool_camera
COPY src /opt/xlerobot_ws/src
COPY upstream/simulation/mujoco /opt/xlerobot_ws/upstream/simulation/mujoco
COPY upstream/simulation/Maniskill/assets/xlerobot /opt/xlerobot_ws/upstream/simulation/Maniskill/assets/xlerobot
COPY assets/robocasa /opt/xlerobot-assets/robocasa
ENV XLEROBOT_WORLD_ASSETS=/opt/xlerobot-assets/robocasa
RUN source /opt/ros/jazzy/setup.bash && cd /opt/xlerobot_ws \
    && colcon build --symlink-install --parallel-workers 2 \
       --base-paths src dependencies --packages-select lekiwi_description lekiwi_mujoco lekiwi_control \
         sts_hardware_interface mod101_description mod101_tool_parallel mod101_tool_pggripper mod101_tool_pincopen mod101_tool_jaws mod101_tool_none mod101_tool_camera robocore_viz xlerobot_description xlerobot_control xlerobot_sim \
       --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF
COPY docker/entrypoint.sh /usr/local/bin/xlerobot-entrypoint
RUN chmod +x /usr/local/bin/xlerobot-entrypoint
ENV ROS_DOMAIN_ID=185 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp MUJOCO_GL=egl
WORKDIR /workspace/xlerobot
ENTRYPOINT ["/usr/local/bin/xlerobot-entrypoint"]
CMD ["sleep", "infinity"]
