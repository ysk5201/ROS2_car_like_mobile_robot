# base image
FROM osrf/ros:jazzy-desktop-full

# install tools needed and delete APT cache
RUN apt-get update && \
    apt-get install -y \
        software-properties-common \
        python3-colcon-common-extensions \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-joint-state-broadcaster \
        ros-jazzy-joy \
        ros-jazzy-ros-gz \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-rqt-robot-steering \
        ros-jazzy-rviz2 \
        x11-apps \
        git && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# create work space
WORKDIR /root/ros2_ws

# ローカルの/ros2_ws/をリモートの/root/ros2_ws/にコピー
COPY ./ros2_ws /root/ros2_ws

# ソースコードの依存関係を解決
RUN apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# build
RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build || echo 'No packages to build'"

# 環境変数の設定
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && source install/setup.bash && bash"]
