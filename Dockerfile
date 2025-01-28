# base image
FROM osrf/ros:jazzy-desktop-full

# install tools needed
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    ros-jazzy-gazebo-ros-pkgs \
    ros-jazzy-rviz2 \
    git \
    && apt clean

# create work space
WORKDIR /ros2_ws
COPY ros2_ws /ros2_ws

RUN mkdir -p src

# ソースコードの依存関係を解決
RUN apt update && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# build
RUN . /opt/ros/jazzy/setup.sh && colcon build

# 環境変数の設定
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && source install/setup.bash && bash"]
