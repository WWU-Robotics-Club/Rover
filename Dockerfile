#FROM ros:melodic
FROM osrf/ros:melodic-desktop-full-bionic

# Install application specific packages
RUN apt-get update && apt-get install -y \
  ros-melodic-rosserial-arduino \
  ros-melodic-rosserial \
  cppcheck \
  gcc \
  python-pip && \
  pip install -U platformio && \
  platformio platform install teensy && \
  python -m pip install cpplint

COPY ROS ROS/
COPY Arduino Arduino/

# reset the entrypoint set by https://github.com/osrf/docker_images/blob/master/ros/melodic/ubuntu/bionic/ros-core/Dockerfile
ENTRYPOINT ["ROS/start.sh"]
