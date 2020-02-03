#FROM ros:melodic
FROM osrf/ros:melodic-desktop-full-bionic

# Install application specific packages
RUN apt-get update && apt-get install -y \
  ros-melodic-rosserial-arduino \
  ros-melodic-rosserial \
  cppcheck \
  python-pip && \
  pip install -U platformio && \
  python -m pip install cpplint

# This is done automatically later but it will speed things up to do it now
RUN platformio platform install teensy

# reset the entrypoint set by https://github.com/osrf/docker_images/blob/master/ros/melodic/ubuntu/bionic/ros-core/Dockerfile
ENTRYPOINT []
# open a shell
CMD ["sh"]
