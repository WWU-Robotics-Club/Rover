#FROM ros:melodic
FROM osrf/ros:melodic-desktop-full-bionic

# Install application specific packages
RUN apt-get update && apt-get install -y \
  ros-melodic-rosserial-arduino \
  ros-melodic-rosserial

# Install development and deployment tools
# Install arduino cli
# https://github.com/arduino/arduino-cli
#RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
# https://www.pjrc.com/teensy/loader_cli.html
RUN apt-get update && apt-get install -y libusb-dev \
  && git clone https://github.com/PaulStoffregen/teensy_loader_cli.git \
  && cd teensy_loader_cli && make \
  && apt-get install -y teensy-loader-cli

# reset the entrypoint set by https://github.com/osrf/docker_images/blob/master/ros/melodic/ubuntu/bionic/ros-core/Dockerfile
ENTRYPOINT []
# open a shell
CMD ["sh"]
