#FROM ros:melodic
FROM osrf/ros:melodic-desktop-full-bionic

# Install application specific packages
RUN apt-get update && apt-get install -y \
  ros-melodic-rosserial-arduino \
  ros-melodic-rosserial \
  ros-melodic-rosbridge-server \
  cppcheck \
  gcc \
  software-properties-common \
  python-pip && \
  pip install -U platformio && \
  platformio platform install teensy && \
  python -m pip install cpplint && \
  pip install pylint

# Install realsense packages
# prevents apt-keye adv warning https://github.com/wv-gis/mudak-wrm-public/issues/2
ENV APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=DontWarn
# install add-apt-repository support
RUN apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
  add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u && \
  apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev
# add librealsense2-dbg if debug symbols are needed. Otherwise it takes a long time to download

COPY ROS ROS/
COPY Arduino Arduino/

# reset the entrypoint set by https://github.com/osrf/docker_images/blob/master/ros/melodic/ubuntu/bionic/ros-core/Dockerfile
ENTRYPOINT ["ROS/start.sh"]
