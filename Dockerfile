# FROM osrf/ros:galactic-desktop
# FROM ros:galactic
FROM ros:humble-perception

SHELL ["/bin/bash", "-c"]

# vim and tmux
RUN apt-get update && apt-get install -y vim tmux curl

# ## install realsense
RUN mkdir -p /etc/apt/keyrings
RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
RUN apt-get install apt-transport-https
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | sudo tee /etc/apt/sources.list.d/librealsense.list
RUN apt-get update && apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg

RUN apt-get install -y \
  ros-${ROS_DISTRO}-realsense2-camera \
  && rm -rf /var/lib/apt/lists/

# ## install octomap
#RUN apt-get install -y --no-install-recommends \
#  ros-${ROS_DISTRO}-octomap

# RUN apt-get install -y --no-install-recommends  ros-${ROS_DISTRO}-octomap-mapping

# WORKDIR "/root/"
# RUN git clone https://github.com/OctoMap/octomap.git
# WORKDIR "/root/octomap/build"
# RUN cmake .. && make -j
# # set an env variable on where octomap is
# ENV octomap_DIR /root/octomap/lib/cmake/octomap/
# #ENV OCTOMAP_DIR /root/colcon_ws/src/octomap/lib/cmake/octomap/

RUN apt-get update 
RUN apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-perception-pcl
RUN apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-octomap ros-${ROS_DISTRO}-octomap-mapping

# install rviz2
RUN apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-rviz2 

# install rqt
RUN apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-rqt-common-plugins

# add the ros2 sourcing to bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source /root/colcon_ws/install/setup.bash" >> /root/.bashrc


WORKDIR "/root/colcon_ws"
