# Udacity capstone project dockerfile
FROM ros:kinetic-robot
LABEL maintainer="olala7846@gmail.com"

# Install Dataspeed DBW https://goo.gl/KFSYi1 from binary
# adding Dataspeed server to apt
RUN sh -c 'echo "deb [ arch=amd64 ] http://packages.dataspeedinc.com/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-dataspeed-public.list'
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys FF6D3CDA
RUN apt-get update

# setup rosdep
RUN sh -c 'echo "yaml http://packages.dataspeedinc.com/ros/ros-public-'$ROS_DISTRO'.yaml '$ROS_DISTRO'" > /etc/ros/rosdep/sources.list.d/30-dataspeed-public-'$ROS_DISTRO'.list'
RUN rosdep update
RUN apt-get install -y ros-$ROS_DISTRO-dbw-mkz
RUN apt-get upgrade -y
# end installing Dataspeed DBW

# install python packages
RUN apt-get install -y python-pip
COPY requirements.txt ./requirements.txt
RUN pip install -r requirements.txt

# install required ros dependencies
RUN apt-get install -y ros-$ROS_DISTRO-cv-bridge
RUN apt-get install -y ros-$ROS_DISTRO-pcl-ros
RUN apt-get install -y ros-$ROS_DISTRO-image-proc

# socket io
RUN apt-get install -y netbase

# Tmux
RUN apt-get install -y tmux

RUN apt-get install -y protobuf-compiler python-setuptools python-dev build-essential python-pip

# Download object detection models
RUN cd /opt && \
     git clone https://github.com/tensorflow/models tensorflow_models

# Install coco
RUN git clone https://github.com/cocodataset/cocoapi.git && \
   cd cocoapi/PythonAPI && \
   make && \
   mkdir -p /usr/local/lib/python2.7/dist-packages/tensorflow/models/research && \
   cp -r pycocotools /opt/tensorflow_models/research && \
   cd /opt/tensorflow_models/research && \
   protoc object_detection/protos/*.proto --python_out=. && \
   echo "export PYTHONPATH=$PYTHONPATH:/opt/tensorflow_models/research:/opt/tensorflow_models/research/slim" >> ~/.bashrc


RUN mkdir /capstone
VOLUME ["/capstone"]
VOLUME ["/root/.ros/log/"]
WORKDIR /capstone/ros
