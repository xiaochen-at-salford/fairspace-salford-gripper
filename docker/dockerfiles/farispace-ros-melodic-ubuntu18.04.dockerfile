FROM osrf/ros:melodic-desktop-full-bionic

ARG uid=1000
ARG gid=1000
RUN groupadd -r -f -g ${gid} fairspace \
    && useradd -r -u ${uid} -g ${gid} -ms /bin/bash hhkb
RUN usermod -aG sudo hhkb 
    # && usermod -aG fairspace  

ENV DEBIAN_FRONTEND=noninteractive

COPY docker/installers /home/hhkb/build/installers

RUN bash /home/hhkb/build/installers/preset_hhkb.sh
RUN bash /home/hhkb/build/installers/init_apt.sh

USER hhkb:fairspace
RUN bash /home/hhkb/build/installers/preset_hhkb_catkin.sh
RUN bash /home/hhkb/build/installers/postset_hhkb.sh

WORKDIR /home/hhkb/catkin_ws

RUN ["/bin/bash"]