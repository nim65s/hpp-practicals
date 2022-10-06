FROM ros:noetic

ADD http://robotpkg.openrobots.org/packages/debian/robotpkg.key /
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    --mount=type=cache,sharing=locked,target=/var/lib/apt \
    apt-key add /robotpkg.key \
 && echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub focal robotpkg" > /etc/apt/sources.list.d/robotpkg.list \
 && apt-get update -y \
 && DEBIAN_FRONTEND=noninteractive apt-get install -qqy emacs-gtk spyder\
    oxygen-icon-theme firefox \
    robotpkg-py38-hpp-environments \
    robotpkg-py38-hpp-manipulation-corba \
    robotpkg-py38-qt5-hpp-gepetto-viewer \
    robotpkg-py38-qt5-hpp-gui \
    robotpkg-py38-qt5-hpp-plot \
    robotpkg-py38-hpp-tutorial \
    ros-noetic-pr2-description

RUN echo "[pyplugins]\nhpp.gui=true" >> /opt/openrobots/etc/gepetto-gui/settings.conf \
 && echo "[pyplugins]\nhpp.gui=true" >> /opt/openrobots/etc/gepetto-gui/basic.conf

ENV PATH=/opt/openrobots/bin:${PATH} \
    LD_LIBRARY_PATH=/opt/openrobots/lib:${LD_LIBRARY_PATH} \
    PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:${PYTHONPATH} \
    ROS_PACKAGE_PATH=/opt/openrobots/share:${ROS_PACKAGE_PATH} \
    CMAKE_PREFIX_PATH=/opt/openrobots:${CMAKE_PREFIX_PATH} \
    PKG_CONFIG_PATH=/opt/openrobots:${PKG_CONFIG_PATH}

RUN --mount=type=cache,sharing=locked,target=/var/cache/apt --mount=type=cache,sharing=locked,target=/var/lib/apt \
    apt-get update -qqy && DEBIAN_FRONTEND=noninteractive apt-get install -qqy \
    git \
    mesa-utils \
    net-tools \
    novnc \
    sudo \
    tigervnc-standalone-server \
    tigervnc-xorg-extension \
    terminator \
    x11-apps \
    xfce4

RUN apt-get remove -y gnome-screensaver light-locker

EXPOSE 5901

RUN ln -s /opt/openrobots/lib/python3.8 /opt/openrobots/lib/python3
RUN ln -s /opt/openrobots/lib/python3.8/site-packages /opt/openrobots/lib/python3/dist-packages
RUN mkdir /workspace
RUN cd /workspace && git clone --recursive -b devel https://github.com/humanoid-path-planner/hpp-practicals
RUN mkdir /workspace/hpp-practicals/build
RUN cd /workspace/hpp-practicals/build && cmake -DCMAKE_INSTALL_PREFIX=/opt/openrobots -DPYTHON_DEB_LAYOUT=ON ..
RUN cd /workspace/hpp-practicals/build && make install
RUN chmod -R a+w /workspace

RUN chmod 777 /usr/share/novnc/utils \
 && useradd -m user \
 && echo "user ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

USER user
ADD --chown=user docker-vnc /usr/local/bin
RUN chmod 777 /usr/local/bin/docker-vnc
COPY config.sh /home/user
RUN echo "source ${HOME}/config.sh\n" >> /home/user/.bashrc

CMD docker-vnc
