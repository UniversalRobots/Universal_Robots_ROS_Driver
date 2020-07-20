# MIT License
#
# Original from https://github.com/ahobsonsayers/DockURSim
# Copyright (c) 2019 Arran Hobson Sayers
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

FROM lsiobase/guacgui:latest

# Set Version Information
ARG BUILD_DATE="15/08/19"
ARG VERSION="5.8.0.10253"
LABEL build_version="URSim Version: ${VERSION} Build Date: ${BUILD_DATE}"
LABEL maintainer="Arran Hobson Sayers"
LABEL MAINTAINER="Arran Hobson Sayers"
ENV APPNAME="URSim"

# Set Timezone
ARG TZ="Europe/London"
ENV TZ ${TZ}

# Setup Environment
ENV DEBIAN_FRONTEND noninteractive

# Set Home Directory
ENV HOME /ursim

# Set robot model - Can be UR3, UR5 or UR10
ENV ROBOT_MODEL UR5

RUN \
    echo "**** Installing Dependencies ****" && \
    apt-get update && \
    apt-get install -qy --no-install-recommends \
    openjdk-8-jre psmisc && \
    # Change java alternatives so we use openjdk8 (required by URSim) not openjdk11 that comes with guacgui
    update-alternatives --install /usr/bin/java java /usr/lib/jvm/java-8-openjdk-amd64/jre/bin/java 10000

# Setup JAVA_HOME
ENV JAVA_HOME /usr/lib/jvm/java-8-openjdk-amd64

RUN \
    echo "**** Downloading URSim ****" && \
    # Make sure we are in the root
    cd / && \
    # Download URSim Linux tar.gz
    curl https://s3-eu-west-1.amazonaws.com/ur-support-site/69987/URSim_Linux-5.8.0.10253.tar.gz -o URSim-Linux.tar.gz && \
    #curl https://s3-eu-west-1.amazonaws.com/ur-support-site/54411/URSim_Linux-5.4.2.76197.tar.gz -o URSim-Linux.tar.gz && \
    # Extract tarball
    tar xvzf URSim-Linux.tar.gz && \
    #Remove the tarball
    rm URSim-Linux.tar.gz && \
    # Rename the URSim folder to jus ursim
    mv  /ursim* /ursim

RUN \
    echo "**** Installing URSim ****" && \
    # cd to ursim folder
    cd /ursim && \
    # Make URControl and all sh files executable
    chmod +x ./*.sh ./URControl && \
    #
    # Stop install of unnecessary packages and install required ones quietly
    sed -i 's|apt-get -y install|apt-get -qy install --no-install-recommends|g' ./install.sh && \
    # Skip xterm command. We dont have a desktop
    sed -i 's|tty -s|(exit 0)|g' install.sh && \
    # Skip Check of Java Version as we have the correct installed and the command will fail
    sed -i 's|needToInstallJava$|(exit 0)|g' install.sh && \
    # Skip install of desktop shortcuts - we dont have a desktop
    sed -i '/for TYPE in UR3 UR5 UR10/,$ d' ./install.sh  && \
    # Remove commands that are not relevant on docker as we are root user
    sed -i 's|pkexec ||g' ./install.sh && \
    sed -i 's|sudo ||g' ./install.sh && \
    sed -i 's|sudo ||g' ./ursim-certificate-check.sh && \
    #
    # Install URSim
    ./install.sh && \
    #
    echo "Installed URSim"

RUN \
    echo "**** Clean Up ****" && \
    rm -rf \
    /tmp/* \
    /var/lib/apt/lists/* \
    /var/tmp/*

# Copy ursim run service script
COPY ursim /etc/services.d/ursim
COPY safety.conf.UR5 /ursim/.urcontrol/
# Expose ports
# Guacamole web browser viewer
EXPOSE 8080
# VNC viewer
EXPOSE 3389
# Modbus Port
EXPOSE 502
# Interface Ports
EXPOSE 29999
EXPOSE 30001-30004

# Mount Volumes
VOLUME /ursim

ENTRYPOINT ["/init"]
