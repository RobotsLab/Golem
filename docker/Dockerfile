# docker image build -t golem:1.0 . -f Dockerfile && docker save golem:1.0 | gzip > golem-v1.0.tgz

FROM ubuntu:16.04

WORKDIR /usr/src/

ENV TZ=Europe/London
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
#ENV DEBIAN_FRONTEND=noninteractive
#ENV DEBIAN_FRONTEND=teletype

RUN apt-get update
RUN apt-get -y install build-essential cmake git
RUN apt-get -y install libboost-dev
RUN apt-get -y install libpopt-dev zlib1g-dev 
RUN apt-get -y install libxmu-dev libxi-dev libproj-dev
RUN apt-get -y install libexpat-dev freeglut3-dev
RUN apt-get -y install libopencv-dev
RUN apt-get -y install libpcl-dev

RUN cd /usr/src && git config --global http.sslverify false && git clone https://github.com/RobotsLab/Golem.git Golem

RUN cd /usr/src/Golem && mkdir build && cd build && cmake build .. -DCMAKE_BUILD_TYPE=RELEASE && make install -j
