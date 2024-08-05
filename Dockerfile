FROM ghcr.io/agri-gaia/seerep_base:0.3.0

RUN apt-get -qq update && apt-get -qq install -y --no-install-recommends -y \
    texlive-latex-extra \
    texlive-fonts-recommended \
    cm-super dvipng \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt

RUN pip3 install -r /tmp/requirements.txt --ignore-installed PyYAML --no-cache-dir \
&& rm /tmp/requirements.txt
    
USER docker
WORKDIR /home/docker

RUN mkdir -p ros_ws/src host_dir

COPY seerep ros_ws/src/seerep
COPY seerep_benchmarking ros_ws/src/seerep_benchmarking

RUN cd ros_ws && source /opt/ros/noetic/setup.bash && \
    catkin build seerep_benchmarking -DCMAKE_BUILD_TYPE=Release

RUN echo "source /home/docker/ros_ws/devel/setup.bash" >> .bashrc
    
ENTRYPOINT ["/bin/bash"]
