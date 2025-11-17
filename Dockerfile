FROM espressif/idf:release-v5.5

WORKDIR /opt/esp/idf/
RUN . ./export.sh && pip3 install catkin_pkg lark-parser colcon-common-extensions empy==3.3.4

