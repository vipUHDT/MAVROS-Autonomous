#!/bin/bash

ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate \
"{stream_id: 0, message_rate: 100, on_off: true}"
