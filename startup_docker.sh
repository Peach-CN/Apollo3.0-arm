#!/bin/bash

# Just for starting ADS when machine power on
sleep 5s

# Start docker
/apollo/docker/scripts/dev_start.sh
sleep 5s

# into docker
if [ -e /apollo/docker/scripts/dev_into.sh ]; then
  /apollo/docker/scripts/dev_into.sh

  sleep 5s
  docker ps --format "{{.Names}}" | grep -Fx 'apollo_dev_nvidia' 1>/dev/null
# docker exec
  if [ $? == 0 ]; then
    docker exec -u $USER apollo_dev_nvidia bash -c 'APOLLO_BIN_PREFIX=/apollo/bazel-bin NVBLAS_CONFIG_FILE=/usr/local/cuda JRE_HOME=/usr/lib/java/jre LD_LIBRARY_PATH=/home/tmp/ros/lib::/usr/local/lib:/usr/lib/aarch64-linux-gnu/tegra:/usr/local/ipopt/lib:/usr/local/cuda/lib64/stubs:/apollo/lib:/apollo/bazel-genfiles/external/caffe/lib:/home/caros/secure_upgrade/depend_lib CPATH=/home/tmp/ros/include PATH=/home/tmp/ros/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/lib/java/bin:/apollo/scripts:/usr/local/miniconda2/bin/ JAVA_HOME=/usr/lib/java HOME=/home/nvidia PYTHONPATH=/usr/local/lib/python2.7/dist-packages:/apollo/py_proto:/usr/local/apollo/snowboy/Python:/apollo/modules/tools:/home/tmp/ros/lib/python2.7/dist-packages CLASSPATH=.:/usr/lib/java/lib:/usr/lib/java/jre/lib PKG_CONFIG_PATH=/home/tmp/ros/lib/pkgconfig CMAKE_PREFIX_PATH=/home/tmp/ros /apollo/scripts/bootstrap.sh'
  fi
fi

