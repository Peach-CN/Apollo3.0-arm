#!/usr/bin/env bash


DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${DIR}"

# the machine type, currently support x86_64, aarch64
MACHINE_ARCH=$(uname -m)
source ${DIR}/scripts/apollo_base.sh

TIME=$(date  +%Y%m%d_%H%M)
if [ -z "${DOCKER_REPO}" ]; then
    DOCKER_REPO=apolloauto/apollo
fi

function print_usage() {
  RED='\033[0;31m'
  BLUE='\033[0;34m'
  BOLD='\033[1m'
  NONE='\033[0m'

  echo -e "\n${RED}Usage${NONE}:
  .${BOLD}/apollo_docker.sh${NONE} [OPTION]"

  echo -e "\n${RED}Options${NONE}:
  ${BLUE}build${NONE}: run build only
  ${BLUE}build_opt${NONE}: build optimized binary for the code
  ${BLUE}build_gpu${NONE}: run build only with Caffe GPU mode support
  ${BLUE}build_opt_gpu${NONE}: build optimized binary with Caffe GPU mode support
  ${BLUE}build_fe${NONE}: compile frontend javascript code, this requires all the node_modules to be installed already
  ${BLUE}buildify${NONE}: fix style of BUILD files
  ${BLUE}check${NONE}: run build/lint/test, please make sure it passes before checking in new code
  ${BLUE}clean${NONE}: run Bazel clean
  ${BLUE}config${NONE}: run configurator tool
  ${BLUE}coverage${NONE}: generate test coverage report
  ${BLUE}doc${NONE}: generate doxygen document
  ${BLUE}lint${NONE}: run code style check
  ${BLUE}usage${NONE}: print this menu
  ${BLUE}release${NONE}: build release version
  ${BLUE}test${NONE}: run all unit tests
  ${BLUE}version${NONE}: display current commit and date
  ${BLUE}push${NONE}: pushes the images to Docker hub
  ${BLUE}gen${NONE}: generate a docker release image
  ${BLUE}ota_gen${NONE}: generate a ota docker release image
  "
}

function start_build_docker() {
  docker ps --format "{{.Names}}" | grep apollo_dev 1>/dev/null 2>&1
  if [ $? != 0 ]; then    
    # If Google is reachable, we fetch the docker image directly. 
    if ping -q -c 1 -W 1 www.google.com 1>/dev/null 2>&1; then
      opt=""
    # If Google is unreachable but Baidu reachable, we fetch the docker image from China. 
    elif ping -q -c 1 -W 1 www.baidu.com 1>/dev/null 2>&1; then
      opt="-C"
    # If Baidu is unreachable, we use local images. 
    else
      opt="-l"
    fi
    #echo ${opt}
    bash docker/scripts/dev_start.sh ${opt}
  fi
}

function gen_docker() {
  IMG="apolloauto/apollo:run-${MACHINE_ARCH}-20180906_2002"
  RELEASE_DIR=${HOME}/.cache/apollo_release
  APOLLO_DIR="${RELEASE_DIR}/apollo"

  if [ ! -d "${APOLLO_DIR}" ]; then
    echo "Release directory does not exist!"
    exit 1
  fi

  RELEASE_NAME="${DOCKER_REPO}:release-${MACHINE_ARCH}-${TIME}"
  DEFAULT_NAME="${DOCKER_REPO}:release-${MACHINE_ARCH}-latest"
  docker pull $IMG
  echo "time : ${TIME}" >> ${APOLLO_DIR}/meta.ini
  echo "tag: ${RELEASE_NAME}" >> ${APOLLO_DIR}/meta.ini

  docker ps -a --format "{{.Names}}" | grep 'apollo_release' 1>/dev/null
  if [ $? == 0 ];then
    docker stop apollo_release 1>/dev/null
    docker rm -f apollo_release 1>/dev/null
  fi
  docker run -it \
      -d \
      --name apollo_release \
      --net host \
      -v $HOME/.cache:/root/mnt \
      -w /apollo \
      "$IMG"

  if [ "$OTA_RELEASE" != "1" ]; then
    docker exec apollo_release bash -c 'cp -Lr /root/mnt/apollo_release/apollo /'
  else
    RELEASE_TGZ="apollo_release.tar.gz"
    SEC_RELEASE_TGZ="sec_apollo_release.tar.gz"

    if [ -e "$HOME/.cache/$RELEASE_TGZ" ]; then
      rm $HOME/.cache/$RELEASE_TGZ
    fi

    if [ -e "$HOME/.cache/$SEC_RELEASE_TGZ" ]; then
      rm $HOME/.cache/$SEC_RELEASE_TGZ
    fi

    # generate security release package
    tar czf $HOME/.cache/$RELEASE_TGZ -C $HOME/.cache apollo_release
    python modules/tools/ota/create_sec_package.py
    docker exec apollo_release cp /root/mnt/${SEC_RELEASE_TGZ} /root
  fi

  CONTAINER_ID=$(docker ps | grep apollo_release | awk '{print $1}')
  docker commit "$CONTAINER_ID" "$DEFAULT_NAME"
  docker tag "$DEFAULT_NAME" "$RELEASE_NAME"
  docker stop "$CONTAINER_ID"
}

function push() {
  DEFAULT_NAME="${DOCKER_REPO}:release-${MACHINE_ARCH}-latest"
  LATEST_IMAG_ID=$(docker images --format "{{.Repository}}:{{.Tag}} {{.ID}}" | grep "${DEFAULT_NAME}" | cut -d " " -f 2)
  RELEASE_NAME=$(docker images --format "{{.Repository}}:{{.Tag}} {{.ID}}" | grep ${LATEST_IMAG_ID}| grep -v "${DEFAULT_NAME}" | cut -d " " -f 1)
  docker push "$DEFAULT_NAME"
  docker push "$RELEASE_NAME"
}

if [ $# == 0 ];then
    print_usage
    exit 1
fi

start_build_docker

case $1 in
  print_usage)
    print_usage
    ;;
  push)
    push
    ;;
  gen)
    gen_docker
    ;;
  ota_gen)
    OTA_RELEASE=1
    gen_docker
    ;;
  *)
    docker exec -u $USER apollo_dev_nvidia bash -c "ROS_ROOT=/home/tmp/ros/share/ros ROS_PACKAGE_PATH=/home/tmp/ros/share:/home/tmp/ros/stacks APOLLO_BIN_PREFIX=/apollo/bazel-bin ROS_MASTER_URI=http://localhost:11311 NVBLAS_CONFIG_FILE=/usr/local/cuda JRE_HOME=/usr/lib/java/jre LD_LIBRARY_PATH=/home/tmp/ros/lib::/usr/local/lib:/usr/lib/aarch64-linux-gnu/tegra:/usr/local/ipopt/lib:/usr/local/cuda/lib64/stubs:/apollo/lib:/apollo/bazel-genfiles/external/caffe/lib:/home/caros/secure_upgrade/depend_lib APOLLO_IN_DOCKER=true CPATH=/home/tmp/ros/include ROS_DOMAIN_ID=68191977 PATH=/home/tmp/ros/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/lib/java/bin:/apollo/scripts:/usr/local/miniconda2/bin/ JAVA_HOME=/usr/lib/java SHLVL=1 HOME=/home/nvidia ROS_DISTRO=indigo PYTHONPATH=/usr/local/lib/python2.7/dist-packages:/apollo/py_proto:/usr/local/apollo/snowboy/Python:/apollo/modules/tools:/home/tmp/ros/lib/python2.7/dist-packages CLASSPATH=.:/usr/lib/java/lib:/usr/lib/java/jre/lib PKG_CONFIG_PATH=/home/tmp/ros/lib/pkgconfig CMAKE_PREFIX_PATH=/home/tmp/ros DISPLAY=localhost:10.0 APOLLO_BASE_SOURCED=1 ROS_ETC_DIR=/home/tmp/ros/etc/ros ./apollo.sh $@"
    ;;
esac
