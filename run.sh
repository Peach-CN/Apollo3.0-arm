注意：带nvidia@in_dev_docker:/apollo$  nvidia@in_dev_docker:/home/tmp/ros2$ 开头的是在docker内操作的，复制命令时不用复制这些



cd apollo

cp -r * /home/nvidia/work/AutoApollo/apollo

cd /home/nvidia/work/AutoApollo/apollo

./docker/scripts/dev_start.sh

/apollo/docker/ scripts/dev_into.sh

nvidia@in_dev_docker:/apollo$./apollo.sh build_opt_gpu




sudo cp ./rules/* /etc/udev/rules.d

sudo cp rc.local /etc




cp -r ros /apollo

/apollo/docker/scripts/dev_into.sh

nvidia@in_dev_docker:/apollo$ mv /home/tmp/ros/ /home/tmp/ros2
nvidia@in_dev_docker:/apollo$ sudo mv  ros/ /home/tmp/ros

nvidia@in_dev_docker:/apollo$ cd /home/tmp/ros2

nvidia@in_dev_docker:/home/tmp/ros2$ cp -r lib/lslidar_c16_d* lib/imu_100d2/ lib/liblslidar_* ../ros/lib/

nvidia@in_dev_docker:/home/tmp/ros2$ cp -r include/lslidar_c16_msgs/ ../ros/include/

nvidia@in_dev_docker:/home/tmp/ros2$ cp -r share/lslidar_c16* share/imu_100d2/ ../ros/share/

