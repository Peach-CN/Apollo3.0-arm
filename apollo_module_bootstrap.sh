#!/bin/bash

case $1 in
  gnss)
    ./bazel-bin/modules/drivers/gnss/gnss  --flagfile=/apollo/modules/drivers/gnss/conf/gnss.conf --log_dir=/apollo/data/log --stderrthreshold=3 --alsologtostderr=1
    ;;
  rtk_planning)
    ./bazel-bin/modules/planning/planning --flagfile=/apollo/modules/planning/conf/planning.conf --log_dir=/apollo/data/log --stderrthreshold=3 --alsologtostderr=1
    ;;
  navi_planning)
    ./bazel-bin/modules/planning/planning --flagfile=/apollo/modules/planning/conf/planning_navi.conf  --log_dir=/apollo/data/log --use_navigation_mode --stderrthreshold=7 --alsologtostderr=1
    ;;
  planning)
    ./bazel-bin/modules/planning/planning --flagfile=/apollo/modules/planning/conf/planning.conf  --log_dir=/apollo/data/log --stderrthreshold=7 --alsologtostderr=1
    ;;
  localization)
    ./bazel-bin/modules/localization/localization --flagfile=/apollo/modules/localization/conf/localization.conf --localization_config_file=/apollo/modules/localization/conf/navi_localization_config.pb.txt --log_dir=/apollo/data/log
    ;;
  relative_map)
    ./bazel-bin/modules/map/relative_map/relative_map --flagfile=/apollo/modules/map/relative_map/conf/relative_map.conf --stderrthreshold=3 --alsologtostderr=1 --log_dir=/apollo/data/log --use_navigation_mode
    ;;
  navi_perception)
    ./bazel-bin/modules/perception/perception --flagfile=/apollo/modules/perception/conf/perception_lowcost.conf --use_navigation_mode --blob_input_width=960 --blob_input_height=384 --enable_zed=false --zed_img_width=1920 --zed_img_high=1080 --log_dir=/apollo/data/log --stderrthreshold=3 --alsologtostderr=1 --use_vision_latern=0 --latern_number=1 --save_framenum=5
    ;;
  bd_perception):
    ./bazel-bin/modules/perception/perception --flagfile=/apollo/modules/perception/conf/perception_lowcost.conf --use_navigation_mode --blob_input_width=960 --blob_input_height=384 --enable_zed=true --zed_img_width=1280 --zed_img_high=720 --log_dir=/apollo/data/log --stderrthreshold=3 --alsologtostderr=1 --use_vision_latern=0 --latern_number=1 --save_framenum=1
    ;;
  zk_perception):
    ./bazel-bin/modules/perception/perception --flagfile=/apollo/modules/perception/conf/perception_lowcost.conf --nouse_navigation_mode --blob_input_width=960 --blob_input_height=384 --enable_zed=true --zed_img_width=1280 --zed_img_high=720 --log_dir=/apollo/data/log --stderrthreshold=3 --alsologtostderr=1 --use_vision_latern=0 --latern_number=1 --save_framenum=1
    ;;
  gr_perception):
    ./bazel-bin/modules/perception/perception --flagfile=/apollo/modules/perception/conf/perception_lowcost.conf --use_navigation_mode --blob_input_width=672 --blob_input_height=376 --enable_zed=true --zed_img_width=1280 --zed_img_high=720 --save_framenum=10 --log_dir=/apollo/data/log --stderrthreshold=3 --alsologtostderr=1 --use_vision_latern=0 --latern_number=1 --save_framenum=5
    ;;
  prediction)
    ./bazel-bin/modules/prediction/prediction --flagfile=/apollo/modules/prediction/conf/prediction_navi.conf --log_dir=/apollo/data/log
    ;;
  navi_control)
    ./bazel-bin/modules/control/control --flagfile=/apollo/modules/control/conf/control.conf --use_navigation_mode=true --stderrthreshold=3 --alsologtostderr=1 --log_dir=/apollo/data/log
    ;;
  canbus)
    ./bazel-bin/modules/canbus/canbus --flagfile=/apollo/modules/canbus/conf/canbus.conf --log_dir=/apollo/data/log --stderrthreshold=3 --alsologtostderr=1
    ;;
  ultrasonic_radar)
    ./bazel-bin/modules/drivers/radar/ultrasonic_radar/ultrasonic_radar  --flagfile=/apollo/modules/drivers/radar/ultrasonic_radar/conf/ultrasonic_radar.conf --log_dir=/apollo/data/log --stderrthreshold=3 --alsologtostderr=1
    ;;
  monitor)
    ./bazel-bin/modules/monitor/monitor --flagfile=/apollo/modules/monitor/conf/monitor.conf --log_dir=/apollo/data/log --stderrthreshold=3 --alsologtostderr=1
    ;;
  control)
    ./bazel-bin/modules/control/control --flagfile=/apollo/modules/control/conf/control.conf --stderrthreshold=3 --alsologtostderr=1 --log_dir=/apollo/data/log
    ;;
  *)
    echo "Unknown module"
esac

