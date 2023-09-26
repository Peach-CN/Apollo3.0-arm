

docker load  -i dev-aarch64-cuda10-trt5-20200318_1200.tar
docker load -i localization_volume-aarch64-latest.tar
docker load -i map_volume-sunnyvale_big_loop-aarch64-latest.tar
docker load -i map_volume-sunnyvale_loop-aarch64-latest.tar
docker load -i yolo3d_volume-aarch64-latest.tar

docker tag 114ca2f30374 apolloauto/apollo:dev-aarch64-cuda10-trt5-20200318_1200
docker tag 7f5894582aee apolloauto/apollo:localization_volume-aarch64-latest
docker tag 239d3e015454 apolloauto/apollo:map_volume-sunnyvale_big_loop-aarch64-latest
docker tag a7a325ab7e63 apolloauto/apollo:map_volume-sunnyvale_loop-aarch64-latest
docker tag 39c06ade2b03 apolloauto/apollo:yolo3d_volume-aarch64-latest


