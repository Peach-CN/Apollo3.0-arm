sudo sh -c 'echo "/dev/nvme0n1p1       /home/nvidia/work     ext4           defaults                                     0 1" >> /etc/fstab'

mkdir -p /home/nvidia/work/AutoApollo/apollo

sudo usermod -aG docker $USER

sudo gpasswd -a $USER docker

newgrp docker


sudo systemctl restart docker.service


sudo systemctl stop docker.service


cd /var/lib

sudo cp -r -v docker /home/nvidia/work/


sudo mv docker docker2

sudo ln -s /home/nvidia/work/docker docker

