#!/usr/bin/env vash

myuid=$(id -u $USER)
mygid=$(id -g $USER)
mygroup=$(id -g -n $USER)

docker build -t "moveit_docker" \
    --no-cache -f ./image_nvidia.dockerfile .

exit 0

