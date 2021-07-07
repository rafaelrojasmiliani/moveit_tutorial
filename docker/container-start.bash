

transfer_x11_permissions() {
    # store X11 access rights in temp file to be passed into docker container
    XAUTH=/tmp/.docker.xauth
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
}

main(){

    scriptdir=$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)
    if [ "$scriptdir" != "$(pwd)" ]; then
      echo "this script must be executed from $scriptdir".
      exit 1
    fi

    transfer_x11_permissions

    docker run -it \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="XAUTHORITY=$XAUTH" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume $(pwd)/../:/catkinws \
        --volume="$XAUTH:$XAUTH" \
        moveit_docker bash
}

main
