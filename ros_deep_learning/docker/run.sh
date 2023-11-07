#!/usr/bin/env bash

source docker/tag.sh

ROS_DISTRO="foxy"
CONTAINER_IMAGE=""

USER_VOLUME=""
USER_COMMAND=""

show_help() {
    echo " "
    echo "usage: Starts the Docker container and runs a user-specified command"
    echo " "
    echo "   ./scripts/docker_run.sh --container DOCKER_IMAGE"
    echo "                           --volume HOST_DIR:MOUNT_DIR"
    echo "                           --run RUN_COMMAND"
    echo " "
    echo "args:"
    echo " "
    echo "   --help                       Show this help text and quit"
    echo " "
    echo "  --ros DISTRO Specifies the ROS distro to use, one of:"
    echo "               'noetic', 'foxy', 'galactic', or 'humble'"
    echo "               The default distro to use is foxy."
    echo " "
    echo "   -c, --container DOCKER_IMAGE Specifies the name of the Docker container"
    echo "                                image to use (default: ros:foxy-pytorch)"
    echo " "
    echo "   -v, --volume HOST_DIR:MOUNT_DIR Mount a path from the host system into"
    echo "                                   the container.  Should be specified as:"
    echo " "
    echo "                                      -v /my/host/path:/my/container/path"
    echo " "
    echo "   -r, --run RUN_COMMAND  Command to run once the container is started."
    echo "                          Note that this argument must be invoked last,"
    echo "                          as all further arguments will form the command."
    echo "                          If no run command is specified, an interactive"
    echo "                          terminal into the container will be provided."
    echo " "
}

die() {
    printf '%s\n' "$1"
    show_help
    exit 1
}

# parse arguments
while :; do
    case $1 in
        -h|-\?|--help)
            show_help    # Display a usage synopsis.
            exit
            ;;
	    --ros)    # Takes an option argument; ensure it has been specified.
            if [ "$2" ]; then
                ROS_DISTRO=$2
                shift
            else
                die 'ERROR: "--ros" requires a non-empty option argument.'
            fi
            ;;
        --ros=?*)
            ROS_DISTRO=${1#*=} # Delete everything up to "=" and assign the remainder.
            ;;
        --distror=)         # Handle the case of an empty --distro=
            die 'ERROR: "--ros" requires a non-empty option argument.'
            ;;
        -c|--container)       # Takes an option argument; ensure it has been specified.
            if [ "$2" ]; then
                CONTAINER_IMAGE=$2
                shift
            else
                die 'ERROR: "--container" requires a non-empty option argument.'
            fi
            ;;
        --container=?*)
            CONTAINER_IMAGE=${1#*=} # Delete everything up to "=" and assign the remainder.
            ;;
        --container=)         # Handle the case of an empty --image=
            die 'ERROR: "--container" requires a non-empty option argument.'
            ;;
        -v|--volume)
            if [ "$2" ]; then
                USER_VOLUME=" -v $2 "
                shift
            else
                die 'ERROR: "--volume" requires a non-empty option argument.'
            fi
            ;;
        --volume=?*)
            USER_VOLUME=" -v ${1#*=} " # Delete everything up to "=" and assign the remainder.
            ;;
        --volume=)         # Handle the case of an empty --image=
            die 'ERROR: "--volume" requires a non-empty option argument.'
            ;;
        -r|--run)
            if [ "$2" ]; then
                shift
                USER_COMMAND=" $@ "
            else
                die 'ERROR: "--run" requires a non-empty option argument.'
            fi
            ;;
        --)              # End of all options.
            shift
            break
            ;;
        -?*)
            printf 'WARN: Unknown option (ignored): %s\n' "$1" >&2
            ;;
        *)               # Default case: No more options, so break out of the loop.
            break
    esac

    shift
done

# if the container image wasn't explicitly set with -c option, set it now
if [[ -z "$CONTAINER_IMAGE" ]]; then
	CONTAINER_IMAGE="ros:$ROS_DISTRO-pytorch-l4t-$CONTAINER_VERSION"
fi

# check for local image
if [[ "$(sudo docker images -q $CONTAINER_IMAGE 2> /dev/null)" == "" ]]; then
	CONTAINER_IMAGE="dustynv/$CONTAINER_IMAGE"
fi

# check for V4L2 devices
V4L2_DEVICES=" "

for i in {0..9}
do
	if [ -a "/dev/video$i" ]; then
		V4L2_DEVICES="$V4L2_DEVICES --device /dev/video$i "
	fi
done

# check for display
DISPLAY_DEVICE=" "

if [ -n "$DISPLAY" ]; then
	# give docker root user X11 permissions
	sudo xhost +si:localuser:root
	
	# enable SSH X11 forwarding inside container (https://stackoverflow.com/q/48235040)
	XAUTH=/tmp/.docker.xauth
	xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
	chmod 777 $XAUTH

	DISPLAY_DEVICE="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix -v $XAUTH:$XAUTH -e XAUTHORITY=$XAUTH"
fi

#echo "CONTAINER_IMAGE: $CONTAINER_IMAGE"
#echo "USER_VOLUME:     $USER_VOLUME"
#echo "USER_COMMAND:    '$USER_COMMAND'"
#echo "V4L2_DEVICES:    $V4L2_DEVICES"
#echo "DISPLAY_DEVICE:  $DISPLAY_DEVICE"

# run the container
sudo docker run --runtime nvidia -it --rm --network host \
	$DISPLAY_DEVICE $V4L2_DEVICES \
	$USER_VOLUME $CONTAINER_IMAGE $USER_COMMAND
