function ros_raven() {
    export ROS_MASTER_URI=http://192.168.0.19:11311
    export ROS_IP=$(hostname -I)
    export ROS_HOSTNAME=$(hostname -I)

    env | grep ROS_MASTER_URI
    env | grep ROS_IP
    env | grep ROS_HOSTNAME
}

function ros_local() {
    export ROS_MASTER_URI=http://localhost:11311
    export ROS_IP=$(hostname -I)
    export ROS_HOSTNAME=$(hostname -I)

    env | grep ROS_MASTER_URI
    env | grep ROS_IP
    env | grep ROS_HOSTNAME
}


