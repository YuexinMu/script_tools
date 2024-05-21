export ROS_ENVIRONMENT="ros2"
echo "ROS_ENVIRONMENT: ${ROS_ENVIRONMENT}"

if [ "$ROS_ENVIRONMENT" == "ros2" ]; then
        source /opt/ros/foxy/setup.bash
        #source /home/myx/develop/Swarm-SLAM/install/setup.bash
        #export PYTHONPATH=$PYTHONPATH:/home/myx/develop/Swarm-SLAM/src/cslam

        export ROS_DOMAIN_ID=1
        echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
elif [ "$ROS_ENVIRONMENT" == "ros1" ]; then
        source  /opt/ros/noetic/setup.bash
        export ROS_IP=`ifconfig | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1' | grep -v '172.17.0.1'`
        if  test -z "${ROS_IP}"
                then
                export ROS_IP=`dig +short localhost`
        fi

#       export ROS_MASTER_URI=http://192.168.43.31:11311
        export ROS_MASTER_URI=http://localhost:11311
        export ROBOT_TYPE=engineer

        echo -e "ROS IP: ${ROS_IP} \nROS Master URI: ${ROS_MASTER_URI} \nROBOT_TYPE: ${ROBOT_TYPE}"
else
        "ROS_ENVIRONMENT: {$ROS_ENVIRNOMENT} is not ros1 or ros2."
fi

