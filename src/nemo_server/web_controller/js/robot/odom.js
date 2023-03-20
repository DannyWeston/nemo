class Odom {
    constructor(rosObj, topic){
        this.ros = rosObj;
        this.topic = topic;

        this.listener = new ROSLIB.Topic({
            ros,
            name: topic,
            messageType: "nav_msgs/Odometry",
        });

        this.listener.subscribe((msg) => this.onMessage(msg));
    }

    onMessage(msg){
        const dp = 2;

        let euler = quaternionToEuler(
            msg.pose.pose.orientation.x, 
            msg.pose.pose.orientation.y, 
            msg.pose.pose.orientation.z, 
            msg.pose.pose.orientation.w
        );

        $("#odomPositionX").text(msg.pose.pose.position.x.toFixed(dp));
        $("#odomPositionY").text(msg.pose.pose.position.y.toFixed(dp));
        $("#odomPositionZ").text(msg.pose.pose.position.z.toFixed(dp));

        $("#odomOrientationX").text(euler[0].toFixed(dp));
        $("#odomOrientationY").text(euler[1].toFixed(dp));
        $("#odomOrientationZ").text(euler[2].toFixed(dp));
    }
}