class Pose {
    constructor(rosObj, topic){
        this.ros = rosObj;
        this.topic = topic;

        this.listener = new ROSLIB.Topic({
            ros: rosObj,
            name: topic,
            messageType: "geometry_msgs/Pose",
        });

        this.listener.subscribe((msg) => this.onMessage(msg));
    }

    onMessage(msg){
        const dp = 2;

        let euler = quaternionToEuler(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        );

        $("#posePosX").text(msg.position.x.toFixed(dp));
        $("#posePosY").text(msg.position.y.toFixed(dp));
        $("#posePosZ").text(msg.position.z.toFixed(dp));

        $("#poseRotX").text(euler[2].toFixed(dp));
        $("#poseRotY").text(euler[1].toFixed(dp));
        $("#poseRotZ").text(euler[0].toFixed(dp));
    }
}