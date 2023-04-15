class Thruster {

    constructor(rosObj, topic){
        this.ros = rosObj;
        this.topic = topic;

        this.conn = new ROSLIB.Topic({
            ros: rosObj,
            name: topic,
            messageType: "geometry_msgs/Twist",
        });

        // Declare initial speeds
        this.linVel = { x: 0.0, y: 0.0, z: 0.0 };

        this.angVel = { x: 0.0, y: 0.0, z: 0.0 };

        this.linVelInc = 0.1;
        this.angVelInc = 0.1;

        this.minLinVel = -1.0;
        this.maxLinVel = 1.0;

        this.minAngVel = -1.0;
        this.maxAngVel = 1.0;
    }

    addForward(){
        this.linVel.x = clamp(this.linVel.x + this.linVelInc, this.minLinVel, this.maxLinVel);
    }

    addBackward(){
        this.linVel.x = clamp(this.linVel.x - this.linVelInc, this.minLinVel, this.maxLinVel);
    }

    setForward(value){
        this.linVel.x = clamp(roundToInc(value, this.linVelInc), this.minLinVel, this.maxLinVel);
    }

    setYaw(value){
        this.angVel.z = clamp(roundToInc(value, this.angVelInc), this.minAngVel, this.maxAngVel);
    }

    setUp(value){
        this.linVel.z = clamp(roundToInc(value, this.linVelInc), this.minLinVel, this.maxLinVel);
    }

    setRoll(value){
        this.angVel.x = clamp(roundToInc(value, this.angVelInc), this.minAngVel, this.maxAngVel);
    }

    reset(){
        this.linVel = { x: 0.0, y: 0.0, z: 0.0 };
        this.angVel = { x: 0.0, y: 0.0, z: 0.0 };
    }

    publish(){
        let msg = {
            linear : this.linVel,
            angular : this.angVel
        };

        this.conn.publish(new ROSLIB.Message(msg));
    }
}