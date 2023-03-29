class Lights {
    constructor(rosObj, topic){
        this.ros = rosObj;
        this.topic = topic;

        $("#btnEnableLights").hide();
        $("#btnDisableLights").show();

        this.listener = new ROSLIB.Topic({
            ros: rosObj,
            name: topic,
            messageType: "std_msgs/Bool",
        });

        this.listener.subscribe((msg) => this.onMessage(msg));
    }

    onMessage(msg){
        msg.data ? this.enable() : this.disable();
    }

    enable(){
        $("#btnEnableLights").hide();
        $("#btnDisableLights").show();
    }

    disable(){
        $("#btnDisableLights").hide();
        $("#btnEnableLights").show();
    }
}