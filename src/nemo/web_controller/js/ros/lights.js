class Lights {
    constructor(rosObj, topic){
        this.ros = rosObj;
        this.topic = topic;

        $("#sliderLights").show();

        this.listener = new ROSLIB.Topic({
            ros: rosObj,
            name: topic,
            messageType: "std_msgs/UInt8",
        });

        $("#sliderLights").on('input', () => {
            this.sendUpdate(parseInt($("#sliderLights").val()))
        });

        this.listener.subscribe((msg) => this.onMessage(msg));

        this.sendUpdate(parseInt($("#sliderLights").val()))
    }

    sendUpdate(value){
        this.listener.publish(new ROSLIB.Message({
            data : value
        }));
    }

    onMessage(msg){
        $("#sliderLights").val(msg.data);
    }
}