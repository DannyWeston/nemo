class Lights {
    constructor(rosObj, topic){
        this.ros = rosObj;
        this.topic = topic;

        this.data = 0;

        this.step = 8;

        this.min = 0;
        this.max = 255;

        $("#sliderLights").show();

        this.listener = new ROSLIB.Topic({
            ros: rosObj,
            name: topic,
            messageType: "std_msgs/UInt8",
        });

        $("#sliderLights").on('input', () => {
            this.publish(parseInt($("#sliderLights").val()))
        });

        this.listener.subscribe((msg) => this.onMessage(msg));

        this.publish(parseInt($("#sliderLights").val()))
    }

    increase(){
        this.data = clamp(this.data + this.step, this.min, this.max);
        
        $("#sliderLights").val(this.data);

        this.publish(this.data);
    }

    decrease(){
        this.data = clamp(this.data - this.step, this.min, this.max);
        $("#sliderLights").val(this.data);

        this.publish(this.data);
    }

    publish(value){
        this.listener.publish(new ROSLIB.Message({
            data : value
        }));
    }

    onMessage(msg){
        this.value = msg.data;

        $("#sliderLights").val(msg.data);
    }
}