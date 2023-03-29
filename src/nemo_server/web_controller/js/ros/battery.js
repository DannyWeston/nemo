class Battery {
    constructor(rosObj, topic){
        this.ros = rosObj;
        this.topic = topic;

        this.listener = new ROSLIB.Topic({
            ros: rosObj,
            name: topic,
            messageType: "sensor_msgs/BatteryState",
        });

        this.listener.subscribe((msg) => this.onMessage(msg));
    }

    onMessage(msg){
        const dp = 2;
        
        $("#batPercentage").text(parseInt(msg.percentage * 100));
        $("#batDischarge").text(msg.current.toFixed(dp));
    
        let chargeMsg = "Unknown";
        if (msg.power_supply_status == 1) chargeMsg = "True";
        else if (msg.power_supply_status == 2) chargeMsg = "False";
        $("#batCharging").text(chargeMsg);
    
        $("#batCharge").text(msg.charge.toFixed(dp));
        $("#batCapacity").text(msg.capacity.toFixed(dp));
        $("#batVoltage").text(msg.voltage.toFixed(dp));
    }
}