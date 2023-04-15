class Planner {
    constructor(rosObj){
        this.ros = rosObj;

        this.manualId = 4;
        this.autoId = 2;

        this.controlTopic = new ROSLIB.Topic({
            ros: rosObj,
            name: "/nemo/control",
            messageType: 'std_msgs/UInt8'
        });

        this.planningTopic = new ROSLIB.Topic({
            ros: rosObj,
            name: "/nemo/planner",
            messageType: 'std_msgs/UInt8'
        });

        this.controlTopic.subscribe((msg) => this.onControlMsg(msg));
        this.planningTopic.subscribe((msg) => this.onPlanningMsg(msg));

        this.manualControl();

        $("#btnStopRobot").hide();
        $("#btnStartRobot").show();

        $("#btnDisableLocRecord").hide();
        $("#btnEnableLocRecord").show();
    }

    startRobot(){
        $("#btnStartRobot").hide();
        $("#btnStopRobot").show();
    }

    pauseRobot(){
        $("#btnStopRobot").hide();
        $("#btnStartRobot").show();
    }

    manualControl(){
        this.inManualUI();

        var msg = new ROSLIB.Message({
            data : this.manualId
        });

        // Activate manual control
        this.controlTopic.publish(msg);
    }

    autonomousControl(){
        this.inAutoUI();

        var msg = new ROSLIB.Message({
            data : this.autoId
        });

        // Activate manual control
        this.controlTopic.publish(msg);
    }

    onControlMsg(msg){
        if (msg.data == this.manualId){
            // Switching to manual control
            this.inManualUI();
        }
        else if (msg.data == this.autoId){
            // Switching to autonomous control
            this.inAutoUI();
        }
    }

    onPlanningMsg(msg){
        
    }

    enableTracking(){
        $("#btnEnableLocRecord").hide();
        $("#btnDisableLocRecord").show();
    }

    disableTracking(){
        $("#btnDisableLocRecord").hide();
        $("#btnEnableLocRecord").show();
    }

    inManualUI(){
        $("#btnManual").hide();
        $("#btnAutonomous").show();
    }

    inAutoUI(){
        $("#btnAutonomous").hide();
        $("#btnManual").show();
    }
}