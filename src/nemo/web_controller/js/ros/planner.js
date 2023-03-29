class Planner {
    constructor(rosObj){
        this.ros = rosObj;

        this.manualId = 5;
        this.autoId = 3;

        this.controlTopic = new ROSLIB.Topic({
            ros: rosObj,
            name: "/nemo/control",
            messageType: 'std_msgs/UInt8'
        });

        this.planningTopic = new ROSLIB.Topic({
            ros: rosObj,
            name: "/nemo/planning",
            messageType: 'std_msgs/UInt8'
        });

        $("#btnStopRobot").hide();
        $("#btnStartRobot").show();

        $("#btnAutonomous").hide();
        $("#btnManual").show();

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
        $("#btnManual").hide();
        $("#btnAutonomous").show();

        var msg = new ROSLIB.Message({
            data : this.manualId
        });

        // Activate manual control
        this.controlTopic.publish(msg);
    }

    autonomousControl(){
        $("#btnAutonomous").hide();
        $("#btnManual").show();

        var msg = new ROSLIB.Message({
            data : this.autoId
        });

        // Activate manual control
        this.controlTopic.publish(msg);
    }

    enableTracking(){
        $("#btnEnableLocRecord").hide();
        $("#btnDisableLocRecord").show();
    }

    disableTracking(){
        $("#btnDisableLocRecord").hide();
        $("#btnEnableLocRecord").show();
    }
}