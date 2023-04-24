class Planner {
    constructor(rosObj){
        this.ros = rosObj;

        this.manualId = 4;
        this.autoId = 2;

        this.auto = false;

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

        this.setManualMode();

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

    setManualMode(){
        this.auto = false;
        
        this.inManualUI();

        var msg = new ROSLIB.Message({
            data : this.manualId
        });

        // Activate manual control
        this.controlTopic.publish(msg);
    }

    setAutonomousMode(){
        this.auto = true;

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
            this.auto = false;
            this.inManualUI();
        }
        else if (msg.data == this.autoId){
            // Switching to autonomous control
            this.auto = true;
            this.inAutoUI();
        }
    }

    onPlanningMsg(msg){
        
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