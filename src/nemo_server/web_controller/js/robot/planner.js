class Planner {
    constructor(rosObj){
        this.ros = rosObj;

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
    }

    autonomousControl(){
        $("#btnAutonomous").hide();
        $("#btnManual").show();
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