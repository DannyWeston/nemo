/*
    Author: DannyWeston

    Makes use of the ./js/roslib.js file in order to interact with a ROS robot 
    on a given endpoint
*/

$(window).on('load', () => $(".section").hide());

class RosConnection {
    constructor(){
        this.ros = null;
        this.endpoint = null;

        this.planner = null;
        this.lights = null;
        this.odom = null;
        this.battery = null;
        this.camera = null;
        this.thruster = null;

        this.controller = new ControllerInput();

        $(this.controller).on("buttonpressed", (e, i) => this.buttonInput(i));
        $(this.controller).on("axismoved", (e, axisId, value) => this.axisInput(axisId, value));

        this.connected = false;

        this.axisAngDampening = 0.5;
        this.axisLinDampening = 0.5;
    }

    onConnect(){
        this.connected = true;

        $("#rosConnStatus > .innerSymbol").css("color", "green");
        $("#rosConnStatus > .innerText").text("Connected to " + this.endpoint);

        $("#btnConnect").hide();
        $("#btnDisconnect").show();

        $(".section").show();
        
        // Setup nodes
        this.battery = new Battery(this.ros, "/nemo/battery");

        this.odom = new Pose(this.ros, "/nemo/pose");

        this.camera = new Camera(this.ros, "/nemo/image");

        this.planner = new Planner(this.ros, "/nemo/planner");

        this.lights = new Lights(this.ros, "/nemo/lights");

        this.thruster = new Thruster(this.ros, "/nemo/props");
    }

    onDisconnect(){
        this.connected = false;
        
        if (this.camera != null) this.camera.receivedImage = false;

        $("#rosConnStatus > .innerSymbol").css("color", "red");
        $("#rosConnStatus > .innerText").text("Disconnected");

        $('#imgCameraView').attr("src", "./img/placeholder.jpg");

        $("#btnDisconnect").hide();
        $("#btnConnect").show();

        $(".section").hide();

        /* Clear metric elements */
        $(".clear-on-disc").text("");

        this.planner = null;
        this.lights = null;
        this.odom = null;
        this.battery = null;
        this.camera = null;
        this.thruster = null;
    }

    connect(){
        this.endpoint = $('#txtEndpoint').val();

        this.ros = new ROSLIB.Ros({ url: "ws://" + this.endpoint });

        // When the Rosbridge server connects, fill the span with id "status" with "successful"
        this.ros.on("connection", () => this.onConnect());

        // When the Rosbridge server experiences an error, fill the "status" span with the returned error
        this.ros.on("error", (error) => {
            // Do nothing for now
        });

        // When the Rosbridge server shuts down, fill the "status" span with "closed"
        this.ros.on("close", () => this.onDisconnect());
    }

    disconnect(){
        this.ros.close();
    }

    buttonInput(buttonId){
        switch (buttonId){
            case 1: // B-button
                if (!!this.thruster && !this.planner.auto) {
                    this.thruster.reset();
                    this.thruster.publish();
                }         
                break;

            case 2:
                if (!!this.camera){
                    if(this.camera.recording){this.camera.stopRecording();}
                    else {this.camera.startRecording();}
                }
                break;

            case 3: // Y-button
                if(!!this.planner){
                    if (this.planner.auto){
                        this.planner.setManualMode();
                    }
                    else {
                        this.planner.setAutonomousMode();
                    }
                }
                break;

            case 5:
                if (!!this.lights){
                    this.lights.increase();
                    console.log("Increase");
                }
                break;
            case 4:
                if (!!this.lights){
                    this.lights.decrease();
                    console.log("Decrease");
                }
                break;

            default:
                return;
        }

        
    }

    axisInput(axisId, value){
        switch (axisId){
            case 0: // Left-right right stick
                if (!!this.thruster && !this.planner.auto) {
                    this.thruster.setYaw(value * this.axisAngDampening);
                    this.thruster.publish();
                }

            case 1: // Up-down left stick
                if (!!this.thruster && !this.planner.auto) {
                    this.thruster.setForward(-value * this.axisLinDampening);
                    this.thruster.publish();
                }
                break;

            case 2: // Up-down right stick
                if (!!this.thruster && !this.planner.auto) {
                    this.thruster.setRoll(-value * this.axisAngDampening);
                    this.thruster.publish();
                }
                break;

            case 3: // Left-right right stick
                if (!!this.thruster && !this.planner.auto) {
                    this.thruster.setUp(-value * this.axisLinDampening);
                    this.thruster.publish();
                }
                break;
        }
    }
}

const rc = new RosConnection();