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

        this.connected = false;
    }

    onConnect(){
        this.connected = true;

        $("#connectionStatus > .innerSymbol").css("color", "green");
        $("#connectionStatus > .innerText").text("Connected to " + this.endpoint);

        $("#btnConnect").hide();
        $("#btnDisconnect").show();

        $(".section").show();
        
        // Setup nodes
        this.battery = new Battery(this.ros, "/nemo/battery");

        this.odom = new Odom(this.ros, "/nemo/odom");

        this.camera = new Camera(this.ros, "/nemo/image");

        this.planner = new Planner(this.ros, "/nemo/planner");

        this.lights = new Lights(this.ros, "/nemo/lights");
    }

    onDisconnect(){
        this.connected = false;
        
        if (this.camera != null) this.camera.receivedImage = false;

        $("#connectionStatus > .innerSymbol").css("color", "red");
        $("#connectionStatus > .innerText").text("Disconnected");

        $('#imgCameraView').attr("src", "./img/placeholder.jpg");

        $("#btnDisconnect").hide();
        $("#btnConnect").show();

        $(".section").hide();

        /* Clear metric elements */
        $(".clear-on-disc").text("");
    }
        
    connect(){
        this.endpoint = $('#txtEndpoint').val();

        this.ros = new ROSLIB.Ros({ url: "ws://" + this.endpoint });

        // When the Rosbridge server connects, fill the span with id "status" with "successful"
        this.ros.on("connection", () => this.onConnect());

        // When the Rosbridge server experiences an error, fill the "status" span with the returned error
        this.ros.on("error", (error) => {
            alert(error);
        });

        // When the Rosbridge server shuts down, fill the "status" span with "closed"
        this.ros.on("close", () => this.onDisconnect());
    }

    disconnect(){
        this.ros.close();
    }
}

const rc = new RosConnection();