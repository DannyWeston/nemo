/*
    Author: DannyWeston

    Makes use of the ./js/roslib.js file in order to interact with a ROS robot 
    on a given endpoint
*/

let connected = false;
let ros = null;
let endpoint = null;

var camera;
var odom;
var battery;
var lights;
var planner;

$(window).on('load', () => $(".section").hide());

function connectToROS(){
    endpoint = document.getElementById('txtEndpoint').value;

    ros = new ROSLIB.Ros({ url: "ws://" + endpoint });

    // When the Rosbridge server connects, fill the span with id "status" with "successful"
    ros.on("connection", onConnect);

    // When the Rosbridge server experiences an error, fill the "status" span with the returned error
    ros.on("error", (error) => {
        document.getElementById("status").innerHTML = `errored out (${error})`;
    });

    // When the Rosbridge server shuts down, fill the "status" span with "closed"
    ros.on("close", onDisconnect);
}

function disconnectFromROS(){
    ros.close();
}

function onConnect(){
    connected = true;

    $("#connectionStatus > .innerSymbol").css("color", "green");
    $("#connectionStatus > .innerText").text("Connected to " + endpoint);

    $("#btnConnect").hide();
    $("#btnDisconnect").show();

    $(".section").show();
    
    // Setup nodes
    battery = new Battery(ros, "/nemo/battery");

    odom = new Odom(ros, "/nemo/odom");

    camera = new Camera(ros, "/nemo/image");

    planner = new Planner(ros, "/nemo/planner");

    lights = new Lights(ros, "/nemo/lights");
}

function onDisconnect(){
    connected = false;
    camera.receivedImage = false;

    $("#connectionStatus > .innerSymbol").css("color", "red");
    $("#connectionStatus > .innerText").text("Disconnected");

    $('#imgCameraView').attr("src", "./img/placeholder.jpg");

    $("#btnDisconnect").hide();
    $("#btnConnect").show();

    $(".section").hide();

    /* Clear metric elements */
    $(".clear-on-disc").text("");
}

function execIfConnected(func){
    if (connected) func();
    else (console.log("Not connected to a robot!"))
}