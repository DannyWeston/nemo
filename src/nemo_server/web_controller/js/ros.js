/*
    Author: DannyWeston

    Makes use of the ./js/roslib.js file in order to interact with a ROS robot 
    on a given endpoint
*/


let connected = false;
let ros = null;
let endpoint = null;
let receivedImage = false;

function processImage(data){
    receivedImage = true;

    document.getElementById('imgCameraView').src = "data:image/jpg;base64," + data;
}

function setupImages(){
    let topic = "/nemo/image";

    const image_listener = new ROSLIB.Topic({
        ros, 
        name: topic,
        messageType: 'sensor_msgs/CompressedImage'
    });

    // Listen for images
    image_listener.subscribe((message) => {
        processImage(message.data)
    });
}

function setupOdometry(){
    let topic = "/nemo/odom";

    const odom_listener = new ROSLIB.Topic({
        ros,
        name: topic,
        messageType: "nav_msgs/Odometry",
    });

    const dp = 2;

    // Listen for odometry
    odom_listener.subscribe((message) => {
        let euler = quaternionToEuler(
            message.pose.pose.orientation.x, 
            message.pose.pose.orientation.y, 
            message.pose.pose.orientation.z, 
            message.pose.pose.orientation.w);

        document.getElementById("odomPositionX").innerHTML = message.pose.pose.position.x.toFixed(dp);
        document.getElementById("odomPositionY").innerHTML = message.pose.pose.position.y.toFixed(dp);
        document.getElementById("odomPositionZ").innerHTML = message.pose.pose.position.z.toFixed(dp);

        document.getElementById("odomOrientationX").innerHTML = euler[0].toFixed(dp);
        document.getElementById("odomOrientationY").innerHTML = euler[1].toFixed(dp);
        document.getElementById("odomOrientationZ").innerHTML = euler[2].toFixed(dp);
    });
}

function connectToROS(){
    endpoint = document.getElementById('txtEndpoint').value;

    ros = new ROSLIB.Ros({ url: "ws://" + endpoint });

    // When the Rosbridge server connects, fill the span with id "status" with "successful"
    ros.on("connection", () => {
        onConnect();
    });

    // When the Rosbridge server experiences an error, fill the "status" span with the returned error
    ros.on("error", (error) => {
        document.getElementById("status").innerHTML = `errored out (${error})`;
    });

    // When the Rosbridge server shuts down, fill the "status" span with "closed"
    ros.on("close", () => {
        onDisconnect();
    });

    setupImages();

    setupOdometry();
}

function disconnectFromROS(){
    ros.close();
}

function onConnect(){
    connected = true;

    document.querySelector("#connectionStatus > .innerSymbol").style.color = "green";
    document.querySelector("#connectionStatus > .innerText").innerText = "Connected to " + endpoint;

    document.getElementById("btnDisconnect").style.display = "inline";
    document.getElementById("btnConnect").style.display = "none";

    document.getElementById("batteryLevel").innerText = "100%";
}

function onDisconnect(){
    connected = false;

    document.querySelector("#connectionStatus > .innerSymbol").style.color = "red";
    document.querySelector("#connectionStatus > .innerText").innerText = "Disconnected";

    document.getElementById('imgCameraView').src = "./img/placeholder.jpg";

    receivedImage = false;

    document.getElementById("btnDisconnect").style.display = "none";
    document.getElementById("btnConnect").style.display = "inline";

    /* Clear metric elements */
    let elements = document.getElementsByClassName("rosMetric");
    for (let i = 0; i < elements.length; i++){
        elements.item(i).innerText = "";
    }
}

function startRobot(){
    if (!connected) return; // Do nothing if not connected
}

function pauseRobot(){
    if (!connected) return; // Do nothing if not connected
}

function manualControl(){
    if (!connected) return; // Do nothing if not connected
}

function autonomousControl(){
    if (!connected) return; // Do nothing if not connected
}

function showFishBoxes(){
    if (!connected) return; // Do nothing if not connected
}

function hideFishBoxes(){
    if (!connected) return; // Do nothing if not connected
}

function startRecording(){
    if (!connected) return; // Do nothing if not connected
}

function stopRecording(){
    if (!connected) return; // Do nothing if not connected
}