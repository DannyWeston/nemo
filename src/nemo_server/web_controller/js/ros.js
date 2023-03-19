var connected = false;
var ros = null;

function setupImages(){
    const image_listener = new ROSLIB.Topic({
        ros, 
        name: '/nemo/image',
        messageType: 'sensor_msgs/CompressedImage'
    });

    // Listen for images
    image_listener.subscribe((message) => {
        document.getElementById('imgCameraView').src = "data:image/jpg;base64," + message.data;
    });
}

function setupOdometry(){
    let topic = document.getElementById('txtTopic').value;

    const odom_listener = new ROSLIB.Topic({
        ros,
        name: topic,
        messageType: "nav_msgs/Odometry",
    });

    const dp = 2;

    // Listen for odometry
    odom_listener.subscribe((message) => {
        document.getElementById("odomPosition").innerHTML = 
            message.pose.pose.position.x.toFixed(dp) + "<br>" + 
            message.pose.pose.position.y.toFixed(dp) + "<br>" + 
            message.pose.pose.position.z.toFixed(dp);

        document.getElementById("odomOrientation").innerHTML = 
            message.pose.pose.orientation.x.toFixed(dp) + "<br>" + 
            message.pose.pose.orientation.y.toFixed(dp) + "<br>" + 
            message.pose.pose.orientation.z.toFixed(dp);
    });
}

function connectToROS(){
    const endpoint = document.getElementById('txtEndpoint').value;

    ros = new ROSLIB.Ros({ url: "ws://" + endpoint });

    // When the Rosbridge server connects, fill the span with id "status" with "successful"
    ros.on("connection", () => {
        document.getElementById("status").innerHTML = "successful";
    });

    // When the Rosbridge server experiences an error, fill the "status" span with the returned error
    ros.on("error", (error) => {
        document.getElementById("status").innerHTML = `errored out (${error})`;
    });

    // When the Rosbridge server shuts down, fill the "status" span with "closed"
    ros.on("close", () => {
        document.getElementById("status").innerHTML = "closed";
    });

    setupImages();

    setupOdometry();
}