class Camera {
    constructor(rosObj, topic){
        this.ros = rosObj;
        this.topic = topic;

        this.receivedImage = false;

        $('#imgCameraView').show();

        $('#btnHideFishBoxes').hide();
        $('#btnShowFishBoxes').show();

        $('#btnStopRecording').hide();
        $('#btnStartRecording').show();

        this.listener = new ROSLIB.Topic({
            ros: rosObj, 
            name: topic,
            messageType: 'sensor_msgs/CompressedImage'
        });

        this.publisher = new ROSLIB.Topic({
            ros: rosObj, 
            name: "/nemo/recorder",
            messageType: 'std_msgs/UInt8'
        });

        this.recording = false;

        this.listener.subscribe((msg) => this.onMessage(msg));
    }

    onMessage(msg){
        this.receivedImage = true;
        $('#imgCameraView').attr("src", "data:image/jpg;base64," + msg.data);
    }

    showFishBoxes(){
        $('#btnShowFishBoxes').hide();
        $('#btnHideFishBoxes').show();
    }
    
    hideFishBoxes(){
        $('#btnShowFishBoxes').show();
        $('#btnHideFishBoxes').hide();
    }

    startRecording(){
        this.recording = true;

        $('#btnStartRecording').hide();
        $('#btnStopRecording').show();

        this.publisher.publish(new ROSLIB.Message({
            data : 1
        }));
    }

    stopRecording(){
        this.recording = false;
        
        $('#btnStopRecording').hide();
        $('#btnStartRecording').show();

        this.publisher.publish(new ROSLIB.Message({
            data : 0
        }));
    }
}