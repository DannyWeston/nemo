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
            ros, 
            name: topic,
            messageType: 'sensor_msgs/CompressedImage'
        });

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
        $('#btnStartRecording').hide();
        $('#btnStopRecording').show();
    }

    stopRecording(){
        $('#btnStopRecording').hide();
        $('#btnStartRecording').show();
    }
}