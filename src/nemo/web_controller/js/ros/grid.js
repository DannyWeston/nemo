class OccupancyGrid {
    constructor(rosObj, topic){
        this.topic = topic;
        this.rosObj = rosObj;

        this.mapViewer = new ROS2D.Viewer({
            divID : 'map',
            width : 512,
            height : 512,
        })

        this.gridClient = new ROS2D.OccupancyGridClient({
            ros : rosObj,
            rootObject : this.mapViewer.scene,
            topic: topic,
            continuous: true
        });

        this.gridClient.on("change", () => {
            this.mapViewer.scaleToDimensions(this.gridClient.currentGrid.width, this.gridClient.currentGrid.height);
            this.mapViewer.shift(this.gridClient.currentGrid.pose.position.x, this.gridClient.currentGrid.pose.position.y);
        });
        
        this.hideMap();
    }

    showMap()
    {
        $("#btnShowMap").hide();
        $("#btnHideMap").show();

        $("#map").show();
    }

    hideMap()
    {    
        $("#btnHideMap").hide();
        $("#btnShowMap").show();

        $("#map").hide();
    }
}