class ControllerInput {
    constructor(){
        $(window).on("gamepadconnected", (e) => this.connected());
        $(window).on("gamepaddisconnected", (e) => this.disconnected());

        this.gamepad = null;

        this.poller = null;

        //this.inputRate = 1000.0 / 30.0;
        this.inputRate = 100.0;
    }   

    connected(){
        this.gamepad = navigator.getGamepads()[0];

        $("#contConn > .innerSymbol").css("color", "green");
        $("#contConn > .innerText").text("Connected");

        this.poller = setInterval(() => this.pollController(), this.inputRate);
    }

    disconnected(){
        $("#contConn > .innerSymbol").css("color", "red");
        $("#contConn > .innerText").text("Disconnected");
        
        if (navigator.getGamepads().length === 0){
            clearInterval(this.poller);
        }
    }

    pollController(){
        const buttons = this.gamepad.buttons;
        const axes = this.gamepad.axes;

        buttons.forEach((button, i) => {
            if (typeof button === "object" && button.value === 1.0) {
                $(this).trigger("buttonpressed", i);
            }
        });

        axes.forEach((axis, i) => {
            $(this).trigger("axismoved", [i, axis]);
        });
    }
}