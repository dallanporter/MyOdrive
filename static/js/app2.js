var socket = null;
var stage = null;


$( () => {
    console.log("The window onload eventhanler has been called");
    socket = io();
    $("#save_button").on("click", onSaveToOdriveClick);
    $("input").on("focus", onInputFocusHandler);
    $("button.button_arm").on("click", onArmButtonClick);
    $("button.button_calibrate_axis").on("click", onCalibrateAxisButtonClick);
    $("button.button_calibrate_motor").on("click", onCalibrateMotorButtonClick);
    if (socket) {
        
        socket.on("connect", () => {
            console.log("Socket connected!");
            socket.on("telem", (payload, callback) => {
                //console.log("Got a telem message!", payload);
                try {
                    payload = JSON.parse(payload);            
                    for(let key in payload) {
                        // Handle special non text/span elements like radio groups first
                        switch (key) {
                            // If the element for the key is in focus or pending a change,
                            // ignore changing it in the update cycle                            
                            case "telem_rate":
                                
                                let e = $("input[name='telem_rate']");
                                if (e) {
                                    if (e.hasClass('pending') || e.hasClass('in_focus')) {
                                        continue;
                                    } else {
                                        // radio group
                                        // uncheck them all first                                
                                        e.prop("checked", null);
                                        $("input[name='telem_rate'][value='" + payload[key] + "']").prop("checked", true);
                                    }
                                }
                                break;
                            default:
                                let val = odrive.formatValue(key, payload[key]);
                                let class_key = key.replaceAll(".", "--"); // hack because we cannot use . in a class name.
                                let element = $("#" + class_key);
                               
                                
                                if (element.length > 0) {
                                    if (element.hasClass("pending") || element.hasClass("in_focus")) {
                                        continue; // ignore updating it since it's being/been modified.
                                    }
                                    if (element[0].nodeName == "INPUT") {
                                        if (element[0].type == "text") {
                                            element[0].value = val;
                                        } else {
                                            console.warn("Unknown input type", key);
                                        }
                                        
                                    } else if (element[0].nodeName == "SPAN") {
                                        element.text(val);
                                    } else {
                                        console.warn("Unknown element type", key);
                                    }
                                }
                                break;
                        }
                        odrive.current_state = payload;
                        updateButtonStates();
                    }
                    
                } catch(e) {
                    console.error(e);
                    odrive.current_state = null;
                }
            });    
        });        
    }

    stage = new createjs.Stage("main_canvas");
    let circle = new createjs.Shape();
    circle.graphics.beginFill("red").drawCircle(0, 0, 40);
    circle.x = circle.y = 50;
    stage.addChild(circle);
    stage.update();
});


function onSaveToOdriveClick() {
    console.log("You clicked the button!");
    if (confirm("Save changes to the Odrive?")) {
        let params = {};
        params.foo = "bar";
        socket.emit("update_params", params);
    }
}

function updateButtonStates() {
    $("button").prop("disabled", true);
    if (odrive.current_state == null) {
        console.warn("odrive state is not set. Disabling the buttons");        
        return;
    }
    if (odrive.current_state["axis0.current_state"] == odrive.states.AXIS_STATE_IDLE) {
        $("button[name='button_calibrate_axis0']").prop("disabled", false);
        $("button[name='button_calibrate_motor0']").prop("disabled", false);
    }
    if (odrive.current_state["axis1.current_state"] == odrive.states.AXIS_STATE_IDLE) {
        $("button[name='button_calibrate_axis1']").prop("disabled", false);
        $("button[name='button_calibrate_motor1']").prop("disabled", false);
    }
}

function onInputFocusHandler(element) {
    console.log("Inside onInputFocusHandler", element);
    $(this).addClass("in_focus");
    $(this).parent().addClass("pending_highlight");
}

function onArmButtonClick() {
    let element = $(this);
    console.log("Clicked the 'arm' button for axis " + element.prop("name"));
}

function onCalibrateAxisButtonClick() {
    let element = $(this);
    let matches = element.prop("name").match(/^button_calibrate_(.+)$/);
    let axis = matches[1];
    console.log("Clicked the 'calibrate' button for " + axis);
    
    if (confirm("Calibrate " + axis)) {
        //socket.emit("calibrate_axis", axis);
        socket.emit("set_param", [axis + ".requested_state", odrive.states.AXIS_STATE_FULL_CALIBRATION_SEQUENCE]);
    }
}

function onCalibrateMotorButtonClick() {
    let element = $(this);
    console.log("Clicked the 'calibrate' button for motor " + element.prop("name"));
}