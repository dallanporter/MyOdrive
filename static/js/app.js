
var update_timer = null;
var update_interval = 5000;
var odrives = {};
var myodrive = new MyOdrive();
var first_odrive = null;
var odrive_element = null;
var plot = null;
var plot_series = [];
var joystick = { x: [], y:[] };
var enable_joystick = false;
var modal_param = null;

$(() => {
    $(document).foundation();
    console.log("Initializing the page...");
    initPlot();
    myodrive.on("odrive_connected", onOdriveConnected);
});

function onOdriveConnected(new_odrive) {
    console.log("App connected odrive");
    first_odrive = new_odrive; // todo, handle multiple someday
    first_odrive.is_loaded = false; // set to true after the UI has been generated.
    //first_odrive.on("vbus_voltage", onVbusVoltage);
    /*
    first_odrive.on("axis0.encoder.pos_circular", (pos) => {
        plot.series[0].addPoint([new Date().getTime(), pos], true);
    });
    first_odrive.on("axis0.encoder.pos_estimate_counts", (pos) => {
        plot.series[1].addPoint([new Date().getTime(), pos], true);
    });
    first_odrive.on("axis1.encoder.pos_abs", (pos) => {
        plot.series[2].addPoint([new Date().getTime(), pos], true);
    });
    first_odrive.on("axis1.encoder.pos_circular", (pos) => {
        plot.series[3].addPoint([new Date().getTime(), pos], true);
    });
    first_odrive.on("axis1.encoder.pos_estimate_counts", (pos) => {
        plot.series[4].addPoint([new Date().getTime(), pos], true);
    });
    first_odrive.on("axis1.encoder.pos_abs", (pos) => {
        plot.series[5].addPoint([new Date().getTime(), pos], true);
    });
    first_odrive.on("vbus_voltage", onVbusVoltage);
    */
    first_odrive.on("odrive_connected", onOdriveConnected);
    $("button.telemetry.button").on("click", () => {
        console.log("Button clicked!");
        first_odrive.toggleTelemetry();
    });
    $("button.sync.button").on("click", () => {
        console.log("Sync button clicked!");
        first_odrive.sync();
    });
    first_odrive.on("telemetry_started", () => {
        console.log("Telemetry started...");
        $("button.telemetry.button").text("Stop Telemetry");
    });
    first_odrive.on("telemetry_stopped", () => {
        console.log("Telemetry stopped.");
        $("button.telemetry.button").text("Start Telemetry");
    });
    first_odrive.on("synced", onOdriveSynced);

    $(".telemetry.button, .sync.button, .save.button").prop("disabled", false);
    buildAxisHTML(new_odrive._odrive, 0);
    buildAxisHTML(new_odrive._odrive, 1);
}

function buildAxisHTML(axis, number) {
    let html = `<ul>
        <li>current_state: <span class="current_state"></span></li>
        <li>error: <span class="error"></span></li>
        <li>is_homed: <span class="is_homed"></span></li>
        <li>last_drv_fault: <span class="last_drv_fault"></span></li>
        <li>requested_state: <span class="requested_state"></span></li>
        <li>step_dir_active: <span class="step_dir_active"></span></li>
        <li>steps: <span class="steps"></span></li>
        </ul>
        `;
    $(".axis" + number + " .axis_properties").html(html);
    buildEncoderHTML(number);
}

function buildEncoderHTML(axis_number) {
    let html = `<ul>
        <li>calib_scan_response: <span class="calib_scan_response"></span></li>
        <li>count_in_cpr: <span class="count_in_cpr"></span></li>
        <li>delta_pos_cpr_counts: <span class="delta_pos_cpr_counts"></span></li>
        <li>error: <span class="error"></span></li>
        <li>hall_state: <span class="hall_state"></span></li>
        <li>index_found: <span class="index_found"></span></li>
        <li>phase: <span class="phase"></span></li>
        <li>pos_abs: <span class="pos_abs"></span></li>
        <li>pos_circular: <span class="pos_circular"></span></li>
        <li>pos_cpr_counts: <span class="pos_cpr_counts"></span></li>
        <li>pos_estimate: <span class="pos_estimate"></span></li>
        <li>pos_estimate_counts: <span class="pos_estimate_counts"></span></li>
        <li>shadow_count: <span class="shadow_count"></span></li>
        <li>spi_error_rate: <span class="spi_error_rate"></span></li>
        <li>vel_estimate: <span class="vel_estimate"></span></li>
        <li>vel_estimate_counts: <span class="vel_estimate_counts"></span></li>
        </ul>
        `;
    $(".axis" + axis_number + " .encoder").html(html);
}

function onVbusVoltage(voltage) {
    //console.log("App got vbus_voltage!", voltage);
    let point = [new Date().getTime(), voltage];
    plot.series[6].addPoint(point, true);
}

function onOdriveDisconnected(serial_number) {
    console.log("App disconnected odrive");
    $("button").prop("disabled", true);
}

function onUpdateTimerTick() {
    console.log("Tick.");
    /*
    if (socket && socket.connected) {
        socket.emit("list_odrives", null);
    }
    */
    update_timer = setTimeout(onUpdateTimerTick, update_interval);
}

function makeAccordion(obj, depth, parent, axis_name) {
    let html = `<ul class="depth_${depth}">`;
    for (let property in obj) {
        if (obj.hasOwnProperty(property)) {
            if (typeof(obj[property]) == "object") {
                html += `<li class="item" >
                <i class="fi-plus"></i>
                <a href="#" class="title" data-content="${axis_name}-${parent}-${property}">${property}</a>
                <div class="foo" id="${axis_name}-${parent}-${property}" >` +                          
                            makeAccordion(obj[property], depth + 1, parent + "-" + property, axis_name) +
                            `</div>
                        </li>`;
            } else {
                
                html += `<li class="item" >
                
                <div id="${axis_name}-${parent}-${property}" >
                <input class="telem" type="checkbox" name="${axis_name}-${parent}-${property}"></input>
                <label>${property}:</label> 
                <a href="#" data-open="parameter_modal" data-property="${axis_name}-${parent}-${property}">
                <span class="${axis_name} ${parent} ${property} value editable">${obj[property]}</span>
                </a>
                            </div>
                        </li>`;
            }
        }
    }
    html += `</ul>`;
    return html;
}

function makeAxis(axis, axis_name) {
    let html = "";
    for (let property in axis) {
        
        if (axis.hasOwnProperty(property)) {
            if (typeof(axis[property]) == "object") {
                html += `<li class="item" >
                <i class="fi-plus"></i>
                <a href="#" class="title" data-content="${axis_name}-${property}">${property}</a>
                <div class="foo" id="${axis_name}-${property}" >
                            ${makeAccordion(axis[property], 0, property, axis_name)}
                            </div>
                        </li>`;
            } else {
                
                html += `<li>
                <input class="telem" type="checkbox" name="${axis_name}-${property}"></input>
                <label>${property}:</label> 
                <a href="#" data-open="parameter_modal" data-property="${axis_name}-${property}">
                <span class="${axis_name} ${property} value editable">${axis[property]}</span>
                </a>
                </li>`;
            }
        }
    }
    return html;
}

function updateProperty(property, key, tree) {
    //console.log(`Updating property ${key}`, tree);    
    if (typeof(property) == "object") {
        for (let p in property) {
            tree.push(key);
            updateProperty(property[p], p, tree);
            tree.pop();
        }
    } else {
        let fullprop = tree.join("-") + key;
        //console.log(`Got to a non-object: ${key} = ${property} (${fullprop})`);
        $(`a[data-property=${fullprop}] span`).text(property);
    }
}

function updateExistingProperties(odrive_json) {
    console.log("udpateExistingProperties()");
    for (let property in odrive_json) {
        updateProperty(odrive_json[property], property, []);
    }
}

function onOdriveSynced(odrive_json) {
    console.log("Inside onOdriveSynced in app.js");
    if (first_odrive.is_loaded) {
        updateExistingProperties(odrive_json);
        return;
    }
    for (let property in odrive_json) {
        if (property == "class_name" || property == "iface") {
            continue;
        }
        if (odrive_json.hasOwnProperty(property)) {
            if (property == "axis0" || property == "axis1") {
                
                let axis = odrive_json[property];
                let axis_html = makeAxis(axis, property);
                $("." + property + " .tree").html(axis_html);
                
            } else if (typeof(odrive_json[property]) == "object") {
                
                let html = `<li class="item" >
                    <i class="fi-plus"></i>
                    <a href="#" class="title" data-content="${property}">${property}</a>
                            <div class="foo" id="${property}" >
                            ${makeAccordion(odrive_json[property], 0, property)}
                            </div>
                        </li>`;
                //$(html).appendTo("#root_properties");
                $("#root_properties").append(html);
                
            } else {
                let html = `<li>
                <input class="telem" type="checkbox" name="${property}"></input>
                <label>${property}:</label> 
                <a href="#" data-open="parameter_modal" data-property="${property}">
                <span class="value editable">${odrive_json[property]}</span>
                </a>
                </li>`;
                $(html).appendTo("ul.base_properties");
            }
        }
    }

    $(".axis0 button").prop("disabled", false);
    $(".axis1 button").prop("disabled", false);

    for (let prop in odrive_json.telemetry) {
        let prop_dashed = (odrive_json.telemetry[prop]).replaceAll(".", "-");
        $(`input[name=${prop_dashed}]`).prop("checked", true);
        addSeries(prop_dashed);
    }

    first_odrive.on("telem_complete", () => {
        plot.redraw(false);
    });

    $(".reveal").on("open.zf.trigger", (e, t) => {
        let prop = (t.data("property")).replaceAll("-", ".");
        let val = t.find(".value").text();
        console.log("Caught the modal open event !", prop);
        $("#parameter_modal .modal_property_name").text(prop);
        $("#parameter_modal input[name=modal_input]").prop("value", val);
        modal_param = { property: prop,
            value: val };
    });

    $(".reveal").on("close.zf.trigger", (e) => {
        console.log("Caught the modal close event!");
        modal_param = null;
    });


    let series_x = plot.addSeries({
        name: "joystick X",
        data: [],
        yAxis: 2,
    });
    let series_y = plot.addSeries({
        name: "joystick Y",
        data: [],
        yAxis: 2
    });
    first_odrive.on("joystick_y", (pos) => {
        let y = pos[1];
        joystick.y.push(y);
        series_y.addPoint([pos[0], y], false, false);
    });
    first_odrive.on("joystick_x", (pos) => {
        let x = pos[1];
        joystick.x.push(x);
        series_x.addPoint([pos[0], x], false, false);                
    });

    // add the checkboxes to the properties.
    $("input[type=checkbox].telem").on("change", (e) => {
        console.log("tick");
        let param_name = (e.currentTarget.name).replaceAll("-", ".");
        if (e.currentTarget.checked) {
            // add it
            first_odrive.addTelemetry(param_name);
            addSeries(param_name);
        } else {
            // remove it.
            first_odrive.removeTelemetry(param_name);
            removeSeries(param_name);
        }
    });

    function addSeries(param_name) {
        
        let series = plot.addSeries({
            name: param_name,
            data: [],
            yAxis: 0
        });
        first_odrive.on(param_name, (pos) => {
            series.addPoint([pos[0], pos[1]], false);            
        });
    }

    function removeSeries(param_name) {
        console.log("TODO - remove series");
    }

    // TEST register some telemetry items
    /*
    first_odrive.addTelemetry(["vbus_voltage", "error", "axis0.requested_state",
        "axis1.requested_state", "axis0.encoder.pos_abs", "axis1.encoder.pos_abs"]);
    */

    $(".foo").toggle();

    // register click handlers to the accordion
    $("li.item a.title").click(function(event) {
        console.log("foo");
        let depth = $(this).data("depth");
        let c = "#" + $(this).data("content");
        console.log("CSS Class: " + c);
        //$(c).removeClass("hide");
        $(c).toggle();
        event.preventDefault();
    });

    first_odrive.is_loaded = true;
}


function onTelemetryButtonClick() {
    console.log("Inside onTelemetry button click");
    if (!this._is_telem_running) {
        myodrive.startTelemetry(this._serial_number);
        this._is_telem_running = true;
    } else {
        myodrive.stopTelemetry(this._serial_number);
        this._is_telem_running = false;
    }
}

function initPlot() {
    console.log("Initializing plot.");
    plot = Highcharts.chart("plot", {
        chart: {
            type: "line",
            animation: false,
        },
        title: {
            text: "Odrive plot"
        },
        xAxis: {
            labels: {
                enabled: false,
            },
        },
        yAxis: [{
            title: {
                text: "Counts"
            },
            max: 2,
            min: -2,
            opposite: true,
        },{
            title: {
                text: "Voltage"
            },
            max: 24,
            min: 0,
        },{
            title: {
                text: "Joystick"
            },
            max: 0.5,
            min: -0.5
        }],
        plotOptions: {            
            series: {
                marker: {
                    enabled: false,
                    states: {
                        hover: {
                            enabled: false,
                        }
                    }
                },
            }        
        },
        series: plot_series
    });
}

function onAxis0MoveIncrementalPos() {
    console.log("onMoveIncrementalPos()");
    first_odrive.callRemoteFunction("axis0.controller.move_incremental", [0.03, true]);
}

function onAxis0MoveIncrementalNeg() {
    console.log("onMoveIncrementalNeg()");
    first_odrive.callRemoteFunction("axis0.controller.move_incremental", [-0.03, true]);
}

function onAxis1MoveIncrementalPos() {
    console.log("onMoveIncrementalPos()");
    first_odrive.callRemoteFunction("axis1.controller.move_incremental", [0.03, true]);
}

function onAxis1MoveIncrementalNeg() {
    console.log("onMoveIncrementalNeg()");
    first_odrive.callRemoteFunction("axis1.controller.move_incremental", [-0.03, true]);
}

function onAxis0Idle() {
    console.log("onAxis0Idle()");
    first_odrive.setRemoteProperty("axis0.requested_state", 1);
}

function onAxis0ClosedLoop() {
    console.log("onAxis0ClosedLoop()");
    first_odrive.setRemoteProperty("axis0.requested_state", 8);
}

function onAxis1Idle() {
    console.log("onAxis1Idle()");
    first_odrive.setRemoteProperty("axis1.requested_state", 1);
}

function onAxis1ClosedLoop() {
    console.log("onAxis1ClosedLoop()");
    first_odrive.setRemoteProperty("axis1.requested_state", 8);
}

function onAxis1Calibrate() {
    first_odrive.setRemoteProperty("axis1.requested_state", 7);
}

function onAxis0Calibrate() {
    first_odrive.setRemoteProperty("axis0.requested_state", 7);
}

function onSaveSettings() {
    if (confirm("Save settings to Odrive firmware?")) {
        first_odrive.callRemoteFunction("save_configuration", []);
    }
}

function onToggleJoystick(e) {
    console.log("onToggleJoystick");
    enable_joystick = !enable_joystick;
    if (enable_joystick) {
        myodrive.enableJoystick();
    } else {
        myodrive.disableJoystick();
    }
}

function onModalCancel(e) {
    console.log("onModalCancel()");
}

function onModalApply(e) {    
    let new_val = $("#parameter_modal input[name=modal_input]").val();
    
    console.log(`onModalApply() TODO: update ${modal_param.property} = ${new_val}`);
    first_odrive.setRemoteProperty(modal_param.property, new_val).then( () => {
        console.log("Promise finished - close this modal");
    });

}