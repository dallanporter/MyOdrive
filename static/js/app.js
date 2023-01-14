
var update_timer = null;
var update_interval = 5000;
var odrives = {};
var myodrive = new MyOdrive();
var first_odrive = null;
var odrive_element = null;
var plot = null;
var plot_series = [{
    name: "Encoder 0",
    data: [],
    yAxis: 0,
},{
    name: "Encoder 1",
    data: [],
    yAxis: 0,
},{
    name: "Vbus Voltage",
    yAxis: 1,
    data: []
}];

$(() => {
    $(document).foundation();
    console.log("Initializing the page...");
    initPlot();
    myodrive.on("odrive_connected", onOdriveConnected);
});

function onOdriveConnected(new_odrive) {
    console.log("App connected odrive");
    first_odrive = new_odrive; // todo, handle multiple someday
    //first_odrive.on("vbus_voltage", onVbusVoltage);
    first_odrive.on("encoder0.shadow_count", (pos) => {
        plot.series[0].addPoint([new Date().getTime(), pos], true);
    });
    first_odrive.on("encoder1.shadow_count", (pos) => {
        plot.series[1].addPoint([new Date().getTime(), pos], true);
    });
    first_odrive.on("vbus_voltage", onVbusVoltage);
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

    $(".telemetry.button, .sync.button").prop("disabled", false);
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
    console.log("App got vbus_voltage!", voltage);
    let point = [new Date().getTime(), voltage];
    plot.series[2].addPoint(point, true);
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
                <label>${property}:</label> <span class="${axis_name} ${parent} ${property} value">${obj[property]}</span>
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
                <label>${property}:</label> <span class="${axis_name} ${property} value">${axis[property]}</span>
                </li>`;
            }
        }
    }
    return html;
}

function onOdriveSynced(odrive_json) {

    for (let property in odrive_json) {
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
                <label>${property}:</label> ${odrive_json[property]}                
                </li>`;
                $(html).appendTo("ul.base_properties");
            }
        }
    }
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
}

/*
function onOdriveSynced(odrive_json) {
    console.log("App synced odrive");
    // loop through the odrive_json and update the html
    for (let property in odrive_json) {
        if (odrive_json.hasOwnProperty(property)) {
            //console.log("property", property);
            if (property == "axis0" || property == "axis1") {
                let axis = odrive_json[property];
                for (let axis_property in axis) {
                    if (axis.hasOwnProperty(axis_property)) {
                        //console.log("axis_property", axis_property);
                        if (typeof(axis[axis_property]) != "object") {
                            $(".axis0 .axis_properties ." + axis_property).html(axis[axis_property]);
                            $(".axis1 .axis_properties ." + axis_property).html(axis[axis_property]);
                        } else if (axis_property == "encoder") {
                            let encoder = axis[axis_property];
                            for (let encoder_property in encoder) {
                                if (encoder.hasOwnProperty(encoder_property)) {
                                    //console.log("encoder_property", encoder_property);
                                    $("." + property + " .encoder ." + encoder_property).html(encoder[encoder_property]);
                                    
                                }
                            }
                        } else if (axis_property == "controller") {

                        } else if (axis_property == "motor") { 

                        } else if (axis_property == "acim_estimator") {

                        } else if (axis_property == "trap_traj") {

                        } else if (axis_property == "config") {

                        }
                    }
                }
            }
        }
    }
    if (odrive_json.axis0.current_state == 1) {
        $(".axis0 .calibrate.button").prop("disabled", false);
        $(".axis0 .closed_loop.button").prop("disabled", false);
    }
    if (odrive_json.axis1.current_state == 1) {
        $(".axis1 .calibrate.button").prop("disabled", false);
        $(".axis1 .closed_loop.button").prop("disabled", false);
    }
}
*/

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
            max: 10000,
            min: 0,
            opposite: true,
        },{
            title: {
                text: "Voltage"
            },
            max: 24,
            min: 0,
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
