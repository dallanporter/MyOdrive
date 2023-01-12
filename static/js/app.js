
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
    first_odrive.on("encoder0.pos_circular", (pos) => {
        plot.series[0].addPoint([new Date().getTime(), pos], true);
    });
    first_odrive.on("encoder1.pos_circular", (pos) => {
        plot.series[1].addPoint([new Date().getTime(), pos], true);
    });
    first_odrive.on("vbus_voltage", onVbusVoltage);
    first_odrive.on("odrive_connected", onOdriveConnected);
    $("button.telemetry.button").on("click", () => {
        console.log("Button clicked!");
        first_odrive.toggleTelemetry();
    });
    first_odrive.on("telemetry_started", () => {
        console.log("Telemetry started...");
        $("button.telemetry.button").text("Stop Telemetry")
    });
    first_odrive.on("telemetry_stopped", () => {
        console.log("Telemetry stopped.");
        $("button.telemetry.button").text("Start Telemetry");
    });
}

function onVbusVoltage(voltage) {
    console.log("App got vbus_voltage!", voltage);
    let point = [new Date().getTime(), voltage];
    plot.series[2].addPoint(point, true);
}

function onOdriveDisconnected(serial_number) {
    console.log("App disconnected odrive");
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
            max: 1,
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

async function foo() {
    console.log("Inside foo");
    for (let sn in myodrive.odrives) {
        let od = myodrive.odrives[sn];
        //if (od.hasOwnProperty("serial_number")) {
            //let temp = await od.serial_number;
            od.serial_number
                .then(() => {
                    console.log("inside then.");
                });
            
        //}
    }
}