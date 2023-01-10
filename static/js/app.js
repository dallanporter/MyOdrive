
var update_timer = null;
var update_interval = 5000;
var odrives = {};
var myodrive = new MyOdrive();
var odrive_element = null;
var plot = null;
var plot_series = {
    name: "Bus Voltage",
    data: []
};

$(() => {
    $(document).foundation();
    console.log("Initializing the page...");
    
    odrive_element = $(".odrive_container").css("display", "none");
    
    
    initPlot();

});

function onUpdateTimerTick() {
    console.log("Tick.");
    /*
    if (socket && socket.connected) {
        socket.emit("list_odrives", null);
    }
    */
    
    update_timer = setTimeout(onUpdateTimerTick, update_interval);
}

function initPlot() {
    console.log("Initializing plot.");
    plot = Highcharts.chart("plot", {
        chart: {
            type: "line"
        },
        title: {
            text: "Odrive plot"
        },
        xAxis: {

        },
        yAxis: {

        },
        plotOptions: {
            series: {}
        },
        series: [plot_series]
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