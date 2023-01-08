
var update_timer = null;
var update_interval = 5000;
var socket;
var odrives = {};
var myodrive = new MyOdrive();

$(() => {
    socket = io();
    console.log("Initializing the page...");
    
    

    if (socket) {        
        
        socket.on("connect", () => {
            console.log("Socket connected!");
            //myodrive.init(socket);
            myodrive.init(socket);
            onUpdateTimerTick(); // kick off the watch timer
        });

        socket.on("disconnect", () => {
            console.log("Socket disconnected. Remove all odrives and unregester events.");
            myodrive.uninit();
            clearTimeout(update_timer);
        });
    }
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