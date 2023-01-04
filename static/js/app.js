
var update_timer = null;
var update_interval = 5000;
var socket;
var odrives = {};

$(() => {
    socket = io();
    console.log("Initializing the page...");
    setTimeout(onUpdateTimerTick, update_interval);
    

    if (socket) {        
        
        socket.on("connect", () => {
            console.log("Socket connected!");
            myodrive.init(socket);
        });
        
        
    }
});

function onUpdateTimerTick() {
    console.log("Tick.");
    if (socket) {
        socket.emit("list_odrives", "NA");
    }
    setTimeout(onUpdateTimerTick, update_interval);
}