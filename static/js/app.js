
var update_timer = null;
var update_interval = 5000;
var socket;
var odrives = {};
var myodrive = new MyOdrive();

$(() => {
    socket = io();
    console.log("Initializing the page...");
    setTimeout(onUpdateTimerTick, update_interval);
    

    if (socket) {        
        
        socket.on("connect", () => {
            console.log("Socket connected!");
            //myodrive.init(socket);
            myodrive.init(socket);
        });
        
        
    }

    setInterval(foo, 4000);
});

function onUpdateTimerTick() {
    console.log("Tick.");
    if (socket && socket.connected) {
        socket.emit("list_odrives", null);
    }
    
    setTimeout(onUpdateTimerTick, update_interval);
}

async function foo() {
    console.log("Inside foo");
    for (let sn in myodrive.odrives) {
        let od = myodrive.odrives[sn];
        //if (od.hasOwnProperty("serial_number")) {
            let temp = await od.serial_number;
            console.log("Foo got temp=" + temp);
        //}
    }
}