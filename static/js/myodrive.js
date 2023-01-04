(function($) {
    var myodrive = {};
    
    myodrive.name = "foobar";
    myodrive.socket = null;
    myodrive.init = (socket) => {
        console.log("Inside myodrive.init()");
        myodrive.socket = socket;
        myodrive.socket.on("list_odrives", myodrive.list_odrives);
        myodrive.socket.on("odrive", myodrive.odrive);
        console.log(this);
    };

    myodrive.list_odrives = (socket_message) => {
        console.log("Inside the mydrive.list_odrives handler.");

    };

    myodrive.odrive = (socket_message) => {
        console.log("Inside the mydrive.odrive handler.");
    };

    window.myodrive = myodrive;
}(jQuery));

var odrive = () => {
    return {
        
    };
};