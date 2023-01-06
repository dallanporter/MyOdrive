/*
(function($) {
    var myodrive = {};
    
    myodrive.name = "foobar";
    myodrive.socket = null;
    myodrive.odrives = {};
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
*/

class MyOdrive {
    constructor() {
        this.odrives = {};
    }

    init(socket) {
        this.socket = socket;
        this.socket.on("list_odrives", this.onListOdrives.bind(this));
        this.socket.on("odrive", this.onOdrive.bind(this));
    }

    onListOdrives(message) {
        console.log("Inside onListOdrives()", message);
        if (Array.isArray(message)) {
            for (let i=0; i<message.length; i++) {
                let sn = message[i];
                if (!this.odrives.hasOwnProperty(sn)) {
                    this.odrives[sn] = new Odrive(sn);
                }
            }
        }
    }

    onOdrive(message) {
        console.log("Inside onOdrive()");
    }
}

class Odrive {
    constructor(serial_number) {
        this._serial_number = serial_number;
    }

    get serial_number() {
        return new Promise( resolve => {
            setTimeout(() => {
                resolve("foobar");
            }, 4000);
        });
    }
}