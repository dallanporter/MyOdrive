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

    uninit() {
        console.log("uninitializing MyOdrive due to socket disconnect");
        if (this.socket) {
            this.socket.off("list_odrives");
            this.socket.off("odrive");
            this.removeAllOdrives();
        }

    }

    onListOdrives(message) {
        console.log("Inside onListOdrives()", message);
        if (Array.isArray(message)) {
            for (let i=0; i<message.length; i++) {
                let sn = message[i];
                if (!this.odrives.hasOwnProperty(sn)) {
                    let od = new Odrive(sn); 
                    od.initSocket(self.socket);
                    this.odrives[sn] = od; 
                }
            }
        }
    }

    onOdrive(message) {
        console.log("Inside onOdrive()");
    }

    removeAllOdrives() {
        console.log("MyOdrive.removeAllDrives() called.");
        this.odrives = {};
    }
}

class Odrive {
    constructor(serial_number) {
        this._serial_number = serial_number;
        this._socket = null;
    }

    initSocket(socket) {
        this._socket = socket;
        if (socket) {
            socket.on(this._serial_number, this.onSocketSync);
        }
    }

    onSocketSync(message) {
        console.log("Inside Odrive.onSocketSync()", message);
    }

    get serial_number() {
        return new Promise( resolve => {
            setTimeout(() => {
                resolve("foobar");
            }, 4000);
        });
    }
}