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
        console.log("MyOdrive.init(socket)");
        this.socket = socket;
        this.socket.on("list_odrives", this.onListOdrives.bind(this));
        this.socket.on("odrive", this.onOdrive.bind(this));
        for (let sn in this.odrives) {
            this.odrives[sn].initSocket(socket);
        }
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
        this._odrive = null;
        this._socket = null;
        this.element = odrive_element.clone().appendTo(".odrive_collection");
        this.element.css("display", "inline-block");
    }

    initSocket(socket) {
        console.log("Inside Odrive.initSocket(socket)");
        this._socket = socket;
        if (socket) {
            socket.on(this._serial_number, this.onSocketSync.bind(this));
            socket.of("/" + this._serial_number).on("telem", this.onSocketTelem.bind(this));
        }
    }

    onSocketTelem(message) {
        console.log("Got telem", message);
    }

    onSocketSync(message) {
        if (this._odrive == null) {
            this._odrive = message;
        } else {
            $.extend(this._odrive, message);
        }
        for (let key in this._odrive) {
            if (this._odrive.hasOwnProperty(key)) {
                let prop = this.element.find(".property." + key);
                if (prop.length == 0) {
                    // create a new one.
                    let val = this._odrive[key];
                    $("<li class=\"property_row\">" + key + ": <span class=\"property " + key + "\">--</span></li>")
                        .appendTo(this.element.find(".property_list"));
                }
            }
        }
        this.element.find(".property.serial_number").text(this._serial_number);
        this.element.find(".property.vbus_voltage").text(message.vbus_voltage);
    }

    get serial_number() {
        return new Promise( resolve => {
            setTimeout(() => {
                resolve("foobar");
            }, 4000);
        });
    }
}