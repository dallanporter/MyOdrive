

class MyOdrive {
    constructor() {
        this.odrives = {};
        this.socket = io();
        this.socket.on("connect", this.onSocketConnect.bind(this));
        this.socket.on("disconnect", this.onSocketDisconnect.bind(this));
        this.socket.on("list_odrives", this.onListOdrives.bind(this));
        this.socket.on("odrive", this.onOdrive.bind(this));
        this.socket.on("telem", this.onSocketTelemetry.bind(this));
        this.socket.on("sync", this.onSocketSync.bind(this));
    }   

    init(socket) {
        console.log("MyOdrive.init(socket)");
        
    }

    onSocketSync(message) {
        console.log("MyOdrive.onSocketSync()");
        try {
            // It should be an objected with key being the serial number
            // and value the odrive object
            let val = null;
            let serial_number = null;
            for (let prop in message) {
                if (message.hasOwnProperty(prop)) {
                    serial_number = prop;
                    val = message[prop];
                    break;
                }
            }
            let odrive = this.getOdrive(serial_number);
            if (odrive == null) {
                console.log("MyOdrive doesn't seem to have an odrive with serial number " + serial_number);
            } else {
                odrive.syncRemote(val);
            }
            
        } catch (ex) {
            console.warn(ex);
        }
    }

    onSocketTelemetry(message) {
        console.log("MyOdrive.onSocketTelemetry()");
        let m = this.helperGetSerialMessage(message);
        if (m) {
            let odrive = this.getOdrive(m.serial_number);
            console.log(m.serial_number, m.payload)
        }
    }

    onSocketConnect(message) {
        console.log("MyOdrive.onSocketConnect()");
        
    }

    onSocketDisconnect(message) {
        console.log("MyOdrive.onSocketDisconnect()");
        console.log("TODO - make the odrives stale or delete them all??");
    }

    uninit() {
        console.log("uninitializing MyOdrive due to socket disconnect");
        if (this.socket) {
            this.socket.off("list_odrives");
            this.socket.off("odrive");
            this.removeAllOdrives();
        }

    }

    startTelemetry(which_odrive) {
        console.log("MyOdrive.startTelemetry for odrive " + which_odrive);
        if (this.socket) {
            this.socket.emit("telem_start", which_odrive);
        }
    }

    stopTelemetry(which_odrive) {
        console.log("MyOdrive.stopTelemetry for odrive " + which_odrive);
        if (this.socket) {
            this.socket.emit("telem_stop", which_odrive);
        }
    }


    onListOdrives(message) {
        console.log("Inside onListOdrives()", message);
        if (Array.isArray(message)) {
            for (let i=0; i<message.length; i++) {
                let sn = message[i];
                if (!this.odrives.hasOwnProperty(sn)) {
                    console.log("MyOdrive creating new Odrive(" + sn + ")");
                    let od = new Odrive(sn, this.socket); 
                    this.odrives[sn] = od; 
                    this.socket.emit("sync", sn); // get this new odrive state
                }
            }
        }
    }

    getOdrive(serial_number) {
        if (this.odrives.hasOwnProperty(serial_number)) {
            return this.odrives[serial_number];
        } else {
            return null;
        }
    }

    onOdrive(message) {
        console.log("Inside onOdrive()");
    }

    removeAllOdrives() {
        console.log("MyOdrive.removeAllDrives() called.");
        this.odrives = {};
    }

    /**
     * A helper that takes an object message from socket.io
     * and returns the odrive serial number and the message
     * payload
     * { <serial_number>: <payload> }
     * @param {Object} obj 
     */
    helperGetSerialMessage(obj) {
        let serial_number = null;
        let payload = null;
        try {
            for (let prop in message) {
                if (message.hasOwnProperty(prop)) {
                    serial_number = prop;
                    payload = message[prop];
                    break;
                }
            }
        } catch (ex) {
            return false;
        }

        let out = {};
        out.serial_number = serial_number;
        out.payload = payload;
        return out;
    }
}

class Odrive {
    constructor(serial_number, socket) {
        this._serial_number = serial_number;
        this._odrive = null;
        this._is_telem_running = false;
        this.element = odrive_element.clone().appendTo(".odrive_collection");
        this.element.css("display", "inline-block");
        this.element.find(".button.telemetry").on("click", this.onTelemetryButtonClick.bind(this));

    }

    onTelemetryButtonClick() {
        console.log("Inside onTelemetry button click");
        if (!this._is_telem_running) {
            myodrive.startTelemetry(this._serial_number);
            this._is_telem_running = true;
        } else {
            myodrive.stopTelemetry(this._serial_number);
            this._is_telem_running = false;
        }
    }


    syncRemote(message) {
        if (this._odrive == null) {
            this._odrive = message;
        } else {
            $.extend(this._odrive, message);
        }
        /*
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
        */
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