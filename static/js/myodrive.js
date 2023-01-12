

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

        this.event_subscribers = {};
        this.on = (event_name, callback) => {
            if (!this.event_subscribers.hasOwnProperty(event_name)) {
                this.event_subscribers[event_name] = [];
            }
            this.event_subscribers[event_name].push(callback);
        };
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
            if (odrive) {
                odrive.syncTelemetry(m.payload);
            }
            //console.log(m.serial_number, m.payload);
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

    callRemoteFunction(serial_number, function_name, args) {
        console.log("MyOdrive.callRemoteFunction()");
        if (this.socket) {
            let request_id = "set_" + Date.now();
            let payload = {
                request_id: request_id,
                serial_number: serial_number,
                function_name: function_name,
                args: args
            };
            this.socket.emit("call", payload);
            return new Promise((resolve, reject) => {
                this.socket.once(request_id, (message) => {
                    console.log("MyOdrive.callRemoteFunction() got response");
                    console.log(message);
                    resolve(true);
                });
                setTimeout(() => {
                    reject("Timeout");
                }, 1000);
            });
        } else {
            return Promise.reject("No socket connection");
        }
    }

    setRemoteProperty(serial_number, property_name, value) {
        console.log("MyOdrive.setRemoteProperty()");
        if (this.socket) {
            let request_id = "set_" + Date.now();
            let payload = {
                request_id: request_id,
                serial_number: serial_number,
                property_name: property_name,
                value: value
            };
            this.socket.emit("set", payload);
            return new Promise((resolve, reject) => {
                this.socket.once(request_id, (message) => {
                    console.log("MyOdrive.setRemoteProperty() got response");
                    console.log(message);
                    resolve(true);
                });
                setTimeout(() => {
                    reject("Timeout");
                }, 1000);
            });
        } else {
            return Promise.reject("No socket connection");
        }

    }

    startTelemetry(which_odrive) {
        console.log("MyOdrive.startTelemetry for odrive " + which_odrive);
        if (this.socket) {
            this.socket.emit("telem_start", which_odrive);
            let od = this.getOdrive(which_odrive);
            if (od) {
                od.broadcastEvent("telemetry_started", true);
            }
        }
    }

    stopTelemetry(which_odrive) {
        console.log("MyOdrive.stopTelemetry for odrive " + which_odrive);
        if (this.socket) {
            this.socket.emit("telem_stop", which_odrive);
            let od = this.getOdrive(which_odrive);
            if (od) {
                od.broadcastEvent("telemetry_stopped", false);
            }
        }
    }

    broadcastEvent(name, value) {
        if (this.event_subscribers.hasOwnProperty(name)) {
            let subs = this.event_subscribers[name];
            for (let i in subs) {
                if (typeof(subs[i]) == "function") {
                    subs[i].call(this, value);
                }
            }
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

                    this.broadcastEvent("odrive_connected", od);
                }
            }
        }
    }

    getOdrive(serial_number) {
        for (let sn in this.odrives) {
            if (sn == serial_number) {
                return this.odrives[sn];
            }
        }
        return null;
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
    helperGetSerialMessage(message) {
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

      
        this.telemetry = null;

        this.event_subscribers = {};
        this.on = (event_name, callback) => {
            if (!this.event_subscribers.hasOwnProperty(event_name)) {
                this.event_subscribers[event_name] = [];   
            }
            this.event_subscribers[event_name].push(callback);                
        };
    }

    toggleTelemetry() {
        if (this._is_telem_running) {
            myodrive.stopTelemetry(this._serial_number);
        } else {
            myodrive.startTelemetry(this._serial_number);
        }
        this._is_telem_running = !this._is_telem_running;
    }


    syncRemote(message) {
        if (this._odrive == null) {
            this._odrive = message;
        } else {
            $.extend(this._odrive, message);
        }
        
    }

    broadcastEvent(name, value) {
        if (this.event_subscribers.hasOwnProperty(name)) {
            let subs = this.event_subscribers[name];
            for (let i in subs) {
                if (typeof(subs[i]) == "function") {
                    subs[i].call(this, value);
                }
            }
        }
    }

    syncTelemetry(telem) {
        this.telemetry = telem;

        //this.element.find(".property.serial_number").text(this._serial_number);
        //this.element.find(".property.vbus_voltage").text(telem.vbus_voltage);


        for (let key in this.event_subscribers) {
            for (let i in this.event_subscribers[key]) {
                // TODO make this better
                let sub = this.event_subscribers[key][i];                
                if (key == "encoder0.pos_circular") {
                    sub.call(this, telem.encoder0.pos_circular);
                } else if (key == "encoder1.pos_circular") {
                    sub.call(this, telem.encoder1.pos_circular);
                }
            }
        }


        if (this.event_subscribers.hasOwnProperty("vbus_voltage")) {
            let subs = this.event_subscribers.vbus_voltage;
            for (let i in subs) {
                if (typeof(subs[i]) == "function") {
                    subs[i].call(this, telem.vbus_voltage);
                }
            }
        }
    }

    // Sync functions to the odrive remote hardware.
    // Calls the remote test_function on the odrive and
    // returns a promise.
    test_function() {
        return myodrive.callRemoteFunction(this._serial_number,
            "test_function", []);
    }

    get_adc_voltage() {

    }

    save_configuration() {

    }

    erase_configuration() {

    }

    reboot() {
    
    }

    enter_dfu_mode() {

    }

    get_interrupt_status() {

    }

    get_dma_status() {

    }

    get_gpio_states() {

    }

    get_drv_fault() {

    }

    clear_errors() {

    }

    /*
    get serial_number() {
        return new Promise( resolve => {
            setTimeout(() => {
                resolve("foobar");
            }, 4000);
        });
    }
    */
}