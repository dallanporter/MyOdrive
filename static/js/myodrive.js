

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
        this.socket.on("joystick", this.onSocketJoystick.bind(this));

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

    enableJoystick() {
        this.socket.emit("joystick", { enable: true });
    }

    disableJoystick() {
        this.socket.emit("joystick", { enable: false });
    }

    onSocketJoystick(message) {
        this.broadcastEvent("joystick", message);
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
        //console.log("MyOdrive.onSocketTelemetry()");
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
                }, 5000);
            });
        } else {
            return Promise.reject("No socket connection");
        }
    }

    emit(event_name, payload) {
        this.socket.emit(event_name, payload);
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
                }, 5000);
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
                    let od = new Odrive(sn); 
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
    constructor(serial_number) {
        this._serial_number = serial_number;
        this._odrive = null;
        this._is_telem_running = false;
        this._is_mapped = false;
      
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

        // Here will map the interface details to the object that came back.
        /*
        if (!this._is_mapped) {
            let mapped_odrive = this._mapInterface(this._odrive);
            this._is_mapped = true;
        }
        */
        

        this.broadcastEvent("synced", this._odrive);
    }

    _mapInterface(property, cname, piface=null) {
        //console.log("_mapInterface", property, cname);
        if (property.hasOwnProperty("class_name")) {
            let class_name = property.class_name;
            //console.log("_mapInterface found class name " + class_name + " in property.");
            if (odrive_map.interfaces.hasOwnProperty(class_name)) {
                let iface = odrive_map.interfaces[class_name];
                for (let propname in property) {
                    let child_prop = property[propname];
                    //console.log(`Checking ${class_name} property ${propname}`);
  
                    let new_iface = iface.attributes[propname];
                    if (typeof(child_prop) == "object") {
                        this._mapInterface(child_prop, class_name, new_iface);
                    } else {
                        //console.log(`Class ${class_name} has property ${propname}`);
                        let newprop = {
                            value: child_prop,
                            iface: new_iface,
                        };
                        //console.log(`${class_name} -> ${propname} setting iface.`);
                        property[propname] = newprop;

                    }
                }
            }

        } else {
            //console.log(`Object from ${cname} class has no class_name property.`);
            if (piface == null) {
                piface = odrive_map.interfaces[cname].attributes;
            }
            for (let propname in property) {
                //console.log(`Checking ${propname}...`);
                let child_prop = property[propname];
                if (piface.hasOwnProperty("attributes")) {
                    piface = piface.attributes;
                }
                if (piface.hasOwnProperty(propname)) {
                    if (typeof(child_prop) == "object") {
                        //console.log(`Found a sub-attribute ${propname}`);
                        this._mapInterface(child_prop, cname, piface[propname].attributes);
                    } else {
                        let newprop = {
                            value: child_prop,
                            iface: piface[propname],
                        };
                        //console.log(`found interface attribute for ${propname}`);
                        property[propname] = newprop;
                    }
                } else {
                    //console.log(`!! didn't find an interface for class ${cname} ${propname}`);
                }
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

    addTelemetry(properties) {
        console.log("Adding telemetry");
        if (typeof(properties) == "string") {
            properties = [properties];
        }
        let payload = {
            serial_number: this._serial_number,
            properties: properties,
        };
        myodrive.emit("add_telemetry", payload);
    }

    removeTelemetry(properties) {
        console.log("Removing telemetry");
        if (typeof(properties) == "string") {
            properties = [properties];
        }
        let payload = {
            serial_number: this._serial_number,
            properties: properties,
        };
        myodrive.emit("remove_telemetry", payload);
    }

    sync() {
        console.log("odrive.sync() called");
        
        myodrive.emit("sync", this._serial_number);
    }

    

    syncTelemetry(telem) {
        this.telemetry = telem;
       // TODO expand the object tree into a single dot separated string.
        let rec_func = (prev_key, obj, tree, out) => {
            if (typeof(obj) == "object") {
                for (let key in obj) {
                    let new_tree = tree + "." + key;
                    rec_func(key, obj[key], new_tree, out);
                }
            } else {
                out[tree] = obj;
                this.broadcastEvent(tree, obj);
                return;
            }
        };

        if (typeof(telem) != "object") {
            console.log("telem is not an object", telem);
            return;
        }
        try {
            let timestamp = telem.timestamp;
            let telem_vals = {};
            for (let key in telem) {
                if (key == "timestamp") {
                    continue;
                }
                let tree = key;
                if (typeof(telem[key]) == "object") {                                    
                    rec_func(key, telem[key], tree, telem_vals);                    
                } else {
                    this.broadcastEvent(tree, [timestamp, telem[key]]);
                }
            }
        } catch (ex) {
            console.log("syncTelemetry() exception: ", ex);
        }
        this.broadcastEvent("telem_complete", true);
    }

    callRemoteFunction(remote_function, value) {
        console.log("setRemoteFunction", value);
        return myodrive.callRemoteFunction(this._serial_number, remote_function, value);
    }

    setRemoteProperty(remote_property, value) {
        return myodrive.setRemoteProperty(this._serial_number, remote_property, value);
    }

}