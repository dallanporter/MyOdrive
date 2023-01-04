import eventlet
import socketio
import odrive
from odrive.enums import *
import time
import asyncio
import json
from aiohttp import web
from threading import Thread
import math
import yaml

sio = socketio.AsyncServer()
telem_rate = 2
command_rate = 2
web_thread = None
delay_telem = 0.5 # sec between telemetry 
loop = None # handle to the async event loop for socketio emits
debugging = False
my_drive = None
odrive_commands = []
odrive_telem = ["vbus_voltage", "error","axis0.current_state",
                "axis0.encoder.shadow_count", "axis0.encoder.index_found",
                "axis0.controller.pos_setpoint", "axis0.controller.vel_setpoint",
                "axis0.controller.torque_setpoint", "axis0.controller.trajectory_done",
                "axis0.controller.vel_integrator_torque", "axis0.controller.config.pos_gain",
                "axis0.controller.config.vel_gain", "axis0.controller.config.vel_integrator_gain", 
                "axis0.controller.config.vel_limit", "axis0.controller.config.vel_ramp_rate",
                "axis0.controller.config.torque_ramp_rate",
                "axis0.trap_traj.config.vel_limit", "axis0.trap_traj.config.accel_limit",
                "axis0.trap_traj.config.decel_limit", "axis0.motor.error", "axis0.motor.is_armed",
                "axis0.motor.config.current_lim","axis0.motor.config.torque_lim","axis0.motor.is_calibrated",
                "axis0.motor.current_meas_phA","axis0.motor.current_meas_phB","axis0.motor.current_meas_phC",
                "axis0.motor.DC_calib_phA","axis0.motor.DC_calib_phB","axis0.motor.DC_calib_phC",
                "axis0.motor.I_bus","axis1.current_state",
                "axis1.encoder.shadow_count", "axis1.encoder.index_found",
                "axis1.controller.pos_setpoint", "axis1.controller.vel_setpoint",
                "axis1.controller.torque_setpoint", "axis1.controller.trajectory_done",
                "axis1.controller.vel_integrator_torque", "axis1.controller.config.pos_gain",
                "axis1.controller.config.vel_gain", "axis1.controller.config.vel_integrator_gain", 
                "axis1.controller.config.vel_limit", "axis1.controller.config.vel_ramp_rate",
                "axis1.controller.config.torque_ramp_rate",
                "axis1.trap_traj.config.vel_limit", "axis1.trap_traj.config.accel_limit",
                "axis1.trap_traj.config.decel_limit","axis1.motor.error", "axis1.motor.is_armed",
                "axis1.motor.config.current_lim","axis1.motor.config.torque_lim","axis1.motor.is_calibrated",
                "axis1.motor.current_meas_phA","axis1.motor.current_meas_phB","axis1.motor.current_meas_phC",
                "axis1.motor.DC_calib_phA","axis1.motor.DC_calib_phB","axis1.motor.DC_calib_phC",
                "axis1.motor.I_bus"
               ]


odrive_yaml = None
with open("odrive-interface.yaml", "r") as file:
    odrive_yaml = yaml.safe_load(file)

odrive_json = json.dumps(odrive_yaml, indent=4)
with open("odrive-interface.json", "w") as json_file:
    json_file.write(odrive_json)
    
print("Done.")

async def getOdriveStatus(drive):
    global telem_rate
    if (drive):
        
        out = {"telem_rate":telem_rate}
        for telem_item in odrive_telem:
            cur_obj = drive
            prop_tree = telem_item.split(".")            
            try:
                for name in prop_tree:                
                    cur_obj = getattr(cur_obj, name)
                if math.isinf(cur_obj) or math.isnan(cur_obj):
                    pass
                    # JSON doesn't handle infinity or nan so dont send parameters with 
                    # those values.
                else:
                    out[telem_item] = cur_obj # should be a string or number hopefully.
                #print("Telem item", telem_item, out[telem_item])
            except:
                print("Caught exception getting odrive property", name, telem_item)
                
        if sio != None:
            #await emit_message("telem", json.dumps(out))
            try:
                await sio.emit("telem", json.dumps(out))
            except:
                print("Caught exception emitting telem from getOdriveStatus")
            #print("Sent the telem to the websocket");
        #print(out)
        #print("Axis0 encoder: ", drive.axis0.encoder.shadow_count)
        #print("Axis1 encoder: ", drive.axis1.encoder.shadow_count)
    else:
        print("Problem: my_drive is None")

async def emit_message(key, message):
    loop = asyncio.get_event_loop()
    if loop:
        pass
    else:
        print("Failed to get the run loop");
        

async def processOdriveCommands(drive):
    count = 0
    for command in odrive_commands:
        cmd = odrive_commands.pop()
        cur_obj = drive
        prev_obj = None
        print("Handling command: ", cmd)
        #drive[cmd[0]] = cmd[1]
        prop_tree = cmd[0].split(".")
        name = ""
        try:
            for name in prop_tree:
                prev_obj = cur_obj
                cur_obj = getattr(cur_obj, name)
            
            setattr(prev_obj, name, cmd[1])
        except Exception as ex:
            print("Caught exception issuing command " + cmd[0] + "=" + cmd[1])
            print(ex)
        
        count += 1
    #print("Finished command stack of {} items", count)
    

def commandOdrive(drive, key, value):
    if drive != None:
        drive[key] = value
    else:
        print("Error pushing command to stack. Odrive is none.")



async def on_startup(app):
    print("Inside the on_startup handler for aiohttp")
    asyncio.create_task(odrive_main(my_drive))

async def index(request):
    """Serve the client-side application."""
    with open('static/index.html') as f:
        return web.Response(text=f.read(), content_type='text/html')

@sio.event
def connect(sid, environ):
    print("connect ", sid)

@sio.event
async def chat_message(sid, data):
    print("message ", data)

@sio.event
def disconnect(sid):
    print('disconnect ', sid)

@sio.event
def update_params(sid, params):
    print("Received a message from the client to update params")
    print(params)
    
@sio.event
def set_param(sid, param):
    # param must be an array with two items: [0] = the object and [1] = the value.
    # i.e. [0] would be "axis0.motor.config.foo" and [1] would be "bar"
    try:
        print("Received a set_param request: " + param[0] + " => " + str(param[1]))
        odrive_commands.append(param)
    except Exception as ex:
        print("Caught exception in set_param")
        print(ex)

@sio.event
def calibrate_axis(sid, params):
    print("Received command to calibrate axis.")
    print(params)
    cmd = [
        params + ".requested_state",
        #"AXIS_STATE_FULL_CALIBRATION" # See odrive_enums. TODO figure out how to handle this
        3
    ]
    odrive_commands.append(cmd)

async def odrive_main(drive):
    print("Inside odrive_main()")
    my_drive = await init_odrive(drive)
    
#async def odrive_command_main(drive):
async def odrive_command_main(drive):
    print("Inside odrive_command_main()")
    while True:
        #print("calling processOdriveCommands inside odrive_command_main")
        await processOdriveCommands(drive)
        await asyncio.sleep(1.0 / command_rate)

async def init_odrive(drive):
    global telem_rate
    if debugging == True:
        drive = OdriveSimulator()
    else: 
        drive = odrive.find_any()
        # Calibrate motor and wait for it to finish
        print("starting Odrive")
        #my_drive.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        #while my_drive.axis1.current_state != AXIS_STATE_IDLE:
        #   time.sleep(0.1)
        #out = my_drive.axis0.controller
        #print(out)
        #while my_drive.axis1.current_state != odrive.enums.AXIS_STATE_IDLE:
        #    print("Sleeping while waiting for odrive")
        #    await asyncio.sleep(1)
            
        print("Odrive got valid state for axis1, starting the Odrive status task")
        #task_status = asyncio.create_task(getOdriveStatus(my_drive))
        asyncio.create_task(odrive_command_main(drive))
        while True:
            await getOdriveStatus(drive)
            if telem_rate <= 0:
                telem_rate = 2
            await asyncio.sleep(1.0 / telem_rate)
        #print(my_drive.axis0)
        return drive

def webTask():
    web.run_app(app)
    loop = asyncio.get_event_loop()
    

if __name__ == '__main__':
    app = web.Application()
    app.on_startup.append(on_startup)
    app.router.add_static('/static', 'static')
    # app.router.add_get('/', index)
    sio.attach(app)
    web.run_app(app)
    
    #web_thread = Thread(target=webTask)
    #web_thread.start()    
    #asyncio.run(odrive_main(my_drive))
    
    