import eventlet
import myodrive
from odrive_enums import *
import time
import asyncio
import json
from aiohttp import web
import socketio
from threading import Thread
import math

# If using Anaconda, install the dependencies with conda first.
# !!! before installing odrive with pip, install the conda stuff first.
# conda install python-socketio aiohttp eventlet matplotlib ipython
#   wcwidth pytprocess pickleshare executing backcall appnope appdirs
#   traitlets pygments propmpt-toolkit pexpect parso decorator asttokens
#   matplotlib-inline jedi


# Note: odrive installs these python packages with pip:
# wcwidth, pyelftools, pure-eval, ptyprocess, pickleshare, 
# executing, backcall, appnope, appdirs, traitlets, PyUSB, 
# pygments, prompt-toolkit, pexpect, parso, decorator, asttokens, 
# stack-data, matplotlib-inline, jedi, ipython, odrive

# First, load the odrive-interface.json into a variable
''''
odrive_interface = None
try:
    f = open("odrive-interface.json")
    odrive_interface = json.load(f)
    f.close()
except Exception:
    print("Error loading odrive-interface.json.")
    exit()
'''

sio = socketio.AsyncServer(async_handlers=True)
telem_rate = 2
command_rate = 2

delay_telem = 0.5 # sec between telemetry 
loop = None # handle to the async event loop for socketio emits
debugging = False

async def emit_message(key, message):
    loop = asyncio.get_event_loop()
    if loop:
       loop.run_in_executor(sio.emit(key, message))
    else:
        print("Failed to get the run loop");
        

async def on_startup(app):
    print("Inside the on_startup handler for aiohttp")
    #asyncio.create_task(odrive_main(my_drive))
    asyncio.create_task(myodrive.MyOdrive.initialize())

async def index(request):
    """Serve the client-side application."""
    with open('static/index.html') as f:
        return web.Response(text=f.read(), content_type='text/html')

@sio.event
async def odrive(sid, message):
    #print("Received socketio odrive message ")
    # The message should be in the form:
    # { serial_number: 12324343, get: [], set: [] }
    result = await myodrive.MyOdrive.handleSocketMessage(message)
    await sio.emit('odrive', result)
    
@sio.event
async def list_odrives(sid, message):
    #print("Received socketio odrive list_odrives message")
    result = await myodrive.MyOdrive.list_odrives()
    await sio.emit("list_odrives", result)
    

@sio.event
def connect(sid, environ):
    print("connect ", sid)

@sio.event
async def chat_message(sid, data):
    print("message ", data)

@sio.event
def disconnect(sid):
    print('disconnect ', sid)


if __name__ == '__main__':
    app = web.Application()
    app.on_startup.append(on_startup)
    app.router.add_static('/static', 'static')
    # app.router.add_get('/', index)
    #myodrive.MyOdrive.setInterface(odrive_interface)
    myodrive.MyOdrive.attachSocketIO(sio)
    thread = Thread(target=myodrive.MyOdrive.detectUSBDevices)
    thread.start()
    sio.attach(app)
    web.run_app(app)
    

    