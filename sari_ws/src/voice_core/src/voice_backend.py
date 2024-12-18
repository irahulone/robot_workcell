#!/home/rsl/archiconda3/bin/python
import os #In order to allow setting environment variables such as the ngrok token below and start tunnel via pyngrok.
import sys # in order to add a directory to path.
sys.path.append('/home/rsl/robot/DexArm_API/pydexarm') #to import pydexarm, the DexArm_API folder needs to be in the robot folder.

os.environ['NGROK_AUTHTOKEN']='2f0uKUyNYIUiTSb1G3SK8S8mSJv_6yjHTcQYJURNfwQypcvY9' #set NGROK_AUTHTOKEN environment variable
from flask import Flask, jsonify, request, make_response #used to create web applications, enabling communication between alexa frontend and backend which is done through flask

import json #provides tools for working with json data

from pydexarm import Dexarm # python library to interact with the dexarm (move/pickup/drop)
from serial.tools import list_ports # get the list of available usb ports
from threading import Thread # used for multiprocessing
import numpy as np

print(list_ports.comports()) # print list of available usb ports

# create Dexarm object using the name of usb port which the DexArm is connected
dexarm = Dexarm(port="/dev/ttyACM0") 

# location of robot home in terms of base frame (global frame)
# robot_home = [30.552, 8.241+2.8]
robot_home = [31.3, 6.584+2.8]

grid = np.loadtxt('/home/rsl/robot/DexArm_API/pydexarm/coverage.csv', delimiter=',')

# this function updates location dictionary with the latest data in json file
def update_location():
    with open('/home/rsl/sari_ws/src/voice_core/src/locations.json', 'r') as f:
        locations = json.load(f) # load json file in Python using a dictionary
    # locations is a dictionary
    #   - Key: object name such as "red boat", "yellow boat"
    #   - Values: a list with 3 elements, 
    #       -- val[0] is the object id
    #       -- val[1] is a list with two elements [x,y]
    #       -- val[2] is 0/1 if the object is present or not
    return locations

# This function provides the connection to ngrok and creates a new url
def start_ngrok():
    from pyngrok import ngrok
    url = ngrok.connect(5000).public_url
    print('tunnel URL:', url)

start_ngrok()

# start Flask application
app = Flask(__name__)

# in Flask each function defines an endpoint 
# each function gets a set of variables from the client in a specific endpoint, processes
# the request and responds with a message

# alexa endpoint #/
"""
@app.route('/alexa')
def get_alexa():
    # get the id from client (alexa skills script)
    id = int( request.args.get('id'))

    if id:
        # if id exists it prints the id and returns sucess message to the client (alexa skills script)
        print(f'Object {id}')
        data = {'message': 'Done', 'code': 'SUCCESS'}
        return make_response(jsonify(data), 200)
    data = {'message': 'Bad Request'}
    return make_response(jsonify(data), 400)
"""

# location endpoint: get object name and return its coordinates
@app.route('/location')
def get_location():
    # get object name
    name = request.args.get('name')
    print(f'get_location: {name}')
    if name:  
        # load the latest information from json file 
        locations = update_location() 
        # if the name is in the json file and it is present
        if name in locations and locations[name][-1]:
            # get x and y locations from the json file and return the message
            x = locations[name][1][0]
            y = locations[name][1][1]
            message = f'{name} is located at ({x},{y}) centimeters'
        else:
            message = f'{name} is not found. try again'
        #print(message)
        data = {'message': message, 'code': 'SUCCESS'}
        return make_response(jsonify(data), 200)        
    data = {'message': 'Bad Request'}
    return make_response(jsonify(data), 400)


# available endpoint 
@app.route('/available')
def get_available():
    name = request.args.get('name')
    print(f'get_available: {name}')
    if name:
        locations = update_location() 
        #print(locations)
        #print(name in locations)
        #print(locations[name])
        if name in locations and locations[name][-1]:           
            message = 'yes'
        else:
            message = 'no'
        #if name in locations:
            #with open('/home/rsl/sari_ws/src/voice_core/src/id.txt', 'w+') as f:               
                #f.write(str(locations[name][0]))
        print(message)
        data = {'message': message, 'code': 'SUCCESS'}
        return make_response(jsonify(data), 200)
    data = {'message': 'Bad Request'}
    return make_response(jsonify(data), 400)

@app.route('/id')
def get_id():
    name = request.args.get('name')
    print(f'get_id: {name}')
    if name:
        locations = update_location() 
        if name in locations and locations[name][-1]:       #if present     
            #with open('/home/rsl/sari_ws/src/voice_core/src/id.txt', 'w+') as f:                
                #f.write(str(locations[name][0]))


            message = f'{locations[name][0]}' #returns ID
        else:
            message = f'{name} is not found. try again.' 
        print(message)
        data = {'message': message, 'code': 'SUCCESS'}
        return make_response(jsonify(data), 200)
    data = {'message': 'Bad Request'}
    return make_response(jsonify(data), 400)


# object endpoint returns the list of available object seen by the camera
@app.route('/objects')
def get_workspace():
    
    print(f'objects is called')
    # load the latest locations
    locations = update_location() 
    message = 'The objects present in workspace are '
    atleastone = False
    # loop over each item in locations dictionary
    for name, (id_, (x, y), present) in locations.items():
        # if the oject is present
        if present:
            # add the object name to the message
            message += name + ', ' # add a comma between each object name    
            atleastone = True  #print message if at least one object is present, otherwise print what is in else
    # return message with name of objects
    if atleastone:        
        data = {'message': message[:-2], 'code': 'SUCCESS'} #-2 removes comma and space after last object found since its the last one 
        # example: The objects present in workspace are red boats, yellow boats, 
    else:
        # if there is no object present, print a message stating that there are currently no objects present in the workspace
        data = {'message':' There are no objects currently present in the workspace ', 'code': 'SUCCESS'}
    return make_response(jsonify(data), 200)

# this function takes the object location as well as the black_roll location and then 
# moves the DexArm to pickup the object and place it on the back_roll
def finish_pickup(object_location, black_roll):
    home = [0, 300]  
    print('** Go to the object')
    dexarm.move_to(object_location[0]+home[0], -object_location[1]+home[1], 0)
    #time.sleep(1)
    #quit()
    print('** wide open soft gripper')
    dexarm.soft_gripper_place() #wide open soft gripper
    #time.sleep(1)

    print('**Go down to the object')
    dexarm.move_to(object_location[0]+home[0], -object_location[1]+home[1], -80)
    #time.sleep(1)

    print('**close soft gripper')
    dexarm.soft_gripper_pick() #close soft gripper
    #time.sleep(1)

    print('**Lift the object')
    dexarm.move_to(object_location[0]+home[0], -object_location[1]+home[1], 0)
    #time.sleep(1)

    print('**Move to the black roll')
    dexarm.move_to(black_roll[0]+ home[0], -black_roll[1]+home[1], 0)
    #time.sleep(1)

    print('**Go down to the black roll')
    dexarm.move_to(black_roll[0]+home[0], -black_roll[1]+home[1], -40)
    #time.sleep(1)

    print('**Release the soft gripper')
    dexarm.soft_gripper_nature() #Release the soft gripper to nature state
    #time.sleep(1)

    print('**Go up')
    dexarm.move_to(black_roll[0]+ home[0], -black_roll[1]+home[1], 0)
    #time.sleep(1)

    print('**Go Home')
    dexarm.go_home()
    print('finished pickup async')
    return 'Sccess'


def is_in_workspace(x, y):
    # calculate the distance of object o all points in the grid
    d = (grid[:,0] - x)**2 + (grid[:,1]- y)**2
    # order the distance and pick the four closest grid points
    indices = np.argsort(d)
    # if all four points in the grid are in workspace, we lalbel the object as in workspace
    return grid[indices[:4],2].mean() == 1

# pickup endpoint which takes the object name and moves the DexArm to place the object on the black roll
@app.route('/pickup')
def pickup():    
    print(f'pickup is called')
    name = request.args.get('name')
    print(f'pickup: {name}')
    if name:
        # load the latest locations information from object detection
        locations = update_location() 

        # if the provided object name is in the json file and it is present (last element is 1)
        if name in locations and locations[name][-1]:  

            # if black_roll is not present
            if locations['black roll'][-1] == 0: 
                # return the following message and stop
                data = {'message': f'black roll not found', 'code': 'SUCCESS'}
                return make_response(jsonify(data), 200)

            # the following lines are executed if both the object and black roll are present

            # extract the coordinate location of black_roll and adjust it based on robot home cooridnates
            pos_x = locations['black roll'][1][0] - robot_home[0]
            pos_y = locations['black roll'][1][1] - robot_home[1]
            black_roll = [pos_x*10, pos_y*10]
            
            # extract the location of specified object and  adjust it based on robot home cooridnate
            pos_x = locations[name][1][0] - robot_home[0]
            pos_y = locations[name][1][1] - robot_home[1]
            
            object_location =  [pos_x*10, pos_y*10]

            print(f'{name} location', object_location[0], 300 - object_location[1])
            print('black_roll location', black_roll[0], 300 - black_roll[1])

            if not is_in_workspace(object_location[0], 300 - object_location[1]): #if object represented by location x and y is not in workspace 
               data = {'message': f'{name} is outside of the workspace', 'code': 'SUCCESS'}
               return make_response(jsonify(data), 200)
            if not is_in_workspace(black_roll[0], 300 - black_roll[1]):
                data = {'message': 'Black Roll is outside of the workspace', 'code': 'SUCCESS'}
                return make_response(jsonify(data), 200)

             #used threading in order for alexa backend to respond back to alexa frontend in less than 8 sec otherwise app will crash
            thread = Thread(target=finish_pickup, args=(object_location, black_roll))
            thread.start()
            
            #time.sleep(1)
            data = {'message': f'{name} will be placed on the black roll', 'code': 'SUCCESS'}
            return make_response(jsonify(data), 200)

            # res = finish_pickup(object_location, black_roll)
            # if res == 'Beyond Limit':
            #     data = {'message': f'{name} is outside of the workspace and cannot be reached', 'code': 'SUCCESS'} 
            # else:
            #     data = {'message': f'{name} is placed on the black roll', 'code': 'SUCCESS'}

            # return make_response(jsonify(data), 200)

        else:
            data = {'message': f'{name} not found', 'code': 'SUCCESS'}
            return make_response(jsonify(data), 200)

        
    data = {'message': 'Bad Request'}
    return make_response(jsonify(data), 400)
    
if __name__ == '__main__':
 
    # run() method of Flask class runs the application 
    # on the local development server.
    app.run()
