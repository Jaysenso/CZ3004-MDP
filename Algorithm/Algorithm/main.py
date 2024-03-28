from algo.algo import pathFinder 
from flask_cors import CORS
from flask import Flask, request, jsonify
from model import *
from helper import build_commands

import time
import json

app = Flask(__name__)
CORS(app)
loaded_model = None

@app.route('/status', methods=['GET'])
def status():
    """
    This is a health check endpoint to check if the server is running
    :return: a json object with a key "result" and value "ok"
    """
    return jsonify({"result": "ok"})


@app.route('/path', methods=['POST'])
def path_finding(): 
    """
    This is the main endpoint for the path finding algorithm
    :return: a json object with a key "data" and value a dictionary with keys "distance", "path", and "commands"
    """
    # Get the json data from the request

    content = request.json #From Android Tablet
    #content : dict = json.loads(request.json) #For postRequest.py

    # Get the obstacles_list, big_turn, retry_flag, robot_x, robot_y, and robot_direction from the json data
    obstacles_list = content['obstacles']
    # big_turn = int(content['big_turn'])
    retry_flag = content['retrying']
    robot_x, robot_y = content['robot_x'], content['robot_y']
    robot_direction = int(content['robot_dir'])

    # Initialize pathFinder object with robot size of 20x20, bottom left corner of robot at (1,1), facing north, and whether to use a big turn or not.
    maze_solver = pathFinder(20, 20, robot_x, robot_y, robot_direction, big_turn=None)

    # Add each obstacle into the pathFinder. Each obstacle is defined by its x,y positions, its direction, and its id
    for ob in obstacles_list:
        maze_solver.add_obstacle(ob['x'], ob['y'], ob['d'], ob['id'])

    start = time.time()
    # Get shortest path
    optimal_path, distance = maze_solver.get_optimal_order_dp(retrying=retry_flag)
    print(optimal_path, distance)
    print(f"Time taken to find shortest path using A* search: {time.time() - start}s")
    print(f"Distance to travel: {distance} units")
    
    # Based on the shortest path, generate commands for the robot
    commands = build_commands(optimal_path, obstacles_list)
    # Get the starting location and add it to path_results
    path_results = [optimal_path[0].get_dict()]
    # Process each command individually and append the location the robot should be after executing that command to path_results
    i = 0

    for command in commands:
        if command.startswith("SNAP"):
            continue
        if command.startswith("FIN"):
            continue
        elif command.startswith("FW") or command.startswith("FS"):
            i += int(command[2:]) // 10
        elif command.startswith("BW") or command.startswith("BS"):
            i += int(command[2:]) // 10
        else:
            i += 1
        path_results.append(optimal_path[i].get_dict())

    return jsonify({
        "data": {
            'distance': distance,
            'path': path_results,
            'commands': commands
        },
        "error": None
    })


@app.route('/image', methods=['POST'])
def image_predict():
    """
    This is the main endpoint for the image prediction algorithm
    :return: a json object with a key "result" and value a dictionary with keys "obstacle_id" and "image_id"
    """
    uploaded_file = request.files['file']
    filename = uploaded_file.filename
    uploaded_file.save(os.path.join('uploads', filename))
    constituents = uploaded_file.filename.split("_")
    obstacle_id = constituents[1]
    image_id = predict_image_week_9(filename,loaded_model)

    # Return the obstacle_id and image_id
    result = {
        "obstacle_id": obstacle_id,
        "image_id": image_id
    }
    return jsonify(result)

@app.route('/stitch', methods=['GET'])
def stitch():
    """
    This is the main endpoint for the stitching command. Stitches the images using two different functions, in effect creating two stitches, just for redundancy purposes
    """
    img = stitch_image()
    img.show()
    img2 = stitch_image_own()
    img2.show()
    return jsonify({"result": "ok"})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
