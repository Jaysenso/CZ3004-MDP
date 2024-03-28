import requests
import json

# Define the URL and the endpoint
url = "http://localhost:5000/path"

# Define JSON data
json_data = json.dumps({
    "obstacles": [
        {"x": 8, "y": 8, "id": 1, "d": 0},
        {"x": 4, "y": 6, "id": 2, "d": 0},
        {"x": 14, "y": 11,"id": 3, "d": 2},
        {"x": 15, "y": 6, "id": 4, "d": 6},
        {"x": 19, "y": 17, "id": 5,  "d": 6}
        # Add more obstacles as needed
    ],
    "retrying": True,
    "robot_x": 1,
    "robot_y": 1,
    "robot_dir": "0"
})
# json_request = json.dumps(json_data)
# response = requests.post(url, json=json_request)
response = requests.post(url, json=json_data)

# Check the response
if response.status_code == 200:
    print("Request successful!")
    print(response.json())
else:
    print("Request failed with status code:", response.status_code)
