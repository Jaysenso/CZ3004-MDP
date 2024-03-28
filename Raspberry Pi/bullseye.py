# Necessary imports
import queue  # Used for holding a set of commands
from serialapi import SerialAPI  # Communication with STM
from imageapi import ImageAPI  # Image capturing and sending to server
import test_pic
import os
import time
import requests
import json
from picamera import PiCamera
from settings import API_IP, API_PORT
from consts import SYMBOL_MAP

class PiAction:
    """
    Class that represents an action that the RPi needs to take.    
    """

    def __init__(self, cat, value):
        """
        :param cat: The category of the action. Can be 'info', 'mode', 'path', 'snap', 'obstacle', 'location', 'failed', 'success'
        :param value: The value of the action. Can be a string, a list of coordinates, or a list of obstacles.
        """
        self._cat = cat
        self._value = value

    @property
    def cat(self):
        return self._cat

    @property
    def value(self):
        return self._value
    


# Define the BullseyeHandler class
class BullseyeHandler:

    # Constructor
    def __init__(self):
        print("[INFO] Initializing BullseyeHandler...")
        
        # Initialize Serial API for communication with STM
        self.serialapi = SerialAPI()
        
        # Initialize Image API for image processing tasks
        self.imageapi = ImageAPI()
        
        # Queue to hold set of instructions
        self.instruction_queue = queue.Queue()

    # Method to connect to STM
    def connect_to_stm(self):
        print("[INFO] Connecting to STM...")
        self.serialapi.connect()
        print("[INFO] Connected to STM!")

    # Method to disconnect from STM
    def disconnect_from_stm(self):
        print("[INFO] Disconnecting from STM...")
        self.serialapi.disconnect()
        print("[INFO] Disconnected from STM!")

    # Method to process instructions based on server response
    def process_instructions(self):
        print("[INFO] Processing instructions...")

        response = '10'
        while response == '10':
            # Attempt to capture an image and send it to the server
            try:
                print("[INFO] Capturing image...")
                # image = self.imageapi.rpiTakePicture()

                print("[INFO] Sending captured image to server...")
                
                rpi_action_queue = []

                command = "SNAP1_FS"
                obstacle_id_with_signal = command.replace("SNAP", "") # should be like '1' or '2' etc

                rpi_action_queue.append(PiAction(cat="snap", value=obstacle_id_with_signal))
                # action: PiAction = self.rpi_action_queue.get()
                response = snap_and_rec(obstacle_id_with_signal=PiAction(cat="snap", value=obstacle_id_with_signal).value)
                # response = self.imageapi.sendImage(image)
            except Exception as e:
                print(f"[ERROR] Failed to capture and send image: {e}")
                return

            # Check server response
            if response == '10':
                print("[WARNING] Obstacle detected. Re-queuing navigation commands...")
                
                # List of commands to execute
                commands = ["RF090", "SF030", "LF090", "SB010", "LF090"]
                for cmd in commands:
                    self.instruction_queue.put(cmd)

                # Send queued commands to STM and await acknowledgment
                while not self.instruction_queue.empty():
                    instruction = self.instruction_queue.get()
                    print(f"[INFO] Processing instruction: {instruction}")
                    self.send_to_stm(instruction)
                    if not self.wait_for_ack():
                        print(f"[ERROR] Failed to receive acknowledgement for {instruction}")
                        break
            else:
                print("[INFO] No obstacle detected. Exiting the loop.")
                break

    # Method to send instruction to STM
    def send_to_stm(self, instruction):
        print(f"[INFO] Sending instruction {instruction} to STM...")
        self.serialapi.write(instruction.encode('utf-8'))

    # Method to wait for acknowledgment from STM
    def wait_for_ack(self):
        ack = None
        print("[INFO] Waiting for acknowledgment from STM...")
        
        while ack is None:
            ack = self.serialapi.read()
            if ack and b'A' in ack:
                print("[INFO] Acknowledgment received!")
                return True
        print("[WARNING] No acknowledgment received!")
        return False

def snap_and_rec(obstacle_id_with_signal):
        """
        RPi snaps an image and calls the API for image-rec.
        The response is then forwarded back to the android
        :param obstacle_id_with_signal: the current obstacle ID followed by underscore followed by signal
        """
        obstacle_id, signal = obstacle_id_with_signal.split("_")
        print("Capturing image for obstacle id:" + obstacle_id)
        url = f"http://{API_IP}:{API_PORT}/image"
        filename = f"{int(time.time())}_{obstacle_id}_{signal}.jpg"

        # Using picamera to take a picture
        with PiCamera() as camera:
            # You can add camera settings here if needed, like camera.resolution = (1024, 768)
            camera.start_preview()
            # Camera warm-up time
            time.sleep(2)
            camera.capture(filename)
            camera.stop_preview()

        print("Requesting from image API")

        try:
            with open(filename, 'rb') as file:
                response = requests.post(url, files={"file": (filename, file)})

                if response.status_code == 200:
                    results = response.json()
                    image_id_value = results['image_id']
                    print(f"results: {results}")
                    # Assuming SYMBOL_MAP is defined elsewhere
                    print(f"Image recognition results: {results} ({SYMBOL_MAP.get(results['image_id'], 'Unknown symbol')})")
                    return image_id_value
                else:
                    print("Something went wrong when requesting path from image-rec API. Please try again.")
                    return None
        except Exception as e:
            print(f"An error occurred: {e}")

# Main execution starts here
if __name__ == "__main__":
    print("[INFO] Starting BullseyeHandler main execution...")
    
    handler = BullseyeHandler()
    handler.connect_to_stm()
    try:
        handler.process_instructions()
    finally:
        # handler.disconnect_from_stm()
        print("[INFO] Cleaning up image resources...")
        handler.imageapi.imageClose()
        print("[INFO] Execution complete!")
