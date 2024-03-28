import os
import time
import requests
import json
from picamera import PiCamera
from settings import API_IP_IMG_REC, API_PORT_IMG_REC
from consts import SYMBOL_MAP
import serial
import bluetooth


ser = serial.Serial("/dev/ttyUSB0",
          baudrate=115200,
          parity=serial.PARITY_NONE,
          stopbits=serial.STOPBITS_ONE,
          bytesize=serial.EIGHTBITS,
          timeout=2,
        )

# Assuming API_IP and API_PORT are defined elsewhere in your script
# API_IP = 'your.api.ip.address'
# API_PORT = 'your_api_port'
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


class RaspberryPi:
    """
    Class that represents the Raspberry Pi.
    """

    def __init__(self):
        """
        Initializes the Raspberry Pi.
        """
        self.logger = prepare_logger()

        # # Messages that need to be processed by RPi
        # self.rpi_action_queue = self.manager.Queue()

        # # Messages that need to be processed by STM32, as well as snap commands
        # self.command_queue = self.manager.Queue()
        # _________________________________________________________________
        # JUST FOR TESTING, REMOVE THIS!!! **
        for c in commands:
            self.command_queue.put("SNAP1")
        # self.rpi_action_queue.put(
        #             PiAction(cat="snap", value=obstacle_id_with_signal))
        # _________________________________________________________________




if __name__ == "__main__":

    def snap_and_rec(obstacle_id_with_signal):
        """
        RPi snaps an image and calls the API for image-rec.
        The response is then forwarded back to the android
        :param obstacle_id_with_signal: the current obstacle ID followed by underscore followed by signal
        """
        obstacle_id, signal = obstacle_id_with_signal.split("_")
        print("Capturing image for obstacle id:" + obstacle_id)
        url = f"http://{API_IP_IMG_REC}:{API_PORT_IMG_REC}/image"
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

# Your existing code for queue and action execution...
    result = '10'
    while result == '10':
    
        rpi_action_queue = []

        command = "SNAP1_FS"
        obstacle_id_with_signal = command.replace("SNAP", "") # should be like '1' or '2' etc

        rpi_action_queue.append(PiAction(cat="snap", value=obstacle_id_with_signal))
        # action: PiAction = self.rpi_action_queue.get()
        result = snap_and_rec(obstacle_id_with_signal=PiAction(cat="snap", value=obstacle_id_with_signal).value)
        
        print(result)
        
        if result == '10':
            command = "C0900"
            ser.write(command.encode("utf-8"))
            time.sleep(9)
            command = "FL900"
            ser.write(command.encode("utf-8"))
            time.sleep(9)
            command = "FW020"
            ser.write(command.encode("utf-8"))
            time.sleep(9)
            command = "BR900"
            ser.write(command.encode("utf-8"))
            time.sleep(9)