import socket
from picamera import PiCamera
from picamera.array import PiRGBArray
import imagezmq
import time
import sys
import queue
import numpy as np
import cv2

from bluetoothapi import BluetoothAPI

# ImageAPI class for handling image capture and communication
class ImageAPI:
    # Network configurations
    HOST = '192.168.34.13'  # Wei Kang laptop
    PORT = '5555'
    READ_BUFFER_SIZE = 2048

    # Constructor
    def __init__(self):
        self.client = None  # Client socket for image server
        self.count = 0  # Image sent counter
        self.camera = PiCamera()  # Initialize camera
        # config = self.camera.create_preview_configuration(main={"size": (1000, 1000)})
        # self.camera.configure(config)  # Configure camera
        # self.camera.start()  # Start camera
        self.camera.resolution = (1280,720)

    # Method to send image to server
    def sendImage(self, image):
        print("[Image] Attempting to connect to Image Server...")
        # Establish connection to Image Server
        sender = imagezmq.ImageSender(connect_to="tcp://"+self.HOST+":"+self.PORT)
        print("[Image] Successfully connected to Image Server: " + str(self.HOST))
        
        rpi_name = socket.gethostname()
        print('[Image] Sending image to server...')
        reply = sender.send_image(rpi_name, image)
        self.count += 1  # Increment image sent counter
        print(f"[Image] We sent picture {self.count}.")
        print(f"[Image] ID for picture {self.count}: ", reply)
        print(f'[Image] Connection with image server closed after picture {self.count}')
        
        reply = reply.decode('utf-8')
        return reply

    # Method to capture image from Raspberry Pi camera
    def rpiTakePicture(self):
        while True:
            try:
                print('[Image] Initializing Camera.')
                print('[Image] Taking Picture')
                # time.sleep(2)

                # Set camera to capture grayscale image
                # self.camera.color_effects = (128, 128)
                self.camera.awb_mode = 'auto'
                self.camera.exposure_mode = 'backlight'
                # self.camera.shutter_speed = 3000
                # self.camera.iso = 1000

                output = PiRGBArray(self.camera)
                self.camera.capture(output, 'bgr')
                image = output.array
                # cv2.imwrite("filename.jpg", image)
                # image = self.camera.capture_array()
                # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                print('[Image] Finished taking picture')

                # Clear the PiRGBArray buffer
                output.truncate(0)
                break

            except Exception as exception:
                print("[Image] Sending image to the server failed: " + str(exception))
                time.sleep(1)
        
        return image

    # Method to close the camera
    def imageClose(self):
        self.camera.close()

    # Method to read data from image server
    def read(self):
        print("[Image] Attempting to read from image server via Wi-Fi...")
        try:
            message = self.client.recv(self.READ_BUFFER_SIZE)
        except Exception as exception:
            print("[Image] Failed to read from image server via Wi-Fi: " + str(exception))
        else:
            if message is not None and len(message) > 0:
                print("[Image] Message read from image server via Wi-Fi:")
                message = message.decode()
                print('[Image] Received: ' + str(message))
                return message

# Main code
if __name__ == '__main__':
    ic = ImageAPI()
    time.sleep(2)
    while True:
        command = input("Execute Image Capturing: ")
        if command == "yes" or command == 'y':
            image = ic.rpiTakePicture()
            imageID = ic.sendImage(image)
            print("Image ID:", imageID)
            if imageID == "N":
                print("no detection result")
        elif command == "exit":
            ic.camera.close()
            print("Exiting")
            exit()
        elif command == "end":
            ic.sendEmptyImage()
            print()

# to test send image id to android
# if __name__ == '__main__':
#     ic = ImageAPI()
#     bt = BluetoothAPI()
#     time.sleep(2)
#     bt.connect()
#     while True:
#         bt.connect()
#         command = input("Execute Image Capturing: ")
#         if command == "yes":
#             image = ic.rpiTakePicture()
#             imageID = ic.sendPictureToServer(image)
#             print("Image ID:", imageID)
#             while (imageID == b"N"):
#                 print("no detection result")
#                 imageID = ic.image()
#             bMsg= "TARGET,1,"+imageID
#             print("[Main] Sending ",bMsg," to Android")
#             failed=bt.write(bMsg.encode('utf-8'))
#             if failed:
#                         print("[Bluetooth] Attempting to reconnect bluetooth")
#                         bt.reconnect_android(bt)
#         elif command == "exit":
#             ic.camera.close()
#             print("exiting")
#             exit()
#         elif command == "end":
#             ic.sendEmptyImage()
#             print()