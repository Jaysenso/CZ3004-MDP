import serial
import time

# SerialAPI class for managing Serial Communication
class SerialAPI:
    # The Serial Port and Baud Rate can be configured here.
    SERIAL_PORT = '/dev/ttyUSB0'  # Change this according to your setup
    BAUD_RATE = 115200  # Baud rate for communication, usually set to 115200 for UART

    # Constructor for SerialAPI
    def __init__(self):
        # serial_connection will hold the serial object once connected
        self.serial_connection = None

    # Method to check if the serial connection is active
    def check_connection(self):
        # Return True if the connection is None (not connected)
        return self.serial_connection is None

    # Method to establish a serial connection
    def connect(self):
        # Will keep trying to connect until successful
        while True:
            try:
                print("[STM] Connecting to STM...")
                # Trying to establish a serial connection
                self.serial_connection = serial.Serial(
                    self.SERIAL_PORT, self.BAUD_RATE)
            except Exception as exception:
                print("[STM] Connection failed: " + str(exception))
                time.sleep(1)  # Wait for a second before trying again
            else:
                print("[STM] Connected successfully")
                break  # Exit the while loop if connection is successful

    # Method to write data to the serial port
    def write(self, message):
        print("[STM] Sending message to STM:", message)
        try:
            # Attempt to write the message to the serial connection
            self.serial_connection.write(message)
            print("[STM] Successfully sent to STM")
        except Exception as exception:
            print("[STM] Failed to send: " + str(exception))

    # Method to read data from the serial port
    def read(self):
        print("[STM] Reading from STM...")
        message = None  # Initialize message as None
        try:
            # Attempt to read until a specific delimiter (in this case, 'A')
            message = self.serial_connection.read_until(b'A')
        except Exception as exception:
            print("[STM] Failed to read: " + str(exception))
        else:
            if message is not None and len(message) > 0:
                print("[STM] Message read: ", message)
                return message  # Return the message if it's not None and has some data
    
    # Method to close the serial connection and clean up resources
    def disconnect(self):
        print("[STM] Attempting to disconnect from STM...")
        try:
            # Check if the serial connection is active
            if self.serial_connection:
                # Close the serial connection
                self.serial_connection.close()
                # Set the serial_connection to None after closing
                self.serial_connection = None
                print("[STM] Successfully disconnected from STM")
            else:
                print("[STM] No active connection found!")
        except Exception as exception:
            print("[STM] Error while disconnecting: " + str(exception))


# Main program
if __name__ == '__main__':
    serialapi = SerialAPI()  # Create a SerialAPI object
    serialapi.connect()  # Connect to the serial port

    # Keep running the loop for user input
    while True:
        command = input("Enter Command: ")
        if command == "close":
            print("Closing Serial Connection")
            serialapi.serial_connection.close()  # Close the serial connection
            exit()  # Exit the program
        # Encode the command and send it
        command = str.encode(command)
        serialapi.write(command)
        print("Sent", command)
