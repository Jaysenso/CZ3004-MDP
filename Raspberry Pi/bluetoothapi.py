# Import the required modules for Bluetooth, time and socket functionalities
import bluetooth
import time
import socket

# Create a class named BluetoothAPI for Bluetooth-related operations
class BluetoothAPI:

    # Initialize static variables: MAC addresses, port, and buffer size
    # MAC_ADDRESS for Raspberry Pi, TABLET_BLUETOOTH for the tablet you may be connecting to
    MAC_ADDRESS = 'E4:5F:01:55:A6:F3'
    TABLET_BlUETOOTH = '90:EE:C7:E7:D3:96' # Change this
    PORT_NUMBER = 1
    READ_BUFFER_SIZE = 5096

    # Constructor: Initializes server and client as None
    def __init__(self):
        self.client = None
        self.server = None

    # Function: Checks if either server or client is None, meaning not connected
    def check_connection(self):
        return self.client is None or self.server is None

    # Function: Initiates connection to the Bluetooth device
    def connect(self):
        try:
            # If there is a pre-existing server, shut it down and close it
            self.server.shutdown(2)
            self.server.close()
        except:
            print("[BT] Trying to connect to the RPI")
        
        # Loop to continually attempt connection
        while True:
            try:
                # Initialize a Bluetooth socket for RFCOMM
                self.server = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
                print("[BT] Initialising Server")
                
                # Bind the server to the given MAC_ADDRESS and port
                self.server.bind((self.MAC_ADDRESS, self.PORT_NUMBER))
                print("[BT] Binded")
                
                # Start listening for incoming connections on the port
                self.server.listen(self.PORT_NUMBER)
                print("[BT] Listening")
                
                # Accept incoming connection and store client socket and address
                self.client, client_address = self.server.accept()
                print("[BT] Server accepted")
                
            except Exception as exception:
                # If anything goes wrong, print the error and restart the connection
                print("[BT] Connection failed: " + str(exception))
                print("[BT] Disconnecting from server")
                self.server.shutdown(1)
                self.server.close()
                time.sleep(1)
            else:
                # Successful connection
                print("[BT] Connected successfully")
                print("[BT] Client address is " + str(client_address))
                break

    # Function: Writes (sends) a message to the connected Bluetooth device
    def write(self, message):
        print("[BT] Attempting to send message:", message)
        try:
            self.client.send(message)
            print("[BT] Sent message:", message)
            return False
        except Exception as exception:
            print("[BT] Failed to send: " + str(exception))
            return True

    # Function: Disconnects both server and client sockets
    def disconnect(self):
        try:
            if self.client:
                self.client.shutdown(socket.SHUT_RDWR)
                self.client.close()
                self.client = None
                print('Android - Disconnecting Client Socket')
            if self.server:
                self.server.shutdown(socket.SHUT_RDWR)
                self.server.close()
                self.server = None
                print('Android - Disconnecting Server Socket')
        except Exception as e:
            print('[Android] Disconnect all error %s' % str(e))

    # Function: Reads a message from the connected Bluetooth device
    def read(self):
        while self.server and self.client:
            print("[BT] Attempting to read...")
            try:
                message = self.client.recv(self.READ_BUFFER_SIZE)
                if message:
                    print("[BT] Message read:")
                    print(message)
                    if message.decode() == "close":
                        print("[BT] Shutting down..")
                        self.server.shutdown(1)
                        self.server = None
                    return message
            except Exception as exception:
                print("[BT] Failed to read: " + str(exception))

# Main function: Instantiates BluetoothAPI and accepts commands for reading or closing
if __name__ == '__main__':
    bluetoothapi = BluetoothAPI()
    bluetoothapi.connect()
    while True:
        command = input("Enter Command(r/close): ")
        if command == "close":
            print("Closing Bluetooth Connection")
            server = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            server.shutdown(2)
            server.close()
        elif command == 'r':
            msg = bluetoothapi.read()
            print("read", msg, "from bluetooth")
        else:
            command = str.encode(command)
            bluetoothapi.write(command)
