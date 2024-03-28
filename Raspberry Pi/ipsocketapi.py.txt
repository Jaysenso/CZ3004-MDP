import socket
import time

# Defines a class named IPSocketAPI
class IPSocketAPI:
    READ_BUFFER_SIZE = 2048  # Defines a constant for buffer size during reads

    # Constructor method
    def __init__(self):
        self.client = None       # Placeholder for the client socket
        self.server = None       # Placeholder for the server socket
        self.client_address = None  # Placeholder for the address of the connecting client

    # Method to check if the client or server is not initialized
    def check_connection(self):
        return self.client is None or self.server is None

    # Method to connect the server to a client
    def connect(self):
        # Keep trying to connect in a loop
        while True:
            print("[Algo] Connecting to Algo...")
            try:
                # Create a new socket for the server
                self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                # Set socket options to reuse the address
                self.server.setsockopt(
                    socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                # Bind the server to a specific IP and port
                self.server.bind(('192.168.34.1', 6000))
                # Start listening for incoming connections
                self.server.listen()
                print("[Algo] Listening...")
                # Accept a client connection
                self.client, self.client_address = self.server.accept()
                print("[Algo] Connected "+str(self.client_address))
            except Exception as exception:  # Catch any errors during the connection process
                print("[Algo] Connection failed: " + str(exception))
                time.sleep(3)  # Wait for 3 seconds before retrying
            else:  # If no exception, the connection is successful
                print("[Algo] Connected successfully")
                print("[Algo] Client address is " +
                      str(self.client_address))
                break  # Exit the loop

    # Method to send a message to the client
    def write(self, message):
        print("[Algo] Sending message:")
        print(message)
        try:
            self.client.send(message)  # Send the message
        except Exception as exception:  # Catch any errors during the send process
            print("[Algo] Failed to send: " + str(exception))

    # Method to read a message from the client
    def read(self):
        print("")
        print("[Algo] Reading message from Algo...")
        try:
            # Receive a message of up to READ_BUFFER_SIZE bytes
            message = self.client.recv(self.READ_BUFFER_SIZE)
        except Exception as exception:  # Catch any errors during the receive process
            print("[Algo] Failed to read: " + str(exception))
        else:
            if message is not None and len(message) > 0:  # Check if the message is not empty
                print("[Algo] Message read:", message)
                return message  # Return the received message

# Main execution starts here
if __name__ == '__main__':
    ipsocketapi = IPSocketAPI()  # Create an instance of IPSocketAPI
    try:
        ipsocketapi.server.close()  # Attempt to close the server socket
    except:
        print("[Algo] Not Connected")  # If an error occurs, the server is not connected
    ipsocketapi.connect()  # Connect the server to a client

    # A sample message
    message = "ALG:0,18,E,0;18,19,S,1;18,0,W,2;5,0,E,3;10,10,E,4;9,10,W,5;"
    ipsocketapi.write(message.encode('utf-8'))  # Send the sample message to the client
    while True:
        msg = input("Enter command:")  # Get a command from the user
        if msg == 'r':  # If command is 'r'
            algo = ipsocketapi.read()  # Read a message from the client
            n = 5
            # Break the received message into chunks of 5 characters
            instr = [algo[i:i+n] for i in range(0, len(algo), n)]
            print(instr)
        else:
            ipsocketapi.write(msg.encode('utf-8'))  # Send the user's command to the client