from bluetoothapi import BluetoothAPI
from serialapi import SerialAPI
import time

# Initialize the Bluetooth and SerialAPI objects
bt_api = BluetoothAPI()
stm_api = SerialAPI()

def main():
    # Connect to Bluetooth
    bt_api.connect()

    # Connect to the STM board
    stm_api.connect()

    while True:
        # Wait for a message from the Android tablet
        message = bt_api.read()
        
        if message:
            decoded_message = message.decode('utf-8').strip()

            # Print the received message
            print("Received message:", decoded_message)

            # Directly send the received message to the STM board
            stm_api.write(decoded_message.encode())

            # Optionally send an acknowledgment back to the tablet
            bt_api.write(f"Relayed command {decoded_message} to STM.".encode())

        # Check for disconnections and reconnect if necessary
        if bt_api.check_connection():
            print("Bluetooth connection lost. Trying to reconnect...")
            bt_api.connect()
        
        if stm_api.check_connection():
            print("STM connection lost. Trying to reconnect...")
            stm_api.connect()

        # Add a slight delay to prevent the loop from running too fast
        time.sleep(1)

if __name__ == "__main__":
    main()