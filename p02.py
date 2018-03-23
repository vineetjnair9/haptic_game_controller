# Provocation 02
# Group: Carolyn Chen, George Moore, Vineet Nair, Cora Zheng
#
# This python script interfaces with an Adafruit Feather M0 board.
# The code is largely adapted from online tutorial code.
#
# Resources: 
# 

# Example of interaction with a BLE UART device using a UART service
# implementation.
# Author: Tony DiCola
import re
import Adafruit_BluefruitLE
from Adafruit_BluefruitLE.services import UART

class UARTReader(object):

    def __init__(self, uart, timeout_sec=1):
        """Initialize the UART Reader.
        """
        self._uart = uart
        # Clear the UART queue
        #with self._uart._queue.mutex:
        #   self._uart._queue.queue.clear()
        self._timeout_sec = timeout_sec
        self._buffer = ''

    def readline(self):
        """Read a line from the UART, up to \n
        """

        # Read until a newline is detected
        while True:
            if re.search(';', self._buffer):
                buffer_parts = self._buffer.split(';', 1)
                self._buffer = buffer_parts[1]
                return buffer_parts[0]
            received = self._uart.read(timeout_sec=self._timeout_sec)
            if received is not None:
                self._buffer += received

# Get the BLE provider for the current platform.
ble = Adafruit_BluefruitLE.get_provider()


# Main function implements the program logic so it can run in a background
# thread.  Most platforms require the main thread to handle GUI events and other
# asyncronous events like BLE actions.  All of the threading logic is taken care
# of automatically though and you just need to provide a main function that uses
# the BLE provider.
def main():
    # Clear any cached data because both bluez and CoreBluetooth have issues with
    # caching data and it going stale.
    ble.clear_cached_data()

    # Get the first available BLE network adapter and make sure it's powered on.
    adapter = ble.get_default_adapter()
    adapter.power_on()
    print('Using adapter: {0}'.format(adapter.name))

    # Disconnect any currently connected UART devices.  Good for cleaning up and
    # starting from a fresh state.
    print('Disconnecting any connected UART devices...')
    UART.disconnect_devices()

    # Scan for UART devices.
    print('Searching for UART device...')
    try:
        adapter.start_scan()
        # Search for the first UART device found (will time out after 60 seconds
        # but you can specify an optional timeout_sec parameter to change it).
        device = UART.find_device()
        if device is None:
            raise RuntimeError('Failed to find UART device!')
    finally:
        # Make sure scanning is stopped before exiting.
        adapter.stop_scan()

    print('Connecting to device...')
    device.connect()  # Will time out after 60 seconds, specify timeout_sec parameter
                      # to change the timeout.

    # Once connected do everything else in a try/finally to make sure the device
    # is disconnected when done.
    try:
        # Wait for service discovery to complete for the UART service.  Will
        # time out after 60 seconds (specify timeout_sec parameter to override).
        print('Discovering services...')
        UART.discover(device)

        # Once service discovery is complete create an instance of the service
        # and start interacting with it.
        uart = UART(device)

        # Write a string to the TX characteristic.
        #uart.write('Hello world!\r\n')
        #print("Sent 'Hello world!' to the device.")

        # Now wait up to one minute to receive data from the device.
        print('Reading from MPU6050 sensor')
        reader = UARTReader(uart)
        inAir = False
        numBounce = 0
        for i in range(1000):
            received = reader.readline()
            if received is not None:
                # Received data, print it out.
                # print('{0}'.format(received))
                data = received.split(",")
                if (len(data) == 3):
                    fsr = int(data[0])
                    pitch = int(data[1])
                    roll = int(data[2])
                    # jumping logic
                    if (fsr > 200 and inAir):
                        inAir = False
                        numBounce += 1
                        print("Bounce %d" %(numBounce))
                    if (fsr < 200 and not inAir):
                        inAir = True
                    if (pitch < -20):
                        print("Right")
                    if (pitch > 20):
                        print("Left")
                    if (roll < -20):
                        print("Forward")
                    if (roll > 20):
                        print("Backward")
                    # print("FSR: %d, Pitch: %d, Roll: %d\n" %(fsr, pitch, roll))
            else:
                # Timeout waiting for data, None is returned.
                print('Received no data!')
    finally:
        # Make sure device is disconnected on exit.
        device.disconnect()


# Initialize the BLE system.  MUST be called before other BLE calls!
ble.initialize()

# Start the mainloop to process BLE events, and run the provided function in
# a background thread.  When the provided main function stops running, returns
# an integer status code, or throws an error the program will exit.
ble.run_mainloop_with(main)
