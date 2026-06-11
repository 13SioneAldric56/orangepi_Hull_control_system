# Import UART and Pin classes for serial communication and GPIO control
from machine import UART, Pin

# Import time module for delays and timing operations
import time

# Initialize UART0 with baudrate 115200, using GPIO0 as TX (transmit) and GPIO1 as RX (receive)
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

# Define function to read GPS data from UART interface
def read_gps(uart):
    # Print startup message with instructions
    print("Reading GPS data... (Press Ctrl+C to stop)")
    
    # Begin try block to handle KeyboardInterrupt exception
    try:
        # Infinite loop to continuously read data
        while True:
            # Check if there is any data available in UART buffer
            if uart.any():
                # Read one line of data from UART
                line = uart.readline()
                
                # Check if line is not empty
                if line:
                    # Try to decode and process the line
                    try:
                        # Convert bytes to ASCII string and remove whitespace
                        line = line.decode('ascii').strip()
                        
                        # Check if line is a valid NMEA sentence (starts with $)
                        if line.startswith('$'):
                            # Print the complete NMEA sentence
                            print(line)
                    
                    # Handle any decoding errors
                    except:
                        # Ignore decoding errors and continue execution
                        pass
            
            # Wait 100ms before checking for new data again
            time.sleep(0.1)
    
    # Handle Ctrl+C keyboard interrupt
    except KeyboardInterrupt:
        # Print exit message when user interrupts the program
        print("\nExiting...")

# Call the read_gps function to start GPS data reading
read_gps(uart)