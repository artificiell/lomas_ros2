#!/usr/bin/python

import serial
import time
import argparse

# Main function
def main(args = None):
    parser = argparse.ArgumentParser(description='This is a basic gcode sender.')
    parser.add_argument('-p', '--port', help='Input USB port', default = '/dev/ttyUSB0', type = str)  # /dev/ttyUSB0 or /dev/ttyACM0
    parser.add_argument('-b', '--baud', help='Baud rate', default = 115200, type = int)               # 9600 or 115200
    parser.add_argument('-d', '--delay', help='Delay between commands', default = 0, type = int)
    args = parser.parse_args()

    print ("USB Port: %s"  % args.port)
    print ("Baud rate: %s" % args.baud)

    s = None
    try:
        # Open serial port
        print ('Opening Serial Port')
        s = serial.Serial(args.port, args.baud)

        # Wake up
        print ('Sending gcode')
        s.write("\r\n\r\n") # Hit enter a few times to wake the Printrbot
        time.sleep(2)       # Wait for Printrbot to initialize
        s.flushInput()      # Flush startup text in serial input

        # List of commands
        commands = [
            "G90",
            "G0 F8000",
            "G0 X200 Y200",
            "G1 F8000",
            "G1 X350 Y350 Z100",
            "G1 X1000 Y1000",
            "G0 Z0",
            "G0 X10 Y10"
        ]
        for cmd in commands:
            s.write(cmd + '\n')     # Send g-code block
            grbl_out = s.readline() # Wait for response with carriage return
            print (' : ' + grbl_out.strip())
            time.sleep(arg.delay)
        
    except Exception as e:
        print ("Error: %s" % str(e))
        if s is not None:
            s.close()
    
if __name__ == '__main__':
    main()
