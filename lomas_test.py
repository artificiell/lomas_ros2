#!/usr/bin/python

import serial
import time
import argparse

# Main function
def main(args = None):
    parser = argparse.ArgumentParser(description='This is a basic gcode sender.')
    parser.add_argument('-p', '--port', help='Input USB port', default = '/dev/ttyACM0', type = str)  # /dev/ttyUSB0 or /dev/ttyACM0
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
        s.write(b"\r\n\r\n") # Hit enter a few times to wake the Printrbot
        time.sleep(2)       # Wait for Printrbot to initialize
        s.flushInput()      # Flush startup text in serial input

        # List of commands
        commands = [
            b"G10 P0 L20 X0 Y0 Z0\n",
            b"G10 P0 L20 X0\n",
            b"G10 P0 L20 Y0\n",
            b"G10 P0 L20 Z0\n",
            b"G90\n",
            b"G01 F800\n",
            b"G01 X5 Y5\n",
            b"G28\n"
          #  b"G01 X0 Y0\n",
          #  "G1 Y0",
          #  "G1 Z0",
          #  "G1 X100",
          #  "G1 Y100",
          #  "G1 Z100",
          #  "G1 X10 Y10 Z10"
          #  "G1 Y-10",
        ]
        cnt = 1
        for cmd in commands:
            s.write(cmd)     # Send g-code block
            grbl_out = s.readline() # Wait for response with carriage return
            #print (str(cnt) + ' : ' + str(cmd) + ' : ' + grbl_out.strip())
            time.sleep(args.delay)
            s.flushInput()      # Flush startup text in serial input
            cnt = cnt + 1
    except Exception as e:
        print ("Error: %s" % str(e))
        if s is not None:
            s.close()
    
if __name__ == '__main__':
    main()
