import socket
import time

flightLanding_module_ip = '192.168.4.1'
flightLanding_port = 1336

GET_FILES_COMMAND = "GET_FILES"
DONE_COMMAND = "DONE"
BUFFER_SIZE = 20000

#just about the right amount of time to wait in between sending bytes

print "Connecting to ESP8266..."
print "Ensure you have connected to the correct network: AI-THINKER-A3A8B3"

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Try to connect to server
try:
    s.connect((flightLanding_module_ip, flightLanding_port))
except:
    print "Failure connecting to flight landing module"
    print "Ensure you are connected to the correct Wifi network and the\
    module is on"
    exit(1)

print "Succesfully connected to ESP8266"

# Get list of files available to download or delete
s.send(GET_FILES_COMMAND)

# Print .CSV files until we get DONE command
while 1:
	received = s.recv(BUFFER_SIZE)

	if(received == DONE_COMMAND):
		break

	print received


print "Done"

s.close()