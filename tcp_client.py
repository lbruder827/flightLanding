import socket
import time

flightLanding_module_ip = '192.168.4.1'
flightLanding_port = 1336

GET_FILES_COMMAND = "GET_FILES"
DONE_COMMAND = "DONE"
DOWNLOAD_COMMAND = "DOWNLOAD~"
BUFFER_SIZE = 20000

file_list = []

#just about the right amount of time to wait in between sending bytes

print "------ Connecting to ESP8266... ------\n"
print "Ensure you have connected to the correct network: AI-THINKER-A3A8B3"
print "If you are not connected to the correct wifi, press control+c to exit"
print "Then connect to the correct wifi network and try again"
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Try to connect to server
try:
    s.connect((flightLanding_module_ip, flightLanding_port))
except:
    print "Failure connecting to flight landing module"
    print "Ensure you are connected to the correct Wifi network and the\
    module is on"
    exit(1)

print "Succesfully connected to ESP8266\n"

# Get list of files available to download or delete
s.send(GET_FILES_COMMAND)

file_count = 0

# Print .CSV files until we get DONE command
while 1:
	received = s.recv(BUFFER_SIZE)

	if(received == DONE_COMMAND):
		break

	print str(file_count) + ": " + received
	file_count += 1

print "\n"

while 1:
	response = raw_input("Type 'download' to download a file\nType 'delete' to delete a file\nType 'exit' to exit this program\n\n")

	# Download
	if(response == 'download'):
		print "\n------ Download ------"
		print "Please enter the number the file you would like to download. For instance, if you want to download '2.csv', type in '2.csv'"
		print "To exit this option, type in 'exit'\n"

		input = raw_input('')

		if(input == "exit"):
			break;

		# Send the input and let it wait 
		s.send(DOWNLOAD_COMMAND + input)
		time.sleep(.2)

		# Get response from system
		response = s.recv(BUFFER_SIZE)

		if(response == "ERROR"):
			print "ERROR: Please select the correct file"
			pass
		else:
			# Get file size and make buffer that big
			print "Size of file: " + response + " bytes"

			size_of_file = int(response)

			response = ""
			while(1):
				response += s.recv(size_of_file+10)
				print response.decode('utf-8')

				if("DONE" in response):
					break

			print response.decode('utf-8')
			print len(response)

	# Delete
	elif(response == 'delete'):
		print "\n------ Delete ------"
		print "Please enter the number representing the file you would like to delete"
		print "If you want to delete all of the files, type in 'ALL'"
		print "To exit this option, type in 'exit'\n"
		input = raw_input('')
		time.sleep(.2)



	# Exit program
	elif(response == 'exit'):
		print "\n------ Exit ------"
		print "Exiting\n"
		break

	else:
		print "Input not recognized, please enter in something else\n"

	print "\n"

print "Bye!"

s.close()