spark_core = '10.0.0.7'
sc_port = 17061
BUFFER_SIZE = 2048
#just about the right amount of time to wait in between sending
perfect_time = 0.015
#print buf
print "Trying to socket and connect"
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

for x in xrange(2):
    try:
        s.connect((spark_core, sc_port))
        print "connected"
        for x in buf:
            s.send(x)
            time.sleep(perfect_time)
        print "Done sending"
        s.close()
        break
    except:
        print "Failure"

#get format from db and make new formatted response
#put this new formatted response into a text file
#send to spark core
'''