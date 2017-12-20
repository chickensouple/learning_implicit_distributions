import zmq
import time

context = zmq.Context()

#  Socket to talk to server
print('Connecting to hello world server...')
socket = context.socket(zmq.PUB)
socket.connect('tcp://192.168.123.200:43354')
# socket.connect('tcp://192.168.123.146:43354')

#  Do 10 requests, waiting each time for a response
for request in range(100):
    print('Sending request %s ...' % request)
    socket.send(b"Hello")
    # socket.send("Hello")
    time.sleep(1)


