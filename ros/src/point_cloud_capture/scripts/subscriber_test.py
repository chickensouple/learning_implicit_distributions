#!/usr/bin/env python 
import msgpack
import zmq

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect('tcp://192.168.123.146:43354')
socket.setsockopt(zmq.SUBSCRIBE, '')

while True:
    data = socket.recv()
    message = msgpack.unpackb(data)
    # message = data
    print('Received reply: ' + str(message))

