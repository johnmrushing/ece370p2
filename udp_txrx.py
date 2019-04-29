import socket
import sys
from ctypes import *

vel = 0
phi = 0

UDP_IP = "192.168.86.244"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.connect((UDP_IP, UDP_PORT))

class cmdPacket(Structure):
	_fields_ = [("velocity", c_double),("theta", c_double),("mode", c_int)]
	
class rtnPacket(Structure):
	_fields_ = [("x", c_double),("y", c_double),("head", c_double)]

# mode 0 = return x,y,head
# mode 1 = driving command
# mode 2 = cardinal command
# mode 3 = stop

while True:
	key = raw_input("command: ")
	
	if ( 'w' == key ) : #forward
		print("fwd")
		
		vel = vel + 10
		
		sock.send( cmdPacket( vel, phi, 1 ) )

		
	elif ( 's' == key ):
		print("back")

		vel = vel - 10
		
		if (vel < 0):
			vel = 0
		
		sock.send( cmdPacket( vel, phi, 1 ) )
		
	elif ( 'a' == key ):
		print("left")
		
		phi = phi - 45
		
		sock.send( cmdPacket( vel, phi, 1 ) )
		
	elif ( 'd' == key ):
		print("right")
		
		phi = phi + 45
		
		sock.send( cmdPacket( vel, phi, 1 ) )
		
	elif ( 'i' == key ):
		print("north")
		
		vel = 0
		phi = 0
		
		sock.send( cmdPacket( vel, phi, 2 ) )
		
	elif ( 'k' == key ):
		print("south")
		
		vel = 0
		phi = 180
		
		sock.send( cmdPacket( vel, phi, 2 ) )
		
	elif ( 'j' == key ):
		print("west")
		
		vel = 0
		phi = 270
		
		sock.send( cmdPacket( vel, phi, 2 ) )
		
	elif ( 'l' == key ):
		print("east")
		
		vel = 0
		phi = 90
		
		sock.send( cmdPacket( vel, phi, 2 ) )
		
	elif ( ' ' == key ):
		print("report")
		
		sock.send( cmdPacket( vel, phi, 0 ) )
		
		buffer = sock.recv( sizeof( rtnPacket ) )
		
		newRtnPacket = rtnPacket.from_buffer_copy( buffer )
		for field_name, field_type in newRtnPacket._fields_:
			print field_name, getattr(newRtnPacket, field_name)
			
	elif ( 'q' == key ):
		print("all stop")
		
		vel = 0
		
		sock.send( cmdPacket( vel, phi, 3 ) )
		
	else:
		print("unrecognized")