import socket
import os
import smbus #necessary for accessing camera interface
from Adafruit_I2C import Adafruit_I2C #access camera interface
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.ADC as ADC #these should be all the modules we need
from Adafruit_BBIO.SPI import SPI
import time

size = 1024


#########################
########################
#Testing input pins
#GPIO.setup("P8_26", GPIO.IN)
#if GPIO.input("P8_26"):
	#print "HIGH"
#else:
	#print "LOW"
#time.sleep(2)
#GPIO.cleanup()

#i2c = Adafruit_I2C(address=0x40, busnum=2)
#i2c.readS16(i2c, 1)
#bus = smbus.SMBus(2)
#data = bus.read_byte(0x15)
#data = bus.readList(0x00, 0x08)
################################
################################


server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(("", 5000))
server_socket.listen(5)


print "TCP Server waiting for client on port 5000"

##################
##DARWIN CODE HERE
##################


f = open('methane_test.txt' ,'r')
numbers = []
for line in f:
	numbers.append(line)

load_cell = []
f2 = open('load_cell_data.txt', 'r')
for line in f2:
	load_cell.append(line)
	
fname = 'test.jpg'
fp = open(fname,'rb') #changed form w
#pixelData = []
#for pixels in fp:
	#pixelData.append(pixels)
image = fp.read()

numBytes = os.stat('test.jpg').st_size


while 1:
	client_socket, address = server_socket.accept()
	print "I got a connection from ", address
	print "Sending methane data..."
	for num in numbers:
		client_socket.send(num)
		print num
	
	client_socket.close()
	
	client_socket, address = server_socket.accept()
	
	print "Now going to send load cell data..."
	for num in load_cell:
		client_socket.send(num)
		print num
		
		
	client_socket.close()
	client_socket, address = server_socket.accept()
	#send image over	
	#fname = raw_input()
    	#client_socket.send(fname)
    	#fname = 'InternetResponse.png'
	#fp = open(fname,'w')
	print "Image to send is", numBytes, "in size"
	#while True:
	#for pixel in pixelData:
		#client_socket.send(numBytes)
		#strng = client_socket.recv(512)
		#client_socket.send(pixel)
		#if not strng:
			#break
		#fp.write(strng)
		
	client_socket.sendall(image) #should work
	client_socket.close()
	fp.close()
	print "Picture sent successfully!"







