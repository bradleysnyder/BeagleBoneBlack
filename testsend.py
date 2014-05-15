import socket
import smbus #necessary for accessing camera interface
from Adafruit_I2C import Adafruit_I2C #access camera interface
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.ADC as ADC #these should be all the modules we need
from Adafruit_BBIO.SPI import SPI
import time

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
bus = smbus.SMBus(2)
#data = bus.read_byte(0x15)
data = bus.readList(0x00, 0x08)


server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(("", 5000))
server_socket.listen(5)


print "TCP Server waiting for client on port 5000"

while 1:
	client_socket, address = server_socket.accept()
	print "Got connection from ", address
	f = open('methane_test.txt', 'r')
	#f.read()
	#numbers = [] data can't be in list
	count = 0
	for line in f:
		#print line
		#numbers.append(line)
		if count == 0:
			client_socket.send(line) #should send the list over
		count += 1
	client_socket.close()
