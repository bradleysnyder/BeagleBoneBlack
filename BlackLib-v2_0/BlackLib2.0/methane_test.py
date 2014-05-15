from Adafruit_BBIO.SPI import SPI

spi = SPI(0,0)
spi.open(0,0)
#spi.msh = 100000
spi.bpw = 8
#spi.mode = b00
try:
#while True:
        # set CS bit high, choose first channel to read from

        #channel = 0;
        #adc = spi.xfer([1,8,0])
        #data = ((adc[1]&3) << 8) + adc[2]

        channelSelect = 0xC0
        channelSelect |= 0x18
        channelSelect <<= 3
        fsr = spi.xfer2([channelSelect, 0x00, 0x00])
        #result = spi.readbytes(2)
        #result2 = spi.readbytes(1)

        resultFinal = 0x0000
        resultFinal = 0x0300 & (resultFinal | (fsr[1] << 8 ))
        resultFinal |= ( ( 0x00FF  & fsr[2]) )
        #print (8 + channel ) << 4
        #print resultFinal

	###################################
	####################################
	resultFinal = 1 #temp
	file = open('methane_test.txt', 'r')
	data = []
	#data.append(str(resultFinal) + '\n')
	#file.read()
	count = 0
	for line in file:
		if not line.strip():
			continue
		if count < 19:
			data.append(line)
			count += 1
		elif count == 19:
			data.pop(0)
			break
	data.append(str(resultFinal))# + '\n')
	print data
	#print len(data)
	#data.pop(0)
	file.close()
	f = open('methane_test.txt' ,'w').close()
	#f = open('methane_test.txt','r+')
	#f.truncate()
	file = open('methane_test.txt', 'w')
	for number in data:
		print number
		#file.write("\n")
		file.write("%s" % number)
		#file.write("\n")
	#####################################
	#####################################

        #print data
        #print result[0] #"EYYYYY"
        #print result[1]
	file.close()
#end while

except KeyboardInterrupt:
        spi.close()
