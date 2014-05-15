CXX = g++
CXXFLAGS = -O2 -std=c++0x
CXXLIBX =
#CXXLINKER = g++

#all: BlackLib.o bbbtest.o
all: bbbtest

bbbtest: bbbtest.o BlackLib.o
	g++ bbbtest.o BlackLib.o -o bbbtest
BlackLib.o: BlackLib.cpp BlackErr.h BlackDef.h ov7670.h
bbbtest.o: bbbtest.cpp BlackLib.h BlackErr.h ov7670.h

clean:
	rm -rf BlackLib.o bbbtest.o bbbtest
