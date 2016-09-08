CC = arm-linux-gnueabihf-g++
CFLAGS = -W -I/home/em/OPRoS_IDE_v2.1.2/OPRoS_external_package/ARM/include

TARGET = libHerkulex.so

$(TARGET) : Herkulex.o
	$(CC) -shared -o $(TARGET) Herkulex.o 

Herkulex.o : Herkulex.cpp Herkulex.h
	$(CC) -fPIC $(CFLAGS) -c Herkulex.cpp
