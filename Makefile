#Makefile ArduRover Project TI3.3
#
#@ Remy Baratte, 2013
#
CC=g++
CFLAGS=-o ardu
SFLAGS=-Wall -pthread
OBJ=main.o

all: ardu

main.o: main.cpp
	$(CC) -c main.cpp $(SFLAGS)

ardu: main.o
	$(CC) $(OBJ) $(SFLAGS) $(CFLAGS)

clean: 
	rm -f $(OBJ) ardu
