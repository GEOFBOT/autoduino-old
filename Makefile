CC = g++
CFLAGS = -std=c++0x
LIBS = -lfann -lwiringPi

all: main datagen

main:
	echo TODO

datagen: datagen.cpp
	$(CC) $(CFLAGS) $(LIBS) datagen.cpp

clean:
	rm datagen