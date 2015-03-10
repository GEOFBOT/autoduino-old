CC = g++
CFLAGS = -std=c++0x
LIBS = -lwiringPi `pkg-config --libs opencv`

all: autoduino

autoduino: src/main.cpp src/linetools.cpp
	$(CC) $(CFLAGS) $(LIBS) src/main.cpp src/linetools.cpp -o autoduino

debug: src/main.cpp src/linetools.cpp
	$(CC) -g $(CFLAGS) $(LIBS) src/main.cpp src/linetools.cpp -o autoduino

clean:
	rm autoduino