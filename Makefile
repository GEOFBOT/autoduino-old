CC = g++
CFLAGS = -std=c++0x
LIBS = -lfann -lwiringPi

all: run datagen train

run: run.cpp
	$(CC) $(CFLAGS) $(LIBS) run.cpp -o run

datagen: datagen.cpp
	$(CC) $(CFLAGS) $(LIBS) datagen.cpp -o datagen
    
train: train.cpp
	$(CC) $(CFLAGS) $(LIBS) train.cpp -o train

clean:
	rm datagen train run