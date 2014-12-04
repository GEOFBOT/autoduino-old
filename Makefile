CC = g++
CFLAGS = -std=c++0x
LIBS = -lfann -lwiringPi `pkg-config --libs opencv`

all: run datagen train opencv-companion

run: run.cpp
	$(CC) $(CFLAGS) $(LIBS) run.cpp -o run

datagen: datagen.cpp
	$(CC) $(CFLAGS) $(LIBS) datagen.cpp -o datagen
    
train: train.cpp
	$(CC) $(CFLAGS) $(LIBS) train.cpp -o train
    
opencv-companion: OpenCV/OpenCV/Source.cpp
	$(CC) $(CFLAGS) $(LIBS) OpenCV/OpenCV/Source.cpp -o opencv-companion

clean:
	rm datagen train run opencv-companion