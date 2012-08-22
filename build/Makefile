#Makefile for ttrack project

CC=g++
SRC=./src/
OBJ=./bin/
TARGET=ttrack

COMPILEFLAGS= -I /usr/local/include/TooN/  `pkg-config --cflags opencv` -g -Wall
LINKFLAGS = -L -lblas -llapack `pkg-config --libs opencv`

OBJECTS=main.o \
	ttrack.o \
	detect.o \
	tracker.o

all:$(OBJ)$(TARGET)

$(OBJ)$(TARGET): $(addprefix $(OBJ),$(OBJECTS))
	$(CC) -o $(OBJ)$(TARGET) $(addprefix $(OBJ),$(OBJECTS)) $(LINKFLAGS)

$(OBJ)%.o:$(SRC)%.cpp
	$(CC) $< -o $@ -c $(COMPILEFLAGS)

clean:
	rm -f $(OBJ)*.o $(OBJ)$(TARGET)