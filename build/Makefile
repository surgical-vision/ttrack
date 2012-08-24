#Makefile for ttrack project

CC=g++
SRC=./src/
OBJ=./bin/
TARGET=ttrack

COMPILEFLAGS= -I /usr/include/boost/  `pkg-config --cflags opencv` -g -Wall -DDEBUG
LINKFLAGS = -L /usr/lib/ -lboost_thread -lboost_system -lboost_filesystem `pkg-config --libs opencv`

OBJECTS=main.o \
	ttrack.o \
	detect.o \
	tracker.o \
	handler.o

all:$(OBJ)$(TARGET)

$(OBJ)$(TARGET): $(addprefix $(OBJ),$(OBJECTS))
	$(CC) -o $(OBJ)$(TARGET) $(addprefix $(OBJ),$(OBJECTS)) $(LINKFLAGS)

$(OBJ)%.o:$(SRC)%.cpp
	$(CC) $< -o $@ -c $(COMPILEFLAGS)

clean:
	rm -f $(OBJ)*.o $(OBJ)$(TARGET)