#Makefile for ttrack project

CC=g++
SRC=./src/
OBJ=./bin/
TARGET=ttrack


COMPILEFLAGS= -I /usr/local/include/boost/ -I deps/image/  `pkg-config --cflags opencv` -g -Wall -DDEBUG -std=c++0x
LINKFLAGS = -L /usr/local/lib/ -lboost_thread -lboost_system -lboost_filesystem `pkg-config --libs opencv`

OBJECTS=main.o \
	ttrack.o \
	detect/detect.o \
	track/tracker.o \
	utils/handler.o \
	utils/helpers.o \
	detect/randomforest.o \
	detect/supportvectormachine.o \
	detect/baseclassifier.o \
	utils/nd_image.o \
	utils/camera.o \
	detect/im_mask_set.o \
	utils/model.o \
	track/kalman.o \
	track/pwp3d/pwp3d.o \
	track/pwp3d/stereo_pwp3d.o \
	track/pwp3d/mono_pwp3d.o \
	track/stt/monocular_tool_tracker.o \
	track/stt/stereo_tool_tracker.o \
	track/stt/surgical_tool_tracker.o \
	utils/quaternion.o

all:$(OBJ)$(TARGET)

$(OBJ)$(TARGET): $(addprefix $(OBJ),$(OBJECTS))
	$(CC) -o $(OBJ)$(TARGET) $(addprefix $(OBJ),$(OBJECTS)) $(LINKFLAGS)

$(OBJ)%.o:$(SRC)%.cpp | MKDIR
	$(CC) $< -o $@ -c $(COMPILEFLAGS)

clean:
	rm -rf $(OBJ)*.o 

MKDIR:
	mkdir -p $(OBJ)
	mkdir -p $(OBJ)detect
	mkdir -p $(OBJ)utils
	mkdir -p $(OBJ)track
	mkdir -p $(OBJ)track/pwp3d
	mkdir -p $(OBJ)track/stt