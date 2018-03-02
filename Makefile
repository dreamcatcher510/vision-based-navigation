CC = g++
CFLAGS = -w -O3 -ggdb -msse3 -fopenmp -DUNIX

VISP_BUILD_DIR = /home/robot/MyLibraries/ViSP/ViSP-build
VISP_CFLAGS =	`$(VISP_BUILD_DIR)/bin/visp-config --cflags`
VISP_LDFLAGS =	`$(VISP_BUILD_DIR)/bin/visp-config --libs`

EXEC_NAME = Test
INCLUDES =  -I/usr/include	-I/usr/local/include/opencv	-I/usr/local/include
LIBS =  -lPocoFoundation -lPocoNet -lturbojpeg -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_calib3d -lopencv_ml -lopencv_nonfree -lopencv_video -lopencv_features2d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_flann -lm -L/usr/local/lib -lvisp-2 -L/usr/lib -llapack -L/usr/lib/ -lblas -L/usr/lib/x86_64-linux-gnu/ -lSM -L/usr/lib/x86_64-linux-gnu/ -lICE -L/usr/lib/x86_64-linux-gnu/ -lX11 -L/usr/lib/x86_64-linux-gnu/ -lXext -L/usr/lib/x86_64-linux-gnu/ -lpthread -L/usr/lib/x86_64-linux-gnu/ -ljpeg -L/usr/lib/x86_64-linux-gnu/ -lpng -L/usr/lib/x86_64-linux-gnu/ -lz -L/usr/lib/x86_64-linux-gnu/ -ldc1394 -L/usr/lib/x86_64-linux-gnu/ -lv4l2 -L/usr/lib/x86_64-linux-gnu/ -lv4lconvert
SRC= $(wildcard *.cpp)	$(wildcard *.c)
OBJ= $(SRC:.c=.o) /home/robot/workspace/linaTest_v1.0/lib/ESMlib.a #/home/robot/MyLibraries/ViSP/ViSP-build/lib/libvisp-2.a  #/usr/local/lib/libvisp-2.a  #/home/robot/MyLibraries/ViSP/ViSP-build/lib/libvisp-2.a
#OBJ= $(SRC: .c)


all : $(EXEC_NAME)


$(EXEC_NAME) : $(OBJ)
	  $(CC) -o $(EXEC_NAME) $(CFLAGS) $(INCLUDES) $(OBJ) $(LIBS)

%.o: %.cpp
	  $(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<

%.o: %.cc
	  $(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<

%.o: %.c
	  gcc $(CFLAGS) $(INCLUDES) -o $@ -c $<

.PHONY: clean mrproper

clean:
		rm *.o

mrproper: 
		rm $(EXEC_NAME)
