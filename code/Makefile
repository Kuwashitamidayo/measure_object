#
# Basic OpenCV example tested on Ubuntu
#
APP = measure_object
MAIN = main
CPPINCLUDE_DIRS =  -I. -I/usr/local/include/opencv
CPPLIBS = -L/usr/local/lib/opencv -lopencv_core
LIBS =

CPP = g++
CFLAGS = -Wall -c -I.
CPPFLAGS = -Wall $(INCLUDE_DIRS)
LFLAGS = -Wall 

CPPSOURCE = $(APP).cpp $(MAIN).cpp
CPPOUTFILE = $(APP)
CPPOBJS = $(CPPSOURCE:.cpp=.o)

all: $(CPPOUTFILE)

$(CPPOUTFILE): $(CPPOBJS)
	$(CPP) $(CPPFLAGS) $(CPPOBJS) -o $(CPPOUTFILE) `pkg-config --libs opencv` $(CPPLIBS)

clean:
	rm -f *.o $(CPPOUTFILE)
