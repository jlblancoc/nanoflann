# Set up basic variables:
CC         = g++
CFLAGS     = -c -Wall -O2 -mtune=native 
LDFLAGS    =

# List of sources:
SOURCES    = test_nanoflann.cpp
OBJECTS    = $(SOURCES:.cpp=.o)

# Name of executable target:
EXECUTABLE = test_nanoflann

# MRPT specific flags:
#  Here we invoke "pkg-config" passing it as argument the list of the 
#  MRPT libraries needed by our program (see available libs 
#   with "pkg-config --list-all | grep mrpt").
#
#CFLAGS     += `pkg-config --cflags mrpt-base`


all: $(SOURCES) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm $(OBJECTS) $(EXECUTABLE)
