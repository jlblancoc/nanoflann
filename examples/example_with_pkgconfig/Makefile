# Example Makefile script
# Purpose: Demonstrate usage of pkg-config with the nanoflann library
# By: Jose Luis Blanco, 2011
#
#   ========================= *IMPORTANT* ================================
#   For this method to work nanoflann must be installed in your
#   system in a path accesible to pkg-config. To check if pkg-config
#   sees nanoflann config files, execute:
#      pkg-config --list-all | grep nanoflann
#   ======================================================================
#

# Set up basic variables:
CC         = g++
CFLAGS     = -c -Wall -O2 -mtune=native
LDFLAGS    =

# List of sources:
SOURCES    = pointcloud_example.cpp
OBJECTS    = $(SOURCES:.cpp=.o)

# Name of executable target:
EXECUTABLE = pointcloud_example

# nanoflann flags:
CFLAGS     += `pkg-config --cflags nanoflann`

all: $(SOURCES) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm $(OBJECTS) $(EXECUTABLE)
	
