# Set up basic variables:
CC         = g++
CFLAGS     = -c -Wall -O2 -mtune=native
LDFLAGS    =

# List of sources:
SOURCES    = test_flann.cpp
OBJECTS    = $(SOURCES:.cpp=.o)

# Name of executable target:
EXECUTABLE = test_flann

CFLAGS     += `pkg-config --cflags flann`
#LDFLAGS    += `pkg-config --libs flann`
LDFLAGS    += -lflann_cpp

all: $(SOURCES) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm $(OBJECTS) $(EXECUTABLE)
	
