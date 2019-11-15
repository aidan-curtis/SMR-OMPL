CXX_FLAGS=-std=c++11 -O2 -Wall -Wextra

INCLUDE_FLAGS=`pkg-config --cflags ompl eigen3`
# Linker options
LD_FLAGS=`pkg-config --libs ompl`

# The c++ compiler to invoke
CXX=c++
all: FloppyNeedle

clean:
	rm -f *.o
	rm -f FloppyNeedle

%.o: src/%.cpp
	$(CXX) -c $(CXX_FLAGS) $(INCLUDE_FLAGS) $< -o $@

FloppyNeedle: FloppyNeedle.o CollisionChecking.o SMR.o
	$(CXX) $(CXX_FLAGS) $(INCLUDE_FLAGS) -o $@ $^ $(LD_FLAGS)
