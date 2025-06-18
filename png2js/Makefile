CC=gcc
CXX=g++
CFLAGS:=-Wall -Wextra -O2 -march=native $(CFLAGS)
CXXFLAGS:=-Wall -Wextra -O2 -march=native $(CXXFLAGS)

png2js: png2js.o findshapes.o readpng.o
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $^ -lpng

%.o: %.cc
	$(CXX) $(CXXFLAGS) $< -c -o $@

%.o: %.c
	$(CC) $(CFLAGS) $< -c -o $@

clean:
	rm *.o png2js
