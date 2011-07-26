CC = gcc
CXX = g++
CFLAGS = -W -Wall -Wextra -O2 -march=native

png2js: png2js.o findshapes.o readpng.o
	${CXX} ${CFLAGS} -o $@ $^ -lpng

%.o: %.cc
	${CXX} ${CFLAGS} $< -c -o $@

%.o: %.c
	${CC} ${CFLAGS} $< -c -o $@

clean:
	rm *.o png2js
