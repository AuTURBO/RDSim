
CFlags=-c -Wall -O3  -std=gnu++11 -Iinclude 
#LDFlags=  -ltinyxml
CC=g++
RM=rm

all: test.o test

test: test.o
	@mkdir -p bin
	$(CC) obj/$< -o bin/$@ $(LDFlags)

test.o: src/test.cpp
	@mkdir -p obj
	$(CC) $(CFlags) $< -o obj/$@

clean:
	$(RM) obj/test.o 
	$(RM) bin/test

install:
	@mkdir -p /usr/local/include/lightsfm
	@cp include/* /usr/local/include/lightsfm



