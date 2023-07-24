CC = gcc
CFLAGS += -Wall

OBJ = omnia-mcutool.o

omnia-mcutool: $(OBJ)
clean:
	rm -f omnia-mcutool $(OBJ)
