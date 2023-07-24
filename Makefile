CC = gcc
CFLAGS += -Wall

override MCUTOOL_VERSION = $(shell git describe --always --dirty --tags)

CPPFLAGS += -DMCUTOOL_VERSION='"'$(MCUTOOL_VERSION)'"'

OBJ = omnia-mcutool.o timeutils.o

omnia-mcutool: $(OBJ)
clean:
	rm -f omnia-mcutool $(OBJ)
