CC = gcc
CFLAGS += -Wall

override MCUTOOL_VERSION = $(shell git describe --always --dirty --tags)

CPPFLAGS += -DMCUTOOL_VERSION='"'$(MCUTOOL_VERSION)'"'
CPPFLAGS += -D_GNU_SOURCE

LDFLAGS = -lcrypto

OBJ = crc32.o omnia-mcutool.o subcommand.o timeutils.o utils.o

omnia-mcutool: $(OBJ)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(OBJ) -o $@ $(LDFLAGS)

clean:
	rm -f omnia-mcutool $(OBJ)
