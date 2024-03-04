CC = gcc
CFLAGS += -Wall

MCUTOOL_VERSION ?= $(shell git describe --always --dirty --tags)
ifeq ($(MCUTOOL_VERSION),)
	MCUTOOL_VERSION = $(error you need to specify MCUTOOL_VERSION variable)
endif

CPPFLAGS += -DMCUTOOL_VERSION='"'$(MCUTOOL_VERSION)'"'
CPPFLAGS += -D_GNU_SOURCE

LDFLAGS = -lcrypto

OBJ = crc32.o omnia-mcutool.o subcommand.o timeutils.o utils.o

omnia-mcutool: $(OBJ)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(OBJ) -o $@ $(LDFLAGS)

clean:
	rm -f omnia-mcutool $(OBJ)
