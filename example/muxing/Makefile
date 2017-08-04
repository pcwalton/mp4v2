CC=g++
CFLAGS += -Werror -Wextra -Wall -g
LDLIBS := $(shell pkg-config --libs mp4v2) $(LDLIBS)

# Debug target to print variables.
print-%:
	@echo '$*=$($*)'

all: camm_muxing.o
	$(CC) camm_muxing.o $(LDLIBS) $(CFLAGS) -o camm_muxing

clean:
	rm camm_muxing camm_muxing.o

# You may need to add something similar to the following package config
# to your system's package config location. This may be located at, e.g.
# "/usr/local/lib/pkgconfig/mp4v2.pc" for mp4v2. (Try "man pkg-config".)
#
#prefix=/usr/local
#exec_prefix=${prefix}
#libdir=/usr/local/lib
#includedir=/usr/local/include
#
#Name: mp4v2
#Description: MP4v2 library
#Version: 1.0.0
#Requires:
#Requires.private:
#Conflicts:
#Libs: -L${libdir} -lmp4v2
#Libs.private:
#Cflags: -I${includedir}
