TARGET = tvgmergepath
SRC = tvgmergepath.cpp

# ThorVG install prefix. Override to link a locally built engine:
#   make THORVG=/path/to/install
THORVG ?= /opt/homebrew

RES_DIR = $(CURDIR)/res

CXXFLAGS = -O3 -std=c++20 -DRES_DIR=\"$(RES_DIR)\" -I$(THORVG)/include
LIBS = $(shell sdl2-config --cflags --libs) -L$(THORVG)/lib -lthorvg -Wl,-rpath,$(THORVG)/lib

# homebrew headers/libs for SDL2 when THORVG points elsewhere
ifneq ($(THORVG), /opt/homebrew)
	CXXFLAGS += -I/opt/homebrew/include
	LIBS += -L/opt/homebrew/lib -Wl,-rpath,/opt/homebrew/lib
endif

all:
	g++ $(SRC) -o $(TARGET) $(CXXFLAGS) $(LIBS)

clean:
	rm -f $(TARGET)
