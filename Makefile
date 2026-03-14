BW16_SOURCES = \
	test_bw16.cpp \
	bw16/data.cpp \
	bw16/data_buf.cpp \
	bw16/hal.cpp

BW16_HEADERS = \
	data_types.h \
	bw16/data.h \
	bw16/hal.h

TEENSY_SOURCES = \
	test_teensy.cpp \
	teensy/data.cpp \
	teensy/hal.cpp

TEENSY_HEADERS = \
	data_types.h \
	teensy/data.h \
	teensy/hal.h

ifeq ($(OS),Windows_NT)
    BW16_EXE = test_bw16.exe
	TEENSY_EXE = test_teensy.exe
else
    BW16_EXE = test_bw16
	TEENSY_EXE = test_teensy
endif

.PHONY: all clean

all: $(BW16_EXE) $(TEENSY_EXE)

$(BW16_EXE): $(BW16_SOURCES) $(BW16_HEADERS)
	g++ -Wall -Wextra -Werror $(BW16_SOURCES) -o $@

$(TEENSY_EXE): $(TEENSY_SOURCES) $(TEENSY_HEADERS)
	g++ -Wall -Wextra -Werror $(TEENSY_SOURCES) -o $@

ifeq ($(OS),Windows_NT)
    RM = del
else
    RM = rm -f
endif

clean:
	$(RM) $(BW16_EXE) $(TEENSY_EXE)