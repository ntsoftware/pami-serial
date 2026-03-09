BW16_SOURCES = \
	main_bw16.cpp \
	data_bw16.cpp \
	data_bw16_hal.cpp

BW16_HEADERS = \
	data.h \
	data_bw16.h \
	data_bw16_hal.h

TEENSY_SOURCES = \
	main_teensy.cpp \
	data_teensy.cpp \
	data_teensy_hal.cpp

TEENSY_HEADERS = \
	data.h \
	data_teensy.h \
	data_teensy_hal.h

all: bw16.exe teensy.exe

bw16.exe: $(BW16_SOURCES) $(BW16_HEADERS)
	g++ -Wall -Wextra -Werror $(BW16_SOURCES) -o $@

teensy.exe: $(TEENSY_SOURCES) $(TEENSY_HEADERS)
	g++ -Wall -Wextra -Werror $(TEENSY_SOURCES) -o $@

clean:
	del bw16.exe teensy.exe
