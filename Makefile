BW16_SOURCES = \
	test_bw16.cpp \
	bw16/capture.cpp \
	bw16/data.cpp \
	bw16/hal.cpp

BW16_HEADERS = \
	data_types.h \
	bw16/capture.h \
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
	g++ -g -Wall -Wextra -Werror \
	    -DHAL_RX_FILE="\"$(CURDIR)/test/bw16_rx\"" \
		-DHAL_TX_FILE="\"$(CURDIR)/test/bw16_tx\"" \
		-DCAPTURE_FILE="\"$(CURDIR)/test/capture\"" \
		$(BW16_SOURCES) -o $@

$(TEENSY_EXE): $(TEENSY_SOURCES) $(TEENSY_HEADERS)
	g++ -g -Wall -Wextra -Werror \
	    -DHAL_RX_FILE="\"$(CURDIR)/test/teensy_rx\"" \
		-DHAL_TX_FILE="\"$(CURDIR)/test/teensy_tx\"" \
		$(TEENSY_SOURCES) -o $@

ifeq ($(OS),Windows_NT)
    RM = del
else
    RM = rm -f
endif

clean:
	$(RM) $(BW16_EXE) $(TEENSY_EXE)
