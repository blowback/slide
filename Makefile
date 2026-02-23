# Makefile for SLIDE
BEAST_DIR = /home/ant/src/microbeast
SJASMPLUS_DIR = $(BEAST_DIR)/sjasmplus
CPMTOOLS_DIR = $(BEAST_DIR)/cpmtools-2.23

SJASMPLUS = ${SJASMPLUS_DIR}/sjasmplus

# CP/M stuff
CPM_DISK = -f memotech-type50

MKFS = $(CPMTOOLS_DIR)/mkfs.cpm
MKFS_OPTS = $(CPM_DISK) -b cpm22.bin 

CP = $(CPMTOOLS_DIR)/cpmcp
CP_OPTS = $(CPM_DISK)

LS = $(CPMTOOLS_DIR)/cpmls
LS_OPTS = $(CPM_DISK)


# NB TARGET set by OUTPUT directive in .asm file
ASM_FILES = $(wildcard *.asm)
TARGET = slide.com
LISTING = slide.lst

all: $(TARGET)

build: all

$(TARGET): $(ASM_FILES)
	$(SJASMPLUS) --nologo --lst=$(LISTING) $^

clean:
	rm -f $(TARGET) $(LISTING)

disk: $(TARGET)
	$(MKFS) $(MKFSOPTS) slide_p24.img
	$(CP) $(CP_OPTS) slide_p24.img slide.com 0:slide.com
	$(LS) $(LS_OPTS) slide_p24.img
	

.PHONY: all clean test disk

