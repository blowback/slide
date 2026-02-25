# Makefile for SLIDE
BEAST_DIR = $(HOME)/src/microbeast
SJASMPLUS_DIR = $(BEAST_DIR)/sjasmplus
CPMTOOLS_DIR = $(BEAST_DIR)/cpmtools-2.23

SJASMPLUS = ${SJASMPLUS_DIR}/sjasmplus

# CP/M stuff
CPM_DISK_TYPE = -f memotech-type50
CPM_IMAGE = slide_p25.img

MKFS = $(CPMTOOLS_DIR)/mkfs.cpm
MKFS_OPTS = $(CPM_DISK_TYPE) -b cpm22.bin 

CP = $(CPMTOOLS_DIR)/cpmcp
CP_OPTS = $(CPM_DISK_TYPE)

LS = $(CPMTOOLS_DIR)/cpmls
LS_OPTS = $(CPM_DISK_TYPE)


# NB TARGET set by OUTPUT directive in .asm file
ASM_FILES = $(wildcard *.asm)

all: slide.com slidiag.com reset.com

build: all

slide.com: slide.asm
	$(SJASMPLUS) --nologo --lst=slide.lst $^

slidiag.com: slidediag.asm
	$(SJASMPLUS) --nologo --lst=slidiag.lst $^

reset.com: reset.asm
	$(SJASMPLUS) --nologo --lst=reset.lst $^

clean:
	rm -f slide.com slidiag.com slide.lst slidiag.lst reset.com reset.lst

disk: slide.com slidiag.com reset.com
	$(MKFS) $(MKFSOPTS) $(CPM_IMAGE)
	$(CP) $(CP_OPTS) $(CPM_IMAGE) slide.com 0:slide.com
	$(CP) $(CP_OPTS) $(CPM_IMAGE) slidiag.com 0:slidiag.com
	$(CP) $(CP_OPTS) $(CPM_IMAGE) reset.com 0:reset.com
	$(LS) $(LS_OPTS) $(CPM_IMAGE)
	

.PHONY: all clean test disk

