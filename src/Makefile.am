CCID_LIB = libacsccid.$(DYN_LIB_EXT)

CCID_VERSION=CCID_VERSION=`$(srcdir)/convert_version.pl $(PACKAGE_VERSION)`

COMMON = ccid.c \
	ccid.h \
	ccid_ifdhandler.h \
	commands.c \
	commands.h \
	acr38cmd.c \
	acr38cmd.h \
	debug.h \
	defs.h \
	ifdhandler.c \
	utils.c \
	utils.h
USB = ccid_usb.c ccid_usb.h
T1 = towitoko/atr.c \
	towitoko/atr.h \
	towitoko/defines.h \
	towitoko/pps.c \
	towitoko/pps.h \
	openct/buffer.c \
	openct/buffer.h \
	openct/checksum.c \
	openct/checksum.h \
	openct/proto-t1.c \
	openct/proto-t1.h
TOKEN_PARSER = tokenparser.l parser.h \
	strlcpy.c \
	misc.h \
	strlcpycat.h \
	simclist.c \
	simclist.h

if WITHOUT_PCSC
PROVIDED_BY_PCSC = debug.c
endif

bundlecontentsdir = $(usbdropdir)/$(bundle)/Contents
bundlecontents_DATA = Info.plist
bundlehostdir = $(bundlecontentsdir)/$(BUNDLE_HOST)
bundlehost_LTLIBRARIES = libacsccid.la

if UDEV_INSTALL
udevrulesdir = $(udevdir)/rules.d
udevrules_DATA = 92_pcscd_acsccid.rules
endif

libacsccid_la_SOURCES = $(COMMON) $(USB) $(TOKEN_PARSER) $(PROVIDED_BY_PCSC) $(T1)
libacsccid_la_LIBADD = $(LIBUSB_LIBS) $(PTHREAD_LIBS) $(LTLIBICONV)
libacsccid_la_CFLAGS = $(PCSC_CFLAGS) $(LIBUSB_CFLAGS) $(PTHREAD_CFLAGS) \
	$(SYMBOL_VISIBILITY) -D$(CCID_VERSION) -DSIMCLIST_NO_DUMPRESTORE
libacsccid_la_LDFLAGS = -avoid-version

EXTRA_DIST = Info.plist.src create_Info_plist.pl supported_readers.txt \
	towitoko/COPYING towitoko/README openct/LICENSE openct/README \
	convert_version.pl 92_pcscd_acsccid.rules

Info.plist: Info.plist.src $(srcdir)/supported_readers.txt
	$(srcdir)/create_Info_plist.pl $(srcdir)/supported_readers.txt $(srcdir)/Info.plist.src --target=$(CCID_LIB) --version=$(VERSION) $(NOCLASS) > Info.plist

DISTCLEANFILES = tokenparser.c Info.plist
