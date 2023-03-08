ACLOCAL_AMFLAGS = -I m4

SUBDIRS = src

EXTRA_DIST = bootstrap \
	ChangeLog \
	config/config.rpath \
	MacOSX/configure \
	MacOSX/convert_reader_h.pl \
	MacOSX/debuglog.h \
	MacOSX/ifdhandler.h \
	MacOSX/pcsclite.h \
	MacOSX/reader.h \
	MacOSX/winscard.h \
	MacOSX/wintypes.h

# Automatically update the libtool script if it becomes out-of-date.
LIBTOOL_DEPS = @LIBTOOL_DEPS@
libtool: $(LIBTOOL_DEPS)
	$(SHELL) ./config.status libtool

ChangeLog:
	git2cl > $@
