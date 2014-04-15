ACLOCAL_AMFLAGS = -I m4

SUBDIRS = src

EXTRA_DIST = bootstrap \
	ChangeLog \
	m4/ax_pthread.m4 \
	MacOSX/configure \
	MacOSX/convert_reader_h.pl \
	MacOSX/debuglog.h \
	MacOSX/ifdhandler.h \
	MacOSX/pcsclite.h \
	MacOSX/reader.h.in \
	MacOSX/winscard.h \
	MacOSX/wintypes.h

DISTCLEANFILES = ChangeLog

ChangeLog:
	git2cl > $@
