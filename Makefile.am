ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include -I/usr/local/ -I/usr/local/include/alvar -I/usr/local/include/opencv

bin_PROGRAMS = detect_marker

# gstreamer
#detect_marker_CFLAGS = $(CFLAGS) $(GLIB_CFLAGS) $(GSTREAMER_CFLAGS)
detect_marker_SOURCES = GlutViewer.cpp kinectAR.cpp detectMarker.cpp
detect_marker_LDADD = -lpthread -lglut -lGLU -lGL -lXi -lX11 -lm -lach -lfreenect_sync -lalvarplatform200 -lalvar200 -lopencv_calib3d -lopencv_core


