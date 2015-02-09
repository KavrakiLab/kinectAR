ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include

bin_PROGRAMS = detect_marker

# gstreamer
#detect_marker_CFLAGS = $(CFLAGS) $(GLIB_CFLAGS) $(GSTREAMER_CFLAGS)
detect_marker_SOURCES = src/kinectMarker.cpp src/kinectAR.cpp src/detectMarker.cpp src/imageReceiver.cpp
detect_marker_LDADD = -losg -losgViewer -losgGA -lboost_system -lpthread -lglut -lGLU -lGL -lXi -lX11 -lm -lach -lpcl_common -lpcl_segmentation -lpcl_sample_consensus -lpcl_filters -lpcl_features -lalvar -ltinyxml -lopencv_calib3d -lopencv_core -lsns -lamino -lgfortran -lblas -llapack

bin_PROGRAMS += streampic

streampic_SOURCES = src/streampic.cpp
streampic_LDADD = -lsns -lamino -lblas -llapack -lach

bin_PROGRAMS += imshow
imshow_SOURCES = src/receive.cpp src/imageReceiver.cpp
imshow_LDADD = -lsns -lamino -lblas -llapack -lach
