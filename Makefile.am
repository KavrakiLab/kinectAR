ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include -I/usr/local/include/alvar -I/usr/local/include/pcl-1.7/ -I/usr/include/eigen3/

bin_PROGRAMS = detect_marker

# gstreamer
#detect_marker_CFLAGS = $(CFLAGS) $(GLIB_CFLAGS) $(GSTREAMER_CFLAGS)
detect_marker_SOURCES = src/kinectMarker.cpp src/kinectAR.cpp src/detectMarker.cpp
detect_marker_LDADD = -losg -losgViewer -losgGA -lboost_system -lpthread -lglut -lGLU -lGL -lXi -lX11 -lm -lach -lpcl_common -lpcl_segmentation -lpcl_sample_consensus -lpcl_filters -lpcl_features -lalvarplatform200 -lalvar200 -lopencv_calib3d -lopencv_core -lsns -lamino -lgfortran -lblas -llapack


