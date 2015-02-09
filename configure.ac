#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.65])
AC_INIT([detect-marker], [0.0.0], [hbenamor@cc.gatech.edu])
AM_INIT_AUTOMAKE([-Wall])
AC_CONFIG_SRCDIR([src/detectMarker.cpp])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

# Checks for programs.
AC_USE_SYSTEM_EXTENSIONS
AC_PROG_CC
AC_PROG_CXX
dnl AC_PROG_CC_C99
AM_PROG_CC_C_O
AC_PROG_LIBTOOL

AC_C_RESTRICT
AC_C_INLINE
AC_C_CONST

# Enable C++11
m4_ifdef([AX_CXX_HEADER_STDCXX_11],
         [AX_CXX_HEADER_STDCXX_11],
         [AX_CXX_COMPILE_STDCXX_0X
         if test x"$ax_cv_cxx_compile_cxx0x_gxx" = xyes; then CXXFLAGS="$CXXFLAGS -std=gnu++0x"; fi])

# Enable maximum warnings
dnl m4_ifdef([AX_CFLAGS_WARN_ALL],
dnl          [AX_CFLAGS_WARN_ALL])

dnl APPEND_FLAG( [FLAG] )
dnl Check if FLAG is supported, and if so, add it

dnl The AX_CHECK_COMPILER_FLAGS maco was renamed to AX_CHECK_COMPILE_FLAG
dnl Checking for both versions
m4_ifdef([AX_CHECK_COMPILE_FLAG],
         [AC_DEFUN([APPEND_FLAG],
                   [AX_CHECK_COMPILE_FLAG([$1], [CFLAGS="$1 $CFLAGS"])])])

m4_ifdef([AX_CHECK_COMPILER_FLAGS],
         [AC_DEFUN([APPEND_FLAG],
                   [AX_CHECK_COMPILER_FLAGS([$1], [CFLAGS="$1 $CFLAGS"])])])


AC_CHECK_HEADERS([GL/glut.h])

# pkg-config does exactly what it was meant to do
PKG_CHECK_MODULES([GLIB], [glib-2.0])
PKG_CHECK_MODULES([OPENCV], [opencv])
PKG_CHECK_MODULES([EIGEN3], [eigen3])

CFLAGS="$CFLAGS $GLIB_CFLAGS $GSTREAMER_CFLAGS $OPENCV_CFLAGS $EIGEN3_CFLAGS $PCL_CFLAGS"
CXXFLAGS="$CXXFLAGS $GLIB_CFLAGS $GSTREAMER_CFLAGS $OPENCV_CFLAGS $EIGEN3_CFLAGS $PCL_CXXFLAGS"
LDFLAGS="$LDFLAGS $GLIB_LIBS $GSTREAMER_LIBS $OPENCV_LIBS $EIGEN3_LDFLAGS $PCL_LDFLAGS"

dnl If we found the flag checking macro, check some flags
dnl m4_ifdef([APPEND_FLAG],
dnl          [APPEND_FLAG([-Wextra])
dnl           APPEND_FLAG([-Wconversion])
dnl           APPEND_FLAG([-Wpointer-arith])
dnl           APPEND_FLAG([-Wfloat-equal])
dnl           APPEND_FLAG([-Wshadow])
dnl           APPEND_FLAG([-Wwrite-strings])
dnl           APPEND_FLAG([-Wc++-compat])])


AC_CONFIG_FILES([Makefile])

AC_OUTPUT

AC_MSG_NOTICE([])
AC_MSG_NOTICE([CONFIGURATION SUMMARY])
AC_MSG_NOTICE([=====================])
AC_MSG_NOTICE([PREFIX:          $prefix])
AC_MSG_NOTICE([CFLAGS:          $CFLAGS])
AC_MSG_NOTICE([CXXFLAGS:        $CXXFLAGS])
AC_MSG_NOTICE([LDFLAGS:         $LDFLAGS])
