#!/bin/bash

DIR=$(cd -P $(dirname $0) && cd .. && pwd)
TMP=$(mktemp -d)

APRILTAG_FILES="Makefile
README
apriltag.c
apriltag.h
apriltag_math.h
apriltag_quad_thresh.c
tag16h5.c
tag16h5.h
tag25h7.c
tag25h7.h
tag25h9.c
tag25h9.h
tag36artoolkit.c
tag36artoolkit.h
tag36h10.c
tag36h10.h
tag36h11.c
tag36h11.h
example/Makefile
example/apriltag_demo.c
example/opencv_demo.cc
apriltag.pc.in
install.sh"

COMMON_FILES="doubles.h
doubles_floats_impl.h
floats.h
g2d.c
g2d.h
getopt.c
getopt.h
homography.c
homography.h
image_f32.c
image_f32.h
image_types.h
image_u8.c
image_u8.h
image_u8x3.c
image_u8x3.h
image_u8x4.c
image_u8x4.h
matd.c
matd.h
math_util.h
pam.c
pam.h
pjpeg.c
pjpeg-idct.c
pjpeg.h
pnm.c
pnm.h
postscript_utils.h
string_util.c
string_util.h
svd22.c
svd22.h
thash_impl.h
timeprofile.h
time_util.c
time_util.h
unionfind.c
unionfind.h
workerpool.c
workerpool.h
zarray.c
zarray.h
zhash.c
zhash.h
zmaxheap.c
zmaxheap.h"

mkdir -p $TMP/apriltag/common
mkdir -p $TMP/apriltag/example
$DIR/bin/licensify -l $DIR/licenses -C $DIR/src/apriltag $APRILTAG_FILES $TMP/apriltag
$DIR/bin/licensify -l $DIR/licenses -C $DIR/src/common $COMMON_FILES $TMP/apriltag/common

OUTDIR=$(pwd)
cd $TMP
tar czvf $OUTDIR/apriltag.tar.gz apriltag

#rm -rf $TMP
echo $TMP
