#!/bin/bash

rm apriltag1.[ch]

# The ugly of this script is that:
#
# 1) it's not general. You can't just add additional files and expect
# it to work. It mostly "just works", but some files have been
# slightly refactored to support this.
#
#    * Header files have to be included in dependency order
#    * #defines in headers will clobber each other.
#
# 2) it doesn't separate the internal APIs from the external APIs
# well. (TODO: Try to move some header files to the .c.)
#
cat apriltag1.h.hdr ../common/math_util.h ../common/pnm.h ../common/pam.h ../common/svd22.h ../common/time_util.h ../common/matd.h ../common/zarray.h ../common/string_util.h ../common/zhash.h ../common/image_types.h ../common/image_f32.h ../common/image_u8.h ../common/image_u8x3.h ../common/image_u8x4.h ../common/unionfind.h ../common/timeprofile.h ../common/workerpool.h ../common/homography.h ../common/zmaxheap.h apriltag_math.h apriltag.h ../common/g2d.h ../common/postscript_utils.h tag16h5.h tag25h7.h tag25h9.h tag36artoolkit.h tag36h10.h tag36h11.h ../common/getopt.h  apriltag1.h.ftr | grep -v "#include \"" > apriltag1.h

cat ../common/string_util.c ../common/time_util.c ../common/pnm.c ../common/pam.c ../common/zmaxheap.c ../common/zarray.c ../common/image_u8.c ../common/image_u8x3.c ../common/image_u8x4.c ../common/workerpool.c ../common/matd.c ../common/svd22.c ../common/g2d.c ../common/homography.c apriltag_quad_thresh.c apriltag.c tag16h5.c tag25h7.c tag25h9.c tag36artoolkit.c tag36h10.c tag36h11.c ../common/getopt.c ../common/zhash.c | grep -v "#include \"" > apriltag1.c.tmp
cat apriltag1.c.hdr apriltag1.c.tmp > apriltag1.c

rm apriltag1.c.tmp

gcc -Wall -c apriltag1.c -lm -lpthread
