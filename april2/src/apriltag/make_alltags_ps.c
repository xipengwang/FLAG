/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <stdio.h>
#include <stdarg.h>

#include "apriltag.h"
#include "tag36h11.h"

int foutf(FILE *f, const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    int res = vfprintf(f, fmt, ap);
    va_end(ap);

    res = fprintf(f, "\n");

    return res;
}

int main(int argc, char *argv[])
{
    double size_inches = 6.8; // consistent with java version

    apriltag_family_t *fam = tag36h11_create();

    FILE *f = fopen("alltags.ps", "w");

    foutf(f, "/pagewidth 8.5 72 mul def");
    foutf(f, "/pageheight 11 72 mul def");
    foutf(f, "");
    foutf(f, "%% how big should the tag be in inches? (black corner to black corner)");
    foutf(f, "/tagdim_in %f def", size_inches);
    foutf(f, "");
    foutf(f, "%% avoid gaps between individual cells. This is in bit-cell fractions");
    foutf(f, "/eps 0.002 def");
    foutf(f, "%% how wide is each tag in bit cells? (from black border to black");
    foutf(f, "%% border, incl.). For a standard 6x6 tag, this would be 8.");
    foutf(f, "/N %d def", fam->d + 2 * fam->black_border);
    foutf(f, "");
    foutf(f, "%% output a black bit cell");
    foutf(f, "/b {");
    foutf(f, "    /ct exch def");
    foutf(f, "    /x ct N mod def");
    foutf(f, "    /y ct x sub N div def");
    foutf(f, "    x eps sub y eps sub moveto");
    foutf(f, "    x 1 add eps add y eps sub lineto");
    foutf(f, "    x 1 add eps add y 1 add eps add lineto");
    foutf(f, "    x eps sub y 1 add eps add lineto");
    foutf(f, "    closepath");
    foutf(f, "    fill");
    foutf(f, "");
    foutf(f, "    ct 1 add");
    foutf(f, "} def");
    foutf(f, "");
    foutf(f, "%% output a white bit cell (actually, clear)");
    foutf(f, "/w {");
    foutf(f, "    1 add");
    foutf(f, "} def");
    foutf(f, "");
    foutf(f, "%% <string> makelabel");
    foutf(f, "/makelabel");
    foutf(f, "{");
    foutf(f, "    gsave");
    foutf(f, "    /Helvetica-Bold findfont 20 scalefont setfont");
    foutf(f, "    pagewidth 2 div 72 translate");
    foutf(f, "    dup");
    foutf(f, "    stringwidth pop -.5 mul 0 moveto");
    foutf(f, "    show");
    foutf(f, "    grestore");
    foutf(f, "} def");
    foutf(f, "");
    foutf(f, "%% <string> A");
    foutf(f, "/A");
    foutf(f, "{");
    foutf(f, "    gsave");
    foutf(f, "    /Helvetica-Bold findfont 20 scalefont setfont");
    foutf(f, "    pagewidth 2 div 72 translate");
    foutf(f, "    dup");
    foutf(f, "    stringwidth pop -.5 mul 0 moveto");
    foutf(f, "    show");
    foutf(f, "    grestore");
    foutf(f, "");
    foutf(f, "    gsave");
    foutf(f, "    %% center tag on page");
    foutf(f, "    pagewidth 2 div pageheight 2 div translate");
    foutf(f, "    tagdim_in 72 mul 2 div neg dup translate");
    foutf(f, "");
    foutf(f, "    %% draw at the right size");
    foutf(f, "    tagdim_in 72 mul N div dup scale");
    foutf(f, "    0");
    foutf(f, "} def");
    foutf(f, "");
    foutf(f, "/B");
    foutf(f, "{");
    foutf(f, "    pop");
    foutf(f, "    grestore");
    foutf(f, "    showpage");
    foutf(f, "");
    foutf(f, "} def");

    for (int i = 0; i < fam->ncodes; i++) {
        fprintf(f, "(%s -- %d) A ", fam->name, i);

        int dim = fam->d + 2 * fam->black_border;

        for (int row = 0; row < dim; row++) {
            for (int col = 0; col < dim; col++) {
                if (row < fam->black_border ||
                    col < fam->black_border ||
                    row >= (dim - fam->black_border) ||
                    col >= (dim - fam->black_border)) {
                    fprintf(f, "b ");
                    continue;
                }

                int X = row - fam->black_border;
                int Y = fam->d - 1 - (col - fam->black_border);
                int idx = X * fam->d + Y;
                if ((fam->codes[i] >> idx) & 1)
                    fprintf(f, "w ");
                else
                    fprintf(f, "b ");
            }
        }

        fprintf(f, "B\n");
    }

    fclose(f);
}
