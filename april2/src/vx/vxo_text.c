/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <math.h>
#include <pthread.h>

#include "common/string_util.h"
#include "common/image_u8.h"
#include "common/april_util.h"

#include "vx.h"
#include "vxo_generic.h"

/** How to create a font:

    1. Export the font from Java, april.vis.VisFont.

    2. convert the png to a pnm
       convert serif__128.png serif__128.pnm

for f in *.png; do
    echo $f
    convert  $f ${f%.png}.pnm
done;


    3. run vx_make_font, passing it the name of the bparam file
       vx_make_font serif__128.bparam
       (vx_make_font *.bparam will work too.)

    4. copy those fonts to the right place.

**/

static zhash_t *font_cache;

static long flength(FILE *f)
{
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    return len;
}

static uint32_t decode32(uint8_t *buf, int *bufpos)
{
    uint32_t v = 0;
    for (int i = 0; i < 4; i++) {
        v = v << 8;
        v |= buf[(*bufpos)];
        (*bufpos)++;
    }
    return v;
}

typedef struct {
    vx_resource_t *texture;
    int height, width; // size of the texture

    int ascii_min, ascii_max;
    int tile_width, tile_height, tile_cols;
    int32_t *widths;
    int32_t *advances;

    float native_points;

} vxo_text_font_t;

typedef struct {
    int justification;
    zarray_t *fragments;
} vxo_text_line_t;

typedef struct {
    // when -1, the size is computed as a function of the string and
    // font. When not -1, the specified fixed width is used.
    int width;

    float rgba[4];
    vxo_text_font_t *font;
    char *s;

    float font_size;

    int invert;
} vxo_text_fragment_t;

// compute the width of a string, rendered in a given font.  It's
// equal to the advance of all the characters plus the width of the
// last character.
static double vxo_text_font_get_width(vxo_text_font_t *vf, const char *s)
{
    int slen = strlen(s);

    double width = 0;

    for (int i = 0; i < slen; i++) {
        int c = s[i];
        if (c < vf->ascii_min || c >= vf->ascii_max)
            c = ' ';

        if (i+1 < slen)
            width += vf->advances[c - vf->ascii_min];
        else
            width += vf->widths[c - vf->ascii_min];
    }

    return width;
}

static double vxo_text_font_get_advance(vxo_text_font_t *vf, const char *s)
{
    int slen = strlen(s);

    double advance = 0;

    for (int i = 0; i < slen; i++) {
        int c = s[i];
        if (c < vf->ascii_min || c >= vf->ascii_max)
            c = ' ';

        advance += vf->advances[c - vf->ascii_min];
    }

    return advance;
}

static int hexchar_to_int(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    assert(0);
    return 0;
}

static double vxo_text_fragment_get_advance(vxo_text_fragment_t *frag)
{
    if (frag->width >= 0)
        return frag->width;

    return vxo_text_font_get_advance(frag->font, frag->s) * frag->font_size / frag->font->native_points;
}

static double vxo_text_fragment_get_width(vxo_text_fragment_t *frag)
{
    if (frag->width >= 0)
        return frag->width;

    return vxo_text_font_get_width(frag->font, frag->s) * frag->font_size / frag->font->native_points;
}

void vxo_text_incref(vx_object_t *vxo)
{
    vx_lock();
    vxo->refcnt++;
    vx_unlock();
}

void vxo_text_decref(vx_object_t *vxo)
{
    vx_lock();

    int c = --vxo->refcnt;
    vx_unlock();

    if (c == 0) {
        vxo->u.text.chain->decref(vxo->u.text.chain);

        memset(vxo, 0, sizeof(struct vx_object)); // fail fast
        free(vxo);
    }
}

void vxo_text_serialize(vx_object_t *vxo, vx_serializer_t *outs)
{
    vxo->u.text.chain->serialize(vxo->u.text.chain, outs);
}

static double vxo_text_line_get_height(vxo_text_line_t *line)
{
    double height = 0;

    for (int fragidx = 0; fragidx < zarray_size(line->fragments); fragidx++) {
        vxo_text_fragment_t *frag;
        zarray_get(line->fragments, fragidx, &frag);

        height = fmax(height, frag->font->tile_height * frag->font_size / frag->font->native_points);
    }

//    if (zarray_size(line->fragments) == 0)
    // XXX doesn't handle font_size
//        height = 16;

    return height;
}

static double vxo_text_line_get_width(vxo_text_line_t *line)
{
    double width = 0;

    for (int fragidx = 0; fragidx < zarray_size(line->fragments); fragidx++) {
        vxo_text_fragment_t *frag;
        zarray_get(line->fragments, fragidx, &frag);

        if (fragidx + 1 < zarray_size(line->fragments))
            width += vxo_text_fragment_get_advance(frag);
        else
            width += vxo_text_fragment_get_width(frag);
    }

    return width;
}


static vxo_text_font_t *font_create(const char *font_name, int style, float points)
{
    vxo_text_font_t *vf = calloc(1, sizeof(vxo_text_font_t));
    vf->native_points = points;

    char pnm_path[1024];
    char vxf_path[1024];

    char font_path[1024];
    char* envpath = april_util_root_path();
    snprintf(font_path, 1023, "%s/web/vx/fonts", envpath);
    fprintf(stderr,
            "Using fonts folder %s\n",
            font_path);


    const char *style_string = "";
    if (style == VXO_TEXT_ITALIC)
        style_string = "i";
    if (style == VXO_TEXT_BOLD)
        style_string = "b";
    if (style == (VXO_TEXT_ITALIC | VXO_TEXT_BOLD))
        style_string = "bi";

    char *font_name_lower = str_tolowercase(strdup(font_name));
    sprintf(vxf_path, "%s/%s_%s_%.0f.vxf", font_path, font_name_lower, style_string, points);

    sprintf(pnm_path, "%s/%s_%s_%.0f.pnm", font_path, font_name_lower, style_string, points);
    free(font_name_lower);

    /////////////////////////////////////////////////////////
    if (1) {
        FILE *f = fopen(vxf_path, "r");
        if (f == NULL) {
            printf("Couldn't open font %s\n", vxf_path);
            exit(-1);
        }

        int cbuflen = flength(f);
        uint8_t *cbuf = malloc(cbuflen + C5_PAD);
        int res = fread(cbuf, 1, cbuflen, f);
        if (res != cbuflen) {
            printf("Unable to read %d bytes from %s, read %d instead\n", res, vxf_path, cbuflen);
            exit(-1);
        }
        fclose(f);

        uint8_t *buf = malloc(uc5_length(cbuf, cbuflen) + C5_PAD);
        int buflen;
        uc5(cbuf, cbuflen, buf,  &buflen);

        int pos = 0;

        uint32_t magic = decode32(buf, &pos);
        assert(magic == 0x0fed01ae);

        vf->ascii_min = decode32(buf, &pos);
        vf->ascii_max = decode32(buf, &pos);
        vf->tile_width = decode32(buf, &pos);
        vf->tile_height = decode32(buf, &pos);
        vf->tile_cols = decode32(buf, &pos);
        vf->width  = decode32(buf, &pos);
        vf->height = decode32(buf, &pos);

        int nwidths = decode32(buf, &pos);
        vf->widths = calloc(nwidths, sizeof(int32_t));
        for (int i = 0; i < nwidths; i++)
            vf->widths[i] = decode32(buf, &pos) / 100.0;

        int nadvances = decode32(buf, &pos);
        vf->advances = calloc(nadvances, sizeof(int32_t));
        for (int i = 0; i < nadvances; i++)
            vf->advances[i] = decode32(buf, &pos) / 100.0;

        uint32_t imformat = decode32(buf, &pos);
        assert(imformat == 0x00000001);

        image_u8_t *im = image_u8_create_alignment(vf->width, vf->height, 1);
        for (int y = 0; y < vf->height; y++)
            memcpy(&im->buf[y*im->stride], &buf[pos+y*vf->width], vf->width);

        assert(im->width == im->stride);

        vf->texture = vx_resource_make_texture_u8_copy(im, VX_TEXTURE_MIPMAP | VX_TEXTURE_MAX_LINEAR);

        image_u8_destroy(im);

        free(cbuf);
        free(buf);
    }

    assert(vf->ascii_min != 0);
    assert(vf->ascii_max != 0);
    assert(vf->tile_width != 0);
    assert(vf->tile_height != 0);
    assert(vf->tile_cols != 0);
    assert(vf->widths != NULL);
    assert(vf->advances != NULL);

    return vf;
}

static vxo_text_font_t *get_font(const char *font_name, int style)
{
    vx_lock();

    if (font_cache == NULL) {
        font_cache = zhash_create(sizeof(char*), sizeof(vxo_text_font_t*),
                                  zhash_str_hash, zhash_str_equals);
    }

    char *buf = malloc(1024);
    snprintf(buf, 1024, "%s-%d", font_name, style);

    vxo_text_font_t *font;
    if (!zhash_get(font_cache, &buf, &font)) {
        font = font_create(font_name, style, 128); // XXXXX Hard coded font size
        char *bufcpy = strdup(buf);
        zhash_put(font_cache, &bufcpy, &font, NULL, NULL);

        // we are maintaining a reference, so don't free these!
        font->texture->incref(font->texture);
    }

    free(buf);
    vx_unlock();

    return font;
}

static vx_object_t* vxo_text_fragment_make_program(vxo_text_fragment_t *frag)
{
    vxo_text_font_t *vf = frag->font;

    int slen = strlen(frag->s);
    float verts[12*slen], texcoords[12*slen];
    int nverts = 6*slen;
    float xpos = 0;

    for (int i = 0; i < slen; i++) {
        char c = frag->s[i];

        if (c < vf->ascii_min || c > vf->ascii_max)
            c = ' ';

        int idx = c - vf->ascii_min;
        int tile_y = idx / vf->tile_cols;
        int tile_x = idx % vf->tile_cols;

        float cwidth = vf->widths[idx];
        float advance = vf->advances[idx];

        // if inverting, we need to render the entire character or
        // we'll have gaps in the background.
        if (frag->invert && cwidth < advance)
            cwidth = advance;

        verts[12*i+0]     = xpos;
        verts[12*i+1]     = 0;
        texcoords[12*i+0] = tile_x*vf->tile_width;
        texcoords[12*i+1] = (tile_y+1)*vf->tile_height;

        verts[12*i+2]     = xpos + cwidth;
        verts[12*i+3]     = 0;
        texcoords[12*i+2] = tile_x*vf->tile_width + cwidth;
        texcoords[12*i+3] = (tile_y+1)*vf->tile_height;

        verts[12*i+4]     = xpos;
        verts[12*i+5]     = vf->tile_height;
        texcoords[12*i+4] = tile_x*vf->tile_width;
        texcoords[12*i+5] = (tile_y+0)*vf->tile_height;

        verts[12*i+6]     = xpos;
        verts[12*i+7]     = vf->tile_height;
        texcoords[12*i+6] = tile_x*vf->tile_width;
        texcoords[12*i+7] = (tile_y+0)*vf->tile_height;

        verts[12*i+8]     = xpos + cwidth;
        verts[12*i+9]     = 0;
        texcoords[12*i+8] = tile_x*vf->tile_width + cwidth;
        texcoords[12*i+9] = (tile_y+1)*vf->tile_height;

        verts[12*i+10]     = xpos + cwidth;
        verts[12*i+11]     = vf->tile_height;
        texcoords[12*i+10] = tile_x*vf->tile_width + cwidth;
        texcoords[12*i+11] = (tile_y+0)*vf->tile_height;

        xpos += advance;
    }

    // normalize texture coordinates
    for (int i = 0; i < nverts; i++) {
        texcoords[2*i+0] /= vf->width;
        texcoords[2*i+1] /= vf->height;
    }

    /////////////////////////////////////////////////////////
    // Create program
    vx_resource_t *position_resource = vx_resource_make_attr_f32_copy(verts, 2*nverts, 2);
    vx_resource_t *texcoord_resource = vx_resource_make_attr_f32_copy(texcoords, 2*nverts, 2);

    vx_lock();

    static vx_resource_t *program_resource = NULL;
    static vx_resource_t *invert_program_resource = NULL;

    if (program_resource == NULL) {

        char vertex_shader_src[] =
            "attribute vec2 aposition; \n"  \
            "attribute vec2 atexcoord; \n"  \
            "varying vec2 vtexcoord; \n"   \
            "uniform mat4 VX_P;\n"         \
            "uniform mat4 VX_V;\n"         \
            "uniform mat4 VX_M;\n"         \
            "void main(void) {\n"          \
            "  vtexcoord = atexcoord; \n " \
            "  gl_Position = VX_P * VX_V * VX_M * vec4(aposition, 0, 1.0);\n" \
            "}";

        char fragment_shader_src[] =
            "precision mediump float; \n"   \
            "varying vec2 vtexcoord; \n"         \
            "uniform sampler2D texture; \n" \
            "uniform vec4 color;\n" \
            "void main(void) {\n"           \
            "  vec4 t = texture2D(texture, vtexcoord);\n" \
            "  if (t.r == 0.0) discard;\n" \
            "  vec4 c = t.r * color;\n" \
            "  gl_FragColor = vec4(c.r, c.g, c.b, t.r);\n" \
            "}\n";

        program_resource = vx_resource_make_program(vertex_shader_src, fragment_shader_src);
        program_resource->incref(program_resource); // make immortal

        char invert_fragment_shader_src[] =
            "precision mediump float; \n"   \
            "varying vec2 vtexcoord; \n"         \
            "uniform sampler2D texture; \n" \
            "uniform vec4 color;\n" \
            "void main(void) {\n"           \
            "  vec4 t = texture2D(texture, vtexcoord);\n" \
            "  t.r = 1.0 - t.r;\n"          \
            "  if (t.r == 0.0) discard;\n"  \
            "  vec4 c = t.r * color;\n" \
            "  gl_FragColor = vec4(c.r, c.g, c.b, t.r);\n" \
            "}\n";

        invert_program_resource = vx_resource_make_program(vertex_shader_src, invert_fragment_shader_src);
        invert_program_resource->incref(invert_program_resource); // make immortal
    }

    vx_unlock();

    return vxo_generic_create(frag->invert ? invert_program_resource : program_resource,
                              (struct vxo_generic_uniformf[]) {
                                  { .name="color", .nrows = 4, .ncols = 1,
                                    .data = (float[]) { frag->rgba[0], frag->rgba[1], frag->rgba[2], frag->rgba[3] } },
                                  { .name=NULL } },
                              (struct vxo_generic_attribute[]) {
                                  { .name="aposition", .resource=position_resource },
                                  { .name="atexcoord", .resource=texcoord_resource },
                                  { .name=NULL } },
                              (struct vxo_generic_texture[]) {
                                  { .name="texture", .resource = vf->texture },
                                  { .name = NULL } },
                              (struct vxo_generic_draw[]) {
                                  { .command = VX_GL_TRIANGLES, .first = 0, .count = nverts },
                                  { .count = 0 }, });
}

vx_object_t *vxo_text(int anchor, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(NULL, 0, fmt, ap);
    char alltext[len+1];
    va_end(ap);
    va_start(ap, fmt);
    vsnprintf(alltext, len+1, fmt, ap);
    va_end(ap);

    assert(anchor >= VXO_TEXT_ANCHOR_TOP_LEFT && anchor <= VXO_TEXT_ANCHOR_BOTTOM_RIGHT);

    int justification = VXO_TEXT_JUSTIFY_LEFT;
    vxo_text_font_t *font = get_font("monospaced", VXO_TEXT_PLAIN);
    float font_size = 18;

    float rgba[4] = { .5, .5, .5, 1 }; // try to make visible regardless of background color
    int width = -1;
    int margin = 0; // how many pixels should we pad on all sides?
    int invert = 0;

    zarray_t *lines = zarray_create(sizeof(vxo_text_line_t*));

    zarray_t *ss = str_split(alltext, "\n");
    for (int ssidx = 0; ssidx < zarray_size(ss); ssidx++) {
        char *s;
        zarray_get(ss, ssidx, &s);
        int slen = strlen(s);

        int pos = 0;
        vxo_text_line_t *line = calloc(1, sizeof(vxo_text_line_t));
        line->fragments = zarray_create(sizeof(vxo_text_fragment_t*));

        line->justification = justification;
        zarray_add(lines, &line);

        if (0) {
            // for every line, always add a dummy fragment at the
            // end. This ensures that if the line has no
            // printable content that there is still at least one
            // fragment present. Without a fragment, the height of the
            // line will collapse to zero and you can't print blank
            // lines!
            vxo_text_fragment_t *frag = calloc(1, sizeof(vxo_text_fragment_t));
            frag->font = font;
            frag->font_size = font_size;
            memcpy(frag->rgba, rgba, 4*sizeof(float));
            frag->width = width;
            frag->invert = invert;
            frag->s = strdup("");
            zarray_add(line->fragments, &frag);
        }

        while (pos >= 0 && pos < slen) {
            // relative to beginning of 's', find the index where << and >> begin.
            int fmtpos = str_indexof(&s[pos], "<<");
            int endfmtpos = str_indexof(&s[pos], ">>");
            if (fmtpos >= 0)
                fmtpos += pos;
            if (endfmtpos >= 0)
                endfmtpos += pos;

            if (fmtpos > pos || fmtpos < 0 || endfmtpos < 0) {
                // here's a block of text that is ready to be rendered.
                vxo_text_fragment_t *frag = calloc(1, sizeof(vxo_text_fragment_t));
                frag->font = font;
                frag->font_size = font_size;
                memcpy(frag->rgba, rgba, 4*sizeof(float));
                frag->width = width;
                frag->invert = invert;

                if (fmtpos < 0)
                    frag->s = str_substring(s, pos, -1); // the whole string
                else
                    frag->s = str_substring(s, pos, fmtpos); // just part of the string

                zarray_add(line->fragments, &frag);
                pos = fmtpos;
                continue;
            }

            // a format specifier begins at pos
            char *chunk = str_substring(s, fmtpos+2, endfmtpos);

            // some format specifiers reset automatically every time we hit a new << >>
            width = -1;
            invert = 0;

            zarray_t *toks = str_split(chunk, ",");
            for (int tokidx = 0; tokidx < zarray_size(toks); tokidx++) {
                char *tok;
                zarray_get(toks, tokidx, &tok);
                str_trim(tok);
                str_tolowercase(tok);

                int tlen = strlen(tok);

                if (tok[0]=='#' && tlen==7) { // #RRGGBB
                    rgba[0] = ((hexchar_to_int(tok[1])<<4) + hexchar_to_int(tok[2])) / 255.0f;
                    rgba[1] = ((hexchar_to_int(tok[3])<<4) + hexchar_to_int(tok[4])) / 255.0f;
                    rgba[2] = ((hexchar_to_int(tok[5])<<4) + hexchar_to_int(tok[6])) / 255.0f;
                    rgba[3] = 1;
                    continue;
                }

                if (tok[0]=='#' && tlen==9) { // #RRGGBBAA
                    rgba[0] = ((hexchar_to_int(tok[1])<<4) + hexchar_to_int(tok[2])) / 255.0f;
                    rgba[1] = ((hexchar_to_int(tok[3])<<4) + hexchar_to_int(tok[4])) / 255.0f;
                    rgba[2] = ((hexchar_to_int(tok[5])<<4) + hexchar_to_int(tok[6])) / 255.0f;
                    rgba[3] = ((hexchar_to_int(tok[7])<<4) + hexchar_to_int(tok[8])) / 255.0f;
                    continue;
                }

                if (!strcmp(tok, "invert")) {
                    invert = 1;
                    continue;
                }

                if (1) {
                    const char *font_names[] = {"serif", "sansserif", "monospaced", NULL };
                    int good = 0;

                    for (int k = 0; font_names[k] != NULL; k++) {
                        if (str_starts_with(tok, font_names[k])) {
                            int style = VXO_TEXT_PLAIN;

                            zarray_t *parts = str_split(tok, "-");
                            for (int tsidx = 1; tsidx < zarray_size(parts); tsidx++) {
                                char *part;
                                zarray_get(parts, tsidx, &part);
                                if (!strcmp(part, "bold"))
                                    style |= VXO_TEXT_BOLD;
                                else if (!strcmp(part, "italic"))
                                    style |= VXO_TEXT_ITALIC;
                                else if (isdigit(part[0]))
                                    font_size = atof(part);
                                else
                                    printf("unknown font specification %s\n", part);
                            }

                            font = get_font(font_names[k], style);

                            zarray_vmap(parts, free);
                            zarray_destroy(parts);
                            good = 1;
                            break;
                        }
                    }
                    if (good)
                        continue;
                }

                if (!strcmp(tok, "left")) {
                    justification = VXO_TEXT_JUSTIFY_LEFT;
                    line->justification = justification;
                    continue;
                }

                if (!strcmp(tok, "right")) {
                    justification = VXO_TEXT_JUSTIFY_RIGHT;
                    line->justification = justification;
                    continue;
                }

                if (!strcmp(tok, "center")) {
                    justification = VXO_TEXT_JUSTIFY_CENTER;
                    line->justification = justification;
                    continue;
                }

                if (str_starts_with(s, "width=")) {
                    width = atoi(&s[6]);
                    continue;
                }

                printf("vx_text_t: unknown format %s\n", tok);
            }

            free(chunk);
            zarray_vmap(toks, free);
            zarray_destroy(toks);

            // skip to end of format specifier.
            pos =  endfmtpos + 2;
        }

        if (zarray_size(line->fragments) == 0) {
            // for every line, always add a dummy fragment at the
            // end. This ensures that if the line has no
            // printable content that there is still at least one
            // fragment present. Without a fragment, the height of the
            // line will collapse to zero and you can't print blank
            // lines!
            vxo_text_fragment_t *frag = calloc(1, sizeof(vxo_text_fragment_t));
            frag->font = font;
            frag->font_size = font_size;
            memcpy(frag->rgba, rgba, 4*sizeof(float));
            frag->width = width;
            frag->invert = invert;
            frag->s = strdup("");
            zarray_add(line->fragments, &frag);
        }
    }
    zarray_vmap(ss, free);
    zarray_destroy(ss);

    if (0) {
        // debug output
        for (int lineidx = 0; lineidx < zarray_size(lines); lineidx++) {
            vxo_text_line_t *line;
            zarray_get(lines, lineidx, &line);

            printf("LINE %d\n", lineidx);
            for (int fragidx = 0; fragidx < zarray_size(line->fragments); fragidx++) {
                vxo_text_fragment_t *frag;
                zarray_get(line->fragments, fragidx, &frag);

                printf(" [Frag %d]%s\n", fragidx, frag->s);
            }
        }
    }

    //////////////////////////////////////////////////////
    // it's render time.
    double total_height = 0;
    double total_width = 0;

    for (int lineidx = 0; lineidx < zarray_size(lines); lineidx++) {
        vxo_text_line_t *line;
        zarray_get(lines, lineidx, &line);

        double this_width = vxo_text_line_get_width(line);
        double this_height = vxo_text_line_get_height(line);

        total_width = fmax(total_width, this_width);
        total_height += this_height;
    }

    double anchorx = 0, anchory = 0;

    switch (anchor) {
        case VXO_TEXT_ANCHOR_TOP_LEFT:
        case VXO_TEXT_ANCHOR_LEFT:
        case VXO_TEXT_ANCHOR_BOTTOM_LEFT:
        case VXO_TEXT_ANCHOR_TOP_LEFT_ROUND:
        case VXO_TEXT_ANCHOR_LEFT_ROUND:
        case VXO_TEXT_ANCHOR_BOTTOM_LEFT_ROUND:
            anchorx = 0;
            break;

        case VXO_TEXT_ANCHOR_TOP_RIGHT:
        case VXO_TEXT_ANCHOR_RIGHT:
        case VXO_TEXT_ANCHOR_BOTTOM_RIGHT:
            anchorx = -total_width;
            break;

        case VXO_TEXT_ANCHOR_TOP_RIGHT_ROUND:
        case VXO_TEXT_ANCHOR_RIGHT_ROUND:
        case VXO_TEXT_ANCHOR_BOTTOM_RIGHT_ROUND:
            anchorx = round(-total_width);
            break;

        case VXO_TEXT_ANCHOR_TOP:
        case VXO_TEXT_ANCHOR_CENTER:
        case VXO_TEXT_ANCHOR_BOTTOM:
            anchorx = -total_width/2;
            break;

        case VXO_TEXT_ANCHOR_TOP_ROUND:
        case VXO_TEXT_ANCHOR_CENTER_ROUND:
        case VXO_TEXT_ANCHOR_BOTTOM_ROUND:
            anchorx = round(-total_width/2);
            break;
    }

    switch (anchor) {
        case VXO_TEXT_ANCHOR_TOP_LEFT:
        case VXO_TEXT_ANCHOR_TOP_RIGHT:
        case VXO_TEXT_ANCHOR_TOP:
            anchory = -total_height;
            break;

        case VXO_TEXT_ANCHOR_TOP_LEFT_ROUND:
        case VXO_TEXT_ANCHOR_TOP_RIGHT_ROUND:
        case VXO_TEXT_ANCHOR_TOP_ROUND:
            anchory = round(-total_height);
            break;

        case VXO_TEXT_ANCHOR_BOTTOM_LEFT:
        case VXO_TEXT_ANCHOR_BOTTOM_RIGHT:
        case VXO_TEXT_ANCHOR_BOTTOM:
        case VXO_TEXT_ANCHOR_BOTTOM_LEFT_ROUND:
        case VXO_TEXT_ANCHOR_BOTTOM_RIGHT_ROUND:
        case VXO_TEXT_ANCHOR_BOTTOM_ROUND:
            anchory = 0;
            break;

        case VXO_TEXT_ANCHOR_RIGHT:
        case VXO_TEXT_ANCHOR_LEFT:
        case VXO_TEXT_ANCHOR_CENTER:
            anchory = -total_height / 2;
            break;

        case VXO_TEXT_ANCHOR_RIGHT_ROUND:
        case VXO_TEXT_ANCHOR_LEFT_ROUND:
        case VXO_TEXT_ANCHOR_CENTER_ROUND:
            anchory = round(-total_height / 2);
            break;
    }

    // now actually construct the vx_object.
    vx_object_t *vt = calloc(1, sizeof(vx_object_t));
    vt->incref = vxo_text_incref;
    vt->decref = vxo_text_decref;
    vt->serialize = vxo_text_serialize;

    vt->u.text.chain = vxo_chain(NULL, NULL);
    vt->u.text.chain->incref(vt->u.text.chain);
    vt->u.text.width  = total_width;
    vt->u.text.height = total_height;

    // draw drop shadow
    if (1) {
        // XXX
    }


    // draw text
    double y = total_height;

    for (int lineidx = 0; lineidx < zarray_size(lines); lineidx++) {
        vxo_text_line_t *line;
        zarray_get(lines, lineidx, &line);

        double line_width = vxo_text_line_get_width(line);
        double x = 0;

        switch (line->justification) {
            case VXO_TEXT_JUSTIFY_LEFT:
                break;
            case VXO_TEXT_JUSTIFY_RIGHT:
                x = total_width - line_width;
                break;
            case VXO_TEXT_JUSTIFY_CENTER:
                x = (total_width - line_width) / 2.0;
                break;
        }

        // move up a line.
        y -= vxo_text_line_get_height(line);

        for (int fragidx = 0; fragidx < zarray_size(line->fragments); fragidx++) {
            vxo_text_fragment_t *frag;
            zarray_get(line->fragments, fragidx, &frag);

            vxo_chain_add(vt->u.text.chain,
                          vxo_chain(vxo_matrix_translate(anchorx + x + margin, anchory + y - margin, 0),
                                    vxo_matrix_scale(frag->font_size / frag->font->native_points),
                                    vxo_text_fragment_make_program(frag),
                                    NULL));

            x += vxo_text_fragment_get_advance(frag);
        }
    }

    //////////////////////////////////////////////////////
    // cleanup
    for (int lineidx = 0; lineidx < zarray_size(lines); lineidx++) {
        vxo_text_line_t *line;
        zarray_get(lines, lineidx, &line);

        for (int fragidx = 0; fragidx < zarray_size(line->fragments); fragidx++) {
            vxo_text_fragment_t *frag;
            zarray_get(line->fragments, fragidx, &frag);

            free(frag->s);
            free(frag);
        }

        zarray_destroy(line->fragments);
        free(line);
    }

    zarray_destroy(lines);

    return vt;
}

double vxo_text_get_width(vx_object_t *vo)
{
    return vo->u.text.width;
}

double vxo_text_get_height(vx_object_t *vo)
{
    return vo->u.text.height;
}
