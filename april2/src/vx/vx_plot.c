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

#include "vx_plot.h"


/* Plot Generation Functions */

vx_plot_t *vx_plot_create(int pixcoords_anchor, double xywh[4])
{
    return vx_plot_create_with_color(pixcoords_anchor, xywh, vx_gray, vx_black, vx_black);
}

vx_plot_t *vx_plot_create_with_color(int pixcoords_anchor, double xywh[4], float *color_bg, float *color_axis, float *color_text)
{
    vx_plot_t *plot = calloc(1, sizeof(vx_plot_t));
    plot->plot_data = zhash_create(sizeof(char *), sizeof(plot_data_t *), zhash_str_hash, zhash_str_equals);

    plot->color_bg = color_bg;
    plot->color_axis = color_axis;
    plot->color_text = color_text;
    
    plot->title = NULL;
    plot->xlabel = NULL;
    plot->ylabel = NULL;

    plot->xmin = plot->xmax = plot->ymin = plot->ymax = 0;

    plot->pixcoords_anchor = pixcoords_anchor;
    plot->x = xywh[0];
    plot->y = xywh[1];
    plot->w = xywh[2];
    plot->h = xywh[3];

    plot->click_x = plot->click_y = 0;
    plot->has_focus = false;
    plot->has_moved = false;

    return plot;
}

void vx_plot_destroy(vx_plot_t *plot)
{
    if (plot->title!=NULL) free(plot->title);
    if (plot->xlabel!=NULL) free(plot->xlabel);
    if (plot->ylabel!=NULL) free(plot->ylabel);

    zhash_iterator_t zit;
    zhash_iterator_init(plot->plot_data, &zit);
    plot_data_t *pd = NULL;

    while (zhash_iterator_next(&zit, NULL, &pd))
    {
        vx_plot_destroy_plot_data(pd);
    }

    zhash_destroy(plot->plot_data);
    free(plot);
}

void vx_plot_add_title(vx_plot_t *plot, char *title)
{
    plot->title = strdup(title);
}


void vx_plot_add_xlabel(vx_plot_t *plot, char *xlabel)
{
    plot->xlabel = strdup(xlabel);
}


void vx_plot_add_ylabel(vx_plot_t *plot, char *ylabel)
{
    plot->ylabel = strdup(ylabel);
}


static void vx_plot_set_draw_xrange(vx_plot_t *plot, float xmin, float xmax) //XMIN, XMAX OR XMAX-XMIN
{
    plot->dxmin = xmin;
    plot->dxmax = xmax;   
}

static void vx_plot_set_draw_yrange(vx_plot_t *plot, float ymin, float ymax)
{
    plot->dymin = ymin;
    plot->dymax = ymax;   
}


static void vx_plot_set_draw_ranges(vx_plot_t *plot, float xmin, float xmax, float ymin, float ymax)
{
    vx_plot_set_draw_xrange(plot, xmin, xmax);
    vx_plot_set_draw_yrange(plot, ymin, ymax);           
}


void vx_plot_add_ranges(vx_plot_t *plot, float xmin, float xmax, float ymin, float ymax)
{
    vx_plot_add_xrange(plot, xmin, xmax);
    vx_plot_add_yrange(plot, ymin, ymax);
    // if (!plot->has_moved) 
        // vx_plot_set_draw_ranges(plot, xmin, xmax, ymin, ymax);
}

void vx_plot_add_xrange(vx_plot_t *plot, float xmin, float xmax) //XMIN, XMAX OR XMAX-XMIN
{
    plot->xmin = xmin;
    plot->xmax = xmax;
    if (!plot->has_moved) 
        vx_plot_set_draw_xrange(plot, xmin, xmax);
}

void vx_plot_add_yrange(vx_plot_t *plot, float ymin, float ymax)
{
    plot->ymin = ymin;
    plot->ymax = ymax;   
    if (!plot->has_moved) 
        vx_plot_set_draw_yrange(plot, ymin, ymax);
}

/* Plot Data Functions */

plot_data_t *vx_plot_create_plot_data(char *name, float *color, int type, float size, float npoints)
{
    plot_data_t *pd = calloc(1, sizeof(plot_data_t));
    pd->points = zarray_create(sizeof(float)); //float
    memcpy(pd->color, color, 4*sizeof(float));
    pd->name = strdup(name); //For legend
    pd->type = type;
    pd->size = size;
    pd->npoints = npoints;
    return pd;
}

void vx_plot_destroy_plot_data(plot_data_t *pd)
{
    zarray_destroy(pd->points);
    free(pd->name);
    free(pd);
}

int vx_plot_init_data(vx_plot_t *plot, char *name, float vals[], int nvals, 
                      float *color, int type, float size, float npoints)
{
    plot_data_t *pd = NULL;

    if (!zhash_contains(plot->plot_data, &name)) {
        pd = vx_plot_create_plot_data(name, color, type, (size==0)?2:(size), npoints);
        zhash_put(plot->plot_data, &name, &pd, NULL, NULL);
        // printf("%s added new data\n", __func__);
    }

    zhash_get(plot->plot_data, &name, &pd);
    // printf("%s name %s type %d\n", __func__, pd->name, pd->type);
    int i;
    float xmin = plot->xmin, xmax = plot->xmax, ymin = plot->ymin, ymax = plot->ymax;
    
    for (i = 0; i < nvals; i+=2)
    {
        if (1){
            xmin = (xmin>vals[i])?vals[i]:xmin; 
            xmax = (xmax<vals[i])?vals[i]:xmax;
            ymin = (ymin>vals[i+1])?vals[i+1]:ymin; 
            ymax = (ymax<vals[i+1])?vals[i+1]:ymax;

        }
        // printf("%s %d %f %f \n", __func__, i, vals[i], vals[i+1]);
        zarray_add(pd->points, &vals[i]);
        zarray_add(pd->points, &vals[i+1]);
    }
    vx_plot_add_ranges(plot, xmin, xmax, ymin, ymax);
    return i;
    // zhash_put(plot->plot_data, &name, &pd, NULL, NULL);
}

int vx_plot_add_data(vx_plot_t *plot, char *name, float vals[], int nvals)
{
    plot_data_t *pd = NULL;

    if (!zhash_contains(plot->plot_data, &name)) {
        return 0;
    }

    zhash_get(plot->plot_data, &name, &pd);
    
    float xmin = plot->xmin, xmax = plot->xmax, ymin = plot->ymin, ymax = plot->ymax;

    int j=0;
    float xy1[2];
    if (pd->npoints!=0 && zarray_size(pd->points) + nvals > pd->npoints*2) {
    //Then, check number of points in memory and delete extra
        int delpts = zarray_size(pd->points) + nvals - pd->npoints*2;
        for (j = 0; j < delpts; j+=2)
        {
        // printf("%s %d %f %f \n", __func__, i, vals[i], vals[i+1]);
            // zarray_get(pd->points, 0, &xy1[0]);
            // zarray_get(pd->points, 0+1, &xy1[1]);
            
            // printf("%s x %f y %f \n", __func__, xy1[0], xy1[1]);
            zarray_remove_index(pd->points, 0, false);
            zarray_remove_index(pd->points, 0, false);
        }
        xmin = ymin = FLT_MAX;
        xmax = ymax = FLT_MIN;
        for (j = 0; j < zarray_size(pd->points); j+=2)
        {
            zarray_get(pd->points, j, &xy1[0]);
            zarray_get(pd->points, j+1, &xy1[1]);
            if (1){
                xmin = (xmin>xy1[0])?xy1[0]:xmin; 
                xmax = (xmax<xy1[0])?xy1[0]:xmax;
                ymin = (ymin>xy1[1])?xy1[1]:ymin; 
                ymax = (ymax<xy1[1])?xy1[1]:ymax;
                // printf("%s xmin x %f %f ymin y %f %f \n", __func__, xmin, xy1[0], ymin, xy1[1]);
            }
        }
    }

    int i;
    for (i = 0; i < nvals; i+=2)
    {
        // printf("%s %d %f %f \n", __func__, i, vals[i], vals[i+1]);
        if (1){
            xmin = (xmin>vals[i])?vals[i]:xmin; 
            xmax = (xmax<vals[i])?vals[i]:xmax;
            ymin = (ymin>vals[i+1])?vals[i+1]:ymin; 
            ymax = (ymax<vals[i+1])?vals[i+1]:ymax;

        }
        zarray_add(pd->points, &vals[i]);
        zarray_add(pd->points, &vals[i+1]);
    }

    vx_plot_add_ranges(plot, xmin, xmax, ymin, ymax);
    return i;
}


int vx_plot_data_size(vx_plot_t *plot, char *name)
{
    plot_data_t *pd = NULL;

    if (!zhash_contains(plot->plot_data, &name)) {
        return 0;
    }

    zhash_get(plot->plot_data, &name, &pd);
    return zarray_size(pd->points);
}

void vx_plot_edit_data(vx_plot_t *plot, char *name, float *color, int type, float size, float npoints)
{

}

// Doesnt work. 
void vx_plot_remove_data(vx_plot_t *plot, char *name)
{
    plot_data_t *pd = NULL;
    // printf("%s attempting %s", __func__, name);
    if (!zhash_contains(plot->plot_data, &name)) {
        fprintf(stderr, "%s : attempting to delete non-existing key\n", __func__);
        return;
    }

    zhash_get(plot->plot_data, &name, &pd);
    printf("%s : status %d", __func__, zhash_remove(plot->plot_data, &name, NULL, NULL));
    vx_plot_destroy_plot_data(pd);
    
}

/* Rendering */

const int INSIDE = 0;
const int LEFT = 1;
const int RIGHT = 1<<1;
const int BOTTOM = 1<<2;
const int TOP = 1<<3;

int vx_plot_render_line_computecode(vx_plot_t *plot, float *xy)
{
    int code;
    code = INSIDE;
    if (xy[0] < plot->dxmin)           // to the left of clip window
        code |= LEFT;
    else if (xy[0] > plot->dxmax)      // to the right of clip window
        code |= RIGHT;
    if (xy[1] < plot->dymin)           // below the clip window
        code |= BOTTOM;
    else if (xy[1] > plot->dymax)      // above the clip window
        code |= TOP;
    return code;
}

bool vx_plot_render_line_clip(vx_plot_t *plot, float *xy1, float *xy2)
{
    int outcode0 = vx_plot_render_line_computecode(plot, xy1);
    int outcode1 = vx_plot_render_line_computecode(plot, xy2);
    bool accept = false;

    while (true) {
        if (!(outcode0 | outcode1)) {
            accept = true;
            break;
        } else if (outcode0 & outcode1) {
            break;
        } else {
            // failed both tests, so calculate the line segment to clip
            // from an outside point to an intersection with clip edge
            float x, y;

            // At least one endpoint is outside the clip rectangle; pick it.
            int outcodeOut = outcode0 ? outcode0 : outcode1;

            float diffx = xy2[0] - xy1[0];
            float diffy = xy2[1] - xy1[1];
            
            if (outcodeOut & TOP) {           // point is above the clip rectangle
                x = xy1[0] + (diffx) * (plot->dymax - xy1[1]) / (diffy);
                y = plot->dymax;
            } else if (outcodeOut & BOTTOM) { // point is below the clip rectangle
                x = xy1[0] + (diffx) * (plot->dymin - xy1[1]) / (diffy);
                y = plot->dymin;
            } else if (outcodeOut & RIGHT) {  // point is to the right of clip rectangle
                y = xy1[1] + (diffy) * (plot->dxmax - xy1[0]) / (diffx);
                x = plot->dxmax;
            } else if (outcodeOut & LEFT) {   // point is to the left of clip rectangle
                y = xy1[1] + (diffy) * (plot->dxmin - xy1[0]) / (diffx);
                x = plot->dxmin;
            }

            if (outcodeOut == outcode0) {
                xy1[0] = x;
                xy1[1] = y;
                outcode0 = vx_plot_render_line_computecode(plot, xy1);
            } else {
                xy2[0] = x;
                xy2[1] = y;
                outcode1 = vx_plot_render_line_computecode(plot, xy2);
            }
        }
    }
    if (accept) {
        return true;    
    }
    return false;
}

void vx_plot_render_line(vx_plot_t *plot, plot_data_t *pd, vx_buffer_t *vb)
{
    if (zarray_size(pd->points)<=2) return;

    int j = 4;
    float xy1[2], xy2[2];
    
    while(j<=zarray_size(pd->points)){
        zarray_get(pd->points, j-4, &xy1[0]);
        zarray_get(pd->points, j-3, &xy1[1]);
        zarray_get(pd->points, j-2, &xy2[0]);
        zarray_get(pd->points, j-1, &xy2[1]);
    
        if (vx_plot_render_line_clip(plot, xy1, xy2)){
            float data[] = {
                plot->dx + plot->dw*(xy1[0]-plot->dxmin)/(plot->dxmax-plot->dxmin), 
                plot->dy + plot->dh*(xy1[1]-plot->dymin)/(plot->dymax-plot->dymin), 
                0.1,
                plot->dx + plot->dw*(xy2[0]-plot->dxmin)/(plot->dxmax-plot->dxmin), 
                plot->dy + plot->dh*(xy2[1]-plot->dymin)/(plot->dymax-plot->dymin), 
                0.1,
            };
            vx_buffer_add_back(vb,
                               vxo_pixcoords(plot->pixcoords_anchor,
                                             VXO_PIXCOORDS_SCALE_MODE_ONE,                           
                                             vxo_lines(vx_resource_make_attr_f32_copy((float*)data, 2*3, 3), 
                                                       pd->color, 
                                                       pd->size),
                                             NULL),
                               NULL);
        }        
        j=j+2;
    }
}

void vx_plot_render_refline(vx_plot_t *plot, plot_data_t *pd, vx_buffer_t *vb)
{
    float xy[2];

    zarray_get(pd->points, 0, &xy[0]);
    zarray_get(pd->points, 1, &xy[1]);
    
    if (xy[1]>plot->dymin && xy[1]<plot->dymax){
        if (xy[1] != 0){
            float data[] = {
                plot->dx + plot->dw*0, 
                plot->dy + plot->dh*(xy[1]-plot->dymin)/(plot->dymax-plot->dymin), 
                0.1,
                plot->dx + plot->dw*1, 
                plot->dy + plot->dh*(xy[1]-plot->dymin)/(plot->dymax-plot->dymin), 
                0.1,
            };
            vx_buffer_add_back(vb,
                               vxo_pixcoords(plot->pixcoords_anchor,
                                             VXO_PIXCOORDS_SCALE_MODE_ONE,                           
                                             vxo_lines(vx_resource_make_attr_f32_copy((float*)data, 2*3, 3), 
                                                       pd->color, 
                                                       pd->size),
                                             NULL),
                               NULL);
        }
    }
    else if (xy[0]>plot->xmin && xy[0]<plot->xmax){
        if (xy[0] != 0){
            printf("%s\n", __func__);

            float data[] = {
                        plot->dx + plot->dw*(xy[0]-plot->xmin)/(plot->xmax-plot->xmin), 
                        plot->dy + plot->dh*(0), 
                        0.1,
                        plot->dx + plot->dw*(xy[0]-plot->xmin)/(plot->xmax-plot->xmin), 
                        plot->dy + plot->dh*(1), 
                        0.1,
                    };
     
            vx_buffer_add_back(vb,
                               vxo_pixcoords(plot->pixcoords_anchor,
                                             VXO_PIXCOORDS_SCALE_MODE_ONE,                           
                                             vxo_lines(vx_resource_make_attr_f32_copy((float*)data, 2*3, 3), 
                                                       pd->color, 
                                                       pd->size),
                                             NULL),
                               NULL);
        }
    }
}

void vx_plot_render_scatter(vx_plot_t *plot, plot_data_t *pd, vx_buffer_t *vb)
{
    float xy[2];
    for(int i=0; i<zarray_size(pd->points); i+=2){
        zarray_get(pd->points, i, &xy[0]);
        zarray_get(pd->points, i+1, &xy[1]);

        if (xy[0]>plot->dxmin && xy[0]<plot->dxmax && xy[1]>plot->dymin && xy[1]<plot->dymax){

            float data[] = {
                plot->dx + plot->dw*(xy[0]-plot->dxmin)/(plot->dxmax-plot->dxmin), 
                plot->dy + plot->dh*(xy[1]-plot->dymin)/(plot->dymax-plot->dymin), 
                0.1,
            };

            vx_buffer_add_back(vb,
                               vxo_pixcoords(plot->pixcoords_anchor,
                                             VXO_PIXCOORDS_SCALE_MODE_ONE,                           
                                             vxo_points(vx_resource_make_attr_f32_copy((float*)data, 3, 3), 
                                                        pd->color, 
                                                        pd->size),
                                             NULL),
                               NULL);
        }               
    }
}

void vx_plot_render_data(vx_plot_t *plot, plot_data_t *pd, vx_buffer_t *vb)
{
    if (plot->xmin == plot->xmax && plot->ymin == plot->ymax){
        fprintf(stderr, "%s : Plot ranges not initialized. Call vx_plot_add_ranges first.\n", __func__);
        return;
    }

    switch(pd->type){ 
        case PLOT_REFLINE:
            vx_plot_render_refline(plot, pd, vb);
            break;

        case PLOT_LINE:
            vx_plot_render_line(plot, pd, vb);
            break;

        case PLOT_SCATTER:
            vx_plot_render_scatter(plot, pd, vb);
            break;

        case PLOT_HISTOGRAM:
            break;

        case PLOT_DISABLE:
        default:
            break;
    }
}

void vx_plot_render(vx_plot_t *plot, vx_buffer_t *vb)
{

    vx_object_t *hack = vxo_text(VXO_TEXT_ANCHOR_CENTER, "<<monospaced-16,#%06x>>%s", plot->color_text, "hack");
    //double hack_tw = vxo_text_get_width(hack);
    double hack_th = vxo_text_get_height(hack);
    double arrow_size = 0.01;
    // printf("%s dx dy %f, %f dh, dw %f %f\n", __func__, plot->dx, plot->dy, plot->dh, plot->dw);

    float lines[] = {
        //Bottom Corner of Draw Area
        plot->x + ((plot->ylabel!=NULL)?hack_th:0) + ((plot->ymin != plot->ymax)?hack_th:0), 
        -plot->y - plot->h + ((plot->xlabel!=NULL)?hack_th:0) + ((plot->xmin != plot->xmax)?hack_th:0), 0,
        //X-AXIS Endpt
        plot->x + plot->w,
        -plot->y - plot->h + ((plot->xlabel!=NULL)?hack_th:0) + ((plot->xmin != plot->xmax)?hack_th:0), 0,
        //Y-AXIS Endpt
        plot->x + ((plot->ylabel!=NULL)?hack_th:0) + ((plot->ymin != plot->ymax)?hack_th:0), 
        -plot->y - (plot->title==NULL?0:hack_th), 0,
        //Bottom Corner of Draw Area
        plot->x + ((plot->ylabel!=NULL)?hack_th:0) + ((plot->ymin != plot->ymax)?hack_th:0), 
        -plot->y - plot->h + ((plot->xlabel!=NULL)?hack_th:0) + ((plot->xmin != plot->xmax)?hack_th:0), 0,

        //X -Axis Arrowhead
        plot->x + (1-arrow_size)*plot->w,
        -plot->y - (1-arrow_size)*plot->h + ((plot->xlabel!=NULL)?hack_th:0) + ((plot->xmin != plot->xmax)?hack_th:0), 0,
        //X-AXIS Endpt
        plot->x + plot->w,
        -plot->y - plot->h + ((plot->xlabel!=NULL)?hack_th:0) + ((plot->xmin != plot->xmax)?hack_th:0), 0,
        //X-AXIS Endpt
        plot->x + plot->w,
        -plot->y - plot->h + ((plot->xlabel!=NULL)?hack_th:0) + ((plot->xmin != plot->xmax)?hack_th:0), 0,
        plot->x + (1-arrow_size)*plot->w,
        -plot->y - (1+arrow_size)*plot->h + ((plot->xlabel!=NULL)?hack_th:0) + ((plot->xmin != plot->xmax)?hack_th:0), 0,
        
        //Y-Axis Arrowhead
        plot->x + (arrow_size)*plot->w + ((plot->ylabel!=NULL)?hack_th:0) + ((plot->ymin != plot->ymax)?hack_th:0), 
        -plot->y - (arrow_size)*plot->h - (plot->title==NULL?0:hack_th), 0,
        //Y-AXIS Endpt
        plot->x + ((plot->ylabel!=NULL)?hack_th:0) + ((plot->ymin != plot->ymax)?hack_th:0), 
        -plot->y - (plot->title==NULL?0:hack_th), 0,
        //Y-AXIS Endpt
        plot->x + ((plot->ylabel!=NULL)?hack_th:0) + ((plot->ymin != plot->ymax)?hack_th:0), 
        -plot->y - (plot->title==NULL?0:hack_th), 0,
        plot->x - (arrow_size)*plot->w + ((plot->ylabel!=NULL)?hack_th:0) + ((plot->ymin != plot->ymax)?hack_th:0), 
        -plot->y - (arrow_size)*plot->h - (plot->title==NULL?0:hack_th), 0,
    };

    plot->dx = lines[0];
    plot->dy = lines[1];
    plot->dh = fabs(lines[7] - lines[1]);
    plot->dw = fabs(lines[3] - lines[0]);

    //Create background and axes
    vx_object_t *bg = vxo_pixcoords(plot->pixcoords_anchor,
                                    VXO_PIXCOORDS_SCALE_MODE_ONE,
                                    vxo_lines(vx_resource_make_attr_f32_copy((float*)lines, 3*12, 3), plot->color_axis, 1.0),
                                    vxo_matrix_translate(plot->x + plot->w/2, -plot->y - plot->h/2, 0),
                                    vxo_matrix_scale3(plot->w, plot->h, 1),
                                    vxo_square_solid(plot->color_bg),
                                    NULL);
    

    if (plot->xlabel!=NULL){
        vx_object_t *xlabel = vxo_text(VXO_TEXT_ANCHOR_CENTER, "<<monospaced-16,#%06x>>%s", plot->color_text, plot->xlabel);
        double xlabel_tw = vxo_text_get_width(xlabel);
        double xlabel_th = vxo_text_get_height(xlabel);
        
        bg = vxo_pixcoords(plot->pixcoords_anchor,
                           VXO_PIXCOORDS_SCALE_MODE_ONE,
                           vxo_matrix_translate(plot->dx + plot->dw/2, 
                                                plot->dy - xlabel_th/2 - ((plot->xmin != plot->xmax)?xlabel_th:0), 
                                                0),
                           xlabel,
                           bg,
                           NULL);        
    }

    if (plot->ylabel!=NULL){
        vx_object_t *ylabel = vxo_text(VXO_TEXT_ANCHOR_CENTER, "<<monospaced-16,#%06x>>%s", plot->color_text, plot->ylabel);
        double ylabel_tw = vxo_text_get_width(ylabel);
        double ylabel_th = vxo_text_get_height(ylabel);
    
        bg = vxo_pixcoords(plot->pixcoords_anchor,
                           VXO_PIXCOORDS_SCALE_MODE_ONE,
                           vxo_matrix_translate(plot->dx - ylabel_th/2 - ((plot->ymin != plot->ymax)?ylabel_th:0), 
                                                plot->dy + plot->dh/2, 
                                                0),
                           vxo_matrix_rotatez(M_PI/2),
                           ylabel,
                           bg,
                           NULL);        
    }

    if (plot->title!=NULL){
        vx_object_t *title = vxo_text(VXO_TEXT_ANCHOR_CENTER, "<<monospaced-16,#%06x>>%s", plot->color_text, plot->title);
        double title_tw = vxo_text_get_width(title);
        double title_th = vxo_text_get_height(title);
    
        bg = vxo_pixcoords(plot->pixcoords_anchor,
                           VXO_PIXCOORDS_SCALE_MODE_ONE,
                           vxo_matrix_translate(plot->dx + plot->dw/2, plot->dy + plot->dh + title_th/2, 0),
                           title,
                           bg,
                           NULL);        
    }


    if (plot->dxmin != plot->dxmax){
        vx_object_t *xmin, *xmax;
        if (plot->dxmin==(int)plot->dxmin)
            xmin = vxo_text(VXO_TEXT_ANCHOR_CENTER, "<<monospaced-16,#%06x>>%-d", plot->color_text, (int)plot->dxmin);
        else
            xmin = vxo_text(VXO_TEXT_ANCHOR_CENTER, "<<monospaced-16,#%06x>>%-.3f", plot->color_text, plot->dxmin);
        if (plot->dxmax==(int)plot->dxmax)
            xmax = vxo_text(VXO_TEXT_ANCHOR_CENTER, "<<monospaced-16,#%06x>>%d", plot->color_text, (int)plot->dxmax);
        else
            xmax = vxo_text(VXO_TEXT_ANCHOR_CENTER, "<<monospaced-16,#%06x>>%.3f", plot->color_text, plot->dxmax);
        
        double xmin_tw = vxo_text_get_width(xmin);
        double xmin_th = vxo_text_get_height(xmin);
        double xmax_tw = vxo_text_get_width(xmax);
        double xmax_th = vxo_text_get_height(xmax);
        
        bg = vxo_pixcoords(plot->pixcoords_anchor,
                           VXO_PIXCOORDS_SCALE_MODE_ONE,
                           vxo_chain(vxo_matrix_translate(plot->dx + xmin_tw/2, plot->dy - xmin_th/2, 0),
                                     xmin, 
                                     NULL),
                           vxo_chain(vxo_matrix_translate(plot->dx + plot->dw - xmax_tw/2, plot->dy - xmax_th/2, 0),
                                     xmax,
                                     NULL),
                           bg,
                           NULL);        

    }

    if (plot->dymin != plot->dymax){
        vx_object_t *ymin, *ymax;
        if (plot->dymin==(int)plot->dymin)
            ymin = vxo_text(VXO_TEXT_ANCHOR_CENTER, "<<monospaced-16,#%06x>>%-d", plot->color_text, (int)plot->dymin);
        else
            ymin = vxo_text(VXO_TEXT_ANCHOR_CENTER, "<<monospaced-16,#%06x>>%-.3f", plot->color_text, plot->dymin);
        if (plot->dymax==(int)plot->dymax)
            ymax = vxo_text(VXO_TEXT_ANCHOR_CENTER, "<<monospaced-16,#%06x>>%d", plot->color_text, (int)plot->dymax);
        else
            ymax = vxo_text(VXO_TEXT_ANCHOR_CENTER, "<<monospaced-16,#%06x>>%.3f", plot->color_text, plot->dymax);
        
        double ymin_tw = vxo_text_get_width(ymin);
        double ymin_th = vxo_text_get_height(ymin);
        double ymax_tw = vxo_text_get_width(ymax);
        double ymax_th = vxo_text_get_height(ymax);
        
        bg = vxo_pixcoords(plot->pixcoords_anchor,
                           VXO_PIXCOORDS_SCALE_MODE_ONE,
                           vxo_chain(vxo_matrix_translate(plot->dx - ymin_th/2, plot->dy + ymin_tw/2, 0),
                                     vxo_matrix_rotatez(M_PI/2),
                                     ymin, 
                                     NULL),
                           vxo_chain(vxo_matrix_translate(plot->dx - ymin_th/2, plot->dy + plot->dh - ymin_tw/2, 0),
                                     vxo_matrix_rotatez(M_PI/2),
                                     ymax, 
                                     NULL),
                           bg,
                           NULL);        
    }

    if (plot->click_x!=0 && plot->click_y!=0){
        vx_object_t *click_value = vxo_text(VXO_TEXT_ANCHOR_CENTER, "<<monospaced-16,#%06x>>(%.3f, %.3f)", plot->color_text, 
                                            (plot->dxmax - plot->dxmin)*(plot->click_x - plot->dx)/plot->dw + plot->dxmin,
                                            (plot->dymax - plot->dymin)*(plot->click_y - plot->dy)/plot->dh + plot->dymin
                                            );

        double click_value_tw = vxo_text_get_width(click_value);
        double click_value_th = vxo_text_get_height(click_value);

        bg = vxo_pixcoords(plot->pixcoords_anchor,
                           VXO_PIXCOORDS_SCALE_MODE_ONE,
                           // vxo_chain(vxo_matrix_translate(plot->dx + plot->click_x, plot->dy + plot->click_y, 0),
                           //           vxo_matrix_scale3(click_value_tw, click_value_tw, 0),
                           //           vxo_square_solid(plot->color_text), 
                           //           NULL),
                           vxo_chain(vxo_matrix_translate(plot->click_x, plot->click_y, 0),
                                     click_value, 
                                     NULL),
                           bg,
                           NULL);  
    }


    vx_buffer_add_back(vb, bg, NULL);

    //Render data
    // fg = zarray_create(sizeof(vx_object_t *));
    zhash_iterator_t zit;
    zhash_iterator_init(plot->plot_data, &zit);
    plot_data_t *pd;
    char *buf;
    while (zhash_iterator_next(&zit, &buf, &pd))
    {
        vx_plot_render_data(plot, pd, vb);
    }
}

/* Event Handling */

bool vx_plot_contains(vx_plot_t *plot, double x, double y)
{
    double px = x - plot->x;
    double py = y - plot->y;

    return px > 0 && px < plot->w && py > 0 && py < plot->h;
}

bool vx_plot_draw_area_contains(vx_plot_t *plot, double x, double y)
{
    double px = x - plot->dx;
    double py = y - plot->dy;

    return px > 0 && px < plot->dw && py > 0 && py < plot->dh;
}

bool vx_plot_on_event(vx_plot_t *plot, vx_layer_t *vl, const vx_event_t *ev, void *user)
{
    int ix = -1, iy = -1;
    switch (ev->type) {
        case VX_EVENT_MOUSE_DOWN:
        case VX_EVENT_MOUSE_UP:
            ix = ev->u.mouse_button.x;
            iy = ev->u.mouse_button.y;
            break;
        case VX_EVENT_TOUCH_START:
        case VX_EVENT_TOUCH_END:
            ix = ev->u.touch.x;
            iy = ev->u.touch.y;
            break;
        case VX_EVENT_KEY_DOWN:
            if (plot->has_focus){
                // Move up X Axis
                if (ev->u.key.key_code == 'W'){
                    float yrange = plot->ymax - plot->ymin;

                    vx_plot_set_draw_ranges(plot, 
                                            plot->dxmin, plot->dxmax,
                                            plot->dymin + 0.1*yrange, plot->dymax + 0.1*yrange
                                            );
                    plot->has_moved = true;
                    return true;
                }
                // Move down X Axis
                else if (ev->u.key.key_code == 'S'){
                    float yrange = plot->ymax - plot->ymin;

                    vx_plot_set_draw_ranges(plot, 
                                            plot->dxmin, plot->dxmax,
                                            plot->dymin - 0.1*yrange, plot->dymax - 0.1*yrange
                                            );
                    plot->has_moved = true;
                    return true;
                }
                // Move left along Y
                else if (ev->u.key.key_code == 'A'){
                    float xrange = plot->xmax - plot->xmin;

                    vx_plot_set_draw_ranges(plot, 
                                            plot->dxmin - 0.1*xrange, plot->dxmax - 0.1*xrange,
                                            plot->dymin, plot->dymax
                                            );
                    plot->has_moved = true;
                    return true;
                }
                // Move right along Y
                else if (ev->u.key.key_code == 'D'){
                    float xrange = plot->xmax - plot->xmin;

                    vx_plot_set_draw_ranges(plot,
                                            plot->dxmin + 0.1*xrange, plot->dxmax + 0.1*xrange,
                                            plot->dymin, plot->dymax
                                            );
                    plot->has_moved = true;
                    return true;
                }
                // Zoom in
                else if (ev->u.key.key_code == 'E'){
                    float xrange = plot->xmax - plot->xmin;
                    float yrange = plot->ymax - plot->ymin;

                    vx_plot_set_draw_ranges(plot, 
                                            plot->dxmin + 0.1*xrange, plot->dxmax - 0.1*xrange,
                                            plot->dymin + 0.1*yrange, plot->dymax - 0.1*yrange
                                            );
                    plot->has_moved = true;
                    return true;
                }
                //Zoom out
                else if (ev->u.key.key_code == 'Q'){
                    float xrange = plot->xmax - plot->xmin;
                    float yrange = plot->ymax - plot->ymin;

                    vx_plot_set_draw_ranges(plot, 
                                            plot->dxmin - 0.1*xrange, plot->dxmax + 0.1*xrange,
                                            plot->dymin - 0.1*yrange, plot->dymax + 0.1*yrange
                                            );
                    plot->has_moved = true;
                    return true;
                }
                //Reset
                else if (ev->u.key.key_code == 'R'){
                    vx_plot_set_draw_ranges(plot, 
                                            plot->xmin, plot->xmax,
                                            plot->ymin, plot->ymax
                                            );
                    plot->has_moved = false;                    
                    return true;
                }

            }
        default:
            return false;
    }

    double vph = ev->viewport[3] - ev->viewport[1];
    double vpw = (ev->viewport[2] - ev->viewport[0]);

    // printf("%s vph %f vpw %f \n", __func__, vph, vpw);

    double x = (ix - ev->viewport[0]);//ev->viewport[3];
    double y = (iy - ev->viewport[1]);//ev->viewport[3];

    // printf("%s ix %d iy %d \n", __func__, ix, iy);
    // printf("%s x %f y %f \n", __func__, x, y);


    switch (plot->pixcoords_anchor) {
        case VXO_PIXCOORDS_TOP_LEFT:
            break;
        case VXO_PIXCOORDS_TOP:
            x -= vpw/2;
            break;
        case VXO_PIXCOORDS_TOP_RIGHT:
            x -= vpw;
            break;
        case VXO_PIXCOORDS_LEFT:
            y -= vph/2;
            break;
        case VXO_PIXCOORDS_CENTER:
            x -= vpw/2;
            y -= vph/2;
            break;
        case VXO_PIXCOORDS_RIGHT:
            x -= vpw;
            y -= vph/2;
            break;
        case VXO_PIXCOORDS_BOTTOM_LEFT:
            y -= vph;
            break;
        case VXO_PIXCOORDS_BOTTOM:
            x -= vpw/2;
            y -= vph;
            break;
        case VXO_PIXCOORDS_BOTTOM_RIGHT:
            x -= vpw;
            y -= vph;
            break;
        default:
            printf("ERR: pixcoords anchor type not supported\n");
            assert(false);
    }

    bool contains = vx_plot_contains(plot, x, y);
    // printf("%s plot click  %d \n", __func__, contains);

    // Flip for draw area co-ordinates
    y = -y;
    // printf("%s plot click draw area %d \n", __func__, vx_plot_draw_area_contains(plot, x, y));

    // printf("%s x y %f %f \n", __func__, x, y);
    // printf("%s dx dy %f %f \n", __func__, plot->dx, plot->dy);
    // printf("%s dw dh %f %f \n", __func__, plot->dw, plot->dh);
    
    switch (ev->type) {
        case VX_EVENT_MOUSE_DOWN:
        case VX_EVENT_TOUCH_START:
            if (contains){
                plot->has_focus = true;
                // printf("%s has focus\n", __func__);

                if (vx_plot_draw_area_contains(plot, x, y)) {
                // plot->click_x_val = (plot->xmax - plot->xmin)*(x - plot->dx)/plot->dw;
                // plot->click_y_val = (plot->ymax - plot->ymin)*(y - plot->dy)/plot->dh;
                    plot->click_x = x;
                    plot->click_y = y;
                // printf("%s x, y value %f ,%f \n", __func__, x_val, y_val);
                }

                return true;
            }
            else{
                plot->has_focus = false;
                plot->has_moved = false;
                // printf("%s lost focus\n", __func__);
            }
            break;

        case VX_EVENT_MOUSE_UP:
        case VX_EVENT_TOUCH_END:
            // To end XY value display
            plot->click_x = plot->click_y = 0;
            break;
    }

    return false;
}
