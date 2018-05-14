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

/**
 * See ./src/vx/example/vx_plot_test.c for sample code.
 */


#include "common/zarray.h"
#include "common/math_util.h"

#include "vx/vx.h"

#include <stdio.h>
#include <math.h>
#include <limits.h>

/* Plot Types */
#define PLOT_DISABLE 0  // Maintain data history but do not draw
#define PLOT_LINE 1     // Standard line plot
#define PLOT_SCATTER 2  // Scatter plot
#define PLOT_REFLINE 3  // Reference line 
#define PLOT_HISTOGRAM 4

typedef struct point point_t;
struct point
{   
    float xy[2];
};

typedef struct plot_data plot_data_t;
struct plot_data
{   
    zarray_t *points; //float
    float npoints; //If fixed memory data
    float color[4];
    char *name; //For legend
    int type;
    float size;
};

typedef struct plot vx_plot_t;
struct plot
{
    int pixcoords_anchor;

    // Area Available
    double w, h;
    double x, y;        // Upper left corner
    
    // Draw Area
    double dx, dy;
    double dw, dh;

    float *color_bg;
    float *color_axis;
    float *color_text;
    
    char *title;
    char *xlabel;
    char *ylabel;

    float xmin, xmax, ymin, ymax; //Data Range
    float dxmin, dxmax, dymin, dymax; //Draw Range

    zhash_t *plot_data; //Key == name

    //Interactivity
    float click_x, click_y;
    bool has_focus, has_moved;
};

/**
 * Creates empty plot structure with default colors (grey background, black text)
 * 
 * pixcoords_anchor 
 * xywh XY position, Width, Height in pixels
 * 
 */
vx_plot_t *vx_plot_create(int pixcoords_anchor, double xywh[4]);

/**
 * Creates empty plot structure with custom colors
 * 
 * pixcoords_anchor
 * xywh should have XY position, Width, Height in pixels 
 * color_bg should point to the desired background color in rgba
 * color_axis should point to the desired axes color in rgba
 * color_text should point to the desired text color in rgba
 * 
 */
vx_plot_t *vx_plot_create_with_color(int pixcoords_anchor, double xywh[4], float *color_bg, float *color_axis, float *color_text);

/**
 * Frees up memory being used by a plot including all of its data buffers.
 */
void vx_plot_destroy(vx_plot_t *plot);

/**
 * (OPTIONAL) Adds title to previously created plot. 
 */
void vx_plot_add_title(vx_plot_t *plot, char *title);

/**
 * (OPTIONAL) Adds label for X axis to previously created plot. 
 */
void vx_plot_add_xlabel(vx_plot_t *plot, char *xlabel);

/**
 * (OPTIONAL) Adds label for Y axis to previously created plot. 
 */
void vx_plot_add_ylabel(vx_plot_t *plot, char *ylabel);

/**
 * (INTERNAL) Defines ranges for previously created plot.
 * Overriden by max-min of data input to plot.  
 */
void vx_plot_add_yrange(vx_plot_t *plot, float ymin, float ymax);
void vx_plot_add_xrange(vx_plot_t *plot, float xmin, float xmax);
void vx_plot_add_ranges(vx_plot_t *plot, float xmin, float xmax, float ymin, float ymax);

/**
 * (INTERNAL) Creates a data buffer that can be used for plotting.
 * 
 * name should refer to a string containing the name of the data buffer.
 * color should point to the desired color for representing the data on the plot.
 * type should be one of the defined plot types.
 * size refers to the size of the line/point being drawn. Ideal values are 2-5. 
 * npoints should be a finite integer which will be the maximum number of points stored in the data buffer.
 * useful when plotting live streaming data. Set to 0, if no memory limit needs to be enforced.
 */
plot_data_t *vx_plot_create_plot_data(char *name, float *color, int type, float size, float npoints);

/**
 * (INTERNAL) Frees up memory belonging to a data buffer.
 */
void vx_plot_destroy_plot_data(plot_data_t *pd);

/**
 * Initializes a data buffer that can be plotted.
 * Should be used once to initialize a data buffer after which vx_plot_add_data should be used.
 * Any uses after the first call for a data buffer will result in no change.
 * 
 * plot should point to a previously initialized plot (created using vx_plot_create*)
 * name should be a string containing the name of the data buffer. Will be used for the legend.
 * vals should be a array of x,y pairs. These will be plotted on the graph.
 * nvals should be the length of vals 
 * color should point to the desired color for representing the data on the plot in rgba format.
 * type should be one of the defined plot types.
 * size refers to the size of the line/point being drawn. Ideal values are 2-5. 
 * npoints should be a finite integer which will be the maximum number of points stored in the data buffer.
 * Useful when plotting live streaming data. Set to 0, if no memory limit needs to be enforced.
 */
int vx_plot_init_data(vx_plot_t *plot, char *name, float vals[], int nvals, float *color, int type, float size, float npoints);

/**
 * Adds data to a pre-existing data buffer for a plot. 
 * Data buffer will be identified using the name parameter. Buffer limits, if enabled, will be enforced here.
 * 
 * plot should point to a previously initialized plot (created using vx_plot_create*)
 * name should be same string used in for initilization
 * vals should be a array of x,y pairs.
 * nvals should be the length of vals 
 */
int vx_plot_add_data(vx_plot_t *plot, char *name, float vals[], int nvals);

/**
 * Returns the number of points (x,y pairs) being stored in a data buffer
 *
 * plot should point to a previously initialized plot (created using vx_plot_create*)
 * name should be same string used for initilization
 */
int vx_plot_data_size(vx_plot_t *plot, char *name);

/**
 * Free up memory being used by a data buffer.
 * 
 * plot should point to a previously initialized plot (created using vx_plot_create*)
 * name should be same string used for initilization
 */
void vx_plot_remove_data(vx_plot_t *plot, char *name);

void vx_plot_edit_data(vx_plot_t *plot, char *name, float *color, int type, float size, float npoints);

/**
 * Draws the plot on the buffer
 * 
 * plot should point to the plot that needs to be drawn
 * vb should point to the buffer being drawn on.
 */
void vx_plot_render(vx_plot_t *plot, vx_buffer_t *vb);

bool vx_plot_contains(vx_plot_t *plot, double x, double y);

/**
 * Performs event handling for a plot.
 * Click on a plot to interact. Following actions are currently supported:
 *  WASD - Move along the X or Y axes.
 *  QE - Zoom level
 *  R - Reset to default view
 *  
 * plot
 * vl
 * ev
 * user
 */
bool vx_plot_on_event(vx_plot_t *plot, vx_layer_t *vl, const vx_event_t *ev, void *user);
