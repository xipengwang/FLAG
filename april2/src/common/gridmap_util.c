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

#include "gridmap_util.h"

#include <assert.h>
#include <zlib.h>
#include <math.h>
#include <stdio.h>
#include <limits.h>

#include <lcm/lcm.h>

#include "common/math_util.h"
#include "common/gridmap.h"
#include "common/zarray.h"
#include "common/doubles.h"
#include "common/image_u8x4.h"
#include "common/image_u8.h"
#include "common/time_util.h"

static void transform_point(double xyt[], double p[], double *rot){

    doubles_xyt_transform_xy(xyt, p, rot);

/*
	double x_j_p[3];
	double x_i_p[3] = {p[0], p[1], 0.0};

	ssc_tail2tail_2d (x_j_p, NULL, xyt,  x_i_p);

	rot[0] = x_j_p[0];
	rot[1] = x_j_p[1];
*/
}

// Doesn't allocate anything for the data field
grid_map_t * grid_map_t_copy_attributes(const grid_map_t * orig)
{
	grid_map_t * msg_out = calloc(1, sizeof(grid_map_t));
	msg_out->utime = orig->utime;
	msg_out->encoding = orig->encoding;
	msg_out->x0 = orig->x0;
	msg_out->y0 = orig->y0;
	msg_out->meters_per_pixel = orig->meters_per_pixel;
	msg_out->width = orig->width;
	msg_out->height = orig->height;
	msg_out->datalen = msg_out->width * msg_out->height;

	return msg_out;
}

grid_map_t * grid_map_t_encode_gzip(const grid_map_t * orig)
{
	if(orig->encoding != GRID_MAP_T_ENCODING_NONE){
		return NULL;
	}

	z_stream strm;
	strm.zalloc = Z_NULL;
	strm.zfree = Z_NULL;
	strm.opaque = Z_NULL;
	strm.total_out = 0;

	strm.next_in = (uint8_t *) orig->data;
	strm.avail_in = orig->datalen;

	int ret = deflateInit2(&strm, Z_DEFAULT_COMPRESSION, Z_DEFLATED, (15+16), 8, Z_DEFAULT_STRATEGY);
	if(ret != Z_OK){
		fprintf(stderr,"grid_map_t_encode_gzip(): Deflate init fail %d\n", ret);
		return NULL;
	}

	grid_map_t * msg_out = grid_map_t_copy_attributes(orig);
    int alloc = 2 * msg_out->width * msg_out->height + 64;
	msg_out->datalen = alloc; // this will be trimmed below
	msg_out->data = malloc(alloc);
	msg_out->encoding = GRID_MAP_T_ENCODING_GZIP;

	strm.next_out = (uint8_t *) msg_out->data;
	strm.avail_out = alloc;
	ret = deflate(&strm, Z_FINISH);

	if(ret != Z_STREAM_END){

		fprintf(stderr,"grid_map_t_encode_gzip(): Deflate map failed %d\n", ret);
		grid_map_t_destroy(msg_out);
		return NULL;
	}

	msg_out->datalen = msg_out->datalen - strm.avail_out;

	deflateEnd(&strm);

	return msg_out;
}

grid_map_t * grid_map_t_decode_gzip(const grid_map_t * orig)
{
	assert(orig->encoding == GRID_MAP_T_ENCODING_GZIP);

	grid_map_t * msg_out = grid_map_t_copy_attributes(orig);
	msg_out->datalen = msg_out->width*msg_out->height;
	msg_out->data = malloc(msg_out->datalen *sizeof(int8_t));
	msg_out->encoding = GRID_MAP_T_ENCODING_NONE;

	z_stream strm;
	strm.zalloc = 0;
	strm.zfree = 0;

	// source
	strm.next_in =  (uint8_t *) orig->data;
	strm.avail_in = orig->datalen;

	//destination
	strm.next_out = (uint8_t *)msg_out->data;
	strm.avail_out = msg_out->datalen;

	if (inflateInit2(&strm,15+32) != Z_OK) {
		printf("Failed to init inflation!\n");
		grid_map_t_destroy(msg_out);
		return NULL;
	} // Add 32 to windowBits to enable zlib and
	//gzip decoding with automatic header detection

	if (inflate(&strm,Z_FINISH)!= Z_STREAM_END) {
		fprintf(stderr,"grid_map_t_decode_gzip(): Failed to inflate map!\n");
		grid_map_t_destroy(msg_out);
		return NULL;
	}

	inflateEnd (&strm);

	return msg_out;
}

/*int gridmap_check_collision (const grid_map_t* gm_dilated, waypoint_t *center)
{
    //For the geometry of the car we need to check only 3 points
    //If the gridmap is already dilated by atleast Width/2, where
    //WIDTH = width of the car
    //1) The center itself
    if (gridmap_get_value (gm_dilated, center->x, center->y) == 255)
        return 0;//Collision

    double theta = center->heading;
    double s, c;
    fsincos(theta, &s, &c);
    //2) A point 1.395m infront of center
    double x = center->x + 1.395*c;
    double y = center->y + 1.395*s;
    if (gridmap_get_value (gm_dilated, x, y) == 255)
        return 0;//Collision

    //3) A point 2.74m infront of center
    x = center->x + 2.74*c;
    y = center->y + 2.74*s;
    if (gridmap_get_value (gm_dilated, x, y) == 255)
        return 0;//Collision

    //Safe
    return 1;
}*/

int gridmap_write_to_text_file (const grid_map_t *gm, const char *filename){

	if(!gm){
		return -1;
	}

	FILE *fp = fopen(filename, "w");

	if(!fp){
		return -1;
	}
	for (int ix = 0; ix < gm->width; ++ix){
		for (int iy = 0; iy < gm->height; ++iy){
			fprintf (fp, "%d ", gm->data[iy * gm->width + ix]);
		}
		fprintf (fp, "\n");
	}


	fclose(fp);
	return 0;

}


int gridmap_write_to_file (const grid_map_t *gm, const char *filename)
{

	FILE *fp = fopen(filename, "w");

	if(!fp){
		return -1;
	}

	uint32_t size = grid_map_t_encoded_size(gm);
	void *buf = malloc(size);

	int ret = grid_map_t_encode(buf, 0, size, gm);

	if(ret != size){
		free(buf);
		fclose(fp);
		return -1;
	}

	fwrite(&size, sizeof(uint32_t), 1, fp);
	fwrite(buf, size, 1, fp);

	free(buf);
	fclose(fp);

	return 0;
}

grid_map_t *gridmap_read_from_file (const char *filename)
{
	FILE *fp = fopen(filename, "r");

	if(!fp){
		fprintf(stderr,"gridmap_read_from_file(): No such file '%s'\n", filename);
		return NULL;
	}

	uint32_t size = 0;
	int ret = fread(&size, sizeof(uint32_t), 1, fp);
	if(ret != 1){
		fprintf(stderr,"gridmap_read_from_file(): Could not get encoded size\n");
		fclose(fp);
		return NULL;
	}

	void *buf = malloc(size);


	ret = fread (buf, size, 1, fp);
	fclose(fp);

	if(ret != 1) {
		fprintf(stderr,"gridmap_read_from_file(): Could not read in data\n");
		free (buf);
		return NULL;
	}

	grid_map_t *gm = malloc(sizeof(grid_map_t));
	ret = grid_map_t_decode(buf, 0, size, gm);

	free(buf);

	if(ret != size){
		fprintf(stderr,"gridmap_read_from_file(): Could not decode lcm type\n");
		free(gm);
		return NULL;
	}

	return gm;
}


typedef struct _search{
	int ix, iy;
}search_t;

void add_neighbor(grid_map_t *gmap, grid_map_t *visited, zarray_t *queue, int ix, int iy){

	if(ix < 0 || ix > gmap->width -1 || iy < 0 || iy > gmap->height - 1){
		return;
	}

	uint8_t vval = gridmap_get_value_index (visited, ix, iy);
	uint8_t gval = gridmap_get_value_index (gmap   , ix, iy);
	if(!vval && gval){
		search_t *s = malloc(sizeof(search_t));
		s->ix = ix;
		s->iy = iy;
		zarray_add(queue, &s);
	}
	gridmap_set_value_index(visited, ix, iy, 1);

}

zarray_t *expand_prune(grid_map_t *gmap, grid_map_t *visited, int ix, int iy){



	zarray_t *queue = zarray_create(sizeof(void*));
	zarray_t *record = zarray_create(sizeof(void*));

	search_t *search = malloc(sizeof(search_t));
	search->ix = ix;
	search->iy = iy;

	zarray_add(queue, &search);


	int num_nodes = 0;
	while(zarray_size(queue)){
		num_nodes++;

		//stack queue, whats really the difference? Hah
				search_t *node;
		zarray_get(queue, 0, &node);
		zarray_remove_index(queue, 0, 0);
		zarray_add(record, &node);
		gridmap_set_value_index(visited, node->ix, node->iy, 1);

		add_neighbor(gmap, visited, queue, node->ix - 1, node->iy);
		add_neighbor(gmap, visited, queue, node->ix + 1, node->iy);
		add_neighbor(gmap, visited, queue, node->ix, node->iy - 1);
		add_neighbor(gmap, visited, queue, node->ix, node->iy + 1);


	}
	return record;

}


grid_map_t *
gridmap_prune (const grid_map_t *gm){

	grid_map_t *visited = grid_map_t_copy_attributes(gm);
	visited->data = calloc(visited->datalen, sizeof(uint8_t));


	lcm_t *lcm = lcm_create(NULL);
	gridmap_publish_encoded(lcm, "ORIG_GRID", gm);
	grid_map_t *gmap = grid_map_t_copy_attributes(gm);
	gmap->data = malloc(sizeof(uint8_t) * gmap->datalen);
	memcpy(gmap->data, gm->data, sizeof(uint8_t) * gmap->datalen);

	for (int i = 0; i < gmap->height; i++){
		for(int j = 0; j < gmap->width; j++){


			int neighbors = 0;
			if (i > 0){
				neighbors += gridmap_get_value_index (gm, j, i - 1) > 0;
			}

			if (i < gm->height - 1){
				neighbors += gridmap_get_value_index (gm, j, i + 1) > 0;
			}

			if (j > 0){
				neighbors += gridmap_get_value_index (gm, j - 1, i) > 0;
			}

			if (j < gm->width - 1){
				neighbors += gridmap_get_value_index (gm, j + 1, i) > 0;
			}

			if(neighbors < 3){
				gridmap_set_value_index (gmap, j, i, 0);
			}
		}
	}
	int minx = INT_MAX;
	int miny = INT_MAX;
	int maxx = INT_MIN;
	int maxy = INT_MIN;

	for (int i = 0; i < gmap->height; i++){
		for(int j = 0; j < gmap->width; j++){

			uint8_t visited_val = gridmap_get_value_index(visited, j, i);
			if(visited_val != 0){
				continue;
			}

			uint8_t grid_val = gridmap_get_value_index(gmap, j, i);
			if(grid_val == 0){
				gridmap_set_value_index(visited, j, i, 1);
				continue;
			}

			zarray_t *connected = expand_prune(gmap, visited, j, i);


			if(zarray_size(connected) < 5000){
				while(zarray_size(connected)){
					search_t *s;
					zarray_get(connected, 0, &s);
					zarray_remove_index(connected, 0, 0);
					gridmap_set_value_index(gmap, s->ix, s->iy, 0);

				}
			}
			else{
				while(zarray_size(connected)){
					search_t *s;
					zarray_get(connected, 0, &s);
					zarray_remove_index(connected, 0, 0);
					if(s->ix < minx){
						minx = s->ix;
					}
					if(s->iy < miny){
						miny = s->iy;
					}
					if(s->ix > maxx){
						maxx = s->ix;
					}
					if(s->iy > maxy){
						maxy = s->iy;
					}
				}

			}

		}
	}

	printf("%d %d %d %d\n", minx, miny, maxx, maxy);
	double meters_per_pixel = gmap->meters_per_pixel;
	double x0 = minx * meters_per_pixel + gmap->x0;
	double y0 = miny * meters_per_pixel + gmap->y0;

	int width = maxx - minx;
	int height = maxy - miny;

	grid_map_t *outmap = gridmap_make_pixels(x0, y0, width, height, meters_per_pixel, 0, 0);

	double x = 0;
	double y = 0;
	for (int i = 0; i < gmap->height; i++){
		for (int j = 0; j < gmap->width; j++){

			uint8_t val = gridmap_get_value_index (gmap, j, i);
			if(val){

				int ret = gridmap_get_meters (gmap, j, i, &x, &y);
				if(ret){
					fprintf(stderr,"gridmap_prune(): Problem getting meters\n");
				}

				gridmap_set_value (outmap, x, y, val);
			}

		}
	}


	gridmap_publish_encoded(lcm, "PRUNE_GRID", outmap);

	return outmap;


}

// copies data from src into dest
// dest and src MUST have the same meters_per_pixel, MUST be grid-aligned, and the
// bounding box of src must be contained in the bounding box of src
// returns 0 on success, 1 otherwise
int
gridmap_cpy_data (grid_map_t *dest, const grid_map_t *src)
{
	// check same meters_per_pixel
	if (fabs(dest->meters_per_pixel - src->meters_per_pixel) > 1e-6) {
		return 1;
	}

	// check that src contained in dest
	if (src->x0 < dest->x0 || src->y0 < dest->y0 ||
			(src->x0+src->meters_per_pixel*src->width) > (dest->x0+dest->meters_per_pixel*dest->width) ||
			(src->y0+src->meters_per_pixel*src->height) > (dest->y0+dest->meters_per_pixel*dest->height)) {
		return 1;
	}

	// check grid-aligned (dest->x0 and dest->y0 integer number of pixels from src->x0
	// and src->y0, respectively)
	double delta_x0 = src->x0 - dest->x0;
	double delta_y0 = src->y0 - dest->y0;
	double smart_fmod_x = delta_x0 - ((int)(delta_x0 / dest->meters_per_pixel)) * dest->meters_per_pixel;
	double smart_fmod_y = delta_y0 - ((int)(delta_y0 / dest->meters_per_pixel)) * dest->meters_per_pixel;
	if (smart_fmod_x > 1e-6 || smart_fmod_y > 1e-6) {
		return 1;
	}

	// all good now -- establish offset between dest and src
	int delta_ix = round(delta_x0 / dest->meters_per_pixel); // round to avoid floating-point weirdness
	int delta_iy = round(delta_y0 / dest->meters_per_pixel); // round to avoid floating-point weirdness

	// actually copy over data -- a row at a time
	for (int src_iy = 0; src_iy < src->height; ++src_iy) {
		int dest_iy = src_iy + delta_iy;
		memcpy (&dest->data[dest_iy*dest->width+delta_ix], &src->data[src_iy*src->width + 0], 1*src->width);
	}

	// alternative copy -- element-wise
	//for (int src_iy = 0; src_iy < src->height; ++src_iy) {
	//    int dest_iy = src_iy + delta_iy;
	//    for (int src_ix = 0; src_ix < src->width; ++src_ix) {
	//        int dest_ix = src_ix + delta_ix;
	//        dest->data[dest_iy*dest->width + dest_ix] = src->data[src_iy*src->width + src_ix];
	//    }
	//}

	return 0;
}

grid_map_t *gridmap_transform(const grid_map_t *gm, double xyh[3]){

	grid_map_t *trans = grid_map_t_copy_attributes(gm);

	double origin[2] = {gm->x0, gm->y0};
	double top_left[2] = {gm->x0, gm->y0 + gm->height * gm->meters_per_pixel};
	double top_right[2] = {gm->x0 + gm->width * gm->meters_per_pixel, gm->y0 + gm->height * gm->meters_per_pixel};
	double bottom_right[2] = {gm->x0 + gm->width * gm->meters_per_pixel, gm->y0};

	double new_origin[2];
	double new_tl[2];
	double new_tr[2];
	double new_br[2];

	transform_point (xyh, origin, new_origin);
	transform_point (xyh, top_left, new_tl);
	transform_point (xyh, top_right, new_tr);
	transform_point (xyh, bottom_right, new_br);

	trans->x0 = fmin(fmin(new_origin[0], new_tl[0]), fmin(new_tr[0], new_br[0]));
	trans->y0 = fmin(fmin(new_origin[1], new_tl[1]), fmin(new_tr[1], new_br[1]));


	trans->width  = (fmax(fmax(new_origin[0], new_tl[0]), fmax(new_tr[0], new_br[0])) - trans->x0) / gm->meters_per_pixel;
	trans->height = (fmax(fmax(new_origin[1], new_tl[1]), fmax(new_tr[1], new_br[1])) - trans->y0) / gm->meters_per_pixel;



	trans->datalen = trans->width * trans->height;
	trans->data = calloc(trans->datalen, sizeof(uint8_t));


	double x = 0;
	double y = 0;
	double *new_pt = malloc(sizeof(double) * 2);
	for (int iy = 0; iy < gm->height; iy++){
		for(int ix = 0; ix < gm->width; ix++){

			int8_t ret = gridmap_get_meters (gm, ix, iy, &x, &y);
			if(ret){
				continue;
			}
			uint8_t val = gridmap_get_value_index (gm, ix, iy);

			//defaults to 0
			if(!val){
				continue;
			}
			double pt[2] = {x, y};
			transform_point(xyh, pt, new_pt);

			gridmap_set_value(trans, new_pt[0], new_pt[1], val);
		}
	}

	free(new_pt);


	return trans;
}


void gridmap_publish_encoded (lcm_t *lcm, const char *channel, const grid_map_t *gm){

	if(!lcm || ! gm){
		return;
	}

	grid_map_t *encoded = grid_map_t_encode_gzip(gm);
	if(!encoded){
		fprintf(stderr,"gridmap_publish_encoded(): Encoding failed\n");
		return;
	}
	grid_map_t_publish (lcm, channel, encoded); // blacklist-ignore
	grid_map_t_destroy(encoded);

	return;

}

grid_map_t *gridmap_decode_and_copy (const grid_map_t *orig)
{
	if (orig->encoding == GRID_MAP_T_ENCODING_GZIP) return grid_map_t_decode_gzip(orig);

	return grid_map_t_copy (orig);
}


void gridmap_decode_in_place (grid_map_t *orig)
{
	if (orig->encoding == GRID_MAP_T_ENCODING_NONE)
        return;

    if (orig->encoding == GRID_MAP_T_ENCODING_GZIP) {
        grid_map_t * gm  = grid_map_t_decode_gzip(orig);

        orig->encoding = gm->encoding;
        orig->datalen = gm->datalen;
        orig->data = realloc(orig->data, orig->datalen);
        memcpy(orig->data, gm->data, orig->datalen);
        grid_map_t_destroy(gm);
    } else {
        assert(0); // XXX need more cases?
    }
}

void gridmap_encode_in_place (grid_map_t *orig)
{
	if (orig->encoding == GRID_MAP_T_ENCODING_GZIP)
        return;

    if (orig->encoding == GRID_MAP_T_ENCODING_NONE) {
        grid_map_t * gm  = grid_map_t_encode_gzip(orig);

        orig->encoding = gm->encoding;
        orig->datalen = gm->datalen;
        orig->data = realloc(orig->data, orig->datalen);
        memcpy(orig->data, gm->data, orig->datalen);
        grid_map_t_destroy(gm);
    } else {
        assert(0); // XXX need more cases?
    }
}


grid_map_t *gridmap_submap (const grid_map_t *gm, double origin[2], double xy[2], uint8_t default_fill)
{


	grid_map_t *out = gridmap_make_pixels (origin[0] - (xy[0] / 2) * gm->meters_per_pixel,
			origin[1] - (xy[1] / 2) * gm->meters_per_pixel,
			xy[0], xy[1],
			gm->meters_per_pixel, default_fill, 0);

	double x = 0;
	double y = 0;
	for (int ix = 0; ix < out->width; ix++) {
		for (int iy = 0; iy < out->height; iy++) {

			gridmap_get_pixel_center (out, ix, iy, &x, &y);

			uint8_t val = gridmap_get_value_safe (gm, x, y, 0);
			gridmap_set_value_index (out, ix, iy, val);



		}
	}

	return out;

}

grid_map_t *gridmap_max_diff (const grid_map_t *gm)
{

	grid_map_t *out = grid_map_t_copy_attributes (gm);
	out->data = calloc (out->height * out->width, sizeof(uint8_t));

	for (int ix = 0; ix < gm->width; ix++) {
		for (int iy = 0; iy < gm->height; iy++) {

			uint8_t max_diff = 0;


			uint8_t val  = gridmap_get_value_index_safe (gm, ix, iy, 0);

			uint8_t val2 = gridmap_get_value_index_safe (gm, ix + 1, iy, 0);
			if (val2) {
				uint8_t diff = abs (val - val2);
				max_diff = diff > max_diff ? diff : max_diff;
			}

			val2 = gridmap_get_value_index_safe (gm, ix - 1, iy, 0);
			if (val2) {
				uint8_t diff = abs (val - val2);
				max_diff = diff > max_diff ? diff : max_diff;
			}
			val2 = gridmap_get_value_index_safe (gm, ix, iy + 1, 0);
			if (val2) {
				uint8_t diff = abs (val - val2);
				max_diff = diff > max_diff ? diff : max_diff;
			}
			val2 = gridmap_get_value_index_safe (gm, ix, iy - 1, 0);
			if (val2) {
				uint8_t diff = abs (val - val2);
				max_diff = diff > max_diff ? diff : max_diff;
			}

			gridmap_set_value_index (out, ix, iy, max_diff);
		}
	}


	return out;

}

grid_map_t *grid_map_from_pnm(const char *filename, double x0, double y0, double meters_per_pixel)
{
	char *full_path = realpath(filename, NULL);
	if(!full_path) {
		printf("WRN: Could not resolve filename\n");
		return NULL;
	}
	image_u8_t *img = image_u8_create_from_pnm(full_path);
	free(full_path);

	grid_map_t *gm = calloc(1, sizeof(grid_map_t));
	gm->utime = utime_now();
	gm->encoding = GRID_MAP_T_ENCODING_NONE;
	gm->x0 = x0;
	gm->y0 = y0;
	gm->meters_per_pixel = meters_per_pixel;
	gm->stride = img->stride;
	gm->width = img->stride;
	gm->height = img->height;
	gm->datalen = img->height * img->stride;
	gm->data = calloc(gm->datalen, sizeof(uint8_t));

        assert(0 && "Compiler warning on next line of code with -O2; overflows destination\n");
	//memcpy(GM_TO_IMU8(gm), img, sizeof(image_u8_t));
	free(img);
	return gm;
}


int
gridmap_zcache_save (const void *tile, const char *filename, const void *user)
{
	return gridmap_write_to_file ((grid_map_t *) tile, filename);
}

void *
gridmap_zcache_load (const char *filename, const void *user)
{
	return gridmap_read_from_file (filename);
}

void *
gridmap_zcache_copy (const void *tile)
{
	return grid_map_t_copy ((grid_map_t*) tile);
}

void
gridmap_zcache_destroy (void *tile)
{
	grid_map_t_destroy ((grid_map_t *) tile);
}
