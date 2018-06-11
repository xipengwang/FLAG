#include <stdio.h>
#include <unistd.h>
#include <float.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <assert.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>

#include "common/doubles.h"
#include "common/floats.h"
#include "common/getopt.h"
#include "common/config.h"
#include "common/rand_util.h"
#include "common/time_util.h"
#include "common/string_util.h"
#include "common/zarray.h"
#include "common/geo_image.h"
#include "common/gps_linearization.h"
#include "common/stype_lcm.h"
#include "common/global_map.h"
#include "common/gridmap.h"
#include "common/gridmap_util.h"



#include "vx/vx.h"
#include "vx/webvx.h"
#include "vx/vxo_generic.h"

#include "lcm/lcm.h"
#include "lcmtypes/pose_t.h"
#include "lcmtypes/gps_t.h"
#include "lcmtypes/global_world_state_t.h"
#include "lcmtypes/global_robot_state_t.h"


/**
 * Running program:
   ** ./FLAG-map-generator -m ../config/mission-gen.config -f ../resc/bbb_floorplan.pnm -o /tmp/features.csv
      ./FLAG-map-generator -f ../resc/bbb_floorplan.pnm -m ../coig/mission-gen.config --world ../resc/global_map.composite
     *** -f: satellite image path
     *** -o: output file storing landmark locations

   ** mission-gen.config:
     *** geo.site (mission location)
 */

#define FIND_FEATURE_THRESH 1.0 //meter

typedef struct state state_t;
struct state{
    lcm_t *lcm;
    vx_world_t *vw;
    webvx_t *webvx;

    const char *input_file_path;
    const char *output_file_path;

    zarray_t *features;
    int selected;

    gps_lin_t *gps_lin;
    geo_image_t *geo_img;

    config_t *mission_config;
    vx_object_t *world_img;
    grid_map_t *gm;

    double lidar_to_img[3];
};

static void signal_handler(int signum)
{
    switch (signum) {
        case SIGINT:
            exit(1);    // XXX Cleanup later, if necessary
            break;
        default:
            break;
    }
}

void write_features(state_t *state)
{
    FILE *f = fopen(state->output_file_path, "w");

    float xy[2];

    int nfeatures = zarray_size(state->features);
    for (int i = 0; i < nfeatures; i++) {
        zarray_get(state->features, i, xy);

        fprintf(f, "%f,%f\n", xy[0], xy[1]);
    }

    printf("wrote %d features to file %s\n", nfeatures, state->output_file_path);

    fclose(f);
}

static void redraw_satellite(state_t *state)
{
    if (state->geo_img == NULL) {
        printf("geo_img empty\n");
        return;
    }

    const matd_t *tr = geo_image_get_matrix(state->geo_img);

    vx_buffer_t *vb = vx_world_get_buffer(state->vw, "satellite");

    vx_object_t *ob = vxo_image_saturated(vx_resource_make_texture_u8x3_copy(geo_image_get_image(state->geo_img),
                                                                             VX_TEXTURE_MAX_LINEAR),
                                          0.5);

    vx_buffer_add_back(vb,
                       vxo_depth_test(0,
                                      vxo_matrix_scale(0.96),
                                      vxo_chain(vxo_matrix_translate(0, 0, 0),
                                                vxo_matrix(tr->data),
                                                ob,
                                                NULL),
                                      NULL),

                       NULL);
    vx_buffer_swap(vb);
}

static void redraw_global_world(state_t *state)
{
    if (state->world_img == NULL) {
        return;
    }
    vx_buffer_t *vb = vx_world_get_buffer(state->vw, "map");
    vx_buffer_add_back(vb,
                       vxo_depth_test(0,
                                      vxo_matrix_xyt(state->lidar_to_img),
                                      vxo_chain(vxo_matrix_translate(state->gm->x0, state->gm->y0, 0),
                                                vxo_matrix_scale(state->gm->meters_per_pixel),
                                                state->world_img,
                                                NULL),
                                      NULL),
                       NULL);

    vx_buffer_swap(vb);

}

static void render_features(state_t *state)
{
    vx_buffer_t* vb = vx_world_get_buffer(state->vw, "features");
    float xy[2];
    for (int i = 0; i < zarray_size(state->features); i++) {
        zarray_get(state->features, i, xy);
        float *color = vx_white;
        if (state->selected == i)
            color = vx_red;

        vx_buffer_add_back(vb,
                           vxo_depth_test(0,
                                          vxo_matrix_translate(xy[0], xy[1], 0),
                                          vxo_matrix_scale(0.5),
                                          vxo_square_solid(color),
                                          NULL),
                           NULL);
    }
    vx_buffer_swap(vb);
}

int find_closest_feature(state_t *state, double xyz[3])
{
    int32_t closest_id = -1;
    double closest_dist = FIND_FEATURE_THRESH;

    float xy[2];
    for (int i = 0; i < zarray_size(state->features); i++) {
        zarray_get(state->features, i, xy);
        double dist = sqrt(sq(xyz[0] - xy[0]) +
                           sq(xyz[1] - xy[1]));

        if (dist < closest_dist) {
            closest_dist = dist;
            closest_id = i;
        }
    }

    return closest_id;
}

static int mouse_down(vx_layer_t *vl, const vx_event_t *ev, void *impl)
{
    state_t *state = impl;

    bool m1 = ev->u.mouse_button.button == 0;
    bool m2 = ev->u.mouse_button.button == 2;
    bool shift = ev->flags & VX_EVENT_FLAGS_SHIFT;

    double r0[3], r1[3];
    vx_util_mouse_event_compute_ray(ev, r0, r1);
    double xyz[3];
    vx_util_ray_intersect_plane(r0, r1, (double[]) { 0, 0, 1, 0 }, xyz);

    printf("MOUSE CLICKED @ (%f,%f)\n", xyz[0], xyz[1]);


    if (m1 && shift){ //API KEY: SHIFT + LEFT CLICK add landmark
            printf("Adding feature @ (%f,%f)\n", xyz[0], xyz[1]);
            float xy[2];
            xy[0] = xyz[0];
            xy[1] = xyz[1];
            zarray_add(state->features, xy);
            render_features(state);
    } else if(m1) { //API KEY: LEFT CLICK select landmark near the cursor.
        state->selected = find_closest_feature(state, xyz);
        render_features(state);
    } else if(m2 && shift){

    } else if(m2) {

    }

    return 0;
}

void on_key_press(vx_layer_t *vl, const vx_event_t *ev, void *user)
{
    state_t *state = user;

    //API KEY: press s to save landmarks into a file;
    if (ev->u.key.key_code == 's') {
        write_features(state);
    }
    //API KEY: LEFT CLICK selecting landmark, press x to delete the selected landmark;
    else if (ev->u.key.key_code == 'x') {
        if (state->selected >= 0) {
            zarray_remove_index(state->features, state->selected, 0);
            state->selected = -1;
            render_features(state);
        }
    } else if(ev->u.key.key_code == 'j') {
        state->lidar_to_img[0] += 0.1;
        redraw_global_world(state);
    } else if(ev->u.key.key_code == 'l') {
        state->lidar_to_img[0] -= 0.1;
        redraw_global_world(state);
    } else if(ev->u.key.key_code == 'i') {
        state->lidar_to_img[1] += 0.1;
        redraw_global_world(state);
    } else if(ev->u.key.key_code == 'k') {
        state->lidar_to_img[1] -= 0.1;
        redraw_global_world(state);
    } else if(ev->u.key.key_code == 'o') {
        state->lidar_to_img[2] += to_radians(1);
        redraw_global_world(state);
    } else if(ev->u.key.key_code == 'p') {
        state->lidar_to_img[2] -= to_radians(1);
        redraw_global_world(state);
    }
    printf("%f, %f, %f \n", state->lidar_to_img[0], state->lidar_to_img[1], state->lidar_to_img[2]);
}

static int on_event(vx_layer_t *vl, const vx_event_t *ev, void *impl)
{
    state_t *state = impl;

    switch (ev->type) {
        case VX_EVENT_MOUSE_DOWN:
            return mouse_down(vl, ev, state);
        case VX_EVENT_KEY_PRESSED:
            on_key_press(vl, ev, state);
            break;
    default:
            break;
    }
    return 0;
}

void on_create_canvas(vx_canvas_t *vc, const char *name, void *impl)
{
    state_t *state = impl;
    vx_layer_t *vl = vx_canvas_get_layer(vc, "default");
    vx_layer_set_world(vl, state->vw);
    vx_layer_add_event_handler(vl, on_event, 1, state);

    int order = 0;
    vx_buffer_set_draw_order(vx_world_get_buffer(state->vw, "satellite"), order++);
    vx_buffer_set_draw_order(vx_world_get_buffer(state->vw, "map"), order++);
    vx_buffer_set_draw_order(vx_world_get_buffer(state->vw, "features"), order++);

}

void on_destroy_canvas(vx_canvas_t *vc, void *impl)
{
    fprintf(stderr,"ON DESTROY CANVAS\n");
}

vx_object_t* make_gws_from_file(state_t *state, const char *path)
{
    global_world_state_t *gws = NULL;
    vx_object_t *vxo = NULL;

    FILE *fp = fopen(path, "r");
    fseek(fp, 0L, SEEK_END);
    long len = ftell(fp);
    fseek(fp, 0L, SEEK_SET);

    uint8_t *buf = malloc(len);
    size_t sz = fread(buf, 1, len, fp);

    if (sz != len)
        goto cleanup;

    uint32_t pos = 0;
    gws = stype_decode_object(buf, &pos, len, NULL);
    printf("NFO: Loaded global_world_state_t from %s - %ld bytes\n", path, len);


    grid_map_t *gm = NULL;
    switch (gws->global_map.encoding) {
        case GRID_MAP_T_ENCODING_NONE:
            gm = gridmap_copy(&gws->global_map);
            break;
        case GRID_MAP_T_ENCODING_GZIP:
            gm = grid_map_t_decode_gzip(&gws->global_map);
            break;
        default:
            printf("ERR: encoding type %d not supported\n", gws->global_map.encoding);
            exit(1);
    }


    image_u8_t *im = image_u8_create(gm->width, gm->height);
    for (int y = 0; y < gm->height; y++)
        memcpy(&im->buf[y*im->stride], &gm->data[y*gm->width], gm->width);

    vx_resource_t *tex = vx_resource_make_texture_u8_copy(im, 0);
    image_u8_destroy(im);

    vxo = vxo_image_tile(tex,
                         vx_nada,
                         vx_gray,
                         vx_maroon,
                         vx_orange);
    state->gm = gm;
    global_world_state_t_destroy(gws);

  cleanup:
    free(buf);
    fclose(fp);
    return vxo;
}

void read_features(state_t *state)
{
    FILE *f = fopen(state->input_file_path, "r");

    float x;
    float y;

    while (fscanf(f, "%f,%f", &x, &y) != EOF) {
        float *xy = (float*) calloc(2, sizeof(float));
        xy[0] = x;
        xy[1] = y;
        zarray_add(state->features, xy);
    }

    fclose(f);
}

void setup_geo(state_t *state, getopt_t *gopt)
{
    state->gps_lin = gps_lin_create();
    double latlon_deg[2] = {0, 0};
    const char *sitename = NULL;
    if (state->mission_config) {
        sitename = config_get_string(state->mission_config, "geo.site", NULL);
    } else {
        printf("No config provided. Operating without site\n");
    }

    const char *image = NULL;
    if (sitename) {
        char *sitepath = sprintf_alloc("geo.%s", sitename);
        char *linpath = sprintf_alloc("%s.linpt", sitepath);
        char *imgpath = sprintf_alloc("%s.image", sitepath);

        printf("loading site %s\n", sitename);
        if (getopt_was_specified(gopt, "image")) {
            image = getopt_get_string(gopt, "image");
        }

        if (config_get_doubles(state->mission_config, linpath, NULL)) {
            config_require_doubles_len(state->mission_config, linpath, latlon_deg, 2);
            printf("%s latlon [%3.6f, %3.6f]\n", sitename, latlon_deg[0], latlon_deg[1]);
        }

        free(sitepath);
        free(linpath);
        free(imgpath);
    }

    gps_lin_setup(state->gps_lin, latlon_deg, (double[]){0,0,0}, utime_now());
    if (image) {
        state->geo_img = geo_image_create(image, state->gps_lin);
        geo_image_update_lin(state->geo_img, state->gps_lin);
    }
}

int main(int argc, char *argv[])
{
    setlinebuf(stderr);
    setlinebuf(stdout);

    stype_register_basic_types();
    stype_register(STYPE_FROM_LCM("grid_map_t", grid_map_t));
    stype_register(STYPE_FROM_LCM("global_world_state_t", global_world_state_t));
    stype_register(STYPE_FROM_LCM("global_robot_state_t", global_robot_state_t));

    getopt_t *gopt = getopt_create();
    getopt_add_int(gopt, 'p', "port", "8890", "jsvx server port");
    getopt_add_string(gopt, 'i', "in-file", "", "input feature file");
    getopt_add_string(gopt, 'o', "out-file", "/var/tmp/flag_landmarks.csv", "output feature file");
    getopt_add_string(gopt, 'f', "image", "", "satellite image file path");
    getopt_add_string(gopt, '\0', "world", "", "global world image file path");
    getopt_add_string(gopt, 'm', "mission-config", "", "Config file for mission");

    if (!getopt_parse(gopt, argc, argv, 1)) {
        getopt_do_usage(gopt);
        return 1;
    }
    signal(SIGINT, signal_handler);
    state_t *state = (state_t*) calloc(1, sizeof(state_t));
    //HARD CODE for evaluation against lidar
    state->lidar_to_img[0] =  -13.9;
    state->lidar_to_img[1] =  -9;
    state->lidar_to_img[2] =  M_PI;

    state->mission_config = config_create_path(getopt_get_string(gopt, "mission-config"));
    if (!state->mission_config) {
        printf("ERR: bad mission config %s\n", getopt_get_string(gopt, "mission-config"));
        return -1;
    }
    state->lcm = lcm_create(NULL);

    // vx satellite image
    setup_geo(state, gopt);
    // vx canvas initialization
    state->vw = vx_world_create();
    state->webvx = webvx_create_server(getopt_get_int(gopt, "port"),
                                       NULL,
                                       "index.html");

    //each element contains (x,y) position of the landmark in global frame.
    state->features = zarray_create(2*sizeof(float));
    webvx_define_canvas(state->webvx,
                        "flag-map-generator-Canvas",
                        on_create_canvas,
                        on_destroy_canvas,
                        state);
    redraw_satellite(state);

    state->input_file_path = getopt_get_string(gopt, "in-file");
    state->selected = -1;
    if (strlen(state->input_file_path)) {
        read_features(state);
        render_features(state);
    }

    if (strlen(getopt_get_string(gopt, "world"))) {
        printf("Load world file from: %s\n", getopt_get_string(gopt, "world"));
        state->world_img = make_gws_from_file(state, getopt_get_string(gopt, "world"));
        state->world_img->decref(state->world_img);
        redraw_global_world(state);
    }


    state->output_file_path = sprintf_alloc("%s_%ld", getopt_get_string(gopt, "out-file"), utime_now());
    while (1) {
        lcm_handle(state->lcm);
    }

    lcm_destroy(state->lcm);
}
