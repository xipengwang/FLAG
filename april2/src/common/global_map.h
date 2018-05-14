#ifndef GLOBAL_MAP
#define GLOBAL_MAP

#include <unistd.h>
#include <inttypes.h>
#include "lcmtypes/grid_map_t.h"

typedef struct global_map global_map_t;

global_map_t * global_map_create(double meters_per_pixel);

void global_map_destroy(global_map_t * glm);

//Add a gm to the global map
//GLM does NOT take ownership of the gm, but does use it's pointer value
//adding a gm that is currently in the map is an assert
//xyt is a double [3] describing where this map goes.
void global_map_add_gridmap(global_map_t * glm, const grid_map_t * gm, double * xyt);

//Move a gm already in the GLM
//moving a gm not in the map is an assert
void global_map_move_gridmap(global_map_t * glm, const grid_map_t * gm, double * xyt);

//Remove a gm from the GLM
//Removing a gm not in the map is an assert
void global_map_remove_gridmap(global_map_t * glm, const grid_map_t * gm);

//Sets xyt to the position last used for GM
//Referencing a GM not in the GLM is an assert
void global_map_get_xyt(global_map_t * glm, const grid_map_t * gm, double *xyt);

//Get a reference to the GLM's MLE
//estimate of the world
//Invalidated after all other operations
const grid_map_t * global_map_get_map(global_map_t * glm);

//Get a copy of the GLM's MLE
grid_map_t * global_map_get_map_copy(global_map_t * glm);

//Trigger a crop operation, useful
//when the global solution shifts massively.
void global_map_crop(global_map_t * glm);

#endif //GLOBAL_MAP
