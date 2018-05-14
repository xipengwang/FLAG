#ifndef POSE_UTIL_H
#define POSE_UTIL_H

#include <stdio.h>

#include "common/doubles.h"


/** Convenience functions for computing transformations.
  A pose can be seen as a representation of a rigid body transform:
  T = [R(q) p
       0    1]
  Where R(q) is the 3x3 rotation matrix represented by the quaternion
  and p is the 3x1 translation vector.

  In this interpretation, a robot's position and orientation represents
  a transformation from robot frame to global frame
*/

typedef struct pose3 pose3_t;
struct pose3 {
    double pos[3];
    double quat[4];
};

static inline pose3_t pose3_init(const double pos[3], const double quat[4])
{
    pose3_t pose = {
        .pos = {pos[0], pos[1], pos[2]},
        .quat = {quat[0], quat[1], quat[2], quat[3]}
    };
    return pose;
}

static inline pose3_t pose3_inverse(pose3_t p)
{
    pose3_t r;
    doubles_quat_inverse(p.quat, r.quat);
    doubles_quat_rotate(r.quat, p.pos, r.pos);
    doubles_scale(-1, r.pos, 3, r.pos);
    return r;
}

/** r = p1 * p2 */
static inline pose3_t pose3_multiply(pose3_t p1, pose3_t p2)
{
    pose3_t r;
    doubles_quat_multiply(p1.quat, p2.quat, r.quat);
    doubles_quat_rotate(p1.quat, p2.pos, r.pos);
    doubles_add(r.pos, p1.pos, 3, r.pos);
    return r;
}

/** Transforms a point [x y z] into coordinate frame described by p */
static inline void pose3_transform(pose3_t p, const double v[3], double r[3])
{
    doubles_quat_rotate(p.quat, v, r);
    doubles_add(r, p.pos, 3, r);
}

static void pose3_print(pose3_t p)
{
    double rpy[3];
    doubles_quat_to_rpy(p.quat, rpy);
    printf("pos:  %10f %10f %10f\n"
           "quat: %10f %10f %10f %10f\n"
           "rpy:  %10f %10f %10f\n",
            p.pos[0], p.pos[1], p.pos[2],
            p.quat[0], p.quat[1], p.quat[2], p.quat[3],
            rpy[0], rpy[1], rpy[2]);
}

static inline void pose3_to_mat44(pose3_t p, double M[16])
{
    doubles_quat_xyz_to_mat44(p.quat, p.pos, M);
}

static inline pose3_t pose3_from_mat44(const double M[16])
{
    pose3_t p;
    doubles_mat_to_xyz(M, p.pos);
    doubles_mat_to_quat(M, p.quat);
    return p;
}
#endif
