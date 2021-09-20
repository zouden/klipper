// Five Bar linkage kinematics
// 
// Pontus Borg <glpontus@gmail.com>
// Eirinn Mackay <eirinn.mackay@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

struct fivebar_stepper {
    struct stepper_kinematics sk;
    double inner_arm_length;
    double outer_arm_length;
    double mount_x_pos;
    char arm;
};

// static inline double sqr(double a) { return a*a; };

// static inline double distance(double x0, double y0,
//                               double x1, double y1)
// {
//     double dx = x1-x0;
//     double dy = y1-y0;
//     return sqrt(sqr(dx) + sqr(dy));
// }



// // Find the angle of the corner opposite to side c
// static inline double triangle_angle(double a, double b, double c)
// {
//     // cosine rule: c^2 = a^2 + b^2 − 2ab cos(C)
//     // C = arccos((a^2 + b^2 - c^2) / 2ab)
//     double cosC = (sqr(a) + sqr(b) - sqr(c)) / (2 * a * b);
//     return acos(cosC);
// }

//circle-intersection code 
//https://stackoverflow.com/questions/29596319/find-the-intersection-points-of-two-circles
static int intersection(double result[], 
                               double x0, double y0, double r0,
                               double x1, double y1, double r1)
{
    //puts the results (two intersection points in the passed array).
    //if circles don't intersect, returns false.
    double a, dx, dy, d, h, rx, ry;
    double x2, y2;
    /* dx and dy are the vertical and horizontal distances between
     * the circle centers.
     */
    dx = x1 - x0;
    dy = y1 - y0;
    /* Determine the straight-line distance between the centers. */
    //d = sqrt((dy*dy) + (dx*dx));
    d = hypot(dx,dy); // Suggested by Keith Briggs

    /* Check for solvability. */
    if (d > (r0 + r1))
    {
        /* no solution. circles do not intersect. */
        return 0;
    }
    if (d < fabs(r0 - r1))
    {
        /* no solution. one circle is contained in the other */
        return 0;
    }
    /* 'point 2' is the point where the line through the circle
     * intersection points crosses the line between the circle
     * centers.  
     */
    /* Determine the distance from point 0 to point 2. */
    a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0 * d) ;

    /* Determine the coordinates of point 2. */
    x2 = x0 + (dx * a/d);
    y2 = y0 + (dy * a/d);

    /* Determine the distance from point 2 to either of the
     * intersection points.
     */
    h = sqrt((r0*r0) - (a*a));

    /* Now determine the offsets of the intersection points from
     * point 2.
     */
    rx = -dy * (h/d);
    ry = dx * (h/d);

    /* Determine the absolute intersection points. */
    result[0] = x2 + rx; //first x
    result[1] = y2 + ry; //first y
    result[2] = x2 - rx; //second x
    result[3] = y2 - ry; //second y
    return 1;
}


// Inverse kinematics
static double
fivebar_stepper_calc_position(
    struct stepper_kinematics *sk,
    struct move *m,
    double move_time)
{
    struct fivebar_stepper *fs =
        container_of(sk, struct fivebar_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    //find the coords of the elbow
    double candidates[4];
    double elbow_x, elbow_y;
    intersection(candidates, fs->mount_x_pos, 0, fs->inner_arm_length,
                             c.x, c.y, fs->outer_arm_length);
    if(fs->arm == 'l'){
        if(candidates[0] < candidates[2]){ //first candidate is left-most
            elbow_x = candidates[0]; elbow_y = candidates[1];
        } else {
            elbow_x = candidates[2]; elbow_y = candidates[3];
        }
    } else {
        if(candidates[0] > candidates[2]){ //first candidate is right-most
            elbow_x = candidates[0]; elbow_y = candidates[1];
        } else {
            elbow_x = candidates[2]; elbow_y = candidates[3];
        }

    }   
    //find the angle to the elbow
    double angle = atan2(elbow_y - 0, elbow_x - fs->mount_x_pos);
    //if the angle is negative, we need to bring it back into range (left side only)
    if(angle<0 && fs->arm=='l'){
        angle+=M_PI*2;
    }                     
    return angle;
}


struct stepper_kinematics * __visible
fivebar_stepper_alloc(char arm,
                           double inner_arm_length,
                           double outer_arm_length,
                           double inner_arms_distance){

    struct fivebar_stepper *fs = malloc(sizeof(*fs));
    memset(fs, 0, sizeof(*fs));

    fs->sk.calc_position_cb = fivebar_stepper_calc_position;
    fs->inner_arm_length = inner_arm_length;
    fs->outer_arm_length = outer_arm_length;
    fs->arm = arm;
    //the left arm is mounted at X=0. The right arm is offset by the inner distance.
    //the Y position for both steppers is 0.
    if (arm == 'l') {
        fs->mount_x_pos = 0;
    } else if (arm == 'r') {
        fs->mount_x_pos = inner_arms_distance;
    }
    fs->sk.active_flags = AF_X | AF_Y;
    return &fs->sk;
}
