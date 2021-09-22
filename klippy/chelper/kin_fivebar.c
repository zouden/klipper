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
#include "pyhelper.h"

struct fivebar_stepper {
    struct stepper_kinematics sk;
    double inner_arm_length;
    double outer_arm_length;
    double inner_arms_distance;
    char arm;
    int toolhead_is_offset;
    double toolhead_to_elbow_length;
    double toolhead_offset_angle;
    char toolhead_attached_to;
};

struct Point{
    double x;
    double y;
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

struct Point _calculate_line(double x0, double y0, double theta, double length){
    struct Point outpoint;
    outpoint.x = x0 + cos(theta) * length;
    outpoint.y = y0 + sin(theta) * length;
    return outpoint;
    }


// Helper function for IK
static struct Point _find_elbow(char arm, double startX, double endX, double endY, 
                        double armlen1, double armlen2)
{
    // Helper function for inverse kinematics. Finds the elbow and the bearing of the line between it and the end XY
    double startY = 0;
    // intersect two circles
    double candidates[4] = {0,0,0,0};
    struct Point elbow;
    int ok = intersection(candidates, startX, startY, armlen1,
                             endX, endY, armlen2);
    if(!ok){
        errorf("IK fail: no elbows found: start(%.2f %.2f) len(%.2f) end(%.2f %.2f) len2(%.2f)", 
                             startX, startY, armlen1,
                             endX, endY, armlen2);
    }
    if(arm == 'l'){
        if(candidates[0] < candidates[2]){ //first candidate is left-most
            elbow.x = candidates[0]; elbow.y = candidates[1];
        } else {
            elbow.x = candidates[2]; elbow.y = candidates[3];
        }
    } else {
        if(candidates[0] > candidates[2]){ //first candidate is right-most
            elbow.x = candidates[0]; elbow.y = candidates[1];
        } else {
            elbow.x = candidates[2]; elbow.y = candidates[3];
        }
    }
    return elbow;
}

// Inverse kinematics
static double fivebar_stepper_calc_position(
                                struct stepper_kinematics *sk,
                                struct move *m,
                                double move_time) 
{
    struct fivebar_stepper *fs =
        container_of(sk, struct fivebar_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    //find the coords of the elbow
    // double left_elbow_x = 0, left_elbow_y = 0;
    // double right_elbow_x = 0, right_elbow_y = 0;
    struct Point endpoint;
    struct Point left_elbow = {0,0};
    struct Point right_elbow = {0,0};

    if(fs->toolhead_attached_to != 'l' && fs->toolhead_attached_to != 'r'){
        errorf("Fivebar kinematics misconfigured");
    }

    if(fs->toolhead_attached_to == 'l') { 
        //toolhead on left arm. Find it first
        left_elbow = _find_elbow('l', 0, c.x, c.y,
                    fs->inner_arm_length, fs->toolhead_to_elbow_length);
        // is there an offset? Then we need to find the endpoint
        if(fs->toolhead_is_offset){
            //find the bearing of this upper arm
            double bearing = atan2(c.y-left_elbow.y, c.x-left_elbow.x);
            //it's offset from this arm, so we need to find the endpoint
            //by subtracting the known theta offset
            endpoint = _calculate_line(left_elbow.x, left_elbow.y,
                            bearing - fs->toolhead_offset_angle,
                            fs->outer_arm_length);
            //now we know where the arm ends. Use this as the target for the right stepper.
        } else {
            endpoint.x = c.x;
            endpoint.y = c.y;
        }
        right_elbow = _find_elbow('r', fs->inner_arms_distance, endpoint.x, endpoint.y,
                fs->inner_arm_length, fs->outer_arm_length);
    } else { 
        //toolhead on right arm. Find it first
        right_elbow = _find_elbow('r', fs->inner_arms_distance, c.x, c.y,
                    fs->inner_arm_length, fs->toolhead_to_elbow_length);
        // is there an offset? Then we need to find the endpoint
        if(fs->toolhead_is_offset){
            //find the bearing of this upper arm
            double bearing = atan2(c.y-right_elbow.y, c.x-right_elbow.x);
            //it's offset from this arm, so we need to find the endpoint
            //by subtracting the known theta offset (note, right side offsets should be negative)
            endpoint = _calculate_line(right_elbow.x, right_elbow.y,
                            bearing - fs->toolhead_offset_angle,
                            fs->outer_arm_length);
            //now we know where the arm ends. Use this as the target for the left stepper.
        }else{
            endpoint.x = c.x;
            endpoint.y = c.y;
        }
        _find_elbow('l', fs->inner_arms_distance, endpoint.x, endpoint.y,
                fs->inner_arm_length, fs->outer_arm_length);
    }
    // now we have both elbow positions.
    double angle;
    if(fs->arm=='l'){
        angle = atan2(left_elbow.y - 0, left_elbow.x - 0);
        if(angle<0) angle+=M_PI*2;
    }else{
        angle = atan2(right_elbow.y - 0, right_elbow.x - fs->inner_arms_distance);
    }
    return angle;
}


struct stepper_kinematics * __visible
fivebar_stepper_alloc(char arm,
                        double inner_arm_length,
                        double outer_arm_length,
                        double inner_arms_distance,
                        int toolhead_is_offset,
                        double toolhead_to_elbow_length,
                        double toolhead_offset_angle,
                        char toolhead_attached_to){

    struct fivebar_stepper *fs = malloc(sizeof(*fs));
    memset(fs, 0, sizeof(*fs));

    fs->sk.calc_position_cb = fivebar_stepper_calc_position;
    fs->inner_arm_length = inner_arm_length;
    fs->outer_arm_length = outer_arm_length;
    fs->arm = arm;
    fs->inner_arms_distance = inner_arms_distance;
    fs->toolhead_attached_to = toolhead_attached_to;
    fs->toolhead_is_offset = toolhead_is_offset;
    fs->toolhead_offset_angle = toolhead_offset_angle;
    fs->toolhead_to_elbow_length = toolhead_to_elbow_length;
    fs->sk.active_flags = AF_X | AF_Y;
    return &fs->sk;
}
