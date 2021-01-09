#pragma once
#include <vector>
#include <iostream>
#include <algorithm>
#define pi 3.1416

using namespace std;

struct dt_point
{
    int type;
    float distance;
    float angle;
};

struct dt_angle
{
    float min;
    float max;
    float inc;
};

// filt to get the best possible pole
// unless otherwise noted, using:
// unit: m for all distances, unit: rad for all angles
class rp_frame
{
private:
    dt_angle angle;
    float closest;
    vector<float> distances;
    vector<dt_point> jmp_pts;
    vector<dt_point> options;
    dt_point history;
    // find jump points, distances --> jmp_pts
    void get_jmp_pts(void);
    // find possible poles, jmp_pts --> options
    void possible_poles(void);
    // filt possible poles, options --> options
    void remove_outline(void);
    // final pair, options --> pole_pair(temp)
    vector<dt_point> sel_best_pair(void);
    // give type to best_pole, check consistency
    int isConsistent(void);
public:
    rp_frame();
    ~rp_frame();
    void get_distance(vector<float> ranges);
    void set_angle(float anglemin, float anglemax, float angleinc);
    // these parameters can be changed outside
    float jump_thres;
    float pole_width;// specially using unit: cm
    float min_gap;
    float consistent_dist;
    float consistent_angle;
    // best_pole can be directly read outside
    // using for old version, where only distance is returned
    // better move this into private, no effection
    dt_point best_pole;
    // give result, pole_pair(temp) --> best_pole & distance
    dt_point get_pole(void);
};
