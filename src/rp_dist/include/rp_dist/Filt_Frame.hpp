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

class rp_frame
{
private:
    dt_angle angle;
    float thres_gap;
    float closest;
    vector<float> distances;
    vector<dt_point> jmp_pts;
    vector<dt_point> options;
    // find jump points, distances --> jmp_pts
    void get_jmp_pts(void);
    // find possible poles, jmp_pts --> options
    void possible_poles(void);
    // filt possible poles, options --> options
    void remove_outline(void);
    vector<dt_point> sel_best_pair(void);
public:
    rp_frame(vector<float> ranges, float anglemin, float angleinc);
    ~rp_frame();
    float jump_thres;// unit: m
    float pole_width;// unit: cm
    dt_point best_pole;
    float get_pole(void);
};
