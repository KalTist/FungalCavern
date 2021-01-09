#include "Filt_Frame.hpp"

using std::cout;

rp_frame::rp_frame(vector<float> ranges, float anglemin, float angleinc)
{
    int i = 0;
    // cout<<"init rp_frame instance"<<endl;
    angle.min = anglemin;
    angle.inc = angleinc;
    for(i=0; i<ranges.size(); i++){
        distances.push_back(ranges[i]);
    }
    jump_thres = 0.2;
    pole_width = 2.5;
    closest = 10;
}

rp_frame::~rp_frame(void)
{
}

void rp_frame::get_jmp_pts(void)
{
    int i = 0;
    dt_point pt;
    float change;
    jmp_pts.clear();
    // cout<<"get_jmp_pts: ";
    for(i = 0; i < distances.size()-1 ; i++)
    {
        // always save the closer point's distance
        // that is, possible poles rather than background
        change = distances[i+1] - distances[i];
        if(change > jump_thres){
            // jump farther
            pt.type = 1;
            pt.angle = angle.min + i * angle.inc;
            pt.distance = distances[i];
            jmp_pts.push_back(pt);
        }
        else if(change < -jump_thres){
            // jump closer
            pt.type = -1;
            pt.angle = angle.min + (i+1) * angle.inc;
            pt.distance = distances[i+1];
            jmp_pts.push_back(pt);
        }
    }
    // cout<<jmp_pts.size()<<endl;
}

float rp_frame::get_pole(void)
{
    vector<dt_point> pole_pair;
    dt_point pole_center;
    float dist;
    int index;
    // cout<<"get_pole: "<<endl;
    get_jmp_pts();
    possible_poles();
    remove_outline();
    pole_pair = sel_best_pair();
    if(pole_pair.empty()){
        // cout<<"empty"<<endl;
        return 0.0;
    }
    // restore best_pair to one pole
    // using original data instead of jmp_pts
    pole_center.angle = (pole_pair[0].angle + pole_pair[1].angle) / 2;
    index = int((pole_center.angle - angle.min) / angle.inc + 0.2);
    dist = distances[index];
    pole_center.type = 0;
    pole_center.distance = dist;
    // cout<<"result: ";
    return dist;
}

void rp_frame::possible_poles(void)
{
    int i = 0;
    float dist_var, angle_var, width_var;
    options.clear();
    cout<<"options: ";
    closest = 10;
    for(i = 0; i < jmp_pts.size()-1; i++)
    {   // rule_one: far-near-far jump points pair
        if(jmp_pts[i].type == -1 && jmp_pts[i+1].type == 1){
            dist_var = jmp_pts[i].distance - jmp_pts[i+1].distance;
            // rule_two: left and right side of pole have same distance
            if(dist_var > - 0.03 && dist_var < 0.03){
                angle_var = jmp_pts[i+1].angle - jmp_pts[i].angle;
                width_var = angle_var / 1.8 * pi * jmp_pts[i].distance - pole_width;
                // rule_three: pole has a certain width
                if( width_var < jmp_pts[i].distance * angle.inc ){
                    options.push_back(jmp_pts[i]);
                    options.push_back(jmp_pts[i+1]);
                    // backup the distance of the closest possible pole
                    if(jmp_pts[i].distance < closest){
                        closest = jmp_pts[i].distance;
                    }
                    cout<<jmp_pts[i].distance<<",";
                }
            }
        }
    }
    // cout<<"num: "<<options.size()<<endl;
    cout<<endl;
    return;
}
void rp_frame::remove_outline(void)
{
    int i = 0;
    float gap_r, gap_a;
    vector<float> gaps;
    vector<float> sortgaps;
    vector<dt_point> alter;
    if(options.empty()){
        return;// no possible pole
    }
    cout<<"Filted: ";
    gaps.clear();
    // calculate gaps between possible poles
    for(i = 2; i < options.size(); i += 2)
    {
        gap_r = options[i].distance - options[i-1].distance;
        gap_a = gap_r < 0 ? options[i].distance : options[i-1].distance;
        gap_a *= pi / 180 * (options[i].angle - options[i-1].angle);
        gaps.push_back(gap_r*gap_r + gap_a*gap_a);
    }
    if(gaps.empty()){
        return;// only one possible pole
    }
    // select a threshold, Apriori
    sortgaps = gaps;
    sort(sortgaps.begin(), sortgaps.end());
    thres_gap = sortgaps[int(sortgaps.size()/2)];
    thres_gap = thres_gap < 0.2 ? 0.2 : thres_gap / 2 + 0.1;
    cout<<"tsh:"<<thres_gap<<";";
    // real pole has both side gaps > threshold
    // and has to be closer, not farther
    for(i = 0; i < gaps.size(); i++)
    {   // considering head and tail
        if( (i == 0 || gaps[i-1] >= thres_gap) && gaps[i] >= thres_gap){
            if( (i == 0 || options[i*2].distance < options[i*2-1].distance) &&\
                (i == gaps.size() - 1 || options[i*2+1].distance < options[i*2+2].distance)){
                alter.push_back(options[i*2]);
                alter.push_back(options[i*2+1]);
            }
        }
    }
    options = alter;
    cout<<"left "<<options.size()/2<<" pair(s): ";
    for(i = 0; i < options.size(); i++){
        cout<<options[i].distance<<",";
    }
    cout<<endl;
}

vector<dt_point> rp_frame::sel_best_pair(void)
{
    int i = 0;
    vector<dt_point> pole;
    // cout<<"sel_best_pair..."<<endl;
    if(options.empty()){
        // cout<<"empty"<<endl;
        pole.clear();
        return pole;
    }
    // cout<<options.size()<<endl;
    pole.push_back(options[0]);
    pole.push_back(options[1]);
    for(i = 2; i < options.size(); i += 2)
    {
        if(pole[0].distance > options[i].distance){
            pole[0] = options[i];
            pole[1] = options[i+1];
        }
    }
    if(pole[0].distance > closest + 0.3 || pole[1].distance > closest + 0.3){
        pole.clear();
    }
    closest = 10;
    return pole;
}
