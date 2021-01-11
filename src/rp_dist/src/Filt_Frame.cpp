#include "Filt_Frame.hpp"

using std::cout;

rp_frame::rp_frame(void)
{
    // default, good in overall
    jump_thres = 0.2;
    pole_width = 2.5;
    toler_depth_var = 0.05;
    toler_consis_ang = 4;
    min_gap = 0.15;
    max_gap = 0.6;
    closest = 10;
    consistent_dist = 0.1;
    consistent_angle = 0.09;
    history.type = -1;
}

rp_frame::~rp_frame(void)
{
}

void rp_frame::get_distance(vector<float> ranges)
{
    distances = ranges;
}

void rp_frame::set_angle(float anglemin, float anglemax, float angleinc)
{
    angle.min = anglemin;
    angle.max = anglemax;
    angle.inc = angleinc;
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
            // cout<<fixed<<setprecision(2)<<pt.distance<<";";
        }
        else if(change < -jump_thres){
            // jump closer
            pt.type = -1;
            pt.angle = angle.min + (i+1) * angle.inc;
            pt.distance = distances[i+1];
            jmp_pts.push_back(pt);
            // cout<<fixed<<setprecision(2)<<pt.distance<<",";
        }
    }
    // cout<<endl;
}

dt_point rp_frame::get_pole(void)
{
    vector<dt_point> pole_pair;
    float dist;
    int index;
    // cout<<"get_pole: "<<endl;
    get_jmp_pts();
    possible_poles();
    remove_outline();
    pole_pair = sel_best_pair();
    if(pole_pair.empty()){
        // define angle in middle, specially used for isConsistent
        best_pole.angle = (angle.min + angle.max) / 2;
        best_pole.distance = 0;
    }
    else{
        // restore best_pair to one pole
        // using original data instead of jmp_pts
        best_pole.angle = (pole_pair[0].angle + pole_pair[1].angle) / 2;
        index = int((best_pole.angle - angle.min) / angle.inc + 0.2);
        best_pole.angle = angle.min + angle.inc * index;
        best_pole.distance = distances[index];
    }
    best_pole.type = isConsistent();
    history = best_pole;
    return best_pole;
}

void rp_frame::possible_poles(void)
{
    int i = 0;
    float dist_var, angle_var, width_var;
    options.clear();
    // cout<<"options: ";
    closest = 10;
    // all high tolerance, considering error and rplidar movement
    for(i = 0; i < jmp_pts.size()-1; i++)
    {   // rule_one: far-near-far jump points pair
        if(jmp_pts[i].type == -1 && jmp_pts[i+1].type == 1){
            dist_var = jmp_pts[i].distance - jmp_pts[i+1].distance;
            // rule_two: left and right side of pole have same distance
            if(dist_var > -toler_depth_var && dist_var < toler_depth_var){
                angle_var = jmp_pts[i+1].angle - jmp_pts[i].angle;
                width_var = angle_var * jmp_pts[i].distance - pole_width;
                // rule_three: pole has a certain width
                if( width_var < jmp_pts[i].distance * angle.inc ){
                    options.push_back(jmp_pts[i]);
                    options.push_back(jmp_pts[i+1]);
                    // backup the distance of the closest possible pole
                    if(jmp_pts[i].distance < closest){
                        closest = jmp_pts[i].distance;
                    }
                    // cout<<jmp_pts[i].distance<<",";
                }
            }
        }
    }
    // cout<<endl;
    return;
}
void rp_frame::remove_outline(void)
{
    int i = 0;
    float gap_r, gap_a, thres_gap;
    vector<float> gaps;
    vector<float> sortgaps;
    vector<dt_point> alter;
    if(options.empty()){
        return;// no possible pole
    }
    // cout<<"Filted: ";
    gaps.clear();
    // calculate gaps between possible poles
    // cout<<"gaps: ";
    for(i = 2; i < options.size(); i += 2)
    {
        gap_r = options[i].distance - options[i-1].distance;
        gap_a = gap_r < 0 ? options[i].distance : options[i-1].distance;
        gap_a *= (options[i].angle - options[i-1].angle);
        gaps.push_back(gap_r*gap_r + gap_a*gap_a);
        // cout<<fixed<<setprecision(3)<<gap_r*gap_r + gap_a*gap_a<<",";
    }
    // cout<<endl;
    if(gaps.empty()){
        return;// only one possible pole
    }
    // select a threshold, Apriori
    sortgaps = gaps;
    sort(sortgaps.begin(), sortgaps.end());
    thres_gap = sortgaps[int(sortgaps.size()/2)];
    thres_gap = thres_gap < min_gap ? min_gap : (thres_gap > max_gap ? max_gap : thres_gap);
    thres_gap = (thres_gap + min_gap) / 2;
    // cout<<"tsh:"<<thres_gap<<endl;
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
    // cout<<"left "<<options.size()/2<<" pair(s): ";
    // for(i = 0; i < options.size(); i += 2){
    //     cout<<options[i].distance<<",";
    // }
    // cout<<endl;
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

int rp_frame::isConsistent(void)
{
    int i = 0;
    float dist_var, angle_var;
    bool near_away;
    dist_var = -1;
    angle_var = -1;
    // wait for data
    if(history.type == -1){
        cout<<"init"<<endl;
        return 2;
    }
    // last data and this data gives no pole
    if(history.distance == 0 && best_pole.distance == 0){
        near_away = true;
    }
    // last pole or this pole is out of sight
    else if(history.distance == 0 || best_pole.distance == 0){
        // check if the other pole is close to the edge
        // angle of the pole out of sight is set to middle
        if((history.angle - angle.min < toler_consis_ang * angle.inc || \
            angle.max - history.angle < toler_consis_ang * angle.inc)  || \
           (best_pole.angle - angle.min < toler_consis_ang * angle.inc || \
            angle.max - best_pole.angle < toler_consis_ang * angle.inc)){
                near_away = true;
        }
        else{ near_away = false; }
    }
    else{
        dist_var = best_pole.distance - history.distance;
        dist_var = dist_var < 0 ? -dist_var : dist_var;
        angle_var = best_pole.angle - history.angle;
        angle_var = angle_var < 0 ? -angle_var : angle_var;
        if(dist_var < consistent_dist && angle_var < consistent_angle){
            near_away = true;
        }
        else{ near_away = false; }
    }
    // cout<<"consistency: "<<dist_var<<","<<angle_var<<","<<near_away<<endl;

    /*      reliable(1)     dubious(2)      discarded(0)
    near(T) reliable        reliable        dubious
    away(F) discarded       dubious         dubious
    */
    if(near_away){
        return history.type == 0 ? 2 : 1;
    }
    else{
        return history.type == 1 ? 0 : 2;
    }
}