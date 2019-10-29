//
// Created by wangxufei on 19-1-29.
//
# include "include/H.h"

void get_s(double s_initial, double speed[N+1], double acceleration[N+1], double so_vf[N+1]){
    so_vf[0] = s_initial + speed[0]*TS + 1/2*acceleration[0]*TS*TS;
    for (int i = 1;i<N+1;i++){
        so_vf[i] = so_vf[i-1] + speed[i]*TS + 1/2*acceleration[i-1]*TS*TS;
    }
}

void get_current_state(vector<double> ego,state_t* current_state) {

    current_state->r_frenet = ego[0];
    current_state->s_frenet = ego[1]*-1;
    current_state->theta_error = ego[2];
    current_state->vel = ego[3];
    current_state->acc = ego[4];
    current_state->delta_rate = ego[5];
}

void Input4ACC(state_t * current_state, double c_curve[4], double lane_ref_ACC[N+1], double speed_ref_ACC[N+1], double so_vf[N+1],
               vector<double> reference_lines, vector<double> ego_pose,vector<double> curve, vector<vector<double > > dynamic_obstacle  ){
    std::cout<<"input start"<<std::endl;
    c_curve[0] = curve[0];
    c_curve[1] = curve[1];
    c_curve[2] = curve[2];
    c_curve[3] = curve[3];

    double s_initial = dynamic_obstacle[0][1];
    double speed_vf[N+1] = {0.0};
    double acceleration_vf[N+1] = {0.0};

    for(int i=0;i<N+1;i++){
        lane_ref_ACC[i] = reference_lines[0];   // 当前车道横坐标
        speed_ref_ACC[i] = ego_pose[6];        // 期望速度
        speed_vf[i] = dynamic_obstacle[0][3];
        acceleration_vf[i] = dynamic_obstacle[0][4];
    }
    get_s(s_initial, speed_vf, acceleration_vf, so_vf);
    get_current_state(ego_pose, current_state);
};

void Input4CL(state_t *current_state, double c_curve[4], double lane_ref_CL[N+1], double speed_ref_CL[N+1], double so_vf[N+1],
              double so_obs[N+1], double ro_obs[N+1], double R[N+1], double r_limit[2],
              vector<double > reference_lines, vector<double > ego_pose, vector<vector<double >> dynamic_obstacle,
              vector<double > curve, int feasiblelane_id, double lane_width) {
    c_curve[0] = curve[0];
    c_curve[1] = curve[1];
    c_curve[2] = curve[2];
    c_curve[3] = curve[3];

    double s_initial_vf = dynamic_obstacle[1][1];
    double s_initial_obs = dynamic_obstacle[0][1];
    double speed_vf[N+1] = {0.0};
    double acceleration_vf[N+1] = {0.0};
    double speed_obs[N+1] = {0.0};
    double acceleration_obs[N+1] = {0.0};

    if(feasiblelane_id == -1){
        r_limit[0] = (reference_lines[1] - lane_width/2);
        r_limit[1] = (reference_lines[0] + lane_width/2);
    }
    if(feasiblelane_id == 1){
        r_limit[0] = (reference_lines[0] - lane_width/2);
        r_limit[1] = (reference_lines[1] + lane_width/2);
    }

    for(int i=0;i<N+1;i++){
        lane_ref_CL[i] = reference_lines[1];   // 目标车道横坐标
        speed_ref_CL[i] = ego_pose[6];        // 期望速度
        speed_vf[i] = dynamic_obstacle[1][3];
        acceleration_vf[i] = dynamic_obstacle[1][4];
        speed_obs[i] = dynamic_obstacle[0][3];
        acceleration_obs[i] = dynamic_obstacle[0][4];
        ro_obs[i] = dynamic_obstacle[0][0] - 1.0;
        R[i] =24.0;
    }
    get_s(s_initial_vf, speed_vf, acceleration_vf, so_vf);
    get_s(s_initial_obs, speed_obs, acceleration_obs, so_obs);
    get_current_state(ego_pose, current_state);
};

void Input4AO(state_t *current_state, double c_curve[4], double lane_ref_AO[N+1], double speed_ref_AO[N+1], double so_vf[N+1],
              double so_obs[N+1], double ro_obs[N+1], double R[N+1], double r_limit[2],
              vector<double> reference_lines, vector<double> ego_pose, vector<double> FrontStaticFrontDynamic,
              vector<double> curve, double d_safe, vector<vector<double>> static_obstacle) {
    c_curve[0] = curve[0];
    c_curve[1] = curve[1];
    c_curve[2] = curve[2];
    c_curve[3] = curve[3];
    //cout<<c_curve[0] <<c_curve[1]<<c_curve[2]<<c_curve[3]<<endl;
    r_limit[0] = reference_lines[0] - 2;
    r_limit[1] = reference_lines[0] + 2;
    double speed_vf[N+1] = {0.0};
    double acceleration_vf[N+1] = {0.0};
    double s_initial_vf = FrontStaticFrontDynamic[1];
    double r = (static_obstacle[0][2])*(static_obstacle[0][2]) + static_obstacle[0][3]*static_obstacle[0][3]/4;
    double rr = (sqrt(4.87306) + d_safe + sqrt(r))*(sqrt(4.87306) + d_safe + sqrt(r));
    for(int i=0;i<N+1;i++){
        lane_ref_AO[i] = reference_lines[0];
        speed_ref_AO[i] = ego_pose[6];
        speed_vf[i] = FrontStaticFrontDynamic[3];
        acceleration_vf[i] = FrontStaticFrontDynamic[4];
        so_obs[i] = static_obstacle[0][1];
        ro_obs[i] = static_obstacle[0][0];
        R[i] = rr;
    }
    get_s(s_initial_vf, speed_vf, acceleration_vf, so_vf);
    get_current_state(ego_pose, current_state);
};
