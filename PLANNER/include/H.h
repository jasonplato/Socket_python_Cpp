# ifndef H_H
# define H_H

#include "acado_common.h"
#include "acado_qpoases_interface.hpp"
#include "acado_auxiliary_functions.h"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "time.h"
#include <dlfcn.h>

#define TS 0.1

using namespace std;

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

typedef struct {
    double s_frenet, r_frenet, theta_error,vel, delta_rate, acc;
} state_t;

typedef struct {
    double s_frenet[N + 1];
    double r_frenet[N + 1];
    double theta_error[N + 1];
    double vel[N + 1];
    double delta_rate[N+1];
    double acc[N+1];
} log_t;


//double c_curve_query(double s_vehicle, double c_curve[4]){
//    return  c_curve[3] * pow(s_vehicle, 3.0) + c_curve[2] * pow(s_vehicle, 2.0)+ c_curve[1] * s_vehicle  + c_curve[0];
//}
//
//double c_curve_trans(double s_vehicle, double r_vehicle, double c_curve[4]){
//    return 1.0 / (1.0 - r_vehicle * c_curve_query(s_vehicle, c_curve));
//}
//
//state_t  motion_function(state_t motion_state_k, double delta_f_rate, double acc, double c_curve[4]){
//    state_t motion_state_k1;
//    motion_state_k1.s_frenet = motion_state_k.s_frenet + TS * motion_state_k.vel * cos(motion_state_k.theta_error) *
//                               c_curve_trans(motion_state_k.s_frenet, motion_state_k.r_frenet, c_curve);
//    motion_state_k1.r_frenet = motion_state_k.r_frenet + TS * motion_state_k.vel * sin(motion_state_k.theta_error);
//    motion_state_k1.theta_error = motion_state_k.theta_error + TS * (delta_f_rate  - motion_state_k.vel *
//                                  cos(motion_state_k.theta_error)*c_curve_trans(motion_state_k.s_frenet, motion_state_k.r_frenet, c_curve) *
//                                  c_curve_query(motion_state_k.s_frenet, c_curve));
//    motion_state_k1.vel = motion_state_k.vel + TS * acc;
//
//    return motion_state_k1;
//}

void get_s(double s_initial, double speed[N+1], double acceleration[N+1], double so_vf[N+1]);
//    so_vf[0] = s_initial + speed[0]*TS + 1/2*acceleration[0]*TS*TS;
//    for (int i = 1;i<N+1;i++){
//        so_vf[i] = so_vf[i-1] + speed[i]*TS + 1/2*acceleration[i-1]*TS*TS;
//        cout<<so_vf[i]<<endl;
//    }
//}

void get_current_state(vector<double> ego,state_t * current_state) ;
//    current_state.s_frenet = ego[0];
//    current_state.r_frenet = ego[1];
//    current_state.theta_error = ego[2];
//    current_state.vel = ego[3];
//    current_state.delta_rate = ego[4];
//    current_state.acc = ego[5];
//}

////The function pointer type is

typedef void (*Init_ACC)(double, double, double, double, double);
typedef int (*Run_ACC)(state_t * , log_t * , double*, double*, double*, double*, int);

typedef void (*Init_CL)(double, double, double, double, double);
typedef int (*Run_CL)(state_t * , log_t * , double*, double*, double*, double*, double*, double*, double*,double, double, int);

typedef void (*Init_AO)(double, double, double, double, double, double);
typedef int (*Run_AO)(state_t * , log_t * , double*, double*, double*, double*, double*, double*, double*,double, double, int);

//// transmit

void Input4ACC(state_t * current_state, double c_curve[4], double lane_ref_ACC[N+1], double speed_ref_ACC[N+1], double so_vf[N+1],
               vector<double> reference_lines, vector<double> ego_pose,vector<double> curve, vector<vector<double > > dynamic_obstacle  );
//    c_curve[0] = curve[0];
//    c_curve[1] = curve[1];
//    c_curve[2] = curve[2];
//    c_curve[3] = curve[3];
//
//    double s_initial = dynamic_obstacle[0][1];
//    double speed_vf[N+1] = {0.0};
//    double acceleration_vf[N+1] = {0.0};
//
//    for(int i=0;i<N+1;i++){
//        lane_ref_ACC[i] = reference_lines[0];   // 当前车道横坐标
//        speed_ref_ACC[i] = ego_pose[6];        // 期望速度
//        speed_vf[i] = dynamic_obstacle[0][3];
//        acceleration_vf[i] = dynamic_obstacle[0][4];
//    }
//    get_s(s_initial, speed_vf, acceleration_vf, so_vf);
//    get_current_state(ego_pose, current_state);
//};

void Input4CL(state_t *current_state, double c_curve[4], double lane_ref_CL[N+1], double speed_ref_CL[N+1], double so_vf[N+1],
              double so_obs[N+1], double ro_obs[N+1], double R[N+1], double r_limit[2],
              vector<double > reference_lines, vector<double > ego_pose, vector<vector<double >> dynamic_obstacle,
              vector<double > curve, int feasiblelane_id, double lane_width);
//    c_curve[0] = curve[0];
//    c_curve[1] = curve[1];
//    c_curve[2] = curve[2];
//    c_curve[3] = curve[3];
//
//    double s_initial_vf = dynamic_obstacle[1][1];
//    double s_initial_obs = dynamic_obstacle[0][1];
//    double speed_vf[N+1] = {0.0};
//    double acceleration_vf[N+1] = {0.0};
//    double speed_obs[N+1] = {0.0};
//    double acceleration_obs[N+1] = {0.0};
//
//    if(feasiblelane_id == -1){
//        r_limit[0] = (reference_lines[1] - lane_width/2);
//        r_limit[1] = (reference_lines[0] + lane_width/2);
//    }
//    if(feasiblelane_id == 1){
//        r_limit[0] = (reference_lines[0] - lane_width/2);
//        r_limit[1] = (reference_lines[1] + lane_width/2);
//    }
//
//    for(int i=0;i<N+1;i++){
//        lane_ref_CL[i] = reference_lines[1];   // 目标车道横坐标
//        speed_ref_CL[i] = ego_pose[6];        // 期望速度
//        speed_vf[i] = dynamic_obstacle[1][3];
//        acceleration_vf[i] = dynamic_obstacle[1][4];
//        speed_obs[i] = dynamic_obstacle[0][3];
//        acceleration_obs[i] = dynamic_obstacle[0][4];
//        ro_obs[i] = dynamic_obstacle[0][0] - 1.0;
//        R[i] =24.0;
//    }
//    get_s(s_initial_vf, speed_vf, acceleration_vf, so_vf);
//    get_s(s_initial_obs, speed_obs, acceleration_obs, so_obs);
//    get_current_state(ego_pose, current_state);
//};

void Input4AO(state_t *current_state, double c_curve[4], double lane_ref_AO[N+1], double speed_ref_AO[N+1], double so_vf[N+1],
              double so_obs[N+1], double ro_obs[N+1], double R[N+1], double r_limit[2],
              vector<double> reference_lines, vector<double> ego_pose, vector<double> FrontStaticFrontDynamic,
              vector<double> curve, double d_safe, vector<vector<double>> static_obstacle);
//    c_curve[0] = curve[0];
//    c_curve[1] = curve[1];
//    c_curve[2] = curve[2];
//    c_curve[3] = curve[3];
//    //cout<<c_curve[0] <<c_curve[1]<<c_curve[2]<<c_curve[3]<<endl;
//    r_limit[0] = reference_lines[0] - 2;
//    r_limit[1] = reference_lines[0] + 2;
//    double speed_vf[N+1] = {0.0};
//    double acceleration_vf[N+1] = {0.0};
//    double s_initial_vf = FrontStaticFrontDynamic[1];
//    double r = (static_obstacle[0][2])*(static_obstacle[0][2]) + static_obstacle[0][3]*static_obstacle[0][3]/4;
//    double rr = (sqrt(4.87306) + d_safe + sqrt(r))*(sqrt(4.87306) + d_safe + sqrt(r));
//    for(int i=0;i<N+1;i++){
//        lane_ref_AO[i] = reference_lines[0];
//        speed_ref_AO[i] = ego_pose[6];
//        speed_vf[i] = FrontStaticFrontDynamic[3];
//        acceleration_vf[i] = FrontStaticFrontDynamic[4];
//        so_obs[i] = static_obstacle[0][1];
//        ro_obs[i] = static_obstacle[0][0];
//        R[i] = rr;
//    }
//    get_s(s_initial_vf, speed_vf, acceleration_vf, so_vf);
//    get_current_state(ego_pose, current_state);
//};

#endif