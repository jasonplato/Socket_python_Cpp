# ifndef FUNCTION_H
# define FUNCTION_H
#include "H.h"


void PlannerACC(state_t * current_state, log_t * current_log, double c_curve[4], double lane_ref_ACC[N+1],
        double speed_ref_ACC[N+1], double so_vf[N+1], int isOK);
//    void *handle = dlopen("/home/wangxufei/CLionProjects/socket_client/PLANNER/libmpc_acc.so", RTLD_LOCAL | RTLD_NOW);
//    if(handle == NULL){
//        cout << "Error: " << endl;
//    }
//
//    //Define function pointer
//    Init_ACC mpc_planner_init_acc;
//    Run_ACC run_mpc_acc;
//
//    //Point pointer to funtion
//    mpc_planner_init_acc = (Init_ACC)dlsym(handle, "mpc_planner_init_ACC");
//    run_mpc_acc = (Run_ACC)dlsym(handle, "run_mpc_ACC");
//
//    mpc_planner_init_acc(1.0, 1.0, 1.0, 1.0, 50.0);
//    cout<< " ACC initialization finish " <<endl;
//
//    run_mpc_acc(current_state, current_log, c_curve, lane_ref_ACC, speed_ref_ACC, so_vf, isOK);
//    for(int i = 0; i<N+1;i++){
//        cout<< "s_frenet " << current_log->s_frenet[i]
//            <<" r_frenet "<<current_log->r_frenet[i]
//            <<" theta_error  "<<current_log->theta_error[i]
//            <<" vel "<< current_log->vel[i]
//            << " delta_rate " << current_log->delta_rate[i]
//            <<" acc "<< current_log->acc[i]<<endl;
//    }
//    cout<< " ACC test finish " <<endl;
//}

void PlannerCL(state_t * current_state, log_t * current_log, double c_curve[4], double lane_ref[N+1],
               double speed_ref[N+1],  double so_vf[N+1], double so_obs[N+1], double ro_obs[N+1], double R[N+1],
               double r_left, double r_right,  int isOK);
//    void *handle = dlopen("/home/wangxufei/CLionProjects/socket_client/PLANNER/libmpc_cl.so", RTLD_LOCAL | RTLD_NOW);
//    if(handle == NULL){
//        cout << "Error: " << endl;
//    }
//
//    //Define function pointer
//    Init_CL init_cl;
//    Run_CL run_cl;
//
//    //Point pointer to funtion
//    init_cl = (Init_CL)dlsym(handle, "mpc_planner_init_CL");
//    run_cl = (Run_CL)dlsym(handle, "run_mpc_CL");
//
//    init_cl(1.0, 1.0, 0.001, 1, 1);
//    cout<< " CL initialization finish " <<endl;
//
//    run_cl(current_state, current_log, c_curve, lane_ref, speed_ref, so_vf, so_obs, ro_obs, R, r_left, r_right, isOK);
//    for(int i = 0; i<N+1;i++){
//        cout<< "s_frenet " << current_log->s_frenet[i]
//            <<" r_frenet "<<current_log->r_frenet[i]
//            <<" theta_error  "<<current_log->theta_error[i]
//            <<" vel "<< current_log->vel[i]
//            << " delta_rate " << current_log->delta_rate[i]
//            <<" acc "<< current_log->acc[i]<<endl;
//    }
//    cout<< " CL test finish " <<endl;
//}

void PlannerAO(state_t * current_state, log_t * current_log, double c_curve[4], double lane_ref[N+1],
               double speed_ref[N+1],  double so_vf[N+1], double so_obs[N+1], double ro_obs[N+1], double R[N+1],
               double r_left, double r_right,  int isOK);
//    void *handle = dlopen("/home/wangxufei/CLionProjects/socket_client/PLANNER/libmpc_ao.so", RTLD_LOCAL | RTLD_NOW);
//    if(handle == NULL){
//        cout << "Error: " << endl;
//    }
//
//    //Define function pointer
//    Init_AO init_ao;
//    Run_AO run_ao;
//
//    //Point pointer to funtion
//    init_ao = (Init_AO)dlsym(handle, "mpc_planner_init_AO");
//    run_ao = (Run_AO)dlsym(handle, "run_mpc_AO");
//
//    init_ao(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
//    cout<< " AO initialization finish " <<endl;
//
//    run_ao(current_state, current_log, c_curve, lane_ref, speed_ref, so_vf, so_obs, ro_obs, R, r_left, r_right, isOK);
//    for(int i = 0; i<N+1;i++){
//        cout<< "s_frenet " << current_log->s_frenet[i]
//            <<" r_frenet "<<current_log->r_frenet[i]
//            <<" theta_error  "<<current_log->theta_error[i]
//            <<" vel "<< current_log->vel[i]
//            << " delta_rate " << current_log->delta_rate[i]
//            <<" acc "<< current_log->acc[i]<<endl;
//    }
//    cout<< " AO test finish " <<endl;
//}

# endif