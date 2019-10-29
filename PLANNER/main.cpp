
#include "C:\Users\wangxufei\Desktop\PLANNER\include\Function.hpp"

int maintest() {

    log_t current_log;
    state_t current_state;
    int i, num_sim;
    const int step = 100;
    clock_t start,finish;
    int isOK = 0;

    //Parameters common
    double c_curve[4] = {0.0, 0.0, 0.0, 0.0};        // curve

    double so_vf[N+1] = {0.0};                       // initial s of vehicle
    double speed_vf[N+1] = {0.0};                    // initial speed of vehicle
    double acceleration_vf[N+1] = {0.0};             // initial acceleration of vehicle

    double lane_ref_ACC[N+1] = {0.0};                // reference lane
    double lane_ref_CL[N+1] = {0.0};                 // reference lane
    double lane_ref_AO[N+1] = {0.0};                 // reference lane

    double speed_ref_ACC[N+1] = {0.0};               // desired speed
    double speed_ref_CL[N+1] = {0.0};                // desired speed
    double speed_ref_AO[N+1] = {0.0};                // desired speed

    double s_initial = 40.0;                         // position initial of vf
    double s_initial_obs_cl = 42.0;                  // position initial of vf
    double s_initial_obs_ao = 40.0;                  // position initial of vf

    double speed_obs_cl[N+1] = {0.0};
    double acceleration_obs_cl[N+1] = {0.0};
    double speed_obs_ao[N+1] = {0.0};
    double acceleration_obs_ao[N+1] = {0.0};

    double so_obs_CL[N+1] = {0.0};                   // position initial of obs
    double ro_obs_CL[N+1] ={0.0};
    double R_CL[N+1] = {0.0};

    double so_obs_AO[N+1] = {0.0};                   // position initial of obs
    double ro_obs_AO[N+1] ={0.0};
    double R_AO[N+1] = {0.0};

    double r_left = -2;
    double r_right = 6;

    for (i=0; i<N+1; i++){
        speed_vf[i] = 5.0 ;                           //
        acceleration_vf[i] = 0.0 ;

        speed_obs_cl[i] = 0.0;
        acceleration_obs_cl[i] = 0.0;

        speed_obs_ao[i] = 5.0;
        acceleration_obs_ao[i] = 0.0;

        speed_ref_ACC[i]= 16.7;
        speed_ref_CL[i]= 16.7;
        speed_ref_AO[i]= 16.7;

        lane_ref_ACC[i] = 1.0;
        lane_ref_CL[i] = 4.0;
        lane_ref_AO[i] = 1.0;

        ro_obs_CL[i] = -1.7;
        ro_obs_AO[i] = -3.5;

        R_CL[i] = 28.0;
        R_AO[i] = 28.0;
    }

    get_s(s_initial, speed_vf, acceleration_vf, so_vf);  // get position of vf
    get_s(s_initial_obs_cl, speed_obs_cl, acceleration_obs_cl, so_obs_CL);  // get position of vf
    get_s(s_initial_obs_ao, speed_obs_ao, acceleration_obs_ao, so_obs_AO);  // get position of vf

    ////************************************  Test ACC  **********************************************////
    PlannerACC(&current_state, &current_log, c_curve, lane_ref_ACC, speed_ref_ACC, so_vf, isOK);


    ////*************************************  Test CL **********************************************////
    PlannerCL(&current_state, &current_log, c_curve, lane_ref_CL, speed_ref_CL,
            so_vf, so_obs_CL, ro_obs_CL, R_CL, r_left, r_right, isOK);


    ////*************************************  Test AO **********************************************////
    PlannerAO(&current_state, &current_log, c_curve, lane_ref_AO, speed_ref_AO,
              so_vf, so_obs_AO, ro_obs_AO, R_AO, r_left, r_right, isOK);


}