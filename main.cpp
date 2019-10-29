#include "trafficFlowSocket.h"

int main() {
    srand((unsigned)time(NULL));
    int sockfd;
    int addrlen;
    const char *seraddr="127.0.0.1";
    struct sockaddr_in cli_addr;

    sockfd= socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd<0)
    {
        fprintf(stderr,"socker error:%s\n",strerror(errno));
        exit(1);
    }
    addrlen = sizeof(struct sockaddr_in);
    bzero(&cli_addr, static_cast<size_t>(addrlen));

    cli_addr.sin_family = AF_INET;
    cli_addr.sin_port = htons(8888);
    inet_pton(AF_INET,seraddr,&cli_addr.sin_addr);

    while (connect(sockfd, (struct sockaddr*)&cli_addr, static_cast<socklen_t>(addrlen)) != 0)
    {
        fprintf(stderr,"connect error:%s\n",strerror(errno));
        //close(sockfd);
        exit(1);
    }

    _command = all_commands[13];
    MPC_acc = 0.0;
    MPC_delta = 0.0;
    //initial_MPC();
    current_state.r_frenet = 0.0;
    current_state.s_frenet = 0.0;
    current_state.acc = 0.0;
    current_state.delta_rate = 0.0;
    current_state.theta_error= 0.0;
    current_state.vel = 0.0;
    double r_left = -2;
    double r_right = 6;

    std::thread t(_socket,sockfd);
    t.detach();

    while(1)    //模拟李芳菲那边的MPC规划，以10HZ的速率更改一次_command的值
    {

//        if ((end - start)*1.0 / CLOCKS_PER_SEC*1000 >=100.0)
//        {
//            int index = (rand() % (b-a))+ a;
//            _command = all_commands[index];
//            start = end;
//        }
//        for (int i=0;i<_recv.length();i++)
//        {
//            std::cout<<_recv[i]<<" ";
//        }
        refreshCommand();

//        std::cout<<_recv<<std::endl;
//        end = clock();
    }

    close(sockfd);
    return 0;
}

void initial_MPC()
{
    int i, num_sim;
    const int step = 100;
    clock_t start,finish;
    isOK = 0;


    //Parameters common
    for (int i =0;i<4;i++)
    {
        c_curve[i] = 0.0;        // curve
    }


    // initial s of vehicle
    double speed_vf[N+1] = {0.0};                    // initial speed of vehicle
    double acceleration_vf[N+1] = {0.0};             // initial acceleration of vehicle

    // reference lane
    // reference lane
    // reference lane

    // desired speed
    // desired speed
    // desired speed

    double s_initial = 40.0;                         // position initial of vf
    double s_initial_obs_cl = 42.0;                  // position initial of vf
    double s_initial_obs_ao = 40.0;                  // position initial of vf

    double speed_obs_cl[N+1] = {0.0};
    double acceleration_obs_cl[N+1] = {0.0};
    double speed_obs_ao[N+1] = {0.0};
    double acceleration_obs_ao[N+1] = {0.0};

    // position initial of obs



    // position initial of obs



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

        so_obs_CL[i] = 0.0;
        so_obs_AO[i] = 0.0;
    }
    get_s(s_initial, speed_vf, acceleration_vf, so_vf);  // get position of vf
    get_s(s_initial_obs_cl, speed_obs_cl, acceleration_obs_cl, so_obs_CL);  // get position of vf
    get_s(s_initial_obs_ao, speed_obs_ao, acceleration_obs_ao, so_obs_AO);  // get position of vf
}




