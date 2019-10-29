////---------------------------------------------------------------------------////
////            This headfile is created for communicating between             ////
////                traffic flow system and the planning module                ////
////                     Copyright @ Jie Chen in Momenta.ai                    ////
////---------------------------------------------------------------------------////

# ifndef TRAFFICFLOWSOCKET_H
# define TRAFFICFLOWSOCKET_H
# include <iostream>
# include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
# include <time.h>
# include <string.h>
# include <sstream>
# include <thread>
# include <vector>
# include <stdlib.h>
#include "/home/pinink/Desktop/socket_client/PLANNER/include/Function.hpp"


// socket 通信
// _command: 需要传输给服务端(python)的指令
// _recv: 服务端(python)根据_command指令返回的结果
static std::string _command;
static std::string _recv;

// _superCommand: 决策规划模块返回的指令
// 用于驱动traffic flow system中的ego-car
static std::string _superCommand;

enum BarrierType {
    None,
    SolidLanes,
    DoubleYellowLanes,
    Physical
};
// 定义各个_command所对应的返回值变量
static int lane_id;

static int feasible_lane_id;

static double dist2connect;

static double dist2frontobs; //本车道上，到前方障碍物的距离

static double obs2frontvehicle; //本车道上，前方静态障碍物与障碍物前方动态障碍物的距离

static double lane_width;

static double dist2stopline;

static std::vector<std::vector<double >> static_obstacles; //本车道及feasible车道上的静态障碍物
//static_obstacles[0]里存放的是本车道上的静态障碍物
//static_obstacles[1]里存放的是feasible车道上的静态障碍物

static bool is_light_green; //导航目标方向的红绿灯信息

static std::vector<double > ego_pose;

static std::vector<double > FrontStatic_Front_Dynamic; //本车前面的静态障碍物前面的动态障碍物的信息

static std::vector<std::vector<double>> dynamic_obstacles; //本车道及feasible车道上的动态障碍物
//dynamic_obstacles[0]里存放的是本车道上的动态障碍物
//dynamic_obstacles[1]里存放的是feasible车道上的动态障碍物

static std::vector<double > reference_lines;  //本车道及feasible车道的中心线
//reference_lines[0]存放的是本车道的中心线
//reference_lines[1]存放的是feasible车道的中心线

static std::vector<double >curve; //道路曲率

// to determine whether the sub-task has been completed
static bool mission_completed;

static double MPC_acc;
static double MPC_delta;

static log_t current_log;
static state_t current_state;
static double c_curve[4];
static double lane_ref_ACC[N+1];
static double lane_ref_CL[N+1];
static double lane_ref_AO[N+1];
static double speed_ref_ACC[N+1];
static double speed_ref_CL[N+1];
static double speed_ref_AO[N+1];
static double so_vf[N+1];
static double so_obs_CL[N+1];
static double so_obs_AO[N+1];
static double ro_obs_CL[N+1];
static double ro_obs_AO[N+1];
static double R_CL[N+1];
static double R_AO[N+1];
static double r_limit[2]={0};
static int isOK;
static double r_left = -2;
static double r_right = 6;

static std::string all_commands[15] = {"mission",
                                       "lane_id",
                                       "feasiblelane_id",
                                       "dist2connect",
                                       "dist2frontobs",
                                       "FrontStaticFrontDynamic",
                                       "lane_width",
                                       "dist2stopline",
                                       "static_obstacles",
                                       "is_light_green",
                                       "ego_pose",
                                       "dynamic_obstacles",
                                       "reference_lines",
                                       "obs2frontvehicle",
                                       "curve",
};


void _socket(int client_sockfd);
void command_match_recv();
std::vector<std::string> split(const std::string &s, const std::string &seperator);
void refreshCommand();
void update_supercommand();
void initial_MPC();
void print_log();

# endif