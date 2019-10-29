////---------------------------------------------------------------------------////
////           This sourcefile is created for communicating between            ////
////                traffic flow system and the planning module                ////
////                     Copyright @ Jie Chen in Momenta.ai                    ////
////---------------------------------------------------------------------------////
# include "trafficFlowSocket.h"
using namespace std;

//此函数用一个单开的线程运行，这样可以保证仿真环境的系统不会因为通信阻塞而影响画图频率
void _socket(int client_sockfd)
{
    mission_completed = true;
    while(1)
    {

        const char* send_data = _command.c_str();

        send(client_sockfd, send_data, static_cast<int>(strlen(send_data) + 1), 0);
        // able_to_send = false;


        char recvBuf[1024]={"\0"};
        recv(client_sockfd, recvBuf, 1024, 0);
        _recv = "";
        for(int i =0;i<strlen(recvBuf);i++) {
            _recv += recvBuf[i];
        }
        command_match_recv();
    }
}


//根据_command，把socket通信接收到的信息转换成给相应的变量赋值。
void command_match_recv()
{
    unsigned long long int position = _recv.find(":");
    //std::cout<<_recv<<std::endl;
    std::string prefix = _recv.substr(0,position);
    //std::cout<<prefix<<std::endl;
    _recv = _recv.substr(position+1);

    //std::cout<<_command<<std::endl;
    if (prefix != _command)
        return ;
    if (prefix == "mission")
    {
        mission_completed = _recv == "Done";
    }
    else if (prefix == "FrontStatic_Front_Dynamic")
    {
        FrontStatic_Front_Dynamic.clear();
        std::vector<std::string> temp;
        temp = split(_recv," ");
        for(std::vector<std::string>::size_type i = 0; i != temp.size(); ++i)
        {
            FrontStatic_Front_Dynamic.push_back(std::stod(temp[i]));
        }

    }
    else if (prefix == "lane_id")
    {
        lane_id = std::stoi(_recv);
    }
    else if (prefix == "feasiblelane_id")
    {
        feasible_lane_id = std::stoi(_recv);
    }
    else if (prefix == "dist2connect")
    {
        dist2connect = std::stod(_recv);
    }
    else if (prefix == "dist2frontobs")
    {
        dist2frontobs = std::stod(_recv);

    }
    else if (prefix == "lane_width")
    {
        lane_width = std::stod(_recv);
    }

    else if (prefix == "dist2stopline")
    {
        dist2stopline = std::stod(_recv);
    }
    else if (prefix == "static_obstacles")
    {
        static_obstacles.clear();
        std::vector<std::string> temp;

        temp = split(_recv,"\n");
        for(std::vector<std::string>::size_type i = 0; i != temp.size(); ++i)
        {
            std::vector<double > obstacle;
            std::vector<std::string> _temp;
            _temp = split(temp[i]," ");

            for(std::vector<std::string>::size_type j = 0; j != _temp.size(); ++j)
            {
                if (j == 0)
                {
                    obstacle.push_back(std::stoi(_temp[j]));
                    continue;
                }
                obstacle.push_back(std::stod(_temp[j]));
            }
            static_obstacles.push_back(obstacle);
        }
    }
    else if (prefix == "is_light_green")
    {
        is_light_green = _recv == "True";
    }
    else if (prefix == "ego_pose")
    {
        ego_pose.clear();
        std::vector<std::string> temp;
        temp = split(_recv," ");
        for(std::vector<std::string>::size_type i = 0; i != temp.size(); ++i)
        {
            ego_pose.push_back(std::stod(temp[i]));
        }
    }
    else if (prefix == "dynamic_obstacles")
    {
        dynamic_obstacles.clear();
        std::vector<std::string> temp;

        temp = split(_recv,"\n");
        for(std::vector<std::string>::size_type i = 0; i != temp.size(); ++i)
        {
            std::vector<double> vehicle;
            std::vector<std::string> _temp;
            _temp = split(temp[i]," ");

            for(std::vector<std::string>::size_type j = 0; j != _temp.size(); ++j)
            {
                if (j == 0 || j == 1)
                {
                    vehicle.push_back(std::stoi(_temp[j]));
                    continue;
                }
                vehicle.push_back(std::stod(_temp[j]));
            }
            dynamic_obstacles.push_back(vehicle);
        }
    }
    else if (prefix == "obs2frontvehicle")
    {
        obs2frontvehicle = std::stod(_recv);
    }
    else if (prefix == "reference_lines")
    {
        reference_lines.clear();
        std::vector<std::string> temp;
        temp = split(_recv," ");
        for(std::vector<std::string>::size_type i = 0; i != temp.size(); ++i)
        {
            reference_lines.push_back(std::stod(temp[i]));
        }
    }
    else if (prefix == "curve")
    {
        curve.clear();
        std::vector<std::string> temp;
        temp = split(_recv," ");
        for(std::vector<std::string>::size_type i = 0; i != temp.size(); ++i)
        {
            curve.push_back(std::stod(temp[i]));
        }
    }
//    else if (prefix == "CRUISE" || prefix == "LEFT_CHANGE" || prefix == "RIGHT_CHANGE" || prefix == "STOP" || prefix == "TAKEOVER")
//    {
//        // std::cout<<"recv:"<<_recv<<std::endl;
//        // mission_completed = _recv == "Done";
//        std::cout<<"process over"<<std::endl;
//    }
    else{
        return;
    }
}

//用于字符串的分割，是一个辅助函数(utils)
std::vector<std::string> split(const std::string &s, const std::string &seperator)
{
    std::vector<std::string> result;
    typedef std::string::size_type string_size;
    string_size i = 0;

    while(i != s.size())
    {
        //找到字符串中首个不等于分隔符的字母；
        int flag = 0;
        while(i != s.size() && flag == 0)
        {
            flag = 1;
            for(string_size x = 0; x < seperator.size(); ++x)
                if(s[i] == seperator[x])
                {
                    ++i;
                    flag = 0;
                    break;
                }
        }

        //找到又一个分隔符，将两个分隔符之间的字符串取出；
        flag = 0;
        string_size j = i;
        while(j != s.size() && flag == 0)
        {
            for(string_size x = 0; x < seperator.size(); ++x)
                if(s[j] == seperator[x])
                {
                    flag = 1;
                    break;
                }
            if(flag == 0)
                ++j;
        }
        if(i != j)
        {
            result.push_back(s.substr(i, j-i));
            i = j;
        }
    }
    return result;
}


void refreshCommand()
{
    for (int i = 0; i < 15; i++)
    {
        _command = all_commands[i];
        if ( i == 0 )
        {
            _command += ":";
            _command += std::to_string(( (double)( (int)( (MPC_acc+0.005)*100 ) ) )/100);
            _command += " ";
            _command += std::to_string(( (double)( (int)( (MPC_delta+0.005)*100 ) ) )/100);
            _command += " ";
        }
        usleep(30000);

    }
    print_log();
    usleep(30000);
    _superCommand = "CRUISE";
//    if(mission_completed && (_superCommand != "RIGHT_CHANGE" && _superCommand != "LEFT_CHANGE"))
//    {
//        //std::cout<<"mission_completed:"<<mission_completed<<std::endl;
//        //update_supercommand();
//        std::cout<<"super_command:"<<_superCommand<<std::endl;
//        _command = _superCommand;
//        usleep(30000);
//    }
//    else if (mission_completed){
//        _superCommand = "CRUISE";
//        _command = _superCommand;
//        usleep(30000);
//    }
    Input4ACC(&current_state,c_curve,lane_ref_ACC,speed_ref_ACC,so_vf,
        reference_lines,ego_pose,curve,dynamic_obstacles);
    PlannerACC(&current_state, &current_log, c_curve, lane_ref_ACC, speed_ref_ACC, so_vf, isOK);
//    Input4CL(&current_state,c_curve,lane_ref_ACC,speed_ref_ACC,so_vf,so_obs_CL,ro_obs_CL,R_CL,r_limit,
//             reference_lines,ego_pose,dynamic_obstacles,curve,feasible_lane_id,lane_width);
//    PlannerCL(&current_state, &current_log, c_curve, lane_ref_CL,speed_ref_CL,so_vf, so_obs_CL, ro_obs_CL, R_CL,
//            r_left, r_right,isOK);
//        else if (_superCommand == "LEFT_CHANGE" || _superCommand == "RIGHT_CHANGE")
//        {
//            PlannerCL(current_state, &current_log, c_curve, lane_ref_CL, speed_ref_CL,
//                      so_vf, so_obs_CL, ro_obs_CL, R_CL, r_left, r_right, isOK);
//        }
//        else {
//            PlannerAO(current_state, &current_log, c_curve, lane_ref_AO, speed_ref_AO,
//                      so_vf, so_obs_AO, ro_obs_AO, R_AO, r_left, r_right, isOK);
//        }
    MPC_delta = current_log.delta_rate[1];
    MPC_acc = current_log.acc[1];

}

void update_supercommand()
{
    //std::cout<<"update begin"<<std::endl;
    //std::cout<<"supercommand:"<<_superCommand<<std::endl;
    if (_superCommand != "TAKEOVER")
    {
        if (feasible_lane_id == 0 )
        {
            if (is_light_green || (!is_light_green && dist2stopline >= 10.0)){
                _superCommand = "CRUISE";
            }
            else{
                _superCommand = "STOP";
            }
        }

        else if (feasible_lane_id == 1){
            _superCommand = "LEFT_CHANGE";
        }
        else if (feasible_lane_id == -1){
            _superCommand = "RIGHT_CHANGE";
        }
        else{
            std::cout<<"warning!!!!!!!!!!!!!!!!"<<std::endl;
            _superCommand = "STOP";
            sleep(3);
            if (feasible_lane_id == 0){
                if (is_light_green){
                    _superCommand = "CRUISE";
                }
                else{
                    _superCommand = "STOP";
                }
            }
            else if (feasible_lane_id == 1){
                _superCommand = "LEFT_CHANGE";
            }
            else if (feasible_lane_id == -1){
                _superCommand = "RIGHT_CHANGE";
            }
            else{
                _superCommand = "STOP";
                sleep(3);
                _superCommand = "TAKEOVER";
            }
        }
    }
    mission_completed = false;
    //std::cout<<"update over"<<std::endl;
}

void print_log()
{
    std::cout<<"lane_id:"<< lane_id<<std::endl;
    std::cout<<"feasiblelane_id:"<< feasible_lane_id<<std::endl;
    std::cout<<"dist2connect:"<< dist2connect<<std::endl;
    std::cout<<"dist2frontobs:"<< dist2frontobs<<std::endl;
    std::cout<<"FrontStaticFrontDynamic:"<<std::endl;
    for (int i =0;i!=FrontStatic_Front_Dynamic.size();i++)
    {
        std::cout<<FrontStatic_Front_Dynamic[i]<<" ";
    }
    std::cout<<std::endl;
    std::cout<<"lane_width:"<< lane_width<<std::endl;
    std::cout<<"dist2stopline:"<<dist2stopline<<std::endl;
    std::cout<<"static_obstacles:"<<std::endl;
    for (int i =0;i!=static_obstacles.size();i++)
    {
        std::cout<<i<<" ";
        for (int j =0;j!=static_obstacles[i].size();j++)
        {
            std::cout<<static_obstacles[i][j]<<" ";
        }
        std::cout<<std::endl;

    }
    std::cout<<"is_light_green:"<< is_light_green<<std::endl;
    std::cout<<std::endl;
    std::cout<<"ego_pose:"<<std::endl;
    for (int i =0;i!=ego_pose.size();i++)
    {
        std::cout<<ego_pose[i]<<" ";
    }
    std::cout<<std::endl;
    std::cout<<"dynamic_obstacles:"<<std::endl;
    for (int i =0;i!=dynamic_obstacles.size();i++)
    {
        std::cout<<i<<" ";
        for (int j =0;j!=dynamic_obstacles[i].size();j++)
        {
            std::cout<<dynamic_obstacles[i][j]<<" ";
        }
        std::cout<<std::endl;

    }
    std::cout<<std::endl;
    std::cout<<"reference_lines:"<<std::endl;
    for (int i =0;i!=reference_lines.size();i++)
    {
        std::cout<<reference_lines[i]<<" ";
    }
    std::cout<<std::endl;
    std::cout<<"obs2frontvehicle:"<<obs2frontvehicle<<std::endl;
    std::cout<<"curve:"<<std::endl;
    for (int i =0;i!=curve.size();i++)
    {
        std::cout<<curve[i]<<" ";
    }
    std::cout<<std::endl;

}