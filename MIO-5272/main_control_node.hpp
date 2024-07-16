#include <posix_rt.h>
#include <math.h>
#include <cmath>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <semaphore.h>
#include <time.h>

#include "task_common.h"
#include "emaster.h"
#include "controller.h"
//#include "main_control.h"
#include "dynamics.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "control_interface/srv/control_interface.hpp"

using namespace std;
using namespace std::placeholders;
using namespace control_interface::srv;

/* globals */
EcatMaster eMaster;

Controller pidPos[INDY7_DOF]; // POS -> VEL
Controller pidVel[INDY7_DOF]; // VEL -> Torque 

PidGain pidGainPos[] = {
	{50.0, 0.000, 0.0},
	{30.0, 0.000, 0.0},
	{30.0, 0.000, 0.0},
	{30.0, 0.000, 0.0},
	{30.0, 0.000, 0.0},
	{30.0, 0.000, 0.0},
};
//double VelLimits[]= {30.,30.,30.,30.,30.,30.}; //[deg/s]
const double indy7_vel = 15.0;
double VelLimits[]= {indy7_vel, indy7_vel, indy7_vel, indy7_vel, indy7_vel, indy7_vel};
//double VelLimits[]= {10.0, 10.0, 10.0, 10.0, 10.0, 10.0}; //[deg/s]

PidGain pidGainVel[] = {
	{80.0, 100.0, 0.0},
	{80.0, 100.0, 0.0},
	{60.0, 60.0, 0.0},
	{20.0, 40.0, 0.0},
	{20.0, 40.0, 0.0},
	{15.0, 40.0, 0.0},
};
double TorLimits[]= {40.0, 110.0, 40.0, 25.0, 25.0, 25.0}; //[Nm]


//void taskEcat(void *arg);
//void init_controller(void);

//MIO5272와 통신하기 위한 message
struct control_interface_response indy7_response_message;

rclcpp::Client<ControlInterface>::SharedPtr MIO5272_main_control_client;
std::shared_ptr<ControlInterface::Request> indy7_ros_request;
rclcpp::Client<ControlInterface>::SharedFuture indy7_ros_response;

//시작 시간, 수행 시간, 대기 시간 
struct timespec duration_delay_start, duration_delay_current, wait_time, test_time;

//joint 이동 결과 확인 변수 
bool complete_bool = true;
int complete_bool_num = 0;

//MoveIt signal 확인 변수 / 0: 시작, 1: 끝  
bool trajectory_bool = false;

//세마포어 
sem_t* ros_semaphore;
char* ros_semaphore_name = "/ros_semaphore1";
int semaphore_state;

int semaphore_i=1;

class main_control_node : public rclcpp::Node
{
	public:
	main_control_node(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions()) : Node("main_control", node_options)
	//main_control_node() : Node("main_control")
	{
		MIO5272_main_control_client = create_client<ControlInterface>("user_control_service");
		indy7_ros_request = std::make_shared<ControlInterface::Request>();
		
		//서버 접속 시도 
		while(!MIO5272_main_control_client->wait_for_service(1s))
		{
			if(!rclcpp::ok())
			{
				RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service.");
				return;
			}
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting agin...");
		}
		//RCLCPP_INFO(this->get_logger(), "MIO5272_main_control_client +++++++++++++++++++++++++++++++");
		
		main_control_init();
		main_control_execute();
	}
	
	void taskEcat(void *arg);
	void init_controller(void);
	void taskEcat_init(void *arg);
	void taskEcat_execute(void *arg);
	
	void main_control_init()
	{
		mlockall(MCL_CURRENT|MCL_FUTURE); 
		create_rt_task(&taskEcat_h, (PCHAR)"EtherCAT Task", 0, 90);
		set_task_period(&taskEcat_h, SET_TM_NOW, PERIOD2NS(0.001));
		
		indy7_ros_request->control_mode = 52;
		
		indy7_ros_request->current_path_num = 0;
	}
	
	void main_control_execute()
	{
		start_task(&taskEcat_h, (void (*)(void*))&main_control_node::taskEcat, NULL);
		
	}
	
	void test_print()
	{
		RCLCPP_INFO(this->get_logger(), "rclcpp &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
	}
	
};

void main_control_node::taskEcat(void *arg)
{	
	double pid_target_vel[INDY7_DOF];
	double pid_target_torque[INDY7_DOF];
	float target_pos_rad;
	
	init_controller();
	int ret = eMaster.init(true, PERIOD2NS(0.01),1);
	if (ret != 0)
	{
		fprintf(stderr, "cannot init EcatMaster with err %d", ret);
		exit(1);
	}
	
	msgDynamicsData dGC;
	
	clock_getres(CLOCK_REALTIME, &test_time);
	test_time.tv_sec = 0;
	test_time.tv_nsec = 0;
	
	complete_bool = true;
	complete_bool_num = 0;
	
	while (1) 
	{
		//perf->get_releaseTime();
		wait_next_period(NULL);

		//perf->measure_period();
		
		eMaster.read_from_slaves();
		
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "");
		
		if(eMaster.is_ecat_ready()==true)
		{
			eMaster.indy7.Endtool[0].set_LEDMode(SlaveNrmkEndtool::BLINK_BLUE);
			
			//Get Queue GC Torque
			if( qDynamics.receive(&dGC) == true ){
				for(int i=0; i<INDY7_DOF;i++){
					eMaster.indy7.set_dynamics_info(i, dGC.G[i], dGC.C[i]);
				}
			}
	        
	        if(trajectory_bool == true)
	        {
				for(int i=0; i<INDY7_DOF;i++)
				{
					eMaster.indy7.set_target_info(i, (float)indy7_response_message.joint_angular[i]*(180/3.141592)*D2R, (float)indy7_response_message.joint_velocity[i]*D2R);
					//printf("%f / ", indy7_response_message.joint_angular[i]);
					//printf("%f / ", indy7_response_message.joint_velocity[i]);
				}
				//printf("\n");
				
	        	trajectory_bool = false;
			}
	        
        	//joint 각도 목표값 도달 여부 확인 
			complete_bool = true;
			for(int i=0;i<INDY7_DOF;i++)
			{
				//모든 joint 각도(라디안) 오차 0.0087266444 이하 (0.5도) 
				complete_bool &= (abs(indy7_response_message.joint_angular[i]-eMaster.indy7.get_pos_rad(i)) < 0.0087266444) ? true : false;
				//printf("%f / ", abs(indy7_response_message.joint_angular[i]-eMaster.indy7.get_pos_rad(i)));
			}
			//printf("\n");
			
			///*
			//속도 측정 
			clock_gettime(CLOCK_REALTIME, &test_time);
			if(duration_delay_start.tv_nsec > 0)
			{
				printf("%ld ", ((test_time.tv_sec - duration_delay_start.tv_sec)*1000 + (test_time.tv_nsec - duration_delay_start.tv_nsec)/1000000));
				for(int i=0;i<INDY7_DOF;i++)
				{
					//속도 정보 출력 
					printf("%f ", eMaster.indy7.get_pos_rad(i));
				}
				printf("\n");
			}
			//*/
			
			if(complete_bool == true)
			{
				complete_bool_num++;
				
				if((complete_bool_num > 10) && (semaphore_i < indy7_ros_request->current_path_num))
				{
					sem_post(ros_semaphore);
					semaphore_i++;
					complete_bool_num = 0;
				}
			}
			else
			{
				complete_bool_num = 0;
			}
			
			//Controller
			for(int i=0; i<INDY7_DOF;i++)
			{
				
				#if false
				if (i==1){
					if (eMaster.indy7.is_servo_on(i) == false){
						eMaster.indy7.servo_on(i);
					}
					else{
						eMaster.indy7.Endtool[0].set_LEDMode(SlaveNrmkEndtool::BLINK_GREEN);
					}	
					target_pos_rad =  eMaster.indy7.get_target_pos(i);
					// printf("%f\n", eMaster.indy7.get_target_pos(i));
					// target_pos_rad =  0.;

					
				}
				#else
				if (eMaster.indy7.is_servo_on(i) == false){
						eMaster.indy7.servo_on(i);
				}
				else{
					eMaster.indy7.Endtool[0].set_LEDMode(SlaveNrmkEndtool::BLINK_GREEN);
				}	
				target_pos_rad =  eMaster.indy7.get_target_pos(i);
				// target_pos_rad = 0.;
				#endif
			 	
				pid_target_vel[i] = pidPos[i].PIDController(pidGainPos[i], eMaster.indy7.get_pos_rad(i), target_pos_rad);
				pid_target_torque[i] = pidVel[i].PIDController(pidGainVel[i], eMaster.indy7.get_vel_rad(i), -pid_target_vel[i]);
				
				eMaster.indy7.set_control_torque(i, pid_target_torque[i]);
			}
			
			eMaster.indy7.set_joint_torque_cmd();
		} 

		eMaster.write_to_slaves();
	}
}

/****************************************************************************/
void main_control_node::init_controller(void){

	for (int i=0; i < INDY7_DOF; i++)
	{
		pidPos[i].setDt(0.001);
		pidPos[i].setExtreme(VelLimits[i]*D2R);
		pidVel[i].setDt(0.001);
		pidVel[i].setExtreme(TorLimits[i]);
	}
}
/****************************************************************************/

