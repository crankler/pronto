#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "RobotStateListener.hpp"

using namespace std;
using namespace boost;
using namespace visualization_utils;

  //==================constructor / destructor
  
  /**Subscribes to Robot URDF Model and to EST_ROBOT_STATE.*/
  RobotStateListenerX::RobotStateListenerX(boost::shared_ptr<lcm::LCM> &lcm, 
          BotViewer *viewer, int operation_mode):
    _urdf_parsed(false),
    _lcm(lcm),
    _viewer(viewer)
  {
   _last_state_msg_system_timestamp = 0;
   _last_state_msg_sim_timestamp = 0;
    //lcm ok?
    if(!lcm->good())
    {
      cerr << "\nLCM Not Good: Robot State Handler" << endl;
      return;
    }

    // Default Greyscale color:
    _robot_color[0] = 0.15;
    _robot_color[1] = 0.15;
    _robot_color[2] = 0.15;

    // Subscribe to Robot_Model.  Will unsubscribe once a single message has been received
    _urdf_subscription = lcm->subscribe("ROBOT_MODEL", 
				       &RobotStateListenerX::handleRobotUrdfMsg,
				       this);    
    _urdf_subscription_on = true;
    //Subscribes to MEAS_JOINT_ANGLES 
    //lcm->subscribe("MEAS_JOINT_ANGLES", &RobotStateListenerX::handleJointAnglesMsg, this); 
    if (operation_mode==0){
      lcm->subscribe("EST_ROBOT_STATE", &RobotStateListenerX::handleRobotStateMsg, this); 
    }else if(operation_mode==1){
      // Alternative Robot State:
      lcm->subscribe("EST_ROBOT_STATE_BDI", &RobotStateListenerX::handleRobotStateMsg, this); 
      _robot_color[0] = 0.15;
      _robot_color[1] = 0.15;
      _robot_color[2] = 0.4;
    }
    
    _jointdof_filter_list.clear();
    _jointdof_filter_list.push_back("l_arm_usy");
    _jointdof_filter_list.push_back("r_arm_usy");
    _jointdof_filter_list.push_back("l_arm_shx");
    _jointdof_filter_list.push_back("r_arm_shx");
    _jointdof_filter_list.push_back("l_arm_ely");
    _jointdof_filter_list.push_back("r_arm_ely");
    _jointdof_filter_list.push_back("l_arm_elx");
    _jointdof_filter_list.push_back("r_arm_elx");
    _jointdof_filter_list.push_back("l_arm_uwy");
    _jointdof_filter_list.push_back("r_arm_uwy");
    _jointdof_filter_list.push_back("l_arm_mwx");
    _jointdof_filter_list.push_back("r_arm_mwx");
    _jointdof_filter_list.push_back("l_leg_hpz");
    _jointdof_filter_list.push_back("r_leg_hpz");
    _jointdof_filter_list.push_back("l_leg_hpx");
    _jointdof_filter_list.push_back("r_leg_hpx");
    _jointdof_filter_list.push_back("l_leg_hpy");
    _jointdof_filter_list.push_back("r_leg_hpy");
    _jointdof_filter_list.push_back("l_leg_kny");
    _jointdof_filter_list.push_back("r_leg_kny");
    _jointdof_filter_list.push_back("l_leg_aky");
    _jointdof_filter_list.push_back("r_leg_aky");
    _jointdof_filter_list.push_back("l_leg_akx");
    _jointdof_filter_list.push_back("r_leg_akx");  
    _jointdof_filter_list.push_back("neck_ay");
    _jointdof_filter_list.push_back("back_bkz");
    _jointdof_filter_list.push_back("back_bky");
    _jointdof_filter_list.push_back("back_bkx");
    
    
    Eigen::Vector3f temp;
    temp << 0,0,0;
    ee_forces_map.insert(make_pair("l_hand", temp));
    ee_forces_map.insert(make_pair("r_hand", temp));
    ee_forces_map.insert(make_pair("l_foot", temp));
    ee_forces_map.insert(make_pair("r_foot", temp));
    ee_torques_map.insert(make_pair("l_hand", temp));
    ee_torques_map.insert(make_pair("r_hand", temp));
    ee_torques_map.insert(make_pair("l_foot", temp));
    ee_torques_map.insert(make_pair("r_foot", temp));

  }

  RobotStateListenerX::~RobotStateListenerX() {
  }

//-------------------------------------------------------------------------------------      
//=============message callbacks

  void RobotStateListenerX::handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf,
						 const string& chan, 
						 const bot_core::robot_state_t* msg)						 
  {
    
    //int64_t tic = bot_timestamp_now();
    if (!_urdf_parsed)
    {
     //cout << msg->utime << endl;
      //cout << "\n handleRobotStateMsg: Waiting for urdf to be parsed" << endl;
      return;
    }
    if(_urdf_subscription_on)
    {			
      cout << "\n handleRobotStateMsg: unsubscribing from _urdf_subscriptionq" << endl;
      _lcm->unsubscribe(_urdf_subscription);     //unsubscribe from urdf messages
      _urdf_subscription_on =  false; 	
    }
    
    // Render at 100Hz of Real Time. Too much rendering will make the viewer less reponsive.
    //cout << msg->utime - _last_state_msg_system_timestamp << endl;
		int64_t now = bot_timestamp_now();//msg->utime
    if(now-_last_state_msg_system_timestamp >= 100000)  // timestamps are in usec
    {
    // cout << now - _last_state_msg_system_timestamp << endl;
    _gl_robot->set_state(*msg);
    
    bot_viewer_request_redraw(_viewer);
     _last_state_msg_system_timestamp = now;//msg->utime;
     _last_state_msg_sim_timestamp = msg->utime;
    }

    //int64_t toc = bot_timestamp_now();
    //cout << bot_timestamp_useconds(toc-tic) << endl;
    
  } // end handleMessage
  

//-------------------------------------------------------------------------------------        
  void RobotStateListenerX::handleRobotUrdfMsg(const lcm::ReceiveBuffer* rbuf, const string& channel, 
					       const  bot_core::robot_urdf_t* msg) 
  {

    if(_urdf_parsed ==false) 
    {
      cout<< "\nurdf handler @ RobotStateListenerX" << endl;
      // Received robot urdf string. Store it internally and get all available joints.
      _robot_name      = msg->robot_name;
      _urdf_xml_string = msg->urdf_xml_string;
      cout<< "\nReceived urdf_xml_string of robot [" 
      << msg->robot_name << "], storing it internally as a param" << endl;

      bot_gtk_gl_drawing_area_set_context(this->_viewer->gl_area); // Prevents conflict with cam renderer which messes with the gl context
      _gl_robot = shared_ptr<visualization_utils::InteractableGlKinematicBodyX>(new visualization_utils::InteractableGlKinematicBodyX(_urdf_xml_string,true,_robot_name));
  
      cout<< "Number of Joints: " << _gl_robot->get_num_joints() <<endl;
      _gl_robot->set_jointdof_marker_filter(_jointdof_filter_list);
      _gl_robot->disable_joint_limit_enforcement();
      _gl_robot->enable_joint_limit_enforcement_for_future_state(); // so that posture markers work within limits.
      
      //remember that we've parsed the urdf already
      _urdf_parsed = true;
    }
 
  } // end urdf handler


