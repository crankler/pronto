#ifndef INTERACTABLE_GL_KINEMATIC_BODY_HPP
#define INTERACTABLE_GL_KINEMATIC_BODY_HPP

#include "GlKinematicBody.hpp"
namespace visualization_utils {

 
class InteractableGlKinematicBodyX: public GlKinematicBody 
{

  public:
    std::string _unique_name;
   

  private:   
   std::string selected_link;
   bool _link_selection_enabled;
   bool _whole_body_selection_enabled; 
   
   std::string selected_marker;
   bool _bodypose_adjustment_enabled;
   bool  _floatingbase_marker_drawoffset_enabled; // enabled if we want body pose markers in a frame other than world frame.
   KDL::Frame _T_world_floatingbase_marker;
   bool _jointdof_adjustment_enabled;
   bool _jointdof_markers_initialized;
   std::vector<std::string> _jointdof_marker_filter;
   bool _jointdof_marker_filter_on;
   bool _jointdof_marker_flip_check_done;
   std::vector<bool> _jointdof_marker_flip;
   
   void init_vars(void);
  public:  
     
  private:  
   Eigen::Vector3f _floatingbase_offset; // visual offset of the root link, if any 
   float _floatingbase_markers_boxsize;
   Eigen::Vector2f _floatingbase_markers_torusdims;
   Eigen::Vector3i _marker_dir_flip;
   
  public:
  //copy Constructors
  InteractableGlKinematicBodyX( const InteractableGlKinematicBodyX& other, std::string unique_name); 
  // create                                                  
  InteractableGlKinematicBodyX(std::string urdf_xml_string,
                              bool enable_selection, std::string unique_name);
  ~InteractableGlKinematicBodyX();

   void set_state(const bot_core::robot_state_t &msg); 
   

   
   // overloaded from GLKinematicBody They call update functions for marker collision objects
   void set_future_state(const bot_core::robot_state_t &msg);
   void set_future_state(const KDL::Frame &T_world_body, std::map<std::string, double> &jointpos_in);

   // double c[3] = {0.3,0.3,0.3};
   // double alpha = self->alpha;


   bool draw_mesh(int linkType);
   void draw_body (float (&c)[3], float alpha);  
   void draw_body_in_frame (float (&c)[3], float alpha,const KDL::Frame &T_drawFrame_currentWorldFrame);
   
  // Interactive markers
   void init_floatingbase_marker_collision_objects(); // requires  as FK needs to be solved atleast once to derive. Called inside setstate.
   void update_floatingbase_marker_collision_objects();
   void draw_floatingbase_markers();
   
   void init_jointdof_marker_collision_objects(); // requires  as FK needs to be solved atleast once to derive. Called inside setstate.
   void update_jointdof_marker_collision_objects();
   void draw_jointdof_markers();

   void draw_interactable_markers(boost::shared_ptr<urdf::Geometry> &_link_shape,const LinkFrameStruct &link_tf);
   
   void enable_bodyorparent_frame_rendering_of_floatingbase_markers(bool value)   {
    // body frame is used in draw_body usage
    // parent frame is used in draw_body_in_frame usage
       _floatingbase_marker_drawoffset_enabled = value; 
   };
   
   bool is_bodyorparent_frame_rendering_of_floatingbase_markers_enabled()   {
       return _floatingbase_marker_drawoffset_enabled; 
   };   

   void set_floatingbasemarker_frame(KDL::Frame &T_currentWorldFrame_markerframe){
      if(!_floatingbase_marker_drawoffset_enabled){
        _floatingbase_marker_drawoffset_enabled = true; 
      }        
      _T_world_floatingbase_marker = T_currentWorldFrame_markerframe;
   };
   
   KDL::Frame get_floatingbasemarker_frame(){
      if(!_floatingbase_marker_drawoffset_enabled)
       return KDL::Frame::Identity();
     return _T_world_floatingbase_marker; // returns T_world_marker
   }
   
   void enable_link_selection(bool value)   {
       _link_selection_enabled = value; 
   };   
   void highlight_link(std::string &link_name)   {
       selected_link = link_name; 
   };  
   
   void highlight_body(std::string &body_name)   {
       selected_link = body_name; 
   };   
    
   void highlight_marker(std::string &marker_name)   {
       selected_marker = marker_name; 
   };   
   void enable_bodypose_adjustment(bool value)   { 
    _bodypose_adjustment_enabled = value;
    if(value)
    {
      _jointdof_adjustment_enabled = false; // mutually exclusive
      if(_root_name=="world"){
        std::cerr << "ERROR: root link pose cannot be adjusted as it is fixed to the world. Enabling jointdof adjusment instead" << std::endl;
        enable_jointdof_adjustment(true);   
       }
    }
   };
   void enable_jointdof_adjustment(bool value)   { 
    _jointdof_adjustment_enabled = value;
    if(value){
      _bodypose_adjustment_enabled = false; // mutually exclusive
    }
   };
   
   bool set_jointdof_marker_filter(std::vector<std::string> joint_filter_list){

    // go through joint name list and verufy that everything is ok.
    std::vector<std::string>::const_iterator found;
    for(size_t i=0;i<_jointdof_marker_filter.size();i++) 
    {
       found = std::find (_joint_names.begin(), _joint_names.end(),_jointdof_marker_filter[i]);
       if(found == _link_geometry_names.end());
       {
         std::cerr << "In InteractableGlKinematicBodyX::set_jointdof_marker_filter: " <<_jointdof_marker_filter[i] << " does not exist in _joint_names\n";
        return false;
       }
    }
    _jointdof_marker_filter.clear();
    _jointdof_marker_filter = joint_filter_list;
    _jointdof_marker_filter_on = true;
    return true;
   };
   

   
   bool is_bodypose_adjustment_enabled()   { 
    return _bodypose_adjustment_enabled;
   };
   bool is_jointdof_adjustment_enabled()   { 
    return _jointdof_adjustment_enabled;
   };
   void enable_whole_body_selection(bool value)   { 
    _whole_body_selection_enabled = value;
   };
   
  bool is_joint_axis_flipped(std::string &joint_name)  
  {
    if(!_jointdof_adjustment_enabled)
       return false;
  
    std::vector<std::string>::const_iterator found;
    found = std::find (_joint_names.begin(), _joint_names.end(), joint_name);
    if (found != _joint_names.end()) {
      unsigned int index = found - _joint_names.begin();
      return _jointdof_marker_flip[index];
    }
    else
    {
       return false;
    }
  };
   
   void flip_trans_marker_xdir(bool value)   { 

     if(value)
      _marker_dir_flip[0] = -1;
    else
     _marker_dir_flip[0] = 1;
   };
   void flip_trans_marker_ydir(bool value)   { 
     if(value)
      _marker_dir_flip[1] = -1;
    else
     _marker_dir_flip[1] = 1;
   };
   void flip_trans_marker_zdir(bool value)   { 
     if(value)
      _marker_dir_flip[2] = -1;
    else
     _marker_dir_flip[2] = 1;
   };
   
   // support for plane markers for planar motion
   public:
   
    bool modify_joint_axis_to_plane_normal(const std::string &joint_name,Eigen::Vector3f &joint_axis);
    

    bool is_planar_coupling_active(const std::string &jointorlink_name)
    {
        std::string token  = "plane::";
        size_t found = jointorlink_name.find(token); 
        return (found!=std::string::npos);
    };
    
    bool extract_plane_name(const std::string &jointorlink_name,std::string &plane_name)
    {
        plane_name = " ";
        std::string token  = "plane::";
        std::string separator  = "::";
        size_t found = jointorlink_name.find(token); 
        if (found!=std::string::npos)
        {
          size_t found2 = jointorlink_name.find(separator,found+token.size()); 
          plane_name = jointorlink_name.substr(found+token.size(),found2-(found+token.size()));
          return true;
        }
        return false;
    };
    
    bool is_first_axis_in_plane(const std::string &joint_name)
    {
      std::string plane_name,axis_name;
      extract_plane_name(joint_name,plane_name);
      std::string token  = "plane::" + plane_name;
      std::string separator  = "::";
      size_t found = joint_name.find(token); 
      size_t found2 = joint_name.find(separator,found+token.size()); 
      axis_name = joint_name.substr(found2+separator.size());
      size_t found3 = plane_name.find(axis_name); 
      if ((found3!=std::string::npos)&&(found3==0))
      {
       return true;
      }
      return false;
    };

    bool is_second_axis_in_plane(const std::string &joint_name)
    {
      std::string plane_name,axis_name;
      extract_plane_name(joint_name,plane_name);
      std::string token  = "plane::" + plane_name;
      std::string separator  = "::";
      size_t found = joint_name.find(token); 
      size_t found2 = joint_name.find(separator,found+token.size()); 
      axis_name = joint_name.substr(found2+separator.size());
      size_t found3 = plane_name.find(axis_name); 
      if ((found3!=std::string::npos)&&(found3==1))
      {
       return true;
      }
      return false;
    };
    
    bool get_first_axis_name(const std::string &jointormarker_name,std::string &axis_name)
    {
    
        std::string token  = "markers::";
        size_t found = jointormarker_name.find(token);  
        std::string joint_name(jointormarker_name);
        if(found!=std::string::npos)
          joint_name =jointormarker_name.substr(found+token.size());
        std::string separator  = "::";
        std::string plane_name;
        extract_plane_name(joint_name,plane_name);
        found = joint_name.find_last_of(separator);
        if (found!=std::string::npos)
        {
         axis_name=joint_name.substr(0,found+1) +plane_name[0];
         return true;
        }
        return false;
    };
    
    bool get_second_axis_name(const std::string &jointormarker_name,std::string &axis_name)
    {
        std::string token  = "markers::";
        size_t found = jointormarker_name.find(token);  
        std::string joint_name(jointormarker_name);
        if(found!=std::string::npos)
          joint_name =jointormarker_name.substr(found+token.size());
        std::string separator  = "::";
        std::string plane_name;
        extract_plane_name(joint_name,plane_name);
        found = joint_name.find_last_of(separator);
        if (found!=std::string::npos)
        {
         axis_name=joint_name.substr(0,found+1) +plane_name[1];
         return true;
        }
        return false;
    };
    
      
    bool get_plane_axis(const std::string &plane_name,Eigen::Vector3f& plane_axis)
    {
      std::string first_axis_name,second_axis_name;
      first_axis_name=plane_name[0];
      second_axis_name=plane_name[1];

      Eigen::Vector3f first_axis,second_axis;
      if(first_axis_name=="x")
        first_axis << 1,0,0;
      else if(first_axis_name=="y")
         first_axis << 0,1,0;
      else if(first_axis_name=="z")
         first_axis << 0,0,1; 
      else {
        std::cerr<<"ERROR: unknown axis name (must be x,y,or z) : " << first_axis_name << std::endl;
        return false;   
      }  
      
      if(second_axis_name=="x")
        second_axis << 1,0,0;
      else if(second_axis_name=="y")
         second_axis << 0,1,0;
      else if(second_axis_name=="z")
         second_axis << 0,0,1; 
      else {
        std::cerr<<"ERROR: unknown axis name (must be x,y,or z) : " << second_axis_name << std::endl;
        return false;   
      }      
         
      plane_axis = first_axis.cross(second_axis);
      return true;
    };      
    
};

} // end namespace 




#endif //INTERACTABLE_GL_KINEMATIC_BODY_HPP
