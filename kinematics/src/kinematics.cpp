/*********************************************************************
  performs the calculations gravity compensation using the KDL library
 *********************************************************************/

#include <cstring>
#include <ros/ros.h>
#include <kinematics.h>

sensor_msgs::JointState jstate_msg;
baxter_core_msgs::SEAJointState arm_gravity;

namespace arm_kinematics 
{

Kinematics::Kinematics()
    : nh_private("~") {
}

bool Kinematics::init_grav() 
{

  std::string urdf_xml, full_urdf_xml;
  nh.param("urdf_xml", urdf_xml, std::string("robot_description"));
  nh.searchParam(urdf_xml, full_urdf_xml);
  ROS_DEBUG("Reading xml file from parameter server");
  std::string result;
  if (!nh.getParam(full_urdf_xml, result)) {
    ROS_FATAL("Could not load the xml from parameter server: %s",
              urdf_xml.c_str());
    return false;
  }

  if (!nh.getParam("root_name", root_name)) {
    ROS_FATAL("GenericIK: No root name for gravity found on parameter server");
    return false;
  }

  if (!nh.getParam("grav_arm_name", grav_arm_name)) {
    ROS_FATAL("GenericIK: No grav_arm_name name for gravity found on parameter server");
    return false;
  }

  //Service client to set the gravity false for the limbs
  ros::ServiceClient get_lp_client = nh.serviceClient<gazebo_msgs::GetLinkProperties>("/gazebo/get_link_properties");

  //Wait for service to become available
  get_lp_client.waitForExistence();

  //Service client to set the gravity false for the limbs
  ros::ServiceClient set_lp_client = nh.serviceClient<gazebo_msgs::SetLinkProperties>("/gazebo/set_link_properties");

  //Wait for service to become availablestd::find(vector.begin(), vector.end(), item)!=vector.end()
  set_lp_client.waitForExistence();

  gazebo_msgs::SetLinkProperties setlinkproperties;
  gazebo_msgs::GetLinkProperties getlinkproperties;

  setlinkproperties.request.gravity_mode=0;

  //Load the arm chain and copy them to Right specific variable
  tip_name = grav_arm_name;
  if (!loadModel(result)) {
    ROS_FATAL("Could not load models!");
    return false;
  }

  grav_chain_arm = chain;
  arm_joint.clear();
  arm_joint.reserve(chain.getNrOfSegments());
  std::vector<std::string>::iterator idx;
  //Update the arm_joint with the fixed joints from the URDF. Get each of the link's properties from GetLinkProperties service and
  //call the SetLinkProperties service with the same set of parameters except for the gravity_mode, which would be disabled. This is
  //to disable the gravity in the links, thereby to eliminate the need for gravity compensation
  for (int i = 0; i < chain.getNrOfSegments(); i++) 
  {
    std::string seg_name = chain.getSegment(i).getName();
    std::string joint_name = chain.getSegment(i).getJoint().getName();
    idx = std::find(info.joint_names.begin(), info.joint_names.end(), joint_name);
    if (idx != info.joint_names.end()) {
      arm_joint.push_back(*idx);
    }
    getlinkproperties.request.link_name=chain.getSegment(i).getName();
    std::string link_name = chain.getSegment(i).getName();
    getlinkproperties.request.link_name=link_name;
    setlinkproperties.request.link_name=link_name;
    get_lp_client.call(getlinkproperties);
    setlinkproperties.request.com = getlinkproperties.response.com;
    setlinkproperties.request.mass = getlinkproperties.response.mass;
    setlinkproperties.request.ixx = getlinkproperties.response.ixx;
    setlinkproperties.request.iyy = getlinkproperties.response.iyy;
    setlinkproperties.request.izz = getlinkproperties.response.izz;
    setlinkproperties.request.ixy = getlinkproperties.response.ixy;
    setlinkproperties.request.iyz = getlinkproperties.response.iyz;
    setlinkproperties.request.ixz = getlinkproperties.response.ixz;
    setlinkproperties.request.gravity_mode = true;//false
    set_lp_client.call(setlinkproperties);
  }

  //Create a gravity solver for the left chain
  gravity_solver_arm = new KDL::ChainIdSolver_RNE(grav_chain_arm,
                                                KDL::Vector(0.0, 0.0, -9.8));

  return true;
}

/* Initializes the solvers and the other variables required
 *  @returns true is successful
 */
bool Kinematics::init(std::string tip, int &no_jts) {
  // Get URDF XML
  std::string urdf_xml, full_urdf_xml;
  tip_name = tip;
  nh.param("urdf_xml", urdf_xml, std::string("robot_description"));
  nh.searchParam(urdf_xml, full_urdf_xml);
  ROS_DEBUG("Reading xml file from parameter server");
  std::string result;
  if (!nh.getParam(full_urdf_xml, result)) {
    ROS_FATAL("Could not load the xml from parameter server: %s",
              urdf_xml.c_str());
    return false;
  }

  // Get Root and Tip From Parameter Service
  if (!nh.getParam("root_name", root_name)) {
    ROS_FATAL("GenericIK: No root name found on parameter server");
    return false;
  }

  // Load and Read Models
  if (!loadModel(result)) {
    ROS_FATAL("Could not load models!");
    return false;
  }

  // Get Solver Parameters
  int maxIterations;
  double epsilon;
  //KDL::Vector grav;

  nh_private.param("maxIterations", maxIterations, 1000);
  nh_private.param("epsilon", epsilon, 1e-2);

  // Build Solvers
  fk_solver = new KDL::ChainFkSolverPos_recursive(chain);
  ik_solver_vel = new KDL::ChainIkSolverVel_pinv(chain);
  ik_solver_pos = new KDL::ChainIkSolverPos_NR_JL(chain, joint_min, joint_max,
                                                  *fk_solver, *ik_solver_vel,
                                                  maxIterations, epsilon);
  no_jts=num_joints;
  return true;
}

/* Method to load all the values from the parameter server
 *  @returns true is successful
 */
bool Kinematics::loadModel(const std::string xml) {
  urdf::Model robot_model;
  KDL::Tree tree;
  if (!robot_model.initString(xml)) {
    ROS_FATAL("Could not initialize robot model");
    return -1;
  }
  if (!kdl_parser::treeFromString(xml, tree)) {
    ROS_ERROR("Could not initialize tree object");
    return false;
  }
  if (!tree.getChain(root_name, tip_name, chain)) {
    ROS_ERROR("Could not initialize chain object for root_name %s and tip_name %s",root_name.c_str(), tip_name.c_str());
    return false;
  }
  if (!readJoints(robot_model)) {
    ROS_FATAL("Could not read information about the joints");
    return false;
  }

  return true;
}

/* Method to read the URDF model and extract the joints
 *  @returns true is successful
 */
bool Kinematics::readJoints(urdf::Model &robot_model) {
  num_joints = 0;
  boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
  boost::shared_ptr<const urdf::Joint> joint;
  for (int i = 0; i < chain.getNrOfSegments(); i++)
    while (link && link->name != root_name) {
      if (!(link->parent_joint)) {
        break;
      }
      joint = robot_model.getJoint(link->parent_joint->name);
      if (!joint) {
        ROS_ERROR("Could not find joint: %s", link->parent_joint->name.c_str());
        return false;
      }
      if (joint->type != urdf::Joint::UNKNOWN
          && joint->type != urdf::Joint::FIXED) {
        ROS_INFO("adding joint: [%s]", joint->name.c_str());
        num_joints++;
      }
      link = robot_model.getLink(link->getParent()->name);
    }
  joint_min.resize(num_joints);
  joint_max.resize(num_joints);
  info.joint_names.resize(num_joints);
  info.link_names.resize(num_joints);

  link = robot_model.getLink(tip_name);
  unsigned int i = 0;
  while (link && i < num_joints) {
    joint = robot_model.getJoint(link->parent_joint->name);
    if (joint->type != urdf::Joint::UNKNOWN
        && joint->type != urdf::Joint::FIXED) {
      ROS_INFO("getting bounds for joint: [%s]", joint->name.c_str());

      float lower, upper;
      int hasLimits;
      if (joint->type != urdf::Joint::CONTINUOUS) {
        lower = joint->limits->lower;
        upper = joint->limits->upper;
        hasLimits = 1;
      } else {
        lower = -M_PI;
        upper = M_PI;
        hasLimits = 0;
      }
      int index = num_joints - i - 1;

      joint_min.data[index] = lower;
      joint_max.data[index] = upper;
      info.joint_names[index] = joint->name;
      info.link_names[index] = link->name;
      i++;
    }
    link = robot_model.getLink(link->getParent()->name);
  }
  return true;
}

/* Method to calculate the torques required to apply at each of the joints for gravity compensation
 *  @returns true is successful
 */
bool Kinematics::getGravityTorques(
    const sensor_msgs::JointState joint_configuration, baxter_core_msgs::SEAJointState &arm_gravity, bool isEnabled) 
{

  bool res;
  KDL::JntArray torques_arm;
  KDL::JntArray jntPosIn_arm;
  arm_gravity.name = arm_joint;
  arm_gravity.gravity_model_effort.resize(num_joints);
  if (isEnabled)
  {
    torques_arm.resize(num_joints);
    jntPosIn_arm.resize(num_joints);

    // Copying the positions of the joints relative to its index in the KDL chain
    for (unsigned int j = 0; j < joint_configuration.name.size(); j++) {
      for (unsigned int i = 0; i < num_joints; i++) {
        if (joint_configuration.name[j] == arm_joint.at(i)) {
          jntPosIn_arm(i) = joint_configuration.position[j];
          break;
        }
      }
    }
    KDL::JntArray jntArrayNull(num_joints);
    KDL::Wrenches wrenchNull_arm(grav_chain_arm.getNrOfSegments(),
                               KDL::Wrench::Zero());
    int solver_check = gravity_solver_arm->CartToJnt(jntPosIn_arm, jntArrayNull,
                                             jntArrayNull, wrenchNull_arm,
                                             torques_arm);

    //Check if the gravity was succesfully calculated by both the solvers
    if (solver_check >= 0)
    {
	     for (unsigned int i = 0; i < num_joints; i++) {
            arm_gravity.gravity_model_effort[i] = torques_arm(i);  
          }
      return true;
    } else {
      ROS_ERROR_THROTTLE(
          1.0,
          "KT: Failed to compute gravity torques from KDL return code for left and right arms %d",
          solver_check);
      return false;
    }

  } else {
    for (unsigned int i = 0; i <  num_joints; i++) {
        arm_gravity.gravity_model_effort[i]=0;
    }
  }
  return true;
}

/* Method to calculate the Joint index of a particular joint from the KDL chain
 *  @returns the index of the joint
 */
int Kinematics::getJointIndex(const std::string &name) {
  for (unsigned int i = 0; i < info.joint_names.size(); i++) {
    if (info.joint_names[i] == name)
      return i;
  }
  return -1;
}

/* Method to calculate the KDL segment index of a particular segment from the KDL chain
 *  @returns the index of the segment
 */
int Kinematics::getKDLSegmentIndex(const std::string &name) {
  int i = 0;
  while (i < (int) chain.getNrOfSegments()) {
    if (chain.getSegment(i).getJoint().getName() == name) {
      return i + 1;
    }
    i++;
  }
  return -1;
}

/* Method to calculate the IK for the required end pose
 *  @returns true if successful
 */
bool Kinematics::getPositionIK(
    const geometry_msgs::PoseStamped &pose_stamp,
    const sensor_msgs::JointState &seed, sensor_msgs::JointState *result) 
{

  geometry_msgs::PoseStamped pose_msg_in = pose_stamp;
  tf::Stamped<tf::Pose> transform;
  tf::Stamped<tf::Pose> transform_root;
  tf::poseStampedMsgToTF(pose_msg_in, transform);

  //Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;

  jnt_pos_in.resize(num_joints);
  // Copying the positions of the joints relative to its index in the KDL chain
  for (unsigned int i = 0; i < num_joints; i++) {
    int tmp_index = getJointIndex(seed.name[i]);
    if (tmp_index >= 0) {
      jnt_pos_in(tmp_index) = seed.position[i];
    } else {
      ROS_ERROR("i: %d, No joint index for %s", i, seed.name[i].c_str());
    }
  }

  //Convert F to our root_frame
  try {
    tf_listener.transformPose(root_name, transform, transform_root);
  } catch (...) {
    ROS_ERROR("Could not transform IK pose to frame: %s", root_name.c_str());
    return false;
  }

  KDL::Frame F_dest;
  tf::transformTFToKDL(transform_root, F_dest);

  int ik_valid = ik_solver_pos->CartToJnt(jnt_pos_in, F_dest, jnt_pos_out);

  if (ik_valid >= 0) {
    result->name = info.joint_names;
    result->position.resize(num_joints);
    for (unsigned int i = 0; i < num_joints; i++) {
      result->position[i] = jnt_pos_out(i);
      ROS_DEBUG("IK Solution: %s %d: %f", result->name[i].c_str(), i,
                jnt_pos_out(i));
    }
    return true;
  } else {
    ROS_DEBUG("An IK solution could not be found");
    return false;
  }
}

/* Method to calculate the FK for the required joint configuration
 *  @returns true if successful
 */
bool Kinematics::getPositionFK(
    std::string frame_id, const sensor_msgs::JointState &joint_configuration,
    geometry_msgs::PoseStamped &result) 
{
  KDL::Frame p_out;
  KDL::JntArray jnt_pos_in;
  tf::Stamped<tf::Pose> tf_pose;

  // Copying the positions of the joints relative to its index in the KDL chain
  jnt_pos_in.resize(num_joints);
  for (unsigned int i = 0; i < num_joints; i++) {
    int tmp_index = getJointIndex(joint_configuration.name[i]);
    if (tmp_index >= 0)
      jnt_pos_in(tmp_index) = joint_configuration.position[i];
  }

  int num_segments = chain.getNrOfSegments();
  ROS_DEBUG("Number of Segments in the KDL chain: %d", num_segments);
  if (fk_solver->JntToCart(jnt_pos_in, p_out, num_segments) >= 0) {
    tf_pose.frame_id_ = root_name;
    tf_pose.stamp_ = ros::Time();
    tf::poseKDLToTF(p_out, tf_pose);
    try {
      tf_listener.transformPose(frame_id, tf_pose, tf_pose);
    } catch (...) {
      ROS_ERROR("Could not transform FK pose to frame: %s", frame_id.c_str());
      return false;
    }
    tf::poseStampedTFToMsg(tf_pose, result);
  } else {
    ROS_ERROR("Could not compute FK for endpoint.");
    return false;
  }
  return true;
}


}  //namespace

void update_jnt_st(const sensor_msgs::JointState &msg) 
{
  jstate_msg = msg;
  arm_gravity.actual_position.resize(arm_gravity.name.size());
  arm_gravity.actual_velocity.resize(arm_gravity.name.size());
  arm_gravity.actual_effort.resize(arm_gravity.name.size());
  for (int i = 0; i < msg.name.size(); i++) 
  {
     for (int j=0;j<arm_gravity.name.size();j++) 
     {
       if (msg.name[i] == arm_gravity.name[j]) {
         arm_gravity.actual_position[j] = msg.position[i];
         arm_gravity.actual_velocity[j] = msg.velocity[i];
         arm_gravity.actual_effort[j] = msg.effort[i];
         break;
       }
       
     }
  }
}

int main(int argc, char* argv[]) 
{
  ros::init(argc, argv, "kinematic_node");
  ros::NodeHandle n;
  ros::Subscriber jnt_st;
  jnt_st = n.subscribe("/rrbot/joint_states", 100, &update_jnt_st);

  ros::Publisher arm_grav_pub;
  arm_grav_pub = n.advertise<std_msgs::Float64>("/rrbot/joint2_effort_controller/command", 1);
  std_msgs::Float64 force;

  arm_gravity.header.frame_id="link1";
  arm_gravity.header.frame_id="link1";

  ros::Rate loop_rate(100);
  
  arm_kinematics::Kinematics kin;
  kin.init_grav();

  while(ros::ok())
  {
    kin.getGravityTorques(jstate_msg, arm_gravity, true);
    arm_gravity.header.stamp = ros::Time::now();
    //arm_grav_pub.publish(arm_gravity);
    force.data = arm_gravity.gravity_model_effort[1] + 0.5;
    arm_grav_pub.publish(force);
    ROS_INFO("gravity: [%f], [%f]",arm_gravity.gravity_model_effort[0],arm_gravity.gravity_model_effort[1]);
    ros::spinOnce();
    loop_rate.sleep();
  }

}