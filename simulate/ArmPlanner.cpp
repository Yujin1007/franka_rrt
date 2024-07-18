#include "ArmPlanner.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
FILE *fp1 = fopen("/home/kendrick/catkin_ws/src/position_data.txt","w");

#define DEGREE M_PI/180.0

using std::ofstream;


ArmPlanner::ArmPlanner(YAMLConfig yaml_config):
config_(yaml_config)
{

}
ArmPlanner::~ArmPlanner()
{
}
void ArmPlanner::initModel()
{

	std::string urdf_absolute_path;
	std::string mod_url = config_.urdf_path;
	// if (config_.urdf_path.find("package://") == 0)
	// {
	// 	mod_url.erase(0, strlen("package://"));
	// 	size_t pos = mod_url.find("/");
	// 	if (pos == std::string::npos)
	// 	{
	// 		cout << "Could not parse package:// format into file:// format" << endl;;
	// 	}
	// 	std::string package = mod_url.substr(0, pos);
	// 	mod_url.erase(0, pos);
	// 	std::string package_path ;//= ros::package::getPath(package);

	// 	if (package_path.empty())
	// 	{
	// 		cout << "Package does not exist" << endl;;
	// 	}

	// 	urdf_absolute_path =  package_path + mod_url;
	// }

	RigidBodyDynamics::Addons::URDFReadFromFile(mod_url.c_str(), &rbdl_model_, false, false);

	//nb_of_joints_ = rbdl_model_.q_size;
	nb_of_joints_ = config_.joint_limit_lower.size(); // dof for using planning
	// Fixed Body : The value of max(unsigned int) is
	// * determined via std::numeric_limits<unsigned int>::max() and the
	// * default value of fixed_body_discriminator is max (unsigned int) / 2.
	rrt_.dofSize = nb_of_joints_;

	// rbdl이 상체 모든 걸 다 받아버리면 말단 장치 위치 계산할때 모든 관절의 위치를 알고 있어야함.
	end_effector_id_ = rbdl_model_.GetBodyId((config_.chain_end).c_str());
	arm_base_frame_id_ = rbdl_model_.GetBodyId((config_.chain_start).c_str());

	if (rbdl_model_.IsFixedBodyId(end_effector_id_))
	{
		end_effector_com_ = rbdl_model_.mFixedBodies[end_effector_id_ - rbdl_model_.fixed_body_discriminator].mCenterOfMass;
	}
	else
	{
		end_effector_com_ = rbdl_model_.mBodies[end_effector_id_].mCenterOfMass;
	}

	if (rbdl_model_.IsFixedBodyId(arm_base_frame_id_))
	{
		arm_base_frame_pos_ = rbdl_model_.mFixedBodies[arm_base_frame_id_ - rbdl_model_.fixed_body_discriminator].mCenterOfMass;
	}
	else
	{
		arm_base_frame_pos_ = rbdl_model_.mBodies[arm_base_frame_id_].mCenterOfMass;
	}

	// end_effector_com_ : 0.0, 0.0, 0.0 for robocare

	//cout << "end_effector_com_" << end_effector_com_.transpose() << endl;
}
void ArmPlanner::initializeData(VectorXd q_init_, VectorXd q_goal_) {

	//cout << "nb_of_joints_" << nb_of_joints_ <<endl;
	//cout << "config_.joint_limit_lower.size()" << config_.joint_limit_lower.size()<< endl;

	// joint limit 
	//assert(nb_of_joints_ == config_.joint_limit_lower.size());
	//assert(nb_of_joints_ == config_.joint_limit_upper.size());
	joint_limit_.lower_.resize(nb_of_joints_);
	joint_limit_.lower_rad_.resize(nb_of_joints_);
	joint_limit_.upper_.resize(nb_of_joints_);
	joint_limit_.upper_rad_.resize(nb_of_joints_);

	for (std::vector<int>::size_type i=0;i<config_.joint_limit_lower.size();i++){
		joint_limit_.lower_(i) = config_.joint_limit_lower[i];
		joint_limit_.upper_(i) = config_.joint_limit_upper[i];
	}
	
	joint_limit_.lower_rad_ = joint_limit_.lower_ / 180.0*M_PI;
	joint_limit_.upper_rad_ = joint_limit_.upper_ / 180.0*M_PI;

	// current joint position
	joint_state_.qInit_.resize(nb_of_joints_);
	joint_state_.qGoal_.resize(nb_of_joints_);

	joint_state_.qInit_ = q_init_;
	joint_state_.qGoal_ = q_goal_;

    output_arm_trajectory_.clear();

	arm_trajectory_point_.setZero(nb_of_joints_);

	// current base position
	// mobile_pose_.qInit_.resize(3);
	// mobile_pose_.qInit_(0) = req.current_mobile_state.x;
	// mobile_pose_.qInit_(1) = req.current_mobile_state.y;
	// mobile_pose_.qInit_(2) = req.current_mobile_state.theta;
	// mobile_pose_.rotInit_.setIdentity();

	// // initial pose &  target pose in Global frame
	// init_pose_.translation().head(2) = mobile_pose_.qInit_.head(2);
	// init_pose_.translation()(2) = 0.0;
	// //init_pose_.translation() += mobile_pose_.rotInit_ * base_frame_pos_;
	// init_pose_.translation() += mobile_pose_.rotInit_ * CalcBodyToBaseCoordinates(rbdl_model_, joint_state_.qInit_, end_effector_id_, end_effector_com_, true);
	// init_pose_.linear() = CalcBodyWorldOrientation(rbdl_model_ , joint_state_.qInit_, end_effector_id_, true).transpose();
	// cout << "init_pose_.translation() \n" << init_pose_.translation().transpose() << endl;
	// cout << "init_pose_ \t" <<init_pose_.linear() << endl;

	//cout << "target_pose_.linear() \t" << target_pose_.linear() << endl;
	//cout << "target_pose_.translation()" << target_pose_.translation().transpose() << endl;

	//Vector3d from_init_to_goal_in_local = mobile_pose_.rotInit_.transpose()*(init_pose_.translation() - target_pose_.translation());
	obs_list_.clear();
	obstacle obs_floor_;
	obs_floor_.dim_(0) = 0.5;
	obs_floor_.dim_(1) = 0.5;
	obs_floor_.dim_(2) = 0.01;

	obs_floor_.pose_.translation() = Eigen::Vector3d(0, 0, -0.02);
	obs_floor_.pose_.linear().setIdentity();

	obs_list_.push_back(obs_floor_);
	obs_list_.push_back(obs_valve_);
	// Obstacles 
	rrt_.box_num_obs = obs_list_.size();
	rrt_.Box_env.clear();
	rrt_.Box_robot.clear();
	
	// for (int i=0;i<rrt_.box_num_obs;i++)
	// {
	// 	Eigen::Quaterniond quat_box(req.Obstacles3D[i].Box_pose.orientation.w, req.Obstacles3D[i].Box_pose.orientation.x, req.Obstacles3D[i].Box_pose.orientation.y, req.Obstacles3D[i].Box_pose.orientation.z);
	// 	Matrix3d Box_rot_temp = quat_box.toRotationMatrix();
	// 	rrt_.Box_obs[i].fAxis = Vector3d(req.Obstacles3D[i].Box_dimension.x, req.Obstacles3D[i].Box_dimension.y, req.Obstacles3D[i].Box_dimension.z);
	// 	rrt_.Box_obs[i].vAxis[0] = mobile_pose_.rotInit_.transpose() * Box_rot_temp.col(0);
	// 	rrt_.Box_obs[i].vAxis[1] = mobile_pose_.rotInit_.transpose() * Box_rot_temp.col(1);
	// 	rrt_.Box_obs[i].vAxis[2] = mobile_pose_.rotInit_.transpose() * Box_rot_temp.col(2);
	// 	rrt_.Box_obs[i].vPos =  mobile_pose_.rotInit_.transpose()* Vector3d(req.Obstacles3D[i].Box_pose.position.x - mobile_pose_.qInit_(0), req.Obstacles3D[i].Box_pose.position.y - mobile_pose_.qInit_(1), req.Obstacles3D[i].Box_pose.position.z); // axis center pos
	// }

	for (int i = 0; i < rrt_.box_num_obs; i++)
	{
		ST_OBB box;
		box.fAxis = obs_list_[i].dim_;
		box.vAxis[0] = obs_list_[i].pose_.linear().col(0);
		box.vAxis[1] = obs_list_[i].pose_.linear().col(1);
		box.vAxis[2] = obs_list_[i].pose_.linear().col(2);
		box.vPos = obs_list_[i].pose_.translation();
		rrt_.Box_env.push_back(box);
	}

	// for (int i=0;i<rrt_.box_num_obs;i++)
	// 	cout << rrt_.Box_env[i].vPos << endl;

	// cout << rrt_.Box_env.size() << endl;
	// cout << rrt_.Box_env[0].fAxis << endl;
	// cout << rrt_.Box_env[1].fAxis << endl;

	// link data (link dimension)
	// 킬때마다 새로 실행됨...
	// rrt_.box_num_link = config_.link_name.size();
	// for (int i=0;i<rrt_.box_num_link;i++){
	// 	Vector3d link_dim;
	// 	for (int j=0;j<config_.link_dimension[i].size();j++){
	// 		link_dim(j) =  config_.link_dimension[i][j];
	// 		rrt_.Box_link[i].vCenter(j) = config_.link_position[i][j];
	// 	}
	// 	rrt_.Box_link[i].vRot = rotateXaxis(config_.link_orientation[i][0]/180.0*M_PI)*rotateYaxis(config_.link_orientation[i][1]/180.0*M_PI)*rotateZaxis(config_.link_orientation[i][2]/180.0*M_PI);
	// 	rrt_.Box_link[i].fAxis = link_dim;
	// }
	
	rrt_.box_num_link = config_.link_name.size();
	for (int i=0;i<rrt_.box_num_link;i++){
		ST_OBB box2;
		Vector3d link_dim;
		for (int j = 0; j < config_.link_dimension[i].size(); j++)
		{
			link_dim(j) = config_.link_dimension[i][j];
			box2.vCenter(j) = config_.link_position[i][j];
		}

		box2.vRot = rotateXaxis(config_.link_orientation[i][0]/180.0*M_PI)*rotateYaxis(config_.link_orientation[i][1]/180.0*M_PI)*rotateZaxis(config_.link_orientation[i][2]/180.0*M_PI);
		box2.fAxis = link_dim;

		rrt_.Box_robot.push_back(box2);
	}
	//for (int i=0;i<rrt_.box_num_link;i++)
		//cout << rrt_.Box_robot[i].fAxis << endl;

	//////////////////////////////// Attach Box to Robot ////////////////////////////////
	Matrix3d ee_rot;
	ee_rot.setZero();
	ee_rot(0,0) = -1.0;
	ee_rot(1,2) = 1.0;
	ee_rot(2,1) = 1.0;

	// if (req.AttachBox.Attach_to_robot.data == true)
	// {
	// 	// Remove the information about last 2 links(Finger links)
	// 	rrt_.Box_robot.pop_back();
	// 	rrt_.Box_robot.pop_back();
	// 	config_.link_dimension.pop_back();
	// 	config_.link_dimension.pop_back();
	// 	config_.link_name.pop_back();
	// 	config_.link_name.pop_back();

	// 	// Convert global pose to local
	// 	Matrix3d global_to_local_ee = (init_pose_.linear() * ee_rot.transpose()).transpose() * mobile_pose_.rotInit_.transpose();
	// 	Vector3d local_position_ = global_to_local_ee * Vector3d(req.AttachBox.Box_pose.position.x - init_pose_.translation()(0), req.AttachBox.Box_pose.position.y - init_pose_.translation()(1), req.AttachBox.Box_pose.position.z - init_pose_.translation()(2));
	// 	attached_box_pose_.translation() = local_position_;
	// 	Eigen::Quaterniond quat_attached_box(req.AttachBox.Box_pose.orientation.w, req.AttachBox.Box_pose.orientation.x, req.AttachBox.Box_pose.orientation.y, req.AttachBox.Box_pose.orientation.z);
	// 	attached_box_pose_.linear() = (init_pose_.linear() * ee_rot.transpose()).transpose() * mobile_pose_.rotInit_.transpose() * quat_attached_box.toRotationMatrix();
	// 	std::vector<double> fake_link_dim;
	// 	fake_link_dim.clear();
	// 	fake_link_dim.push_back(0.1);
	// 	fake_link_dim.push_back(0.1);
	// 	fake_link_dim.push_back(0.1);

	// 	config_.link_dimension.push_back(fake_link_dim);
	// 	ST_OBB attached_box;
	// 	attached_box.fAxis(0) = req.AttachBox.Box_dimension.x;
	// 	attached_box.fAxis(1) = req.AttachBox.Box_dimension.y;
	// 	attached_box.fAxis(2) = req.AttachBox.Box_dimension.z;
	// 	attached_box.vRot = attached_box_pose_.linear();

	// 	if (req.AttachBox.Right_arm.data == true)
	// 	{
	// 		std::string link_name = "RWrist_Roll";
	// 		config_.link_name.push_back(link_name);
	// 		attached_box.vCenter(0) = attached_box_pose_.translation()(0); 
	// 		attached_box.vCenter(1) = attached_box_pose_.translation()(1) - 0.075; 
	// 		attached_box.vCenter(2) = attached_box_pose_.translation()(2); 
	// 		rrt_.box_num_link = config_.link_name.size();
	// 	}
	// 	else
	// 	{
	// 		std::string link_name = "LWrist_Roll";
	// 		config_.link_name.push_back(link_name);
	// 		attached_box.vCenter(0) = attached_box_pose_.translation()(0); 
	// 		attached_box.vCenter(1) = attached_box_pose_.translation()(1) + 0.075; 
	// 		attached_box.vCenter(2) = attached_box_pose_.translation()(2);
	// 		rrt_.box_num_link = config_.link_name.size();
	// 	}
	// 	rrt_.Box_robot.push_back(attached_box);
	// }

	////////////////////////////////////////////////////////////////////////////////////////////
	//this->initializeIKparam(config_.chain_start, config_.chain_end, urdf_param_);

	//////////////////////////////// Comparison between RBDL and KDL forward kinematics
	// Eigen::Isometry3d pose_from_rbdl;
	// pose_from_rbdl.translation() = CalcBodyToBaseCoordinates(rbdl_model_, joint_state_.qInit_, arm_base_frame_id_, arm_base_frame_pos_, true);
	// pose_from_rbdl.linear() = CalcBodyWorldOrientation(rbdl_model_ , joint_state_.qInit_, arm_base_frame_id_, true).transpose();

	// Eigen::Isometry3d init_pose_ee;
	// init_pose_ee.translation() = CalcBodyToBaseCoordinates(rbdl_model_, joint_state_.qInit_, end_effector_id_, end_effector_com_, true);
	// init_pose_ee.linear() = CalcBodyWorldOrientation(rbdl_model_ , joint_state_.qInit_, end_effector_id_, true).transpose();

	// cout << "pose_from_rbdl.translation() \n" << pose_from_rbdl.translation().transpose() << endl;
	// cout << "pose_from_rbdl \n" <<pose_from_rbdl.linear() << endl;	

	// Eigen::Isometry3d pose_from_base = pose_from_rbdl.inverse()*init_pose_ee;

	// cout << "pose_from_base.translation() \n" << pose_from_base.translation().transpose() << endl;
	// cout << "pose_from_base \n" <<pose_from_base.linear() << endl;	
	// KDL::ChainFkSolverPos_recursive fk_solver(IK_chain);									  // Forward kin. solver

	// KDL::JntArray jarr_q;
	// KDL::Frame frame;
	// jarr_q.resize(IK_chain.getNrOfJoints());
	// jarr_q.data = joint_state_.qInit_.tail(7);
	// fk_solver.JntToCart(jarr_q, frame);

  	// Eigen::Isometry3d transform;
  	// transform.translation() = getEigenVector(frame.p);
  	// transform.linear() = getEigenRotation(frame.M);
	  
	// cout << "trac ik frame" << endl;
	// cout << transform.translation().transpose() << endl;
	// cout << transform.linear() << endl;
	////////////////////////////////////////////////////////////////////////////////////////////

	// Constraint bound
	constrain_pose_ = false;
	rrt_.C.resize(6,2);
	// for (int i = 0; i < 3; i++)
	// {
	// 	if (from_init_to_goal_in_local(i) > 0)
	// 	{
	// 		rrt_.C(i, 0) = from_init_to_goal_in_local(i) + 0.05;
	// 		rrt_.C(i, 1) = -0.05;
	// 	}
	// 	else
	// 	{
	// 		rrt_.C(i, 0) = 0.05;
	// 		rrt_.C(i, 1) = from_init_to_goal_in_local(i) - 0.05;
	// 	}
	// }

	// rrt_.C(0,0) = req.Pose_bound.position_bound_upper.x;
	// rrt_.C(1,0) = req.Pose_bound.position_bound_upper.y;
	// rrt_.C(2,0) = req.Pose_bound.position_bound_upper.z;

	// rrt_.C(0,1) = req.Pose_bound.position_bound_lower.x;
	// rrt_.C(1,1) = req.Pose_bound.position_bound_lower.y;
	// rrt_.C(2,1) = req.Pose_bound.position_bound_lower.z;

	// rrt_.C(3,0) = req.Pose_bound.orientation_bound_upper.x;
	// rrt_.C(4,0) = req.Pose_bound.orientation_bound_upper.y;
	// rrt_.C(5,0) = req.Pose_bound.orientation_bound_upper.z;

	// rrt_.C(3,1) = req.Pose_bound.orientation_bound_lower.x;
	// rrt_.C(4,1) = req.Pose_bound.orientation_bound_lower.y;
	// rrt_.C(5,1) = req.Pose_bound.orientation_bound_lower.z;


	// Trajectory library 
	interpolate_path_ = false;
	maxVelocity.resize(nb_of_joints_);
	maxVelocity.setZero();
	maxAcceleration.resize(nb_of_joints_);
	maxAcceleration.setZero();

	for (size_t i = 0; i < nb_of_joints_; i++)
	{
		maxAcceleration(i) = 10.0;
		maxVelocity(i) = 10.0;
	}
	wayPoints.clear();
	playTime_ = 0.0;
}


// bool ArmPlanner::initializeIKparam(const std::string& base_link, const std::string& tip_link, const std::string& URDF_param){
// 	ros::NodeHandle node_handle("~");

// 	std::string xml_string;

// 	std::string urdf_xml, full_urdf_xml;
// 	node_handle.param("urdf_xml", urdf_xml, URDF_param);
// 	node_handle.searchParam(urdf_xml, full_urdf_xml);

// 	//ROS_DEBUG_NAMED("IK", "Reading xml file from parameter server");
// 	if (!node_handle.getParam(full_urdf_xml, xml_string))
// 	{
// 		ROS_INFO("Could not load the xml from parameter server");
// 		return false;
// 	}

// 	node_handle.param(full_urdf_xml, xml_string, std::string());
// 	IK_robot_model.initString(xml_string);

// 	//ROS_DEBUG_STREAM_NAMED("trac_ik", "Reading joints and links from URDF");

// 	if (!kdl_parser::treeFromUrdfModel(IK_robot_model, IK_tree))
// 		ROS_INFO("Failed to extract kdl tree from xml robot description");

// 	if (!IK_tree.getChain(base_link, tip_link, IK_chain))
// 		ROS_INFO("Couldn't find chain");

// 	std::vector<KDL::Segment> chain_segs = IK_chain.segments;

// 	urdf::JointConstSharedPtr joint;

// 	IK_lb.resize(IK_chain.getNrOfJoints());
// 	IK_ub.resize(IK_chain.getNrOfJoints());

// 	uint joint_num = 0;
// 	for (unsigned int i = 0; i < chain_segs.size(); ++i)
// 	{
// 		joint = IK_robot_model.getJoint(chain_segs[i].getJoint().getName());
// 		if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
// 		{
// 			joint_num++;
// 			float lower, upper;
// 			int hasLimits;
// 			if (joint->type != urdf::Joint::CONTINUOUS)
// 			{
// 				if (joint->safety)
// 				{
// 					lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
// 					upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
// 				}
// 				else
// 				{
// 					lower = joint->limits->lower;
// 					upper = joint->limits->upper;
// 				}
// 				hasLimits = 1;
// 			}
// 			else
// 			{
// 				hasLimits = 0;
// 			}
// 			if (hasLimits)
// 			{
// 				IK_lb(joint_num - 1) = lower;
// 				IK_ub(joint_num - 1) = upper;
// 			}
// 			else
// 			{
// 				IK_lb(joint_num - 1) = std::numeric_limits<float>::lowest();
// 				IK_ub(joint_num - 1) = std::numeric_limits<float>::max();
// 			}
// 			//ROS_DEBUG_STREAM_NAMED("trac_ik", "IK Using joint " << joint->name << " " << lb(joint_num - 1) << " " << ub(joint_num - 1));
// 		}
// 	}
// }

void ArmPlanner::compute(std::vector<VectorXd>& arm_trajectory)
{
	// Get body Id of Link 
	body_id_collision_.clear();
	body_com_position_.clear();

	for (std::vector<int>::size_type i = 0; i < config_.link_dimension.size(); i++)
	{
		body_id_collision_.push_back(rbdl_model_.GetBodyId(config_.link_name[i].c_str()));
		body_com_position_.push_back(rbdl_model_.mBodies[rbdl_model_.GetBodyId(config_.link_name[i].c_str())].mCenterOfMass);
	};

	// push back end_effector
	body_id_collision_.push_back(end_effector_id_);
	body_com_position_.push_back(end_effector_com_);
	// Write the model data
	rrt_model_.model_ = rbdl_model_;
	rrt_model_.body_id_vec.clear();
	rrt_model_.body_id_vec.assign(body_id_collision_.begin(), body_id_collision_.end());
	rrt_model_.body_com_pos.clear();
	rrt_model_.body_com_pos.assign(body_com_position_.begin(), body_com_position_.end());


	if (setupRRT(joint_state_.qInit_, joint_state_.qGoal_, joint_target_))
	{
		if (interpolate_path_)
		{
			//output_torso_trajectory_.points.clear();
			output_arm_trajectory_.clear();
			for (int i = 0; i < joint_target_.rows(); i++)
			{
				wayPoints.push_back(joint_target_.row(i));
			}
			trajectory_generator_ = new Trajectory(Path(wayPoints, 0.1), maxVelocity, maxAcceleration);
			//	trajectory_generator_->outputPhasePlaneTrajectory();
			duration_ = trajectory_generator_->getDuration();
			// cout <<"duration" << duration_ << endl;
			while (playTime_ / 10.0 < duration_)
			{
				for (int i = 0; i < nb_of_joints_; i++)
				{
					arm_trajectory_point_(i) = trajectory_generator_->getPosition(playTime_ / 10.0)[i];
				}

				output_arm_trajectory_.push_back(arm_trajectory_point_);

				Vector3d ee_pos = CalcBodyToBaseCoordinates(rbdl_model_, trajectory_generator_->getPosition(playTime_ / 10.0) / 180.0 * M_PI, end_effector_id_, end_effector_com_, true);
				//cout << ee_pos.transpose() << endl;
				playTime_++;
			}
		}
		else
		{
			output_arm_trajectory_.clear();
			for (int i = 0; i < joint_target_.rows(); i++)
			{
				// q_tra = joint_target_.row(i) / 180.0 * M_PI;
				//  ee_pos = CalcBodyToBaseCoordinates(rbdl_model_, q_tra, end_effector_id_, end_effector_com_, true);
				//  cout << ee_pos.transpose() << endl;
				// cout << "Path\t" << i << "\t"
				// 	 << "rad:" << joint_target_.row(i) / 180.0 * M_PI << endl;
				// cout << "Path\t" << i << "\t"
				// 	 << "angle:" << joint_target_.row(i) << endl;
				// fprintf(fp1, "%f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t \n", joint_target2_(i, 0), joint_target2_(i, 1), joint_target2_(i, 2), joint_target2_(i, 3), joint_target2_(i, 4), joint_target2_(i, 5), joint_target2_(i, 6), ee_pos(0), ee_pos(1), ee_pos(2));

				for (int j = 0; j < nb_of_joints_; j++)
					arm_trajectory_point_(j) = joint_target_(i, j) / 180.0 * M_PI;

				output_arm_trajectory_.push_back(arm_trajectory_point_);
			}
		}
	}
	else
	{
		std::cout <<"2" << std::endl;
		output_arm_trajectory_.clear();
	}
	arm_trajectory = output_arm_trajectory_;
	//return output_arm_trajectory_;
}


// bool ArmPlanner::solveIK(Transform3d pose, Robotmodel& model) // from panda_arm.xacro
// {
// 	double eps = 5e-3;
// 	double num_samples = 100;

// 	// Set up KDL IK
// 	KDL::ChainFkSolverPos_recursive fk_solver(IK_chain);									  // Forward kin. solver
// 	KDL::ChainIkSolverVel_pinv vik_solver(IK_chain);										  // PseudoInverse vel solver
// 	KDL::ChainIkSolverPos_NR_JL kdl_solver(IK_chain, IK_lb, IK_ub, fk_solver, vik_solver, 100, eps); // Joint Limit Solver

// 	// Create Nominal chain configuration midway between all joint limits
// 	KDL::JntArray nominal(IK_chain.getNrOfJoints());
		
// 	KDL::JntArray result;
// 	KDL::Frame end_effector_pose;

// 	Matrix4d target_from_base;
// 	Matrix4d target_global;
// 	Matrix4d base_global;
// 	base_global.setIdentity();
// 	target_from_base.setIdentity();
// 	target_global.setIdentity();

// 	Transform3d pose_from_rbdl;
// 	pose_from_rbdl.translation() = CalcBodyToBaseCoordinates(rbdl_model_, joint_state_.qInit_, arm_base_frame_id_, arm_base_frame_pos_, true);
// 	pose_from_rbdl.linear() = CalcBodyWorldOrientation(rbdl_model_ , joint_state_.qInit_, arm_base_frame_id_, true).transpose();

// 	base_global.block(0,0,3,3) = rotateZaxis(mobile_pose_.qInit_(2))*pose_from_rbdl.linear();
// 	base_global(0, 3) = mobile_pose_.qInit_(0) + pose_from_rbdl.translation()(0);
// 	base_global(1, 3) = mobile_pose_.qInit_(1) + pose_from_rbdl.translation()(1);
// 	base_global(2, 3) = pose_from_rbdl.translation()(2);

// 	target_global.block(0, 0, 3, 3) = pose.linear();
// 	target_global.block(0, 3, 3, 1) = pose.translation();

// 	target_from_base = base_global.inverse() * target_global;

// 	//cout << "target_global" << target_global << endl;
// 	//cout << "base_global" << base_global << endl;
// 	//cout << "target_from_base \n" << target_from_base << endl;

// 	for (int i = 0; i < 3; i++)
// 	{
// 		end_effector_pose.p(i) = target_from_base(i,3);
// 	}
// 	//cout << "waist to desired ee in local : " << target_from_base.block(0, 3, 3, 1).transpose() << endl;

// 	KDL::Rotation A;
// 	A.data[0] = target_from_base(0, 0);
// 	A.data[1] = target_from_base(0, 1);
// 	A.data[2] = target_from_base(0, 2);
// 	A.data[3] = target_from_base(1, 0);
// 	A.data[4] = target_from_base(1, 1);
// 	A.data[5] = target_from_base(1, 2);
// 	A.data[6] = target_from_base(2, 0);
// 	A.data[7] = target_from_base(2, 1);
// 	A.data[8] = target_from_base(2, 2);
// 	end_effector_pose.M = A;

// 	int rc;

// 	double total_time = 0;
// 	uint success = 0;
// 	bool solved = true;
// 	while(true) //for (uint i = 0; i < num_samples; i++)
// 	{
// 		if(--num_samples == 0){
// 			solved = false;
// 			break;
// 		}

// 		std::vector<double> R;
// 		for (int i = 0; i < IK_chain.getNrOfJoints(); i++)
// 		{
// 			double jointrange = joint_limit_.upper_rad_(i) - joint_limit_.lower_rad_(i); // angle
// 			double r = ((double)rand() / (double)RAND_MAX) * jointrange;
// 			R.push_back(joint_limit_.lower_rad_(i) + r);
// 		}

// 		for (size_t j = 0; j < nominal.data.size(); j++)
// 		{
// 			nominal(j) = R[j];
// 		}

// 		//cout <<"iteration?" << endl;
// 		double elapsed = 0;
// 		//start_time = boost::posix_time::microsec_clock::local_time();
// 		rc = kdl_solver.CartToJnt(nominal, end_effector_pose, result);
// 		// int CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist& bounds=KDL::Twist::Zero());

// 		std::vector<double> config;
// 		config.clear();
// 		if (rc >= 0)
// 		{
// 			//ROS_INFO_STREAM("IK solution is : " << result.data.transpose() * 180 / M_PI);
// 			for (int i = 0; i < nb_of_joints_; i++)
// 			{
// 				config.push_back(result.data(i) * 180 / M_PI);
// 			}

// 			// joint_state_.qGoal_ = result.data;
// 			// solved = true;
// 			// break;
// 			if (!rrt_.checkExternalCollision(model, config) && !rrt_.checkSelfCollision(model, config) )
// 			{

// 				ROS_INFO_STREAM("IK solution is : " << result.data.transpose() * 180 / M_PI);
// 				joint_state_.qGoal_ = result.data;
// 				solved = true;
// 				break;
// 			}
// 			else
// 			{
// 				continue;
// 			}
// 		}
// 		else
// 		{
// 			continue;
// 		}
// 	}
// 	return solved;
// }
bool ArmPlanner::setupRRT(VectorXd q_start, VectorXd q_goal, MatrixXd& joint_target)
{
	rrt_.lower_limit = joint_limit_.lower_;
	rrt_.upper_limit = joint_limit_.upper_;

	rrt_.qinit = q_start;
	rrt_.qgoal = q_goal;

	ofstream outFile("path_result.txt", ios::out); // open file for "writing"

	if (rrt_.solveRRT(rrt_model_, outFile))
	{
		// ofstream outFile2("path_result2.txt", ios::out); // "writing"
		// // path_result -> Smooth -> path_result2
		// ifstream inFile("path_result.txt"); // "reading"
		// rrt_.smoothPath(outFile2, inFile);
		// outFile2.close();
		outFile.close();
		MatrixXd joint_temp(100, nb_of_joints_);

		ifstream inFile2("path_result.txt"); // "reading"
		int size = 0;
		std::vector<std::string> parameters;
		char inputString[1000];
		while (!inFile2.eof())
		{ // eof : end of file
			inFile2.getline(inputString, 1000);
			boost::split(parameters, inputString, boost::is_any_of(","));
			if (parameters.size() == nb_of_joints_)
			{
				for (int j = 0; j < parameters.size(); j++)
					joint_temp(size, j) = atof(parameters[j].c_str()); // c_str() : string -> char* // atof : char* -> float
				size++;
			}
		}
		cout << "trajectory size" << size << endl;
		inFile2.close();
		joint_target = joint_temp.topLeftCorner(size, nb_of_joints_);
		return true;
	}
	else
	{
		cout << "Time out!!" << endl;
		return false;
	}
}
bool ArmPlanner::setupCRRT(VectorXd q_start, VectorXd q_goal, MatrixXd &joint_target)
{
	rrt_.lower_limit = joint_limit_.lower_;
	rrt_.upper_limit = joint_limit_.upper_;

	rrt_.qinit = q_start;
	rrt_.qgoal = q_goal;

	ofstream outFile3("path_result3.txt", ios::out);

	if (rrt_.solveCRRT(rrt_model_, outFile3))
	{
		outFile3.close();

		MatrixXd joint_temp(5000, nb_of_joints_);

		ifstream inFile3("path_result3.txt");
		int size = 0;
		std::vector<std::string> parameters;
		char inputString[50000];
		while (!inFile3.eof())
		{
			inFile3.getline(inputString, 50000);
			boost::split(parameters, inputString, boost::is_any_of(","));
			if (parameters.size() == nb_of_joints_)
			{
				for (int j = 0; j < parameters.size(); j++)
					joint_temp(size, j) = atof(parameters[j].c_str());
				size++;
			}
		}
		inFile3.close();
		//cout << "size" << size << endl;
		joint_target = joint_temp.topLeftCorner(size, nb_of_joints_);
		
		return true;
	}
	else
	{
		cout << "Time out!!" << endl;
		return false;
	}

}



bool ArmPlanner::solveRRTwithMultipleGoals(VectorXd q_start, std::vector<VectorXd> q_goal_set, MatrixXd &joint_target)
{
	rrt_.lower_limit = joint_limit_.lower_;
	rrt_.upper_limit = joint_limit_.upper_;

	rrt_.qinit = q_start;
	rrt_.qgoals.assign(q_goal_set.begin(), q_goal_set.end());

	ofstream outFile("path_result.txt", ios::out); // open file for "writing"

	if (rrt_.solveRRT(rrt_model_, outFile))
	{

		// outFile.close();
		// MatrixXd joint_temp(5000, nb_of_joints_);

		// ifstream inFile2("path_result.txt"); // "reading"
		// int size = 0;
		// std::vector<std::string> parameters;
		// char inputString[1000];


		ofstream outFile2("path_result2.txt", ios::out); // "writing"
		// path_result -> Smooth -> path_result2
		ifstream inFile("path_result.txt"); // "reading"
		rrt_.smoothPath(outFile2, inFile);

		outFile2.close();
		MatrixXd joint_temp(100, nb_of_joints_);

		ifstream inFile2("path_result2.txt"); // "reading"
		int size = 0;
		std::vector<std::string> parameters;
		char inputString[1000];

		while (!inFile2.eof())
		{
			inFile2.getline(inputString, 50000);
			boost::split(parameters, inputString, boost::is_any_of(","));
			if (parameters.size() == nb_of_joints_)
			{
				for (int j = 0; j < parameters.size(); j++)
					joint_temp(size, j) = atof(parameters[j].c_str());
				size++;
			}
		}
		inFile2.close();
		//cout << "size" << size << endl;
		joint_target = joint_temp.topLeftCorner(size, nb_of_joints_);
		
		return true;
	}
	else
	{
		cout << "Time out!!" << endl;
		return false;
	}

}

bool ArmPlanner::solveCRRTwithMultipleGoals(VectorXd q_start, std::vector<VectorXd> q_goal_set, MatrixXd &joint_target)
{
	rrt_.lower_limit = joint_limit_.lower_;
	rrt_.upper_limit = joint_limit_.upper_;

	rrt_.qinit = q_start;
	rrt_.qgoals.assign(q_goal_set.begin(), q_goal_set.end());

	ofstream outFile3("path_result3.txt", ios::out);

	if (rrt_.solveCRRT(rrt_model_, outFile3))
	{
		outFile3.close();

		//cout << "111"<<endl;
		MatrixXd joint_temp(5000, nb_of_joints_);

		ifstream inFile3("path_result3.txt");
		int size = 0;
		std::vector<std::string> parameters;
		char inputString[50000];
		while (!inFile3.eof())
		{
			inFile3.getline(inputString, 50000);
			boost::split(parameters, inputString, boost::is_any_of(","));
			if (parameters.size() == nb_of_joints_)
			{
				for (int j = 0; j < parameters.size(); j++)
					joint_temp(size, j) = atof(parameters[j].c_str());
				size++;
			}
		}
		inFile3.close();
		//cout << "size" << size << endl;
		joint_target = joint_temp.topLeftCorner(size, nb_of_joints_);
		return true;
	}
	else
	{
		cout << "Time out!!" << endl;
		return false;
	}

}

Eigen::VectorXd ArmPlanner::interpolate(const std::vector<Eigen::VectorXd> &data, double t, double dt, double t0, double tf){
    int N = data.size();
    double timeSpan = tf - t0;
    int totalPoints = static_cast<int>(timeSpan / dt) + 1; // Total number of interpolated points

    // Calculate the index for the lower bound
    int index = static_cast<int>((t - t0) / (timeSpan / (N - 1)));
    if (index >= N - 1) {
        index = N - 2;
    }

    // Calculate the local fraction
    double exactPosition = (t - t0) / (timeSpan / (N - 1));
    double fraction = exactPosition - index;

    return data[index] * (1 - fraction) + data[index + 1] * fraction;
}

Eigen::VectorXd ArmPlanner::generateRandomGoal(){

	Eigen::VectorXd R(nb_of_joints_);
	for (int i = 0; i < nb_of_joints_; i++)
	{
		double jointrange = config_.joint_limit_upper[i] - config_.joint_limit_lower[i]; // angle
		double r = ((double)rand() / (double)RAND_MAX) * jointrange;
		R(i) = config_.joint_limit_lower[i] + r;
	}

	return R;
}