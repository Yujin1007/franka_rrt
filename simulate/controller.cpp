

#include "controller.h"

#include <chrono>
#include <vector>
// #include <pybind11/operators.h>
// #include <pybind11/pybind11.h>
// #include <pybind11/stl.h>

#include <fstream> // ifstream header
#include <iostream>
#include <string> // getline header
#include <math.h>
#include <time.h>
// #include <torch/extension.h>
// #include <torch/torch.h>
// #include <torch/script.h>

CController::CController(int JDOF)
{
	_k = JDOF;
    std::string yaml_path = "/home/kist-robot2/Franka/franka_rrt/config/franka_arm_param.yaml";
    config.loadConfig(yaml_path);

	arm_planner_ = new ArmPlanner(config);


    VectorXd q_init_(7), q_goal_(7);
    q_init_.setZero();
    q_goal_(0) = 0.78;
    q_goal_(1) = 0.78;
    q_goal_(2) = 0.78;
    q_goal_(3) = -0.78;
    q_goal_(4) = 0.78;
    q_goal_(5) = 0.78;
    q_goal_(6) = 0.78;


    std::vector<VectorXd> output_path_;
    arm_planner_->initModel();

	Initialize();
}

CController::~CController()
{
}

void CController::read(double t, double *q, double *qdot, double timestep)
{
	_dt = timestep;
	_t = t;

	if (_bool_init == true)
	{
		_init_t = _t;
		for (int i = 0; i < 1; i++)
		{
			_init_gripper = q[_k + i];
		}
		_bool_init = false;
	}

	_dt = t - _pre_t;
	_pre_t = t;

	for (int i = 0; i < _k; i++)
	{
		_q(i) = q[i];
		_qdot(i) = qdot[i];
	}
	for (int i = 0; i < 1; i++)
	{
		_gripper = q[_k + i];
		_gripperdot = qdot[_k + i];
	}
}

void CController::read(double t, double *q, double *qdot, double timestep, double *pos, double *quat)
{
	int robot_base = 2;
	int valve = 14;
	int handle_valve = 18;

	_dt = timestep;
	_t = t;
	_door_angle = q[9];
	if (_bool_init == true)
	{
		_init_t = _t;
		for (int i = 0; i < 1; i++)
		{
			_init_gripper = q[_k + i];
		}
		
		for (int i = 0; i < 3; i++)
		{
			_valve(i) = pos[valve*3 + i];
			_handle_valve(i) = pos[handle_valve*3 + i];
			_robot_base(i) = pos[robot_base*3 + i];
		}
		Vector4d quat_handle;
		Vector4d quat_valve;
		for (int i=0; i<4; i++){
			quat_handle(i) = quat[handle_valve*4+i];
			quat_valve(i) = quat[valve*4+i];
		}
		Eigen::Quaterniond quat_h(quat_handle(0), quat_handle(1), quat_handle(2),quat_handle(3)); // (w, x, y, z)
    	Eigen::Matrix3d rotation_handle = quat_h.toRotationMatrix();
		
		Eigen::Quaterniond quat_v(quat_valve(0), quat_valve(1), quat_valve(2),quat_valve(3)); // (w, x, y, z)
    	Eigen::Matrix3d rotation_valve = quat_v.toRotationMatrix();
		
    	_rotation_handle << rotation_handle;
		_rotation_valve << rotation_valve;
		
		_bool_init = false;
	}

	_dt = t - _pre_t;
	_pre_t = t;

	for (int i = 0; i < _k; i++)
	{
		_q(i) = q[i];
		_qdot(i) = qdot[i];
	}
	for (int i = 0; i < 1; i++)
	{
		_gripper = q[_k + i];
		_gripperdot = qdot[_k + i];
	}
}

void CController::write(double *torque)
{
	for (int i = 0; i < _k; i++)
	{
		torque[i] = _torque(i);
	}
	for (int i = 0; i < 1; i++)
	{
		torque[i + _k] = _grippertorque;
	}
}


////////////////////////////////////////////////////////////////////////////////////////////////

void CController::control_mujoco()
{
	ModelUpdate();
	
	// double init_angle = 5.410520681182422;// handle 0
	
	// double init_angle = 3.8397243543875246;// handle 1 
	
	// double init_angle = 0.6981317007977318;// handle 2
	
	// double init_angle =  5.410520681182422;// handle 3 
	
	
	// motionPlan_Heuristic("HANDLE_VALVE",  init_angle, 2*PI + init_angle);
	motionPlan();
	
	// motionPlan_RL(_object_name);
	if (_control_mode == 1) // joint space control
	{
		// if (_bool_joint_motion == false)
		// {
		// 	// visit once
		// 	VectorXd tmp;
		// 	tmp.setZero(7);

		// 	_start_time = _init_t;
		// 	_end_time = _start_time + _motion_time;
		// 	JointTrajectory.reset_initial(_start_time, _q, tmp);
		// 	JointTrajectory.update_goal(_q_goal, _qdot_goal, _end_time);
		// 	_bool_joint_motion = true;
		// 	_x_des_hand = _x_hand;
		// 	_xdot_des_hand = _xdot_hand;
		// 	_q_des = _q;
		// 	_qdot_des = _qdot;
		// 	output_path_.clear();

		// 	if (_cnt_plan == 1)
		// 	{
		// 		// Eigen::Affine3d valve_pose_;
		// 		// valve_pose_.translation() = _handle_valve;
		// 		// valve_pose_.linear() = _rotation_obj;
		// 		// Vector3d valve_dim_;
		// 		// valve_dim_(0) = 0.1;
		// 		// valve_dim_(1) = 0.3;
		// 		// valve_dim_(2) = 0.1;
		// 		struct timespec spec;
		// 		clock_gettime(CLOCK_REALTIME, &spec);
		// 		srand(spec.tv_nsec);
		// 		// arm_planner_->updateValvePose(valve_pose_, valve_dim_);
		// 		// _q_goal = arm_planner_->generateRandomGoal()/180.0*M_PI;
		// 		_q_goal<<_q_home;
		// 		// std::cout << "_q_goal \t" << _q_goal.transpose() << endl;
		// 		arm_planner_->initializeData(_q, _q_goal);
		// 		arm_planner_->compute(output_path_);

		// 		// if (output_path_.size() == 0)
					
		// 	}
		// 			//	std::cout <<_cnt_plan<<"\t" << _motion_time<<std::endl;
		// } 

		// // _q_des = arm_planner_->interpolate(output_path_, _t, 0.001, _start_time, _end_time);	
		// // 	std::cout <<"!"<<std::endl;

		// _q_des = JointTrajectory.position_cubicSpline();
		// cout<<"_q_des:" <<_q_des.transpose()<<endl;
		// // JointTrajectory.update_time(_t);
		// // if (_cnt_plan == 1)
		// // {
		// // 	_q_des = arm_planner_->interpolate(output_path_, _t, 0.002, _start_time, _end_time);			
		// // }
		// // else
		// // {
		// // 	_q_des = JointTrajectory.position_cubicSpline();
		// // }
		// // 	std::cout << _q_des.transpose() << endl;

		// _qdot_des = JointTrajectory.velocity_cubicSpline();

		// JointControl();
		// GripperControl(); // planning + torque generation
		// if (JointTrajectory.check_trajectory_complete() == 1)
		// {

		// 	_bool_plan(_cnt_plan) = 1;
		// 	_bool_init = true;
			
		// }
		if (_t - _init_t < 0.1 && _bool_joint_motion == false)
		{
			VectorXd tmp;
			tmp.setZero(7);

			_start_time = _init_t;
			_end_time = _start_time + _motion_time;
			// JointTrajectory.reset_initial(_start_time, _q, tmp);
			// JointTrajectory.update_goal(_q_goal, _qdot_goal, _end_time);
			_bool_joint_motion = true;
			_x_des_hand = _x_hand;
			_xdot_des_hand = _xdot_hand;
			_q_des = _q;
			_q_init = _q;
			// _qdot_des = _qdot;
			_qdot_des << (_q_goal - _q)/_motion_time;
			// cout<<"_qdot_des:"<<_qdot_des.transpose()<<endl;
			// cout<<"_qdot_des:"<<_q_goal.transpose()<<endl;
			
		}
		double duration = _t-_start_time;
		// JointTrajectory.update_time(_t);
		_q_des << (_q_goal - _q_init) / _motion_time * duration + _q_init;
		// _q_des = JointTrajectory.position_cubicSpline();
		// _qdot_des = JointTrajectory.velocity_cubicSpline();

		JointControl();
		GripperControl(); // planning + torque generation
		// if (JointTrajectory.check_trajectory_complete() == 1)
		if (_motion_time <= duration)
		{
			_bool_plan(_cnt_plan) = 1;
			_bool_init = true;
		}
	}
	// else{
	// 			JointControl();

	// }
	else if (_control_mode == 2) // task space control
	{
		if (_t - _init_t < 0.1 && _bool_ee_motion == false)
		{
			_start_time = _init_t;
			_end_time = _start_time + _motion_time;
			HandTrajectory.reset_initial(_start_time, _x_hand, _xdot_hand);
			HandTrajectory.update_goal(_x_goal_hand, _xdot_goal_hand, _end_time);
			_bool_ee_motion = true;

			_x_des_hand = _x_hand;
			_xdot_des_hand = _xdot_hand;
			_q_des = _q;
			_qdot_des = _qdot;
			
		}

		HandTrajectory.update_time(_t);
		_x_des_hand.head(3) = HandTrajectory.position_cubicSpline();
		_xdot_des_hand.head(3) = HandTrajectory.velocity_cubicSpline();
		_R_des_hand = HandTrajectory.rotationCubic();
		_x_des_hand.segment<3>(3) = CustomMath::GetBodyRotationAngle(_R_des_hand);
		_xdot_des_hand.segment<3>(3) = HandTrajectory.rotationCubicDot();

		// CLIK();
		OperationalSpaceControl();
		GripperControl();
		if (HandTrajectory.check_trajectory_complete() == 1)
		{
			_bool_plan(_cnt_plan) = 1;
			_bool_init = true;
		}
	}
	// else if (_control_mode == 3) // circular trajectory heuristic
	// {
	// 	if (_t - _init_t < 0.1 && _bool_ee_motion == false)
	// 	{
	// 		_start_time = _init_t;
	// 		_end_time = _start_time + _motion_time;
	// 		CircularTrajectory.reset_initial(_start_time, _grab_vector, _normal_vector, _radius, _Tvr, _dt);
	// 		CircularTrajectory.update_goal(_end_time, _init_theta, _goal_theta); // _init_theta = 0 -> change to learned result later
	// 		_bool_ee_motion = true;
	// 		_x_des_hand = _x_hand;
	// 		_q_des = _q;
	// 		_xdot_des_hand = _xdot_hand;
	// 		_qdot_des = _qdot;
	// 	}
	// 	// _x_force.head(3) = AddTorque();
	// 	_theta_des = CircularTrajectory.update_time(_t);
	// 	_x_des_hand.head(3) = CircularTrajectory.position_circular();
	// 	_xdot_des_hand.head(3) = CircularTrajectory.velocity_circular();
	// 	_x_des_hand.tail(3) = CircularTrajectory.rotation_circular();
	// 	_xdot_des_hand.tail(3) = CircularTrajectory.rotationdot_circular();
	// 	_R_des_hand = CustomMath::GetBodyRotationMatrix(_x_des_hand(3), _x_des_hand(4), _x_des_hand(5));
	// 	// cout<<"3";
	// 	// cout<<"rpy :"<<_xdot_des_hand.tail(3).transpose()<<endl;
	// 	OperationalSpaceControl();
	// 	// CLIK();
	// 	GripperControl();
	// 	if (CircularTrajectory.check_trajectory_complete() == 1)
	// 	{
	// 		_bool_plan(_cnt_plan) = 1;
	// 		_bool_init = true;
	// 		// cout<<"time : "<< _t<<"  (circular"<<endl;
	// 	}


	// 	if((_t - _print_time )> _print_interval)
	// 	{
	// 		_print_time = _t;
	// 		// cout<<"des hand :"<<_x_des_hand.tail(3).transpose()<<endl;

	// 		Matrix3d Trot, temp_rot;
	// 		Trot << cos(-_robot.ee_align), -sin(-_robot.ee_align), 0,
	// 			sin(-_robot.ee_align), cos(-_robot.ee_align), 0,
	// 			0, 0, 1;
			
	// 		temp_rot << _R_des_hand * Trot;
	// 		double r,p,y;
	// 		y = atan2(temp_rot(1, 0), temp_rot(0, 0)) ;
	// 		p = atan2(-temp_rot(2, 0), sqrt(pow(temp_rot(2, 1), 2) + pow(temp_rot(2, 2), 2)));
	// 		r = atan2(temp_rot(2, 1), temp_rot(2, 2));
	// 		// cout<<"rot hand : "<<r<<" "<<p<<" "<<y<<endl;
	// 	}
	// }
	
	_q_pre = _q;
	_qdot_pre = _qdot;
}

void CController::ModelUpdate()
{
	Model.update_kinematics(_q, _qdot);
	Model.update_dynamics();

	Model.calculate_EE_Jacobians();

	Model.calculate_EE_positions_orientations();

	Model.calculate_EE_velocity();

	_J_hands = Model._J_hand;
	_x_hand.head(3) = Model._x_hand;
	_R_hand = Model._R_hand;	
	_x_hand.tail(3) = CustomMath::GetBodyRotationAngle(_R_hand);
	
	_xdot_hand = Model._xdot_hand;
	for (int i = 0; i < 7; i++)
	{
		Model._A(i, i) += 0.1;
	}
}

// Vector3d CController::AddTorque()
// {
// 	Vector4d tangential_vector;
// 	tangential_vector << -_obj.o_margin.normalized(), 0;
// 	if (_init_theta < _goal_theta)
// 	{
// 		// 반시계방향
// 		tangential_vector << -_obj.o_margin.normalized().cross((_obj.pos + _obj.o_margin - _x_hand.head(3)).normalized()), 0;
// 	}
// 	else
// 	{
// 		tangential_vector << _obj.o_margin.normalized().cross((_obj.pos + _obj.o_margin - _x_hand.head(3)).normalized()), 0;
// 	}
// 	tangential_vector = _Tur * tangential_vector; // robot

// 	return tangential_vector.head(3);
// }



Matrix3d CController::R3D(Objects obj, Vector3d unitVec, double angle)
{
	Matrix3d _Tug;
	angle = -angle; // frame은 반대 방향으로 회전 해야지, gripper방향이 유지된다.
	double cosAngle = cos(angle);
	double sinAngle = sin(angle);
	double x = unitVec(0);
	double y = unitVec(1);
	double z = unitVec(2);
	Matrix3d rotMatrix;
	rotMatrix << cosAngle + (1 - cosAngle) * x * x, (1 - cosAngle) * x * y - sinAngle * z, (1 - cosAngle) * x * z + sinAngle * y,
		(1 - cosAngle) * y * x + sinAngle * z, cosAngle + (1 - cosAngle) * y * y, (1 - cosAngle) * y * z - sinAngle * x,
		(1 - cosAngle) * z * x - sinAngle * y, (1 - cosAngle) * z * y + sinAngle * x, cosAngle + (1 - cosAngle) * z * z;

	_Tug << rotMatrix * obj.grab_dir.normalized(),
		rotMatrix * -obj.o_margin.normalized().cross(obj.grab_dir.normalized()),
		rotMatrix * -obj.o_margin.normalized();
	return _Tug;
}

CController::Target CController::TargetTransformMatrix(Objects obj, Robot robot, double angle)
{
	Target target;

	// position x,y,z
	Vector4d _xaxis;
	Vector4d _yaxis;
	Vector4d _zaxis;
	Vector4d _porg;
	Vector4d tmp;
	Matrix4d Tvb; // valve handle -> valve base
	Matrix4d Tbu; // valve base -> universal
	Matrix4d Tur; // universal -> robot
	Matrix4d Tvr; // valve handle -> valve vase -> universal -> robot!!

	// roll pitch yaw
	Matrix3d _Tug; // universal -> gripper
	Matrix3d _Tge; // gripper -> end-effector
	Matrix3d _Tue; // universal -> gripper -> end-effector

	// calc target x,y,z
	_xaxis << obj.r_margin.normalized(), 0;
	_yaxis << obj.o_margin.normalized().cross(obj.r_margin.normalized()), 0;
	_zaxis << obj.o_margin.normalized(), 0;
	_porg << obj.o_margin, 1;

	Tvb << _xaxis, _yaxis, _zaxis, _porg;

	Tbu << 1, 0, 0, obj.pos(0),
		0, 1, 0, obj.pos(1),
		0, 0, 1, obj.pos(2),
		0, 0, 0, 1;

	Tur << cos(-robot.zrot), sin(-robot.zrot), 0, robot.pos(0),
		-sin(-robot.zrot), cos(-robot.zrot), 0, robot.pos(1),
		0, 0, 1, -robot.pos(2),
		0, 0, 0, 1;

	Tvr << Tur * Tbu * Tvb;

	tmp << obj.r_margin.norm() * cos(angle), obj.r_margin.norm() * sin(angle), 0, 1;
	tmp << Tvr * tmp;

	target.x = tmp(0);
	target.y = tmp(1);
	target.z = tmp(2);
	_x_plan.push_back(target.x);
	_y_plan.push_back(target.y);
	_z_plan.push_back(target.z);

	// calc target r,p,y
	_Tge << cos(robot.ee_align), -sin(robot.ee_align), 0,
		sin(robot.ee_align), cos(robot.ee_align), 0,
		0, 0, 1;
	_Tug = CController::R3D(obj, -obj.o_margin.normalized(), angle);
	_Tue << _Tug * _Tge;

	target.yaw = atan2(_Tue(1, 0), _Tue(0, 0)) + robot.zrot;
	target.pitch = atan2(-_Tue(2, 0), sqrt(pow(_Tue(2, 1), 2) + pow(_Tue(2, 2), 2)));
	target.roll = atan2(_Tue(2, 1), _Tue(2, 2));

	target.yaw = fmod(target.yaw + M_PI, 2 * M_PI);
	if (target.yaw < 0)
	{
		target.yaw += 2 * M_PI;
	}
	target.yaw = target.yaw - M_PI;

	target.pitch = fmod(target.pitch + M_PI, 2 * M_PI);
	if (target.pitch < 0)
	{
		target.pitch += 2 * M_PI;
	}
	target.pitch = target.pitch - M_PI;

	target.roll = fmod(target.roll + M_PI, 2 * M_PI);

	if (target.roll < 0)
	{
		target.roll += 2 * M_PI;
	}
	target.roll = target.roll - M_PI;

	target.gripper = _gripper_close;
	target.time = 0.5;

	target.target_velocity << 0.02, 0.02, 0.02;
	target.state = "taskspace";

	return target;
}


void CController::TargetPlanHeuristic(Objects obj, Robot robot, double init_theta, double goal_theta)
{
	double motion_time_const = 10.0;
	double motion_time;

	Target onvalve;
	onvalve.state = "onvalve_heuristic";

	Target home;

	home.time = -1;
	Objects obj_above = obj;
	obj_above.o_margin = obj_above.o_margin + obj_above.o_margin.normalized() * 0.05;
	_target_plan.push_back(TargetTransformMatrix(obj_above, robot, init_theta));
	_target_plan.back().gripper = 0.04;
	_target_plan.back().time = 3.0;
	// 3s for moving to the valve

	_target_plan.push_back(TargetTransformMatrix(obj, robot, init_theta));
	_target_plan.back().gripper = _gripper_close;
	_target_plan.back().time = 1.0;
	// 1s for closing gripper

	_target_plan.push_back(onvalve);
	motion_time = abs(motion_time_const * abs(goal_theta - init_theta) * _obj.r_margin.norm());
	// Xs for rotating valve

	_target_plan.back().time = motion_time;
	_target_plan.back().gripper = _gripper_close;

	_target_plan.push_back(TargetTransformMatrix(obj, robot, goal_theta));
	_target_plan.back().gripper = 0.04;
	// open the gripper

	_target_plan.push_back(TargetTransformMatrix(obj_above, robot, goal_theta));
	_target_plan.back().gripper = 0.04;
	_target_plan.back().time = .5;
	// move off the valve

	_target_plan.push_back(home);
	// back to initial config
	
}



// Joint space and Task space motion.

// void CController::motionPlan()
// {
// 	_time_plan(1) = 2.0;	  // move home position
// 	_time_plan(2) = 1.0;	  // wait
// 	_time_plan(3) = 2.0;	  // joint goal motion
// 	_time_plan(4) = 1.0;	  // wait
// 	_time_plan(5) = 2.0;	  // task goal motion
// 	_time_plan(6) = 100000.0; // wait
// 	if (_bool_plan(_cnt_plan) == 1)
// 	{
// 		_cnt_plan = _cnt_plan + 1;

// 		if (_cnt_plan == 1)
// 		{
// 			reset_target(_time_plan(_cnt_plan), _q_home);
// 			_gripper_goal = 0.04;
// 		}
// 		else if (_cnt_plan == 2)
// 		{
// 			_gripper_goal = 0.04;
// 			reset_target(_time_plan(_cnt_plan), _q);
// 		}
// 		else if (_cnt_plan == 3)
// 		{
// 			_pos_goal_hand(0) = _x_hand(0) + 0.2;
// 			_pos_goal_hand(1) = _x_hand(1) - 0.2;
// 			_pos_goal_hand(2) = _x_hand(2) + 0.1;

// 			_rpy_goal_hand(0) = _x_hand(3) - 0.5;
// 			_rpy_goal_hand(1) = _x_hand(4) + 0.3;
// 			_rpy_goal_hand(2) = _x_hand(5) - 0.5;
// 			_gripper_goal = 0.0;
			
// 			reset_target(_time_plan(_cnt_plan), _pos_goal_hand, _rpy_goal_hand);
// 		}
// 		else if (_cnt_plan == 4)
// 		{
// 			reset_target(_time_plan(_cnt_plan), _q);
// 		}
// 		else if (_cnt_plan == 5)
// 		{
// 			_pos_goal_hand(0) = _x_hand(0) - 0.2;
// 			_pos_goal_hand(1) = _x_hand(1) + 0.2;
// 			_pos_goal_hand(2) = _x_hand(2);

// 			_rpy_goal_hand(0) = _x_hand(3) + 0.5;
// 			_rpy_goal_hand(1) = _x_hand(4) - 0.3;
// 			_rpy_goal_hand(2) = _x_hand(5) + 0.5;
// 			_gripper_goal = 0.04;
			
// 			reset_target(_time_plan(_cnt_plan), _pos_goal_hand, _rpy_goal_hand);
// 		}
// 		else if (_cnt_plan == 6)
// 		{
// 			// cout<<"current hand : "<<_x_hand.transpose()<<endl;

// 			reset_target(_time_plan(_cnt_plan), _q);
// 		}
// 	}
// }

void CController::motionPlan()
{
	
	
	if (_bool_plan(_cnt_plan) == 1)
	{
		cout<<"plan: "<<_cnt_plan<< "  current door angle :"<<RAD2DEG*_door_angle<<endl;
		if (_cnt_plan == 0){
			reset_target(5.0, _q_home);
		}
		else if (_cnt_plan >= _rrt_goals.rows()){
			printf("done!  ");
		}
		else{
			VectorXd q_goal = _rrt_goals.row(_cnt_plan-1);
			VectorXd abs_diff = (q_goal - _q).cwiseAbs();
			double max_q = abs_diff.maxCoeff();
			double plan_time = max_q * 5;
			
			reset_target(plan_time, _rrt_goals.row(_cnt_plan-1));
		}
		_cnt_plan = _cnt_plan + 1;
	}
}



void CController::motionPlan_Heuristic(const char *object, double init_theta, double goal_theta)
{

	// initialize for motion plan 3
	if (_init_mp)
	{

		Robot robot_base;
		Objects obj;

		robot_base.id = 6;
		robot_base.pos << _robot_base;
		robot_base.zrot = 0;	 // M_PI;
		robot_base.ee_align = DEG2RAD*(45);

		if (object == "VALVE")
		{
			_rotation_obj << _rotation_valve;
			obj.id = 42; // 39;
			obj.name = "VALVE";
			// obj.o_margin << 0, 0, 0.017;//0.015 ;
			obj.o_margin << 0, 0, -0.017;
			obj.o_margin = _rotation_obj * obj.o_margin;
			// obj.r_margin << 0, 0.1, 0;
			obj.r_margin << 0.1, 0, 0;
			obj.r_margin = _rotation_obj * obj.r_margin;
			// obj.grab_dir << obj.o_margin.cross(obj.r_margin);//obj.r_margin;
			obj.grab_dir << obj.r_margin;
			obj.pos << _valve;
			_gripper_close = 0.017 - 0.007;
		}
		else if (object == "HANDLE_VALVE")
		{
			_rotation_obj << _rotation_handle;
			obj.id = 54; // 51;
			obj.name = "HANDLE_VALVE";
			obj.o_margin << 0, 0.149, 0;
			obj.o_margin = _rotation_obj * obj.o_margin;
			obj.r_margin << 0.119, 0, 0; // East side of the origin
			obj.r_margin = _rotation_obj * obj.r_margin;
			obj.grab_dir << obj.o_margin.cross(obj.r_margin);
			// obj.grab_dir << obj.r_margin;//obj.o_margin.cross(obj.r_margin);
			obj.pos << _handle_valve;
			_gripper_close = 0.01 - 0.005;
		}
		else
		{
			printf("%s\n\n", object);
			printf("cannot find an object\n");
			return;
		}

		Vector4d xaxis;
		Vector4d yaxis;
		Vector4d zaxis;
		Vector4d origin;
		Vector4d porg;
		// Matrix4d Tvb; // valve handle -> valve base
		// Matrix4d Tbu; // valve base -> universal
		// Matrix4d Tur; // universal -> robot
		// Matrix4d Tvr; // valve handle -> valve vase -> universal -> robot!!

		xaxis << obj.r_margin.normalized(), 0;
		yaxis << obj.o_margin.normalized().cross(obj.r_margin.normalized()), 0;
		zaxis << obj.o_margin.normalized(), 0;
		porg << obj.o_margin, 1;

		_Tvb << xaxis, yaxis, zaxis, porg;

		_Tbu << 1, 0, 0, obj.pos(0),
			0, 1, 0, obj.pos(1),
			0, 0, 1, obj.pos(2),
			0, 0, 0, 1;

		_Tur << cos(-robot_base.zrot), sin(-robot_base.zrot), 0, robot_base.pos(0),
			-sin(-robot_base.zrot), cos(-robot_base.zrot), 0, robot_base.pos(1),
			0, 0, 1, -robot_base.pos(2),
			0, 0, 0, 1;

		_Tvr << _Tur * _Tbu * _Tvb;
		_init_theta = init_theta;
		_goal_theta = goal_theta;
		_obj = obj;
		_robot = robot_base;
		_origin << obj.o_margin;
		_radius = obj.r_margin.norm();
		_grab_vector = _obj.grab_dir.normalized();
		_normal_vector = -_obj.o_margin.normalized();
		TargetPlanHeuristic(obj, robot_base, _init_theta, _goal_theta);
		_init_mp = false;
	}

	Target target;

	if (_bool_plan(_cnt_plan) == 1)
	{
		if (_cnt_plan == 0)
		{
			reset_target(2.0, _q_home);		
		}

		else if (_cnt_plan > _target_plan.size())
		{

			printf("plan terminated\n\n");

			reset_target(100000000, _q);
		}
		else
		{

			target = _target_plan[_cnt_plan - 1];

			if (target.time == -1)
			{
				printf("reset position\n\n");
				reset_target(3.0, _q_home);
			}
			else
			{
				if (target.state == "onvalve_heuristic")
				{
					_time_plan(_cnt_plan) = target.time;
					reset_target(target.time, target.state);
				}
				else
				{
					_pos_goal_hand(0) = target.x;
					_pos_goal_hand(1) = target.y;
					_pos_goal_hand(2) = target.z;

					_rpy_goal_hand(0) = target.roll;
					_rpy_goal_hand(1) = target.pitch;
					_rpy_goal_hand(2) = target.yaw;

					_gripper_goal = target.gripper;
					_time_plan(_cnt_plan) = target.time;

					reset_target(target.time, _pos_goal_hand, _rpy_goal_hand, target.target_velocity);
				}
			}
		}
		_cnt_plan = _cnt_plan + 1;
	}
}


void CController::reset_target(double motion_time, VectorXd target_joint_position)
{
	_control_mode = 1;
	_motion_time = motion_time;
	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_q_goal = target_joint_position.head(7);
	_qdot_goal.setZero();
}

void CController::reset_target(double motion_time, Vector3d target_pos, Vector3d target_ori)
{
	_control_mode = 2;
	_motion_time = motion_time;
	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_x_goal_hand.head(3) = target_pos;
	_x_goal_hand.tail(3) = target_ori;
	_xdot_goal_hand.setZero();
}

void CController::reset_target(double motion_time, Vector3d target_pos, Vector3d target_ori, Vector3d target_velocity)
{
	_control_mode = 2;
	_motion_time = motion_time;
	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_x_goal_hand.head(3) = target_pos;
	_x_goal_hand.tail(3) = target_ori;

	_xdot_goal_hand.tail(3) = target_velocity;
}



void CController::reset_target(double motion_time, string state)
{
	if (state == "onvalve_heuristic")
	{
		_control_mode = 3;
	}
	

	_bool_joint_motion = false;
	_bool_ee_motion = false;
	_motion_time = motion_time;
}

void CController::JointControl()
{
	_torque.setZero();
	_torque = Model._A * (400 * (_q_des - _q) + 40 * ( - _qdot)) + Model._bg;
}

void CController::GripperControl()
{
	// position control with mujoco
	du = _t - _start_time;
	if (_motion_time == 0.0){
		_motion_time = 0.5;
	}
	if (du >= (_motion_time))
	{
		du = _motion_time;
	}
	
	
	else
	{
		_gripper_des = _init_gripper + (_gripper_goal - _init_gripper) * du / _motion_time;
	}
	// cout<<"gripper now :"<<_gripper.transpose()<<"  gripper des:"<<_gripper_des.transpose()<<"  _gripper dot:"<<_gripperdot.transpose()<<endl;
	_grippertorque = _kpj_gripper * (_gripper_des - _gripper) - _kdj_gripper * (_gripperdot); // PD simple damping control (_gripperdot_goal = 0 0)
	// cout<<"gripper goal : "<<_gripper_goal<<"  gripper des : "<<_gripper_des<<"  du : "<<du<<" torque : "<<_grippertorque<<endl;
	// cout<<"init gripper : "<<_init_gripper<<endl;
}

// Closed Loop Inverse Kinematics
void CController::CLIK()
{
	_torque.setZero();

	_x_err_hand.segment(0, 3) = _x_des_hand.head(3) - _x_hand.head(3);

	_x_err_hand.segment(3, 3) = -CustomMath::getPhi(Model._R_hand, _R_des_hand);

	_J_bar_hands = CustomMath::pseudoInverseQR(_J_hands);
	_qdot_des = _J_bar_hands * (_xdot_des_hand + _x_kp * (_x_err_hand)); // + _x_err_hand.norm()*_x_force);
	_q_des = _q_des + _dt * _qdot_des;

	_torque = Model._A * (_kpj * (_q_des - _q) + _kdj * (_qdot_des - _qdot)) + Model._bg;

	_accum_err_q = _accum_err_q + (_q - _q_des).cwiseAbs();
	_accum_err_x = _accum_err_x + (_x_hand - _x_des_hand).cwiseAbs();

	_Rdot_des_hand = CustomMath::GetBodyRotationMatrix(_xdot_des_hand(3), _xdot_des_hand(4), _xdot_des_hand(5));
	_Rdot_hand = CustomMath::GetBodyRotationMatrix(_xdot_hand(3), _xdot_hand(4), _xdot_hand(5));

	_xdot_err_hand.segment(0, 3) = _xdot_des_hand.head(3) - _xdot_hand.head(3);
	_xdot_err_hand.segment(3, 3) = -CustomMath::getPhi(_Rdot_hand, _Rdot_des_hand);

}

void CController::OperationalSpaceControl()
{
	// cout<<"_xdot :"<<_xdot_des_hand.transpose()<<endl;
	_J_bar_hands = CustomMath::pseudoInverseQR(_J_hands);
	_lambda = CustomMath::pseudoInverseQR(_J_hands.transpose()) * Model._A * _J_bar_hands;

	_x_err_hand.segment(0, 3) = _x_des_hand.head(3) - _x_hand.head(3);
	_x_err_hand.segment(3, 3) = -CustomMath::getPhi(_R_hand, _R_des_hand);
	// _x_err_hand.segment(3, 3) = -CustomMath::getPhi(Model._R_hand, _R_des_hand);

	_Rdot_des_hand = CustomMath::GetBodyRotationMatrix(_xdot_des_hand(3), _xdot_des_hand(4), _xdot_des_hand(5));
	_Rdot_hand = CustomMath::GetBodyRotationMatrix(_xdot_hand(3), _xdot_hand(4), _xdot_hand(5));

	_xdot_err_hand.segment(0, 3) = _xdot_des_hand.head(3) - _xdot_hand.head(3);
	_xdot_err_hand.segment(3, 3) = -CustomMath::getPhi(_Rdot_hand, _Rdot_des_hand);
	_force = _kpj * _x_err_hand + _kdj * _xdot_err_hand;
	_torque = _J_hands.transpose() * _lambda * _force + Model._bg;

	// cout<<"force :"<<_force.transpose()<<" torque :"<<_torque.transpose()<<endl;
}


void CController::Initialize()
{
	_control_mode = 0; // 1: joint space, 2: task space(CLIK)
	_init_mp = true;
	_bool_init = true;
	_t = 0.0;
	_init_t = 0.0;
	_pre_t = 0.0;
	_dt = 0.0;

	_kpj = 400;
	_kdj = 40;
	_kpj_gripper = 30000.0; // 100;
	_kdj_gripper = 10.0;	// 10;
	_door_angle = 0.0;
	_x_kp = 0.1; // 작게 0.1

	_q.setZero(_k);
	_qdot.setZero(_k);
	_q_init.setZero(_k);
	_torque.setZero(_k);

	_gripper=0;
	_gripperdot=0; // gripper 속도 : 50mm/s
	_grippertorque = 0;


	// _gripper.setZero(2);
	// _gripperdot.setZero(2); // gripper 속도 : 50mm/s
	// _grippertorque.setZero(2);
	_valve.setZero(3);
	_handle_valve.setZero(3);
	_robot_base.setZero(3);

	_J_hands.setZero(6, _k);
	_J_bar_hands.setZero(_k, 6);

	_x_hand.setZero(6);
	_xdot_hand.setZero(6);

	_cnt_plan = 0;
	_bool_plan.setZero(1000);
	_bool_plan(0) = 1;
	_time_plan.resize(1000);
	_time_plan.setConstant(-1);

	_q_home.setZero(_k);

	// _q_home(0) =  	0.0;
	// _q_home(1) = -60*DEG2RAD; // -0.7853981633974483; //-45
	// _q_home(2) = 0.0;
	// _q_home(3) = -90*DEG2RAD;//-2.356194490192345; //-135
	// _q_home(4) = 0.0;
	// _q_home(5) = 90*DEG2RAD;//1.5707963267948966; // 90
	// _q_home(6) = 45*DEG2RAD;//0.7853981633974483; // 45
	_q_home << 0.384, -1.267 ,-0.451, -2.182 ,-0.412 , 2.578 , 0.692;

	_gripper_close = 0.0;

	
	_grab_vector.setZero(3);
	_normal_vector.setZero(3);
	_origin.setZero(3);
	_radius = 0.0;
	_goal_theta = 0.0;
	_init_theta = 0.0;

	_start_time = 0.0;
	_end_time = 0.0;
	_motion_time = 0.0;

	_bool_joint_motion = false;
	_bool_ee_motion = false;
	// _q_pre.
	_q_des.setZero(_k);
	_qdot_des.setZero(_k);
	_q_pre.setZero(_k);
	_qdot_pre.setZero(_k);
	_q_goal.setZero(_k);
	_qdot_goal.setZero(_k);
	_gripper_des=0;
	_gripper_goal=0.0;
	_gripperdot_goal=0;
	_init_gripper=0;
	
	_x_des_hand.setZero(6);
	_xdot_des_hand.setZero(6);
	_x_goal_hand.setZero(6);
	_xdot_goal_hand.setZero(6);

	_pos_goal_hand.setZero(); // 3x1
	_rpy_goal_hand.setZero(); // 3x1

	JointTrajectory.set_size(_k);

	_x_err_hand.setZero(6);
	_xdot_err_hand.setZero(6);
	_R_des_hand.setZero();
	_R_hand.setZero(3, 3);
	_Rdot_des_hand.setZero();
	_Rdot_hand.setZero();
	_lambda.setZero(6, 6);
	_force.setZero(6);
	_rotation_obj.setIdentity(3, 3);
	_Tvr.setIdentity(4, 4);
	_Tvb.setIdentity(4, 4);
	_Tbu.setIdentity(4, 4);
	_Tur.setIdentity(4, 4);

	du = 0.0;
	
	_print_time = 0;
	_print_interval = 0.1;
	_target_plan.clear();

	_x_plan.clear();
	_y_plan.clear();
	_z_plan.clear();
	_theta_des = 0;
	_accum_err_x.setZero(6);
	_accum_err_q.setZero(7);
	// _x_force.setZero(6);

	_rotation_handle.setIdentity(3,3);
	_rotation_valve.setIdentity(3,3);

/////////////////////////////////////////////////////////////////////////////////////////////////////////
	std::string file_path = "/home/kist-robot2/Franka/franka_rrt/waypoint/position_data_30_tsr2_2.txt";
	std::ifstream file(file_path);

    // Check if the file was opened successfully
    if (!file.is_open()) {
        std::cerr << "Unable to open file: " << file_path << std::endl;
        
    }

    std::vector<std::vector<double>> data; // To store the values temporarily
    std::string line;

    // Read the file line by line
    while (std::getline(file, line)) {
        std::vector<double> row;
        std::stringstream ss(line);
        std::string value;

        // Split the line by spaces (or other delimiters)
        while (ss >> value) {
            row.push_back(std::stod(value)); // Convert to double and add to the row
        }

        data.push_back(row);
    }

    // Close the file
    file.close();

    // Determine the dimensions of the matrix
    size_t rows = data.size();
    size_t cols = data.empty() ? 0 : data[0].size();
	cout<<"rows:"<<rows<<" cols:"<<cols<<endl;
    // Create an Eigen matrix and populate it with the data
    _rrt_goals.setZero(rows, cols);
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            _rrt_goals(i, j) = data[i][j]*DEG2RAD;
			
        }
    }
	_time_plan.head(rows).setConstant(0.5);

}
