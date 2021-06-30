#include<stdio.h>
#include<iostream>
#include<chrono>
#include<utility>
#include<cmath>
#include<opencv2/opencv.hpp>
#include<time.h>
#define PI 3.141592653589793


typedef struct {
	int val;
	bool ret;
} isinStruct;

typedef struct {
	int val;
	std::pair<double,double> pt;
} nextStruct;

// We compute the reachable avoidance velocities (RAV) at every instant by considering the location,orientation,tyre angle,bot velocity,obstacle location and velocity

class velocityObstacles
{
public:
	std::pair<double,double> bot_loc;
	std::pair<double,double> obs_loc;
	std::pair<double,double> bot_vel;
	std::pair<double,double> obs_vel;
	double bot_orien,bot_radius,delta_max,bot_vel_max,obs_radius,radius,bot_delta,delta_dot;

	velocityObstacles(std::pair<double,double> bot_loc,double bot_orien,double bot_delta,std::pair<double,double> bot_vel,double bot_radius,double delta_max,double delta_dot,double bot_vel_max,std::pair<double,double> obs_loc,std::pair<double,double> obs_vel,double obs_radius)
	{
		this->bot_loc = bot_loc;
		this->obs_loc = obs_loc;
		this->bot_vel = bot_vel;
		this->obs_vel = obs_vel;
		this->bot_orien = bot_orien;
		this->bot_radius = bot_radius;
		this->delta_max = delta_max;
		this->bot_vel_max = bot_vel_max;
		this->obs_radius = obs_radius;
		this->radius = bot_radius + obs_radius;
		this->bot_delta = bot_delta;
		this->delta_dot = delta_dot;
	}

// This does same as '%' does, but this can also be used for float values.
	double mod(double inp,double inp_wrt){
		while(std::abs(inp) > inp_wrt){
			inp = inp - inp_wrt*inp/std::abs(inp);
		}
		return inp;
	}

// Computes the distance between two points.
	double Distance(std::pair<double,double> pt1, std::pair<double,double> pt2){
		double value = sqrt((pt2.second - pt1.second)*(pt2.second - pt1.second) + (pt2.first - pt1.first)*(pt2.first - pt1.first));
		return value;
	}

// Computing the vector [ i + 2j => (1,2) ] for given direction and magnitude
	std::pair<double,double> get_vector(double direc,double mag){
		return std::make_pair(mag*cos(direc*PI/180),mag*sin(direc*PI/180));
	}

// This finds the angle of tangents globally for given bot, obstacle location and the sum of bot and obstacle radius
	std::pair<double,double> tangent_angles(std::pair<double,double> bot_loc,std::pair<double,double> obs_loc)
	{
		std::pair<double,double> centre = obs_loc;
		double radius = this->radius;
	
		double theta_arg = radius/this->Distance(bot_loc,centre);
		if(std::abs(theta_arg) > 1){
			theta_arg = int(round(theta_arg));
		}
	
		double theta = asin(theta_arg)*180/PI;
		double alpha = atan2(centre.second-bot_loc.second,centre.first-bot_loc.first)*180/PI;
		alpha = this->mod(alpha,360);

		return std::make_pair(alpha-theta,alpha+theta);
	}

// Just the magnitude of a vector
	double mag(std::pair<double,double> pt){
		return this->Distance(pt,std::make_pair(0,0));
	}

// Outputs the angle between two given input vectors
	double meas_ang(std::pair<double,double> pt1,std::pair<double,double> pt2)
	{
		double dot_pro = pt1.first*pt2.first + pt1.second*pt2.second;
		double val;

		if(dot_pro != 0){val = dot_pro/(mag(pt1)*mag(pt2));}
		else{val = 0;}

		if(std::abs(val) >= 1){val = int(round(val));}

		return acos(val)*180/PI;
	}

// return true if the given angle is in between two angles, (30,90,60 => True)
	bool check_in(double ang1,double ang2,double angle)
	{
		std::pair<double,double> vec1 = std::make_pair(cos(ang1*PI/180),sin(ang1*PI/180));
		std::pair<double,double> vec2 = std::make_pair(cos(ang2*PI/180),sin(ang2*PI/180));
		std::pair<double,double> vect = std::make_pair(cos(angle*PI/180),sin(angle*PI/180));

		double theta1 = this->meas_ang(vec1,vect);
		double theta2 = this->meas_ang(vec2,vect);
		double theta = this->meas_ang(vec1,vec2);

		if(round(theta1 + theta2 - theta) == 0){return true;}
		else{return false;}

	}

// Computes the location w.r.t new axis
	std::pair<double,double> axis_transform(std::pair<double,double> origin_new,double orien,std::pair<double,double> pt){
		orien = orien*PI/180;
		return std::make_pair(cos(orien)*(pt.first-origin_new.first)+sin(orien)*(pt.second-origin_new.second),cos(orien)*(pt.second-origin_new.second)-sin(orien)*(pt.first-origin_new.first));
	}

// Returns True if the given point is between two points.
	bool ptisout(std::pair<double,double> pt1,std::pair<double,double> pt2,std::pair<double,double> pt){
		double dist_pt_1 = this->Distance(pt,pt1);
		double dist_pt_2 = this->Distance(pt,pt2);
		double dist_1_2 = this->Distance(pt1,pt2);

		if(dist_pt_1 > dist_1_2 || dist_pt_2 > dist_1_2){return true;}
		else{return false;}
	}

	// bool collision_check(std::pair<std::pair<double,double>,std::pair<double,double> > pts_bot,std::pair<double,double> obs_loc,double radius)
	// {
	// 	std::pair<double,double> drop;

	// 	if(this->ptisout(pts_bot.first,pts_bot.second,drop) == false){
	// 		if(this->Distance(drop,obs_loc) <= radius){return true;}
	// 		else{return false;}
	// 	}
	// 	else{return false;}
	// }

// For a given obstacle location,velocity, bot location,velocity and the steering angle the function computes whether there is gonna be a collision with the constant steering angle
	bool ackermen_check_in_post(std::pair<double,double> bot_loc,double bot_orien,std::pair<double,double> bot_action,std::pair<double,double> obs_loc,std::pair<double,double> obs_vel)
	{
		std::pair<double,double> p_b = this->axis_transform(bot_loc,bot_orien,obs_loc);
		std::pair<double,double> v_b = this->axis_transform(std::make_pair(0,0),bot_orien,obs_vel);
		double u_s = bot_action.first;
		double u_phi = bot_action.second*PI/180;
		double L = 2*this->bot_radius;
		double radii = this->radius;
		double prev = 0;

		double gamma = 0.00008;
		double t = 0;
		while(std::abs(t) < 100)
		{
			// obj = D'(t),differential of distance between bot and obs w.r.t time.
			// finding the time t where the minimal distance between bot and obstace occur using gradient descent
			double obj1 = 2*((L*sin(u_s*tan(u_phi)*t/L)/tan(u_phi)) - p_b.first - v_b.first*t)*((u_s*cos(u_s*tan(u_phi)*t/L)) - v_b.first);
			double obj2 = 2*((-L*cos(u_s*tan(u_phi)*t/L)/tan(u_phi)) + L/tan(u_phi) - p_b.second - v_b.second*t)*((u_s*sin(u_s*tan(u_phi)*t/L)) - v_b.second);
			double obj = obj1 + obj2;			
			
			if(std::abs(obj) < 0.0001){break;}
			
			t = t - gamma*(obj-prev);
			if(t >= 100){return NAN;}
		}
		if(t < 0){return false;}

		double D1 = pow((L*sin(u_s*tan(u_phi)*t/L)/tan(u_phi) - p_b.first - v_b.first*t),2);
		double D2 = pow(((-L*cos(u_s*tan(u_phi)*t/L)/tan(u_phi)) + L/tan(u_phi) - p_b.second - v_b.second*t),2);
		double D = sqrt(D1 + D2);
		// D is the minimum distance between robot and obstacle, if D < sum of bot and obstacle radius means collision
		if(D < radii){return true;}
		else{return false;}

	}

// Computes whether there is gonna be a collision with varying steering angle (d(delta)/dt)
	bool ackermen_check_in(std::pair<double,double> bot_action){
		double delta_c = this->bot_delta;
		double delta_r = bot_action.second;
		double length = 2*this->bot_radius;
		double velocity = bot_action.first;
		float time_del = 0.1;
		std::pair<double,double> obs_loc = this->obs_loc;
		double x_bot = bot_loc.first;
		double y_bot = bot_loc.second;
		double bot_orientation = this->bot_orien*PI/180;
		double pt_new_x,pt_new_y;
		double bot_orientation_new;
		double theta_dot;
		double delta_dot = this->delta_dot;

		while(true)
		{
			// We can't use the same approach that we are using in the above function because we couldn't find the equation for (x,y) in presence of delta_dot
			// Whenever the difference between required and current delta is more than delta_dot*time then it forward integrates the ackermen dynamics 
			// of the vehicle in presence of delta dot and the obstacle using while loop to check whether there is any potential collision
			if(std::abs(delta_r-delta_c) > delta_dot*time_del)
			{
				theta_dot = velocity*tan(delta_c*PI/180)/length;
				if(theta_dot != 0){
					pt_new_x = x_bot + 2*velocity*cos(bot_orientation + theta_dot*time_del/2)*(sin(theta_dot*time_del/2))/theta_dot;
					pt_new_y = y_bot + 2*velocity*sin(bot_orientation + theta_dot*time_del/2)*(sin(theta_dot*time_del/2))/theta_dot;
					bot_orientation_new = bot_orientation + theta_dot*time_del;
				}
				else{
					pt_new_x = x_bot + velocity*cos(bot_orientation)*time_del;
					pt_new_y = y_bot + velocity*sin(bot_orientation)*time_del;
					bot_orientation_new = bot_orientation;
				}
				obs_loc = std::make_pair(obs_loc.first+time_del*this->obs_vel.first,obs_loc.second+time_del*this->obs_vel.second);
				delta_c = delta_c + delta_dot*time_del*(delta_r-delta_c)/std::abs(delta_r-delta_c);
				if(this->Distance(std::make_pair(pt_new_x,pt_new_y),obs_loc) <= this->radius){return true;}
			}

			// Once the required delta is achieved the delta is fixed and we can use the previous function (ackermen_check_in_post) 
			// to check the collision for the remaining motion
			else
			{
				// time_rem is the remaining time left to achieve the required delta from current delta
				// and forward integrates the vehicle dynamics and obstacle motion for time_rem
				double time_rem = std::abs(delta_c-delta_r)/delta_dot;
				theta_dot = velocity*tan(delta_c*PI/180)/length;
				if(theta_dot != 0){
					pt_new_x = x_bot + 2*velocity*cos(bot_orientation + theta_dot*time_rem/2)*(sin(theta_dot*time_rem/2))/theta_dot;
					pt_new_y = y_bot + 2*velocity*sin(bot_orientation + theta_dot*time_rem/2)*(sin(theta_dot*time_rem/2))/theta_dot;
					bot_orientation_new = bot_orientation + theta_dot*time_rem;
				}
				else{
					pt_new_x = x_bot + velocity*cos(bot_orientation)*time_rem;
					pt_new_y = y_bot + velocity*sin(bot_orientation)*time_rem;
					bot_orientation_new = bot_orientation;
				}
				obs_loc = std::make_pair(obs_loc.first+time_rem*this->obs_vel.first,obs_loc.second+time_rem*this->obs_vel.second);
				delta_c = delta_r;

				if(this->Distance(std::make_pair(pt_new_x,pt_new_y),obs_loc) <= this->radius){return true;}

				// if delta is non-zero
				if(bot_action.second != 0){
					return this->ackermen_check_in_post(std::make_pair(pt_new_x,pt_new_y),bot_orientation_new*180/PI,bot_action,obs_loc,this->obs_vel);
				}
				// if delta is zero then the vehicle will go along the orientation, to check for the collision we just 
			 	// need to check whether the orientation lies between two tangents of normal veloctiy obstacles 
				else
				{
					double angle1,angle2,ang_low,ang_high;
					std::pair<double,double> tgt_ret = this->tangent_angles(std::make_pair(pt_new_x,pt_new_y),obs_loc);
					angle1 = tgt_ret.first;
					angle2 = tgt_ret.second;
					if(angle1 > angle2){
						ang_low = angle2;
						ang_high = angle1;
					}
					else{
						ang_low = angle1;
						ang_high = angle2;
					}
					std::pair<double,double> vel_vec = this->get_vector(bot_orientation_new*180/PI,bot_action.first);
					std::pair<double,double> vel_vec_rel = std::make_pair(vel_vec.first-this->obs_vel.first,vel_vec.second-this->obs_vel.second);
					double ang_rel = atan2(vel_vec_rel.second,vel_vec_rel.first)*180/PI;
					ang_rel = mod(ang_rel,360);
					return this->check_in(ang_low,ang_high,ang_rel);
				}
			}

			x_bot = pt_new_x;
			y_bot = pt_new_y;
			bot_orientation = bot_orientation_new;
		}
	}

// Computes RAV
	std::vector<std::pair<double,double> > RAV()
	{
		std::pair<double,double> bot_vel = this->bot_vel;
		double bot_orien = this->bot_orien;
		std::pair<double,double> obs_vel = this->obs_vel;
		double R = this->radius;

		double velocity = bot_vel.first;

		//accelerating and reaching the max velocity (not properly written)
		if(velocity > this->bot_vel_max-0.6){
			velocity = this->bot_vel_max-0.6;
		}

		// double velocity_low = velocity - 0.6;
		double velocity_high = velocity + 0.6;

		if(velocity_low < 0){
			velocity_low = 0;
		}

		std::vector<std::pair<double,double> > rav;
		double i = -this->delta_max;
		while(i <= this->delta_max)
		{
			bool val_check = this->ackermen_check_in(std::make_pair(velocity_high,i));
			if(val_check == false)
			{			
				rav.push_back(std::make_pair(velocity_high,i));
			}
			i = i + 1;
		}
		return rav;
	}

};

double mod(double inp,double inp_wrt){
	while(std::abs(inp) > inp_wrt){
		inp = inp - inp_wrt*inp/std::abs(inp);
	}
	return inp;
}

double Distance(std::pair<double,double> pt1, std::pair<double,double> pt2){
	double value = sqrt((pt2.second - pt1.second)*(pt2.second - pt1.second) + (pt2.first - pt1.first)*(pt2.first - pt1.first));
	return value;
}

double mag(std::pair<double,double> pt){
	return Distance(pt,std::make_pair(0,0));
}

std::pair<double,double> get_vector(double direc,double mag){
	return std::make_pair(mag*cos(direc*PI/180),mag*sin(direc*PI/180));
}

double meas_ang(std::pair<double,double> pt1,std::pair<double,double> pt2)
{
	double dot_pro = pt1.first*pt2.first + pt1.second*pt2.second;
	double val;

	if(dot_pro != 0){val = dot_pro/(mag(pt1)*mag(pt2));}
	else{val = 0;}

	if(std::abs(val) >= 1){val = int(round(val));}

	return acos(val)*180/PI;
}

// chooses the action from rav which is pointed to the goal_loc
std::pair<double,double> select_vel(std::pair<double,double> bot_loc,double bot_orien,std::pair<double,double> goal_loc,std::vector<std::pair<double,double> > pts_vel){
	double goal_dir = atan2(goal_loc.second-bot_loc.second,goal_loc.first-bot_loc.first);
	std::pair<double,double> goal_dir_vec = std::make_pair(cos(goal_dir),sin(goal_dir));
	double ang_min = 10000; //large value
	std::pair<double,double> vel_ret;
	double tyre_orien,ang;

	int i = 0;
	int j = 0;
	while(i < pts_vel.size()){
		tyre_orien = bot_orien + pts_vel[i].second;
		ang = meas_ang(goal_dir_vec,std::make_pair(cos(tyre_orien*PI/180),sin(tyre_orien*PI/180)));
		if(ang < ang_min){
			ang_min = ang;
			vel_ret = pts_vel[i];
			j = 1;
		}
		i = i + 1;
	}
	if(j != 1){
		std::cout<<"ERROR"<<std::endl;
	}
	return vel_ret;
}

std::pair<double,double> axis_transform(std::pair<double,double> origin_new,double orien,std::pair<double,double> pt){
	orien = orien*PI/180;
	return std::make_pair(cos(orien)*(pt.first-origin_new.first)+sin(orien)*(pt.second-origin_new.second),cos(orien)*(pt.second-origin_new.second)-sin(orien)*(pt.first-origin_new.first));
}

// chooses the action which is pointed towards goal in case of no RAV
double choose_delta(std::pair<double,double> bot_loc,double bot_orien,std::pair<double,double> goal,double delta_max){
	double goal_ang = atan2(goal.second-bot_loc.second,goal.first-bot_loc.first);
	bot_orien = bot_orien*PI/180;

	std::pair<double,double> goal_vec = get_vector(goal_ang*180/PI,1);

	std::pair<double,double> goal_vec_new = axis_transform(std::make_pair(0,0),bot_orien*180/PI,goal_vec);
	double ang_wrt_orien = atan2(goal_vec_new.second,goal_vec_new.first)*180/PI;

	if(std::abs(ang_wrt_orien) > delta_max){
		ang_wrt_orien = delta_max*std::abs(ang_wrt_orien)/ang_wrt_orien;
	}

	return ang_wrt_orien;
}

// Computes the perpendicular from a point on to a line
std::pair<double,double> drop_point(std::pair<std::pair<double,double>,std::pair<double,double>>pts_line,std::pair<double,double> pt){
	std::pair<double,double> pt1 = pts_line.first;
	std::pair<double,double> pt2 = pts_line.second;
	std::pair<double,double> pt_ret;

	if(pt1.first != pt2.first){
		if(pt1.second != pt2.second){
			double m1 = (pt2.second - pt1.second)/(pt2.first - pt1.first);
			double c1 = (pt1.second*pt2.first - pt1.first*pt2.second)/(pt2.first - pt1.first);
			double c2 = pt.second - ((pt1.first-pt2.first)/(pt2.second-pt1.second))*pt.first;
			pt_ret = std::make_pair(float(m1*c2 - m1*c1)/float(pow(m1,2) + 1),float(m1*m1*c2 + c1)/float(pow(m1,2) + 1));
		}
		else{
			pt_ret = std::make_pair(pt.first,pt1.second);	//since pt1.second = pt2.second;
		}
	}
	else{
		pt_ret = std::make_pair(pt1.first,pt.second);
	}

	return std::make_pair(float(pt_ret.first),float(pt_ret.second));
}


isinStruct pt_isin(std::pair<double,double> pt1,std::pair<double,double> pt2,std::pair<double,double> pt){
	double dist_pt_1 = Distance(pt,pt1);
	double dist_pt_2 = Distance(pt,pt2);
	double dist_1_2 = Distance(pt1,pt2);
	isinStruct ret;

	if(dist_pt_1 > dist_1_2 or dist_pt_2 > dist_1_2){
		if(dist_pt_1 > dist_1_2){
			ret.val = 2;
			ret.ret = false;
			return ret;
		}
		if(dist_pt_2 > dist_1_2){
			ret.val = 1;
			ret.ret = false;
			return ret;
		}
	}
	else{
		ret.val = 0;
		ret.ret = true;
		return ret;
	}
}

// computes the point from a given point and the direction
std::pair<double,double> point_dir(std::pair<double,double> pt,double dir,double dist){
	return std::make_pair(pt.first + dist*cos(dir),pt.second + dist*sin(dir));
}

// Computes the point to pursue, it is 'distFROMdrop' away from perpendicular drop on the line
nextStruct next_pt(std::vector<std::pair<double,double> > pts,std::pair<double,double> loc,double distFROMdrop){
	std::pair<double,double> drop = drop_point(std::make_pair(pts[0],pts[1]),loc);
	std::pair<double,double> pt_ret = point_dir(drop,atan2(pts[1].second-pts[0].second,pts[1].first-pts[0].first),distFROMdrop);
	isinStruct isinRet = pt_isin(pts[0],pts[1],pt_ret);
	int val = isinRet.val;
	bool bool_in = isinRet.ret;
	nextStruct ret;


	if(val == 1){
		ret.val = 0;
		ret.pt = pts[0];
		return ret;
	}

	int i = 1;
	while(bool_in == false){
		if(i == pts.size()-1){
			pt_ret = pts[pts.size()-1];
			break;
		}
		distFROMdrop = distFROMdrop - Distance(drop,pts[i]);
		pt_ret = point_dir(pts[i],atan2(pts[i+1].second-pts[i].second,pts[i+1].first-pts[i].first),distFROMdrop);
		isinStruct isinRet2 = pt_isin(pts[i],pts[i+1],pt_ret);
		val = isinRet2.val;
		bool_in = isinRet2.ret;
		if(val == 1){
			ret.val = i;
			ret.pt = pts[i];
			return ret;
		}
		drop = pts[i];
		i = i + 1;
	}

	ret.val = i-1;
	ret.pt = pt_ret;
	return ret;
}

// (x1,y1)-(x2,y2) => (x1-x2,y1-y2)
std::pair<double,double> subt(std::pair<double,double> pt1,std::pair<double,double> pt2){
	std::pair<double,double> ret;
	ret.first = pt1.first - pt2.first;
	ret.second = pt1.second - pt2.second;
	return ret;
}

std::vector<std::pair<double,double> > red_pts(std::vector<std::pair<double,double> > pts){
	std::vector<std::pair<double,double> > pts_ret;
	pts_ret.push_back(pts[0]);

	int i = 1;
	while(i < pts.size()){
		if(i == pts.size() - 1){
			pts_ret.push_back(pts[i]);
			break;
		}
		if(round(meas_ang(subt(pts[i],pts_ret[pts_ret.size()-1]),subt(pts[i+1],pts_ret[pts_ret.size()]))) != 0){
			pts_ret.push_back(pts[i]);
		}

		i = i + 1;
	}
	return pts_ret;
}


int main(){

	std:: pair<double,double> bot_loc = std::make_pair(360,20);
	double bot_orien = 0;
	double bot_delta = 0;
	std:: pair<double,double> obs_loc = std::make_pair(20,480);
	std:: pair<double,double> goal = std::make_pair(20,460);
	std:: pair<double,double> bot_vel = std::make_pair(10,bot_delta*180/PI);
	std:: pair<double,double> obs_vel = std::make_pair(9,-0.2);
	double bot_radius = 10;
	double obs_radius = 10;
	double offset = 0;
	double delta_max = 30;
	double delta_dot = 20;
	double bot_vel_max = 20;
	double dist_thresh_VO = 150;
	double L = 2*bot_radius;
	double dist_drop = 30;
	double obs_len = 25;
	float time_del = 0.1;
	std::vector<std::pair<double,double> > rav;

	cv::Mat img = cv::Mat::zeros(500,500,CV_8UC3);
	cv::Mat steer_img = cv::Mat::zeros(100,100,CV_8UC3);
	cv::line(steer_img,cv::Point(50+int(round(35*cos(-PI/3))),50+int(round(35*sin(-PI/3)))),cv::Point(50+int(round(45*cos(-PI/3))),50+int(round(45*sin(-PI/3)))),cv::Scalar(0,0,255),2,8);
	cv::line(steer_img,cv::Point(50+int(round(35*cos(-2*PI/3))),50+int(round(35*sin(-2*PI/3)))),cv::Point(50+int(round(45*cos(-2*PI/3))),50+int(round(45*sin(-2*PI/3)))),cv::Scalar(0,0,255),2,8);
	cv::circle(img,cv::Point(int(bot_loc.first),int(bot_loc.second)),3,cv::Scalar(255,255,255),-1,8);
	cv::circle(img,cv::Point(int(goal.first),int(goal.second)),3,cv::Scalar(255,255,255),-1,8);

	double bot_vel_mag;
	double theta_dot;
	std::pair<double,double> bot_loc_new;
	std::pair<double,double> obs_loc_new;
	double bot_orien_new;
	double bot_delta_new;
	double bot_loc_new_x;
	double bot_loc_new_y;
	std::pair<double,double> pt_follow;
	int key,ind;

	std::pair<std::pair<double,double>,std::pair<double,double> > inf_stgo = std::make_pair(bot_loc,goal);
	std::vector<std::pair<double,double> > obstacle_pts;

	std::vector<std::pair<double,double> > path_pts;
	path_pts.push_back(std::make_pair(360,20));
	path_pts.push_back(std::make_pair(360,460));
	path_pts.push_back(std::make_pair(20,460));

	std::pair<double,double> prev_pt = path_pts[0];

	for(int k=1;k < path_pts.size();k++){
		cv::line(img,cv::Point(int(prev_pt.first),int(prev_pt.second)),cv::Point(int(path_pts[k].first),int(path_pts[k].second)),cv::Scalar(255,255,255),2,8);
		prev_pt = path_pts[k];
	}

	cv::imshow("image",img);
	cv::imshow("Steer",steer_img);
	cv::waitKey(0);

	double t = 0;

	//forward integrating the vehicle dynamics based on the best control action in each iteration
	while(true){
		// std::cout<<'1'<<std::endl;
		cv::Mat steer_copy = steer_img.clone();
		cv::line(steer_copy,cv::Point(50,50),cv::Point(50+int(40*cos(-PI/2 + bot_delta*PI/180)),int(50+40*sin(-PI/2 + bot_delta*PI/180))),cv::Scalar(255,0,0),3,8);
		if(Distance(goal,bot_loc) < 10){
			std::cout<<"reached"<<std::endl;
			break;
		}

		if(Distance(bot_loc,obs_loc) < bot_radius+obs_radius){
			std::cout<<"collision"<<std::endl;
			break;
		}

		nextStruct next_ret = next_pt(path_pts,bot_loc,dist_drop);
		ind = next_ret.val;
		// this is just to give one point to follow at every instant
		pt_follow = next_ret.pt;
		path_pts.erase(path_pts.begin(), path_pts.begin()+ind);

		img.at<cv::Vec3b>(int(round(bot_loc.second)),int(round(bot_loc.first))) = cv::Vec3b(255,0,0);
		img.at<cv::Vec3b>(int(round(obs_loc.second)),int(round(obs_loc.first))) = cv::Vec3b(0,0,255);
		cv::Mat img_copy = img.clone();

		cv::circle(img_copy,cv::Point(int(bot_loc.first),int(bot_loc.second)),bot_radius,cv::Scalar(255,0,0),-1,8);
		cv::circle(img_copy,cv::Point(int(obs_loc.first),int(obs_loc.second)),obs_radius,cv::Scalar(0,0,255),-1,8);
		cv::circle(img_copy,cv::Point(int(pt_follow.first),int(pt_follow.second)),7,cv::Scalar(255,255,255),-1,8);
		cv::imshow("image",img_copy);
		cv::imshow("Steer",steer_copy);

		key = cv::waitKey(1);
		if(key == 27){
			break;
		}
		else if(key == 32){
			cv::waitKey(0);
		}

		// Computing RAV only if the obstacle at some threshold distance to bot
		// After computing the rav we are choosing the action with such delta which points directly or close to the pt_follow, this
		// is just a simple planner to test the code.
		if(Distance(bot_loc,obs_loc) < dist_thresh_VO){
			velocityObstacles vel_obs(bot_loc,bot_orien,bot_delta,bot_vel,bot_radius,delta_max,delta_dot,bot_vel_max,obs_loc,obs_vel,obs_radius+offset);
			rav = vel_obs.RAV();
			bot_vel = select_vel(bot_loc,bot_orien,pt_follow,rav);
			if(bot_vel.second != 0){
				std::cout<<vel_obs.ackermen_check_in(bot_vel)<<std::endl;
			}
		}
		// if the bot is far from obstacle then it just chooses the delta that points to pt_follow
		else{
			bot_vel_mag = mag(bot_vel) + 0.6;
			if(bot_vel_mag > bot_vel_max){
				bot_vel_mag = bot_vel_max;
			}
			bot_vel = std::make_pair(bot_vel_mag,choose_delta(bot_loc,bot_orien,pt_follow,delta_max));
		}
		std::cout<<"Bot vel : "<<bot_vel.first<<','<<bot_vel.second<<std::endl;
		// std::cout<<"Theta dot : "<<theta_dot<<std::endl;
		// std::cout<<"Delta : "<<bot_delta<<std::endl;

		//forward integrating the dynamics based on the given control action
		theta_dot = bot_vel.first*tan(bot_delta*PI/180)/L;
		if(theta_dot != 0){
			bot_loc_new_x = bot_loc.first + (2*bot_vel.first/theta_dot)*(cos(bot_orien*PI/180 + theta_dot*time_del/2)*sin(theta_dot*time_del/2));
			bot_loc_new_y = bot_loc.second + (2*bot_vel.first/theta_dot)*(sin(bot_orien*PI/180 + theta_dot*time_del/2)*sin(theta_dot*time_del/2));
			bot_loc_new = std::make_pair(bot_loc_new_x,bot_loc_new_y);
		}
		else{
			bot_loc_new = std::make_pair(bot_loc.first+time_del*bot_vel.first*cos(bot_orien*PI/180),bot_loc.second+time_del*bot_vel.first*sin(bot_orien*PI/180));
		}
		bot_orien_new = bot_orien + theta_dot*time_del*180/PI;
		obs_loc_new = std::make_pair(obs_loc.first+time_del*obs_vel.first,obs_loc.second+time_del*obs_vel.second);

		if(std::abs(bot_vel.second - bot_delta) > delta_dot*time_del){
			if(bot_vel.second > bot_delta){
				bot_delta_new = bot_delta + delta_dot*time_del;
			}
			else{
				bot_delta_new = bot_delta - delta_dot*time_del;
			}
		}
		else{
			bot_delta_new = bot_vel.second;
		}

		if(std::abs(bot_delta_new) > delta_max){
			bot_delta_new = delta_max*bot_delta_new/std::abs(bot_delta_new);
		}
		bot_loc = bot_loc_new;
		bot_orien = bot_orien_new;
		obs_loc = obs_loc_new;
		bot_delta = bot_delta_new;
		t = t + time_del;
		std::cout<<"###################"<<std::endl;
	}





	// [374.1968414852692, 481.4970074724578] , 77.9647510841 , 0.0174532925199 , [20.0, 30] , 10 , 30 , 0.349065850399 , 20 , [354.1000000000018, 477.43000000000234] , [13, -0.1] , 10
	// velocityObstacles vel_obs(std::make_pair(374.9968414852692, 482.4970074724578) , 77.9647510841 , 0.0174532925199 , std::make_pair(20.0, 30) , 10 , 30 , 0.349065850399 , 20 , std::make_pair(354.1000000000018, 477.43000000000234) , std::make_pair(13, -0.1) , 10);
	// std::vector<std::pair<double,double> > rav = vel_obs.RAV();

	// for(int i=0;i<rav.size();i++){
	// 	std::cout<<rav[i].first<<','<<rav[i].second<<std::endl;
	// }

	// std::cout<<rav.size()<<std::endl;
	// std::cout<<vel_obs.ackermen_check_in(std::make_pair(20,30))<<std::endl;
	return 0;
}