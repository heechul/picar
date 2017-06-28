#include <string>
#include <cmath>
#include <chrono>
#include <thread>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>




class CarPublisher
{

        public:

                CarPublisher();
                ~CarPublisher();
                void initialize_joint_states(sensor_msgs::JointState* joint_state);
                void initialize_transformation(geometry_msgs::TransformStamped* trans);
                void receive_Twist(const geometry_msgs::Twist msg);
                void publish_Odometry();
                static void move_car(CarPublisher* car_pub);

                sensor_msgs::JointState* joint_state;
                geometry_msgs::TransformStamped* trans;

                ros::NodeHandle node;
                ros::Publisher joint_pub;
                ros::Publisher odom_pub;
                ros::Subscriber get_twist;
                tf::TransformBroadcaster broadcaster;

                double f_left, f_right, back_wheels;
                double original_speed, current_linx, current_angz, turn, angle;
                double px, py, th, vx, vy, vth;
                bool first_message;

};




CarPublisher::CarPublisher()
{
        this->joint_state = new sensor_msgs::JointState();
        initialize_joint_states(joint_state);

        this->trans = new geometry_msgs::TransformStamped();
        initialize_transformation(trans);

        joint_pub = node.advertise<sensor_msgs::JointState>("joint_states", 1);
        //odom_pub = node.advertise<nav_msgs::Odometry>("odom", 1);
        get_twist = node.subscribe("/cmd_vel", 1, &CarPublisher::receive_Twist, this);

        this->f_left = 0; this->f_right = 0; this->back_wheels = 0;
        this->original_speed = 0; this->current_linx = 0; this->current_angz = 0;
        this->turn = 1; this->angle += (M_PI/4);
        this->first_message = false;
        this->px = 0; this->py = 0; this->th = 0; this->vx = 0.1; this->vy = -0.1; this->vth = 0.1;

}



CarPublisher::~CarPublisher()
{
        delete this->joint_state;
        delete this->trans;
}



void CarPublisher::initialize_joint_states(sensor_msgs::JointState* joint_state)
{

        joint_state->header.frame_id = "base_footprint";
        joint_state->header.stamp = ros::Time::now();
        joint_state->name.resize(7);
        joint_state->position.resize(7);
        std::string names[7] = { "base_footprint_joint" , "front_axle_to_base" , "back_axle_to_base" ,
        "front_left_wheel_to_front_axle" , "front_right_wheel_to_front_axle" ,
        "back_left_wheel_to_back_axle" , "back_right_wheel_to_back_axle" };

        for( int i = 0 ; i < 7 ; i++ )
        {
                joint_state->name[i] = names[i];
                joint_state->position[i] = 0;
        }

}


void CarPublisher::initialize_transformation(geometry_msgs::TransformStamped* trans)
{

        trans->header.frame_id = "odom";
        trans->child_frame_id = "base_footprint";
        trans->header.stamp = ros::Time::now();
        trans->transform.translation.x = 0;
        trans->transform.translation.y = 0;
        trans->transform.translation.z = 0;
        trans->transform.rotation = tf::createQuaternionMsgFromYaw(M_PI*4);

}


void CarPublisher::receive_Twist(const geometry_msgs::Twist msg)
{

        this->first_message = true;

        if( std::abs(msg.linear.x) != std::abs(current_linx) )
        {
                current_linx = msg.linear.x;
        }

        if( std::abs(msg.angular.z) != std::abs(current_angz) ) // else, only a change in angular speed has been detected
        {
                current_angz = msg.angular.z;
                turn = msg.angular.z / original_speed;
        }



        if( msg.linear.y == 0 )// move
        {

                if( msg.linear.x > 0 )// if( forward
                {
                        if( msg.angular.z > 0 )// u
                        {
                                f_left = 0.785;
                                f_right = 0.393;
                                back_wheels += 3.14;
                                angle += (M_PI/4);
                        }

                        else if( msg.angular.z == 0 )// i
                        {
                                f_left = 0;
                                f_right = 0;
                                back_wheels += 3.14;
                        }

                        else // o
                        {
                                f_left = -0.393;
                                f_right = -0.785;
                                back_wheels += 3.14;
                                angle -= (M_PI/4);
                        }

                }

                else if( msg.linear.x == 0 )// if( rotate
                {
                        if( msg.angular.z > 0 )// j
                        {
                                f_left = 0.785;
                                f_right = 0.393;
                                angle += (M_PI/4);
                        }

                        else if( msg.angular.z == 0 )// k
                        {
                                f_left = 0;
                                f_right = 0;
                        }

                        else // l
                        {
                                f_left = -0.393;
                                f_right = -0.785;
                                angle -= (M_PI/4);
                        }
                }

                else // if( reverse
                {
                        if( msg.angular.z < 0 )// m
                        {
                                f_left = 0.785;
                                f_right = 0.393;
                                back_wheels -= 3.14;
                                angle += (M_PI/4);
                        }

                        else if( msg.angular.z == 0 )// ,
                        {
                                f_left = 0;
                                f_right = 0;
                                back_wheels -= 3.14;
                        }

                        else // .
                        {
                                f_left = -0.393;
                                f_right = -0.785;
                                back_wheels -= 3.14;
                                angle -= (M_PI/4);
                        }
                }

        }

        if(angle < 0)
        {
                angle = 2*M_PI + angle;
        }
        else if(angle > 2*M_PI)
        {
                angle -= 2*M_PI;
        }

        int x_dir, y_dir;
        if(angle => 0 && angle < 90)
        {
                x_dir = 1;
                y_dir = 1;
        }
        else if(angle >= 90 && angle < 180)
        {
                x_dir = 1;
                y_dir = 1;
        }
        else if(angle >= 180 && angle < 270)
        {
                x_dir = 1;
                y_dir = 1;
        }
        else
        {
                x_dir = 1;
                y_dir = 1;
        }


        this->joint_state->position[0] = 0;
        this->joint_state->position[1] = 0;
        this->joint_state->position[2] = 0;
        this->joint_state->position[3] = f_left;
        this->joint_state->position[4] = f_right;
        this->joint_state->position[5] = back_wheels;
        this->joint_state->position[6] = back_wheels;

        this->joint_state->header.stamp = ros::Time::now();
        joint_pub.publish(*this->joint_state);

        //tf::Quaternion quaternion = tf::Quaternion(0, 0, 0, 1);
        this->trans->transform.translation.x += x_dir*msg.linear.x*cos(angle);
        this->trans->transform.translation.y -= y_dir*msg.linear.x*sin(angle);
        this->trans->transform.translation.z += msg.linear.z;
        this->trans->transform.rotation = tf::createQuaternionMsgFromYaw(angle);
        this->trans->header.stamp = ros::Time::now();
        broadcaster.sendTransform(*this->trans);

}



void CarPublisher::publish_Odometry()
{

/*
        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        px += delta_x;
        py += delta_y;
        th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
*/

}


void CarPublisher::move_car(CarPublisher* car_pub)
{

        for(;;)
        {
                if(car_pub->first_message) {break;}
                car_pub->joint_state->header.stamp = ros::Time::now();
                car_pub->joint_pub.publish(*car_pub->joint_state);
                car_pub->trans->header.stamp = ros::Time::now();
                car_pub->broadcaster.sendTransform(*car_pub->trans);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }

}


int main(int argc, char** argv)
{

        ros::init(argc, argv, "car_publisher");
        CarPublisher* car_pub = new CarPublisher();


        std::thread thr(CarPublisher::move_car, car_pub);
        //thr.join();

        if(car_pub->first_message)
        {
                thr.detach();
        }

        ros::spin();
        delete car_pub;

        return 0;

}
