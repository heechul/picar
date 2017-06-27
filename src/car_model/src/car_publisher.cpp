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
                static void move_car(CarPublisher* car_pub);

                sensor_msgs::JointState* joint_state;
                geometry_msgs::TransformStamped* trans;
                ros::NodeHandle node;
                ros::Publisher joint_pub;
                ros::Publisher odom_pub;
                ros::Subscriber get_twist;
                tf::TransformBroadcaster broadcaster;

                int f_left, f_right, back_wheels;
                int original_linx, previous_linx, current_linx, turn, angle;
                bool first_message;

};




CarPublisher::CarPublisher()
{
        this->joint_state = new sensor_msgs::JointState();
        initialize_joint_states(joint_state);

        this->trans = new geometry_msgs::TransformStamped();
        initialize_transformation(trans);

        joint_pub = node.advertise<sensor_msgs::JointState>("joint_states", 1);
        //ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>("odom", 1);
        get_twist = node.subscribe("/cmd_vel", 1, &CarPublisher::receive_Twist, this);

        this->f_left = 0;
        this->f_right = 0;
        this->back_wheels = 0;
        this->original_linx = 0;
        this->current_linx = 0;
        this->previous_linx = 0;
        this->turn = 1;
        this->angle = 1;
        this->first_message = false;

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
        trans->transform.rotation = tf::createQuaternionMsgFromYaw(M_PI*2);

}


void CarPublisher::receive_Twist(const geometry_msgs::Twist msg)
{

        this->first_message = true;

        if( std::abs(msg.linear.x) != std::abs(current_linx) )
        {
                previous_linx = current_linx;
                current_linx = msg.linear.x;
        }

        if( std::abs(msg.angular.z) != std::abs(turn) ) // else, only a change in angular speed has been detected
        {
                turn = msg.angular.z / original_linx;
                angle += msg.angular.z*(M_PI/2);
        }



        if( msg.linear.y == 0 )// move
        {

                if( msg.linear.x > 0 )// if( forward
                {
                        if( msg.angular.z > 0 )// u
                        {
                                f_left = 1;
                                f_right = 1;
                                back_wheels += 3.14;
                        }

                        else if( msg.angular.z == 0 )// i
                        {
                                f_left = 0;
                                f_right = 0;
                                back_wheels += 3.14;
                        }

                        else // o
                        {
                                f_left = -1;
                                f_right = -1;
                                back_wheels += 3.14;
                        }
                }

                else if( msg.linear.x == 0 )// if( rotate
                {
                        if( msg.angular.z > 0 )// j
                        {
                                f_left = 1;
                                f_right = 1;
                        }

                        else if( msg.angular.z == 0 )// k
                        {
                                f_left = 0;
                                f_right = 0;
                        }

                        else // l
                        {
                                f_left = -1;
                                f_right = -1;
                        }
                }

                else // if( reverse
                {
                        if( msg.angular.z < 0 )// m
                        {
                                f_left = 1;
                                f_right = 1;
                                back_wheels -= 3.14;
                        }

                        else if( msg.angular.z == 0 )// ,
                        {
                                f_left = 0;
                                f_right = 0;
                                back_wheels -= 3.14;
                        }

                        else // .
                        {
                                f_left = -1;
                                f_right = -1;
                                back_wheels -= 3.14;
                        }
                }

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
        this->trans->transform.translation.x += msg.linear.x*cos(angle);
        this->trans->transform.translation.y += msg.linear.x*sin(angle);
        this->trans->transform.translation.z += msg.linear.z;
        trans->transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
        this->trans->header.stamp = ros::Time::now();
        broadcaster.sendTransform(*this->trans);

}


void CarPublisher::move_car(CarPublisher* car_pub)
{

        for(;;)
        {
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
