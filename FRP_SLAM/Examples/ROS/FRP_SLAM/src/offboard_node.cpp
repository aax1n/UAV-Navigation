#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

int count;
int flag=1;
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("iris_0/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("iris_0/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("iris_0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("iris_0/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(8.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.6;
   
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    
    while(ros::ok())
   {
      
   if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
      
            if((flag == 1)  && (ros::Time::now() - last_request > ros::Duration(5.0)))
      { 
         ROS_INFO("position1(0 , 0,  1.6)");
                  pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 1.6;
         last_request = ros::Time::now();
                        flag=2;
         //local_pos_pub.publish(pose);

                }

      if((flag ==2) && (ros::Time::now() - last_request > ros::Duration(5.0)))
      {
         ROS_INFO("position2(1 , 0,  1.6)");
                  pose.pose.position.x = 1;
            pose.pose.position.y = 0.0;
            pose.pose.position.z = 1.6;
         last_request = ros::Time::now();
         flag=3;
         //local_pos_pub.publish(pose);
                }
                       
       if((flag ==3) && (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                        ROS_INFO("position3(1 , 1,  1.6)");
                        pose.pose.position.x = 1;
                        pose.pose.position.y = 1;
                        pose.pose.position.z = 1.6;
                        last_request = ros::Time::now();
         flag=4;
                        //local_pos_pub.publish(pose);
                }

       if((flag ==4) && (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                        ROS_INFO("position4(0 , 1,  1.6)");
                        pose.pose.position.x = 0;
                        pose.pose.position.y = 1;
                        pose.pose.position.z = 1.6;
                        last_request = ros::Time::now();
         flag=5;
                        //local_pos_pub.publish(pose);
                }


       if((flag ==5) && (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                        ROS_INFO("position5(0 ,0,  1.6)");
                        pose.pose.position.x = 0;
                        pose.pose.position.y = 0;
                        pose.pose.position.z = 1.6;
                        last_request = ros::Time::now();
         flag=6;
                        //local_pos_pub.publish(pose);
                }
       if((flag ==6) && (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                	offb_set_mode.request.custom_mode = "AUTO.LAND";
       		if( current_state.mode != "AUTO.LAND" &&
            		(ros::Time::now() - last_request > ros::Duration(5.0))){
            			if( set_mode_client.call(offb_set_mode) &&
                		offb_set_mode.response.mode_sent){
                			ROS_INFO("AUTO.LAND enabled");
            			}
            			last_request = ros::Time::now();
        		} 
        flag=7;
        	}
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
