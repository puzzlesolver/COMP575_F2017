#include <ros/ros.h>

// ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>

// ROS messages
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "Pose.h"
#include "TargetState.h"

// Custom messages
#include <shared_messages/TagsImage.h>

// To handle shutdown signals so the node quits properly in response to "rosnode kill"

#include <signal.h>
#include <math.h>

using namespace std;

// Random number generator
random_numbers::RandomNumberGenerator *rng;



string rover_name;
char host[128];
bool is_published_name = false;
int simulation_mode = 0;
float mobility_loop_time_step = 0.1;
float status_publish_interval = 5;
float kill_switch_timeout = 10;

pose current_location;

int transitions_to_auto = 0;
double time_stamp_transition_to_auto = 0.0;

// state machine states
#define STATE_MACHINE_TRANSLATE 0
int state_machine_state = STATE_MACHINE_TRANSLATE;

//Publishers
ros::Publisher velocityPublish;
ros::Publisher stateMachinePublish;
ros::Publisher status_publisher;
ros::Publisher target_collected_publisher;
ros::Publisher angular_publisher;
ros::Publisher messagePublish;
ros::Publisher debug_publisher;
ros::Publisher posePublish;
ros::Publisher global_average_heading_publisher;
ros::Publisher local_average_heading_publisher;

//Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;

ros::Subscriber messageSubscriber;
ros::Subscriber poseSubscriber;
ros::Subscriber globalAverageHeadingSubscriber;
ros::Subscriber localAverageHeadingSubscriber;

//Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer killSwitchTimer;

// Mobility Logic Functions
void setVelocity(double linearVel, double angularVel);

// OS Signal Handler
void sigintEventHandler(int signal);

// Callback handlers
void joyCmdHandler(const geometry_msgs::Twist::ConstPtr &message);
void modeHandler(const std_msgs::UInt8::ConstPtr &message);
void targetHandler(const shared_messages::TagsImage::ConstPtr &tagInfo);
void obstacleHandler(const std_msgs::UInt8::ConstPtr &message); //
void odometryHandler(const nav_msgs::Odometry::ConstPtr &message);
void mobilityStateMachine(const ros::TimerEvent &);
void publishStatusTimerEventHandler(const ros::TimerEvent &event);
void killSwitchTimerEventHandler(const ros::TimerEvent &event);
void messageHandler(const std_msgs::String::ConstPtr &message);
void poseHandler(const std_msgs::String::ConstPtr &message);
void globalAverageHeadingHandler(const std_msgs::String::ConstPtr &message);
void localAverageHeadingHandler(const std::msgs::String::ConstPtr &message);

class Rover // I received help from Jerry Zhang creating this class
{
    //constructor that contains all rover attributes
    Rover(string roverName, float x, float y, float theta)
    {
        name = roverName;
        X(x);
        Y(y);
        Theta(theta);
    }
    Rover(string roverName, pose p)
    {
        name = roverName;
        Pose(p);
    }
    //x, y, and theta functions to be used in the constructor function
    void X(float x) {location.x=x;}
    void Y(float y) {location.y=y;}
    void Theta(float theta) {location.theta=theta;}
    void Pose(pose p)
    {
        X(p.x);
        Y(p.y);
        Theta(p.theta);
    }

    inline float X(void) {return location.x;}
    inline float Y(void) {return location.Y;}
    inline float Theta(void) {return location.theta;}
    inline pose Pose(void) {return location;}
    inline string Name(void) {return name;}
    // Calculate the neighbors of a rover
    bool isRoverClose(Rover, otherRover, float distance)
    {
        //pythagorean theorem and solve for c^2 instead of solving for c
        float distX = X() -otherRover.X();
        float distY = Y() -otherRover.Y();
        return distX*distX + DistY*DistY <=distance*distance ;
    }
private:
    string name;
    pose location;
};

class Rovers
{
public:
    void addRover(string name, float x, float y, float theta)
    {
        for (std::vector<Rover>::iterator currentRover = theRovers.begin();
             currentRover != theRovers.end();
             ++currentRover)
        {
            if ((*currentRover).Name() == name)
            {
                (*currentRover).X(x);
                (*currentRover).Y(y);
                (*currentRover).Theta(theta);
                return;
            }
        }
        Rover newRover(name, x, y, theta);
        theRovers.push_back(newRover);
    }
    inline void addRover(Rover r)
    {
        addRover(r.Name(), r.X(), r.Y(), r.Theta());
    }
    inline void updateRover(string name, float x, float y, float theta)
    {
        addRover(name, x, y, theta);
    }
    float calculateAverageBearing()
    {
        int roverCount = theRovers.size();
        switch(roverCount)
        {
        case 0:
            return 0.0;
        case 1:
            return theRovers.front().Theta();
        default:
            for (std::vector<Rover>::iterator currentRover = theRovers.begin(); currentRover != theRovers.end();
                 ++currentRover)
            {
                x_comp +=cos((*currentRover).Theta());
                y_comp +=sin((*currentRover).Theta());
            }
            return atan2(y_comp/roverCount, x_comp/roverCount);
        }
    }
    float calculateAverageNeighborBearing(float x, float y)
    {
        for (std::vector<Rover>::iterator currentRover = theRovers.begin(); currentRover != theRovers.end();
             ++currentRover)
        {
            Rover centerPoint("centerPoint", x, y, 0.0);
            int roverCount = 0;
            float x_comp = 0.0;
            float y_comp = 0.0;
            for (std::vector<Rover>::iterator neighborRover = theRovers.begin(); neighborRover != theRovers.end();
                 ++neighborRover)
            {
                if (*currentRover != *neighborRover)
                {
                    if (*currentRover.isRoverClose(*neighborRover),2.0)
                    {
                        x_comp +=cos(*neighborRover);
                        y_comp +=sin(*neighborRover);
                        roverCount++;
                    }
                }
            }
            switch(roverCount)
            {
            case 0:
                return 0.0;
            case 1:
                return *currentRover;
            default:
                return atan2(y_comp/roverCount, x_comp/roverCount)
            }
        }
    }
private:
    std::vector<Rover> theRovers;
}all_rovers;

int main(int argc, char **argv)
{
    gethostname(host, sizeof(host));
    string hostName(host);

    rng = new random_numbers::RandomNumberGenerator(); // instantiate random number generator

    if (argc >= 2)
    {
        rover_name = argv[1];
        cout << "Welcome to the world of tomorrow " << rover_name << "!  Mobility module started." << endl;
    } else
    {
        rover_name = hostName;
        cout << "No Name Selected. Default is: " << rover_name << endl;
    }
    // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
    ros::init(argc, argv, (rover_name + "_MOBILITY"), ros::init_options::NoSigintHandler);
    ros::NodeHandle mNH;

    signal(SIGINT, sigintEventHandler); // Register the SIGINT event handler so the node can shutdown properly

    joySubscriber = mNH.subscribe((rover_name + "/joystick"), 10, joyCmdHandler);
    modeSubscriber = mNH.subscribe((rover_name + "/mode"), 1, modeHandler);
    targetSubscriber = mNH.subscribe((rover_name + "/targets"), 10, targetHandler);
    obstacleSubscriber = mNH.subscribe((rover_name + "/obstacle"), 10, obstacleHandler);
    odometrySubscriber = mNH.subscribe((rover_name + "/odom/ekf"), 10, odometryHandler);
    messageSubscriber = mNH.subscribe(("messages"), 10, messageHandler);
    poseSubscriber = mNH.subscribe(("poses"), 10, poseHandler);

    status_publisher = mNH.advertise<std_msgs::String>((rover_name + "/status"), 1, true);
    velocityPublish = mNH.advertise<geometry_msgs::Twist>((rover_name + "/velocity"), 10);
    stateMachinePublish = mNH.advertise<std_msgs::String>((rover_name + "/state_machine"), 1, true);
    messagePublish = mNH.advertise<std_msgs::String>(("messages"), 10, true);
    target_collected_publisher = mNH.advertise<std_msgs::Int16>(("targetsCollected"), 1, true);
    angular_publisher = mNH.advertise<std_msgs::String>((rover_name + "/angular"),1,true);
    publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
    killSwitchTimer = mNH.createTimer(ros::Duration(kill_switch_timeout), killSwitchTimerEventHandler);
    stateMachineTimer = mNH.createTimer(ros::Duration(mobility_loop_time_step), mobilityStateMachine);
    debug_publisher = mNH.advertise<std_msgs::String>("/debug", 1, true);
    messagePublish = mNH.advertise<std_msgs::String>(("messages"), 10 , true);

    posePublish = mNH.advertise<std_msgs::String>(("poses"), 10, true);
    globalAverageHeadingSubscriber = mNH.subscribe(("globalAverageHeading"), 10, globalAverageHeadingHandler);
    localAverageHeadingSubscriber = mNH.subscribe(("localAverageHeading"),10, localAverageHeadingHandler);
    global_average_heading_publisher = mNH.advertise<std_msgs::String>(("globalAverageHeading"), 10, true);
    local_average_heading_publisher = mNH.advertise<std_msgs::String>(("localAverageHeading"), 10, true);

    ros::spin();
    return EXIT_SUCCESS;
}

void mobilityStateMachine(const ros::TimerEvent &)
{
    std_msgs::String state_machine_msg;
    std_msgs::String pose_msg;

    if ((simulation_mode == 2 || simulation_mode == 3)) // Robot is in automode
    {
        if (transitions_to_auto == 0)
        {
            // This is the first time we have clicked the Autonomous Button. Log the time and increment the counter.
            transitions_to_auto++;
            time_stamp_transition_to_auto = ros::Time::now().toSec();
        }
        switch (state_machine_state)
        {
        case STATE_MACHINE_TRANSLATE:
        {
            state_machine_msg.data = "TRANSLATING";//, " + converter.str();
            float angular_velocity = 0.2;
            float linear_velocity = 0.1;
            setVelocity(linear_velocity, angular_velocity);
            break;
        }
        default:
        {
            state_machine_msg.data = "DEFAULT CASE: SOMETHING WRONG!!!!";
            break;
        }
        }

    }
    else
    { // mode is NOT auto

        // publish current state for the operator to seerotational_controller
        std::stringstream converter;
        converter <<"CURRENT MODE: " << simulation_mode;

        state_machine_msg.data = "WAITING, " + converter.str();
    }
    std::stringstream rover_info; //start setting up the positions of the rover
    rover_info << rover_name << " - ";
    rover_info << current_location.x << ", ";
    rover_info << current_location.y << ", ";
    rover_info << current_location.theta;
    pose_msg.data = rover_info.str(); //done setting up the positions
    stateMachinePublish.publish(state_machine_msg);
    posePublish.publish(pose_msg);
}

void setVelocity(double linearVel, double angularVel)
{
    geometry_msgs::Twist velocity;
    // Stopping and starting the timer causes it to start counting from 0 again.
    // As long as this is called before the kill switch timer reaches kill_switch_timeout seconds
    // the rover's kill switch wont be called.
    killSwitchTimer.stop();
    killSwitchTimer.start();

    velocity.linear.x = linearVel * 1.5;
    velocity.angular.z = angularVel * 8; //scaling factor for sim; removed by aBridge node
    velocityPublish.publish(velocity);
}

/***********************
 * ROS CALLBACK HANDLERS
 ************************/
void targetHandler(const shared_messages::TagsImage::ConstPtr &message) {
    // Only used if we want to take action after seeing an April Tag.
}

void modeHandler(const std_msgs::UInt8::ConstPtr &message)
{
    simulation_mode = message->data;
    setVelocity(0.0, 0.0);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr &message)
{
    if ( message->data > 0 )
    {
        if (message->data == 1)
        {
            // obstacle on right side
        }
        else
        {
            //obstacle in front or on left side
        }
    }
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr &message)
{
    //Get (x,y) location directly from pose
    current_location.x = message->pose.pose.position.x;
    current_location.y = message->pose.pose.position.y;

    //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y,
                     message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_location.theta = yaw;
}

void joyCmdHandler(const geometry_msgs::Twist::ConstPtr &message)
{
    if (simulation_mode == 0 || simulation_mode == 1)
    {
        setVelocity(message->linear.x, message->angular.z);
    }
}

void publishStatusTimerEventHandler(const ros::TimerEvent &)
{
    if (!is_published_name)
    {
        std_msgs::String name_msg;
        name_msg.data = "I ";
        name_msg.data = name_msg.data + rover_name;
        messagePublish.publish(name_msg);
        is_published_name = true;
    }

    std_msgs::String msg;
    msg.data = "online";
    status_publisher.publish(msg);
}

// Safety precaution. No movement commands - might have lost contact with ROS. Stop the rover.
// Also might no longer be receiving manual movement commands so stop the rover.
void killSwitchTimerEventHandler(const ros::TimerEvent &t)
{
    // No movement commands for killSwitchTime seconds so stop the rover
    setVelocity(0.0, 0.0);
    double current_time = ros::Time::now().toSec();
    ROS_INFO("In mobility.cpp:: killSwitchTimerEventHander(): Movement input timeout. Stopping the rover at %6.4f.",
             current_time);
}

void sigintEventHandler(int sig)
{
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

void messageHandler(const std_msgs::String::ConstPtr &message)
{
}

void poseHandler(const std_msgs::String::ConstPtr &message)
{
    string msg = message->data.c_str();
    cout <<"Message " << msg << endl;
    char name[32];
    float x, y, theta = 0.0;
    all_rovers.updateRover(name, x, y, theta);
    std_msgs::String globalAverageHeading_msg;
    std::stringstream globalInfo;
    globalInfo << "Global Average Heading:" << all_rovers.calculateAverageBearing();
    globalAverageHeading_msg.data = localInfo.str();
    global_average_heading_publisher.publish(globalAverageHeading_msg);
    std_msgs::String localAverageHeading_msgs;
    std::stringstream localInfo;
    localInfo << "Local Average Heading (" << name <<"): " << all_rovers.calculateAverageNeighborBearing(x,y);
    localAverageHeading_msgs.data = localInfo.str();
    local_average_heading_publisher.publish(localAverageHeading_msgs);
}
void globalAverageHeadingHandler(const std_msgs::String::ConstPtr &message)
{
}

void localAverageHeadingHandler(const std::msgs::String::ConstPtr &message);
