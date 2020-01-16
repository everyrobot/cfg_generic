
// =============================================================================
//		Includes
// =============================================================================
#include <dynamic_reconfigure/server.h>
#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
// include nessecary headers
#include <er_tactile/er_tactile_parametersConfig.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64MultiArray.h>
// during calibration
//#include <std_msgs/UInt16MultiArray.h>

// =============================================================================
//	Defines
// =============================================================================
#define PCB_WIDTH 0.6 / 10 //cm
#define PCB_LEGTH 0.6 / 10 // cm
#define GRAMS_TO_KG 1.0 / 1000
#define NO_OF_SENSORS 9
#define PCB_AREA PCB_WIDTH *PCB_LEGTH // cm^2
#define SENSOR_AREA PCB_AREA / NO_OF_SENSORS
#define MAX_VOLTAGE_DIGITIAL_VALUE 4096
#define g 9.81
// =============================================================================
//	Structs
// =============================================================================

// =============================================================================
//	functions Prototypes
// =============================================================================
void reconfigure_callback(er_tactile::er_tactile_parametersConfig &config, uint32_t level);
void sensors_function_callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
// =============================================================================
//	Global variables
static std::vector<double_t> a, b, c, zero_v, pcb1_voltages, pcb1_sensors_forces;
static std_msgs::Float64MultiArray pcb1_sensors_forces_msg;
static geometry_msgs::WrenchStamped pcb1_force_msg;
static double exponent;
//  after calibration
static ros::Publisher sensors_forces_pub;
static ros::Subscriber sensors_voltage_sub;
static ros::Publisher pcb1_force_pub;
// during calibration
//static ros::Subscriber sensors_pressure_sub;
//static ros::Publisher sensors_voltage_pub;

// =============================================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "er_tactile");

    ros::NodeHandle nh_;

    ros::NodeHandle nh_private("~");
    a.resize(NO_OF_SENSORS);
    b.resize(NO_OF_SENSORS);
    c.resize(NO_OF_SENSORS);
    zero_v.resize(NO_OF_SENSORS);
    pcb1_voltages.resize(NO_OF_SENSORS);
    pcb1_sensors_forces.resize(NO_OF_SENSORS);

    // if (!nh_private.getParam("rate", g_rate))
    // {
    //   ROS_WARN("Rate is not defined, using maximum 1.2 kHz");
    //   g_rate = 1200;
    // }
    //ros::Rate loop_rate(g_rate);

    nh_private.getParam("a1", a[0]);
    nh_private.getParam("b1", b[0]);
    nh_private.getParam("c1", c[0]);
    nh_private.getParam("zero_v1", zero_v[0]);
    nh_private.getParam("a2", a[1]);
    nh_private.getParam("b2", b[1]);
    nh_private.getParam("c2", c[1]);
    nh_private.getParam("zero_v2", zero_v[1]);
    nh_private.getParam("a3", a[2]);
    nh_private.getParam("b3", b[2]);
    nh_private.getParam("c3", c[2]);
    nh_private.getParam("zero_v3", zero_v[2]);
    nh_private.getParam("a4", a[3]);
    nh_private.getParam("b4", b[3]);
    nh_private.getParam("c4", c[3]);
    nh_private.getParam("zero_v4", zero_v[3]);
    nh_private.getParam("a5", a[4]);
    nh_private.getParam("b5", b[4]);
    nh_private.getParam("c5", c[4]);
    nh_private.getParam("zero_v5", zero_v[4]);
    nh_private.getParam("a6", a[5]);
    nh_private.getParam("b6", b[5]);
    nh_private.getParam("c6", c[5]);
    nh_private.getParam("zero_v6", zero_v[5]);
    nh_private.getParam("a7", a[6]);
    nh_private.getParam("b7", b[6]);
    nh_private.getParam("c7", c[6]);
    nh_private.getParam("zero_v7", zero_v[6]);
    nh_private.getParam("a8", a[7]);
    nh_private.getParam("b8", b[7]);
    nh_private.getParam("c8", c[7]);
    nh_private.getParam("zero_v8", zero_v[7]);
    nh_private.getParam("a9", a[8]);
    nh_private.getParam("b9", b[8]);
    nh_private.getParam("c9", c[8]);
    nh_private.getParam("zero_v9", zero_v[8]);

    ROS_INFO("Parameters Update: %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f "
             "%f %f %f %f %f %f %f %f %f"
             "%f %f %f %f %f",
             a[0],
             b[0],
             c[0],
             zero_v[0],
             a[1],
             b[1],
             c[1],
             zero_v[1],
             a[2],
             b[2],
             c[2],
             zero_v[2],
             a[3],
             b[3],
             c[3],
             zero_v[3],
             a[4],
             b[4],
             c[4],
             zero_v[4],
             a[5],
             b[5],
             c[5],
             zero_v[5],
             a[6],
             b[6],
             c[6],
             zero_v[6],
             a[7],
             b[7],
             c[7],
             zero_v[7],
             a[8],
             b[8],
             c[8],
             zero_v[8]);

    // create dynamic_reconfigure server
    dynamic_reconfigure::Server<er_tactile::er_tactile_parametersConfig> reconfigureServer;
    dynamic_reconfigure::Server<er_tactile::er_tactile_parametersConfig>::CallbackType cb;

    ros::Rate loop_rate(100);
    sensors_voltage_sub = nh_.subscribe("sensors_voltage", 1000, &sensors_function_callback);
    sensors_forces_pub = nh_.advertise<std_msgs::Float64MultiArray>("sensors_force", 1000);
    pcb1_force_pub = nh_.advertise<geometry_msgs::WrenchStamped>("pcb1_force", 1000);
    pcb1_force_msg.header.frame_id = "/map";
    cb = boost::bind(&reconfigure_callback, _1, _2);
    reconfigureServer.setCallback(cb);

    while (ros::ok()) {
        loop_rate.sleep();

        ros::spinOnce();
    } //while

    return 0;
}

// ----------------------------------------------------------------------------------
//	Other Functions
// ----------------------------------------------------------------------------------

/*********************************************************************************
 * Function Name  : reconfigureCallback
 * Description    : 
 * Input          : 
 *                : 
 * Output         :
 * Return         :
 *********************************************************************************/
void reconfigure_callback(er_tactile::er_tactile_parametersConfig &config, uint32_t)
{
    a[0] = config.a1;
    b[0] = config.b1;
    c[0] = config.c1;
    zero_v[0] = config.zero_v1;
    a[1] = config.a2;
    b[1] = config.b2;
    c[1] = config.c2;
    zero_v[1] = config.zero_v2;
    a[2] = config.a3;
    b[2] = config.b3;
    c[2] = config.c3;
    zero_v[2] = config.zero_v3;
    a[3] = config.a4;
    b[3] = config.b4;
    c[3] = config.c4;
    zero_v[3] = config.zero_v4;
    a[4] = config.a5;
    b[4] = config.b5;
    c[4] = config.c5;
    zero_v[4] = config.zero_v5;
    a[5] = config.a6;
    b[5] = config.b6;
    c[5] = config.c6;
    zero_v[5] = config.zero_v6;
    a[6] = config.a7;
    b[6] = config.b7;
    c[6] = config.c7;
    zero_v[6] = config.zero_v7;
    a[7] = config.a8;
    b[7] = config.b8;
    c[7] = config.c8;
    zero_v[7] = config.zero_v8;
    a[8] = config.a9;
    b[8] = config.b9;
    c[8] = config.c9;
    zero_v[8] = config.zero_v9;

    ROS_INFO("Parameters Update: %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f "
             "%f %f %f %f %f %f %f %f %f"
             "%f %f %f %f %f",
             a[0],
             b[0],
             c[0],
             zero_v[0],
             a[1],
             b[1],
             c[1],
             zero_v[1],
             a[2],
             b[2],
             c[2],
             zero_v[2],
             a[3],
             b[3],
             c[3],
             zero_v[3],
             a[4],
             b[4],
             c[4],
             zero_v[4],
             a[5],
             b[5],
             c[5],
             zero_v[5],
             a[6],
             b[6],
             c[6],
             zero_v[6],
             a[7],
             b[7],
             c[7],
             zero_v[7],
             a[8],
             b[8],
             c[8],
             zero_v[8]);
    //config.str_param.c_str(), config.bool_param ? "True" : "False",config.size
}
/*********************************************************************************
 * Function Name  : sensors_function_callback
 * Description    : 
 * Input          : 
 *                : 
 * Output         :
 * Return         :
 *********************************************************************************/
void sensors_function_callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    pcb1_force_msg.wrench.force.z = 0;
    pcb1_sensors_forces_msg.data.clear();
    for (int sensor_no = 0; sensor_no < NO_OF_SENSORS; sensor_no++) {
        //ROS_INFO("data[%d]: %f", sensor_no, msg->data[sensor_no]);
        pcb1_voltages[sensor_no] = ((msg->data[sensor_no]) - zero_v[sensor_no])
                                   / MAX_VOLTAGE_DIGITIAL_VALUE;
        exponent = (pcb1_voltages[sensor_no] - c[sensor_no]) / a[sensor_no];
        pcb1_sensors_forces[sensor_no] = (1 / b[sensor_no]) * exp(exponent) * SENSOR_AREA
                                         * GRAMS_TO_KG * g;
        pcb1_sensors_forces_msg.data.push_back(pcb1_sensors_forces[sensor_no]);
        pcb1_force_msg.header.stamp = ros::Time::now();
        pcb1_force_msg.wrench.force.z += pcb1_sensors_forces_msg.data[sensor_no];
    }
    sensors_forces_pub.publish(pcb1_sensors_forces_msg);
    pcb1_force_pub.publish(pcb1_force_msg);
}
