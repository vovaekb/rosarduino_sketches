#include "DFRobot_OzoneSensor.h"

#define COLLECT_NUMBER 20 // collect number, the collection range is 1-100
/**
 * select i2c device address
 *   OZONE_ADDRESS_0  0x70
 *   OZONE_ADDRESS_1  0x71
 *   OZONE_ADDRESS_2  0x72
 *   OZONE_ADDRESS_3  0x73
 */
#define Ozone_IICAddress OZONE_ADDRESS_3
DFRobot_OzoneSensor Ozone;

ros::NodeHandle nh;

std_msgs::Int16 ozone_data_msg;
ros::Publisher imu_pub("ozone_level", &ozone_data_msg);

void setup()
{
    nh.initNode();
    nh.advertise(imu_pub);
    Serial.begin(9600);
    while (!Ozone.begin(Ozone_IICAddress))
    {
        Serial.println("I2c device number error !");
        delay(1000);
    }
    Serial.println("I2c connect success !");

    /**
     * set measuer mode
     * MEASURE_MODE_AUTOMATIC         active  mode
     * MEASURE_MODE_PASSIVE           passive mode
     */
    Ozone.setModes(MEASURE_MODE_PASSIVE);
}

void loop()
{
    ozone_data_msg.header.stamp = nh.now();
    ozone_data_msg.header.frame_id = "ozone_link";

    int16_t ozoneConcentration = Ozone.readOzoneData(COLLECT_NUMBER);
    ozone_data_msg.data = ozoneConcentration;
    Serial.print("Ozone concentration is ");
    Serial.print(ozoneConcentration);
    Serial.println(" PPB.");

    imu_pub.publish(&ozone_data_msg);
    nh.spinOnce();
}