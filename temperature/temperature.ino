#include <limits>
#include <stdlib.h>

// ros and message types
#include <ros.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/RelativeHumidity.h>
#include <std_msgs/UInt8MultiArray.h>

#define THERMAL_SIZE 384

// publisher declarations
ros::NodeHandle nh;
sensor_msgs::Temperature temp_msg;
ros::Publisher temp_pub("intern_temp", &temp_msg);
sensor_msgs::RelativeHumidity humid_msg;
ros::Publisher humid_pub("intern_humidity", &humid_msg);
std_msgs::UInt8MultiArray thermal_msg;
std_msgs::MultiArrayLayout thermal_msg_layout;
std_msgs::MultiArrayDimension thermal_msg_dim[1];
std_msgs::MultiArrayDimension thermal_msg_row;
uint8_t thermal_data[THERMAL_SIZE];
ros::Publisher thermal_pub0("thermal_0u", &thermal_msg);
ros::Publisher thermal_pub1("thermal_1u", &thermal_msg);
ros::Publisher thermal_pub2("thermal_2u", &thermal_msg);

// I2C lib
#include <Wire.h>

// sensor libraries and class initializations
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
// https://github.com/sparkfun/SparkFun_MLX90640_Arduino_Example/issues/2
const byte MLX90640_address = 0x33; // Default 7-bit unshifted address of the MLX90640
#define TA_SHIFT 8                  // Default shift for MLX90640 in open air
static float mlx90640To[768];
paramsMLX90640 mlx90640[4];

#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme; // I2C

#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

#include <Adafruit_BNO055.h>
Adafruit_BNO055 bno = Adafruit_BNO055();

// multiplexer address
#define TCAADDR 0x70

// muliplexer port assignments
#define THERMAL_0 0
#define THERMAL_1 1
#define THERMAL_2 2
#define BME 4
#define BNO 5

// initialize ROS publishers and each sensor
void setup()
{
    // ros initializations
    nh.initNode();
    nh.advertise(temp_pub);
    nh.advertise(humid_pub);
    nh.advertise(thermal_pub0);
    nh.advertise(thermal_pub1);
    nh.advertise(thermal_pub2);

    // I2C init
    Wire.begin();
    Serial.begin(57600);
    // BME init
    tcaselect(BME);
    if (!bme.begin())
    {
        // could not find BME
        Serial.println("Failed to connect to BME");
        while (1)
            ;
    }

    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms

    // BNO init
    tcaselect(BNO);
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1)
            ;
    }

    // thermal inits
    MLX_init(THERMAL_0);
    MLX_init(THERMAL_1);
    MLX_init(THERMAL_2);

    // init some constant data for ROS messages
    temp_msg.variance = 1.0;
    humid_msg.variance = 0.03;

    const char row[4] = "row";
    thermal_msg_row.label = row;
    thermal_msg_row.size = THERMAL_SIZE;
    thermal_msg_row.stride = THERMAL_SIZE;

    thermal_msg_dim[0] = thermal_msg_row;

    thermal_msg_layout.dim = thermal_msg_dim;
    thermal_msg_layout.dim_length = 1;

    thermal_msg.layout = thermal_msg_layout;
    thermal_msg.data_length = THERMAL_SIZE;
    Serial.println("all sensors connected");
}

// repeatedly read from each sensor and publish to ROS
void loop()
{

    // read from BME sensor
    tcaselect(BME);
    temp_msg.header.stamp = nh.now();
    humid_msg.header.stamp = nh.now();
    if (bme.performReading())
    {
        temp_msg.temperature = bme.temperature;
        humid_msg.relative_humidity = bme.humidity / 100.0;
    }
    else
    {
        temp_msg.temperature = 0.0;
        humid_msg.relative_humidity = -1.0;
    }
    temp_pub.publish(&temp_msg);
    humid_pub.publish(&humid_msg);

    // IR Array readings
    MLX_read(THERMAL_0);
    for (int j = 0; j < 768; j += THERMAL_SIZE)
    {
        thermal_msg_layout.data_offset = j;
        thermal_msg.layout = thermal_msg_layout;
        for (int i = j; i < THERMAL_SIZE + j; i++)
        {
            float temp = mlx90640To[i];
            if (temp <= 25.0)
            {
                temp = 25;
            }
            else if (temp >= 280.0)
            {
                temp = 280;
            }
            temp -= 25;
            uint8_t temp2 = static_cast<uint8_t>(temp);
            thermal_data[i % THERMAL_SIZE] = temp2;
        }
        thermal_msg.data = thermal_data;
        thermal_pub0.publish(&thermal_msg);
    }
    MLX_read(THERMAL_1);
    for (int j = 0; j < 768; j += THERMAL_SIZE)
    {
        thermal_msg_layout.data_offset = j;
        thermal_msg.layout = thermal_msg_layout;
        for (int i = j; i < THERMAL_SIZE + j; i++)
        {
            float temp = mlx90640To[i];
            if (temp <= 25.0)
            {
                temp = 25;
            }
            else if (temp >= 280.0)
            {
                temp = 280;
            }
            temp -= 25;
            uint8_t temp2 = static_cast<uint8_t>(temp);
            thermal_data[i % THERMAL_SIZE] = temp2;
        }
        thermal_msg.data = thermal_data;
        thermal_pub1.publish(&thermal_msg);
    }
    MLX_read(THERMAL_2);
    for (int j = 0; j < 768; j += THERMAL_SIZE)
    {
        thermal_msg_layout.data_offset = j;
        thermal_msg.layout = thermal_msg_layout;
        for (int i = j; i < THERMAL_SIZE + j; i++)
        {
            float temp = mlx90640To[i];
            if (temp <= 25.0)
            {
                temp = 25;
            }
            else if (temp >= 280.0)
            {
                temp = 280;
            }
            temp -= 25;
            uint8_t temp2 = static_cast<uint8_t>(temp);
            thermal_data[i % THERMAL_SIZE] = temp2;
        }
        thermal_msg.data = thermal_data;
        thermal_pub2.publish(&thermal_msg);
    }

    nh.spinOnce();
}

// initialize an MLX sensor
void MLX_init(int port)
{
    tcaselect(port);
    if (isConnected() == false)
    {
        // sensor not detected
        Serial.printf("Failed to connect to MLX %d\n", port);
        while (1)
            ;
    }
    int status;
    uint16_t eeMLX90640[832];
    status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
    if (status != 0)
    {
        Serial.printf("Failed to load params MLX %d\n", port);
        while (1)
            ; // failed to load system parameters
    }
    status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640[port]);
    if (status != 0)
    {
        Serial.printf("Failed to extract params MLX %d\n", port);
        while (1)
            ; // failed to extract parameters
    }
    // Serial.printf("gainEE: %d\n", mlx90640[port].gainEE);
    // Serial.println("");
    // Serial.printf("%d, %d, %d\n", mlx90640[port].offset[64], mlx90640[port].offset[357], mlx90640[port].offset[746]);
}

void MLX_read(int port)
{
    tcaselect(port);
    for (byte x = 0; x < 2; x++) // Read both subpages
    {
        uint16_t mlx90640Frame[834];

        int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
        if (status >= 0)
        {

            float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640[port]);
            float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640[port]);

            float tr = Ta - TA_SHIFT; // Reflected temperature based on the sensor ambient temperature
            float emissivity = 0.95;

            MLX90640_CalculateTo(mlx90640Frame, &mlx90640[port], emissivity, tr, mlx90640To);
        }
        // Serial.printf("%d mode: %f\n", port, (mlx90640Frame[832] & 0x1000) >> 5);
    }
}

// Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected()
{
    Wire.beginTransmission((uint8_t)MLX90640_address);
    if (Wire.endTransmission() != 0)
        return (false); // Sensor did not ACK
    return (true);
}

// select multiplexer port
void tcaselect(uint8_t i)
{
    if (i > 7)
        return;

    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
}
