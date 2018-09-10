/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#define LIGHTTELEMETRY_START1 0x24 //$
#define LIGHTTELEMETRY_START2 0x54 //T
#define LIGHTTELEMETRY_GFRAME 0x47 //G GPS + Baro altitude data ( Lat, Lon, Speed, Alt, Sats, Sat fix)
#define LIGHTTELEMETRY_AFRAME 0x41 //A Attitude data ( Roll,Pitch, Heading )
#define LIGHTTELEMETRY_SFRAME 0x53 //S Sensors/Status data ( VBat, Consumed current, Rssi, Airspeed, Arm status, Failsafe status, Flight mode )
#define LIGHTTELEMETRY_GFRAMELENGTH 18
#define LIGHTTELEMETRY_AFRAMELENGTH 10
#define LIGHTTELEMETRY_SFRAMELENGTH 11
#define LIGHTTELEMETRY_OFRAMELENGTH 18

int32_t      uav_lat = 0;                    // latitude
int32_t      uav_lon = 0;                    // longitude
float        lonScaleDown=0.0;               // longitude scaling
uint8_t      uav_satellites_visible = 0;     // number of satellites
uint8_t      uav_fix_type = 0;               // GPS lock 0-1=no fix, 2=2D, 3=3D
int32_t      uav_alt = 0;                    // altitude (dm)
int32_t      rel_alt = 0;                    // relative altitude to home
uint16_t     uav_groundspeed = 0;            // ground speed in km/h
uint8_t      uav_groundspeedms = 0;          // ground speed in m/s
int16_t      uav_pitch = 0;                  // attitude pitch
int16_t      uav_roll = 0;                   // attitude roll
int16_t      uav_heading = 0;                // attitude heading
int16_t      uav_gpsheading=0;               // gps heading
uint16_t     uav_bat = 0;                    // battery voltage (mv)
uint16_t     uav_amp = 0;                    // consumed mah.
uint16_t     uav_current = 0;                // actual current
uint8_t      uav_rssi = 0;                   // radio RSSI (%)
uint8_t      uav_linkquality = 0;            // radio link quality
uint8_t      uav_airspeed = 0;               // Airspeed sensor (m/s)
uint8_t      ltm_armfsmode = 0;
uint8_t      uav_arm = 0;                    // 0: disarmed, 1: armed
uint8_t      uav_failsafe = 0;               // 0: normal,   1: failsafe 
uint8_t      uav_flightmode = 19;            // Flight mode(0-19): 0: Manual, 1: Rate, 2: Attitude/Angle, 3: Horizon, 4: Acro, 5: Stabilized1, 6: Stabilized2, 7: Stabilized3,
                                             // 8: Altitude Hold, 9: Loiter/GPS Hold, 10: Auto/Waypoints, 11: Heading Hold / headFree, 12: Circle, 13: RTH, 14: FollowMe, 15: LAND, 
                                             // 16:FlybyWireA, 17: FlybywireB, 18: Cruise, 19: Unknown


static uint8_t LTMserialBuffer[LIGHTTELEMETRY_GFRAMELENGTH-4];
static uint8_t LTMreceiverIndex;
static uint8_t LTMcmd;
static uint8_t LTMrcvChecksum;
static uint8_t LTMreadIndex;
static uint8_t LTMframelength;
serial::Serial ser;

void write_callback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO_STREAM("Writing to serial port" << msg->data);
	ser.write(msg->data);
}

uint8_t ltmread_u8()  {
  return LTMserialBuffer[LTMreadIndex++];
}

uint16_t ltmread_u16() {
  uint16_t t = ltmread_u8();
  t |= (uint16_t)ltmread_u8()<<8;
  return t;
}

uint32_t ltmread_u32() {
  uint32_t t = ltmread_u16();
  t |= (uint32_t)ltmread_u16()<<16;
  return t;
}

void ltm_check() {
  LTMreadIndex = 0;
                    
  if (LTMcmd==LIGHTTELEMETRY_GFRAME)
  {
    
    uav_lat = (int32_t)ltmread_u32();
    uav_lon = (int32_t)ltmread_u32();
    uav_groundspeedms = ltmread_u8();
    uav_groundspeed = (uint16_t) round((float)(uav_groundspeedms * 3.6f)); // convert to kmh
    uav_alt = (int32_t)ltmread_u32();
    uint8_t ltm_satsfix = ltmread_u8();
    uav_satellites_visible         = (ltm_satsfix >> 2) & 0xFF;
    uav_fix_type                   = ltm_satsfix & 0b00000011;
    memset(LTMserialBuffer, 0, LIGHTTELEMETRY_GFRAMELENGTH-4); 

  }
  
  if (LTMcmd==LIGHTTELEMETRY_AFRAME)
  {
    uav_pitch = (int16_t)ltmread_u16();
    uav_roll =  (int16_t)ltmread_u16();
    uav_heading = (int16_t)ltmread_u16();
    if (uav_heading < 0 ) uav_heading = uav_heading + 360; //convert from -180/180 to 0/360°
    memset(LTMserialBuffer, 0, LIGHTTELEMETRY_AFRAMELENGTH-4); 
  }
  if (LTMcmd==LIGHTTELEMETRY_SFRAME)
  {
    uav_bat = ltmread_u16();
    uav_amp = ltmread_u16();
    uav_rssi = ltmread_u8();
    uav_airspeed = ltmread_u8();
    ltm_armfsmode = ltmread_u8();
    uav_arm = ltm_armfsmode & 0b00000001;
    uav_failsafe = (ltm_armfsmode >> 1) & 0b00000001;
    uav_flightmode = (ltm_armfsmode >> 2) & 0b00111111;
    memset(LTMserialBuffer, 0, LIGHTTELEMETRY_SFRAMELENGTH-4);     
  }
}

void ltm_read() {
  uint8_t c;


  static enum _serial_state {
    IDLE,
    HEADER_START1,
    HEADER_START2,
    HEADER_MSGTYPE,
    HEADER_DATA
  }
  c_state = IDLE;
  
  while (ser.available()) {
    c = char(ser.read(1)[0]);

    if (c_state == IDLE) {
      c_state = (c=='$') ? HEADER_START1 : IDLE;
        //Serial.println("header $" );
    }
    else if (c_state == HEADER_START1) {
      c_state = (c=='T') ? HEADER_START2 : IDLE;
        //Serial.println("header T" );
    }
    else if (c_state == HEADER_START2) {
      switch (c) {
         case 'G':
           LTMframelength = LIGHTTELEMETRY_GFRAMELENGTH;
           c_state = HEADER_MSGTYPE;
           break;
         case 'A':
           LTMframelength = LIGHTTELEMETRY_AFRAMELENGTH;
           c_state = HEADER_MSGTYPE;
           break;
         case 'S':
           LTMframelength = LIGHTTELEMETRY_SFRAMELENGTH;
           c_state = HEADER_MSGTYPE;
           break;
         default:
           c_state = IDLE;
      }
      LTMcmd = c;
      LTMreceiverIndex=0;
    }
    else if (c_state == HEADER_MSGTYPE) {
	  if(LTMreceiverIndex == 0) {
	  LTMrcvChecksum = c;
	  } 
	  else {
	  LTMrcvChecksum ^= c;
	  }
      if(LTMreceiverIndex == LTMframelength-4) {   // received checksum byte
        if(LTMrcvChecksum == 0) {
	    // telemetry_ok = true;
            // lastpacketreceived = millis();
	    // protocol = "LTM"; 
            ltm_check();
            c_state = IDLE;
        }
        else {                                                   // wrong checksum, drop packet
        c_state = IDLE; 
        
        }
      }
      else LTMserialBuffer[LTMreceiverIndex++]=c;
    }
  }
}






int main (int argc, char** argv){
	ros::init(argc, argv, "serial_example_node");
	ros::NodeHandle nh;

	ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
	ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);

	try
	{
		ser.setPort("/dev/ttyACM0");
		ser.setBaudrate(9600);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(to);
		ser.open();
	}
	catch (serial::IOException& e)
	{
		ROS_ERROR_STREAM("Unable to open port ");
		return -1;
	}

	if(ser.isOpen()){
		ROS_INFO_STREAM("Serial Port initialized");
	}else{
		return -1;
	}

	ros::Rate loop_rate(100);
	while(ros::ok()){

		ros::spinOnce();
		ltm_read();
		ROS_INFO_STREAM("Read: " << uav_pitch);
		// if(ser.available()){
		// 	ROS_INFO_STREAM("Reading from serial port");
		// 	std_msgs::String result;
		// 	result.data = ser.read(ser.available());
		// 	ROS_INFO_STREAM("Read: " << result.data);
		// 	read_pub.publish(result);
		// }
		loop_rate.sleep();

	}
}

