#include <Arduino.h>
#include "ODriveCAN.h"

#include <IBusBM.h>
IBusBM IBus; // IBus object

// Documentation for this example can be found here:

// CAN bus baudrate. Make sure this matches for every device on the bus
#define CAN_BAUDRATE 250000

// ODrive node_id for odrv0
#define ODRV0_NODE_ID 0
#define ODRV1_NODE_ID 1
#define ODRV2_NODE_ID 2
#define ODRV3_NODE_ID 3

// Uncomment below the line that corresponds to your hardware.
// See also "Board-specific settings" to adapt the details for your hardware setup.

#define IS_TEENSY_BUILTIN // Teensy boards with built-in CAN interface (e.g. Teensy 4.1). See below to select which interface to use.
// #define IS_ARDUINO_BUILTIN // Arduino boards with built-in CAN interface (e.g. Arduino Uno R4 Minima)
// #define IS_MCP2515 // Any board with external MCP2515 based extension module. See below to configure the module.


/* Board-specific includes ---------------------------------------------------*/

#if defined(IS_TEENSY_BUILTIN) + defined(IS_ARDUINO_BUILTIN) + defined(IS_MCP2515) != 1
#warning "Select exactly one hardware option at the top of this file."

#if CAN_HOWMANY > 0 || CANFD_HOWMANY > 0
#define IS_ARDUINO_BUILTIN
#warning "guessing that this uses HardwareCAN"
#else
#error "cannot guess hardware version"
#endif

#endif

#ifdef IS_ARDUINO_BUILTIN
// See https://github.com/arduino/ArduinoCore-API/blob/master/api/HardwareCAN.h
// and https://github.com/arduino/ArduinoCore-renesas/tree/main/libraries/Arduino_CAN

#include <Arduino_CAN.h>
#include <ODriveHardwareCAN.hpp>
#endif // IS_ARDUINO_BUILTIN

#ifdef IS_MCP2515
// See https://github.com/sandeepmistry/arduino-CAN/
#include "MCP2515.h"
#include "ODriveMCPCAN.hpp"
#endif // IS_MCP2515

# ifdef IS_TEENSY_BUILTIN
// See https://github.com/tonton81/FlexCAN_T4
// clone https://github.com/tonton81/FlexCAN_T4.git into /src
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
struct ODriveStatus; // hack to prevent teensy compile error
#endif // IS_TEENSY_BUILTIN

/* Board-specific settings ---------------------------------------------------*/

/* Teensy */

#ifdef IS_TEENSY_BUILTIN

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

bool setupCan() {
  can_intf.begin();
  can_intf.setBaudRate(CAN_BAUDRATE);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}

#endif // IS_TEENSY_BUILTIN

/* MCP2515-based extension modules -*/

#ifdef IS_MCP2515

MCP2515Class& can_intf = CAN;

// chip select pin used for the MCP2515
#define MCP2515_CS 10

// interrupt pin used for the MCP2515
// NOTE: not all Arduino pins are interruptable, check the documentation for your board!
#define MCP2515_INT 2

// freqeuncy of the crystal oscillator on the MCP2515 breakout board. 
// common values are: 16 MHz, 12 MHz, 8 MHz
#define MCP2515_CLK_HZ 8000000


static inline void receiveCallback(int packet_size) {
  if (packet_size > 8) {
    return; // not supported
  }
  CanMsg msg = {.id = (unsigned int)CAN.packetId(), .len = (uint8_t)packet_size};
  CAN.readBytes(msg.buffer, packet_size);
  onCanMessage(msg);
}

bool setupCan() {
  // configure and initialize the CAN bus interface
  CAN.setPins(MCP2515_CS, MCP2515_INT);
  CAN.setClockFrequency(MCP2515_CLK_HZ);
  if (!CAN.begin(CAN_BAUDRATE)) {
    return false;
  }

  CAN.onReceive(receiveCallback);
  return true;
}

#endif // IS_MCP2515


/* Arduinos with built-in CAN */

#ifdef IS_ARDUINO_BUILTIN

HardwareCAN& can_intf = CAN;

bool setupCan() {
  return can_intf.begin((CanBitRate)CAN_BAUDRATE);
}

#endif


/* Example sketch ------------------------------------------------------------*/

// Instantiate ODrive objects
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID); // Standard CAN message ID
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID); // Standard CAN message ID
ODriveCAN odrv2(wrap_can_intf(can_intf), ODRV2_NODE_ID); // Standard CAN message ID
ODriveCAN odrv3(wrap_can_intf(can_intf), ODRV3_NODE_ID); // Standard CAN message ID
ODriveCAN* odrives[] = {&odrv0, &odrv1, &odrv2, &odrv3}; // Make sure all ODriveCAN instances are accounted for here

struct ODriveUserData0 {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

struct ODriveUserData1 {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

struct ODriveUserData2 {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

struct ODriveUserData3 {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

// Keep some application-specific user data for every ODrive.
ODriveUserData0 odrv0_user_data;
ODriveUserData1 odrv1_user_data;
ODriveUserData2 odrv2_user_data;
ODriveUserData3 odrv3_user_data;

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  
  ODriveUserData0* odrv_user_data0 = static_cast<ODriveUserData0*>(user_data);
  odrv_user_data0->last_heartbeat = msg;
  odrv_user_data0->received_heartbeat = true;

  ODriveUserData1* odrv_user_data1 = static_cast<ODriveUserData1*>(user_data);
  odrv_user_data1->last_heartbeat = msg;
  odrv_user_data1->received_heartbeat = true;

  ODriveUserData2* odrv_user_data2 = static_cast<ODriveUserData2*>(user_data);
  odrv_user_data2->last_heartbeat = msg;
  odrv_user_data2->received_heartbeat = true;

  ODriveUserData3* odrv_user_data3 = static_cast<ODriveUserData3*>(user_data);
  odrv_user_data3->last_heartbeat = msg;
  odrv_user_data3->received_heartbeat = true;
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData0* odrv_user_data0 = static_cast<ODriveUserData0*>(user_data);
  odrv_user_data0->last_feedback = msg;
  odrv_user_data0->received_feedback = true;

  ODriveUserData1* odrv_user_data1 = static_cast<ODriveUserData1*>(user_data);
  odrv_user_data1->last_feedback = msg;
  odrv_user_data1->received_feedback = true;

  ODriveUserData2* odrv_user_data2 = static_cast<ODriveUserData2*>(user_data);
  odrv_user_data2->last_feedback = msg;
  odrv_user_data2->received_feedback = true;

  ODriveUserData3* odrv_user_data3 = static_cast<ODriveUserData3*>(user_data);
  odrv_user_data3->last_feedback = msg;
  odrv_user_data3->received_feedback = true;
}

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) {
  for (auto odrive: odrives) {
    onReceive(msg, *odrive);
  }

}

// *** variables ***

int ch1;
int ch2;
int ch3;
int ch4;
int ch5;
int ch6;
int ch7;
int ch8;
int ch9;
int ch10;

int ch8a;
int ch8b;

float RFB;
float RFBa;
float RFBFiltered;
float RLR;
float RLRa;
float RLRFiltered;
float RT;
float RTa;
float RTFiltered;

float LFB;
float LFBa;
float LFBFiltered;
float LLR;
float LLRa;
float LLRFiltered;
float LT;
float LTa;
float LTFiltered;

float forward;
float diff;
float right;
float left;

int actuator1;
int actuator2;
int actuator3;
int actuator4;

int actuator1a;
int actuator2a;
int actuator3a;
int actuator4a;

int clFlag = 0;

unsigned long currentMillis;
unsigned long previousMillis = 0;        // set up timers
long interval = 10;             // time constant for timer
  


void setup() {
  Serial.begin(115200);

  IBus.begin(Serial3, IBUSBM_NOTIMER);    // change to Serial1 or Serial2 port when required

  pinMode(0,OUTPUT);
  pinMode(1,OUTPUT);
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);

  // Wait for up to 3 seconds for the serial port to be opened on the PC side.
  // If no PC connects, continue anyway.
  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(100);
  }
  delay(200);


  Serial.println("Starting ODriveCAN demo");

  // Register callbacks for the heartbeat and encoder feedback messages
  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);
  odrv1.onFeedback(onFeedback, &odrv1_user_data);
  odrv1.onStatus(onHeartbeat, &odrv1_user_data);
  odrv2.onFeedback(onFeedback, &odrv2_user_data);
  odrv2.onStatus(onHeartbeat, &odrv2_user_data);
  odrv3.onFeedback(onFeedback, &odrv3_user_data);
  odrv3.onStatus(onHeartbeat, &odrv3_user_data);

  // Configure and initialize the CAN bus interface. This function depends on
  // your hardware and the CAN stack that you're using.
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true); // spin indefinitely
  }

  // Check for Odrives

  Serial.println("Waiting for ODrive 0...");
  while (!odrv0_user_data.received_heartbeat) {
    pumpEvents(can_intf);
    delay(100);
  }
  Serial.println("found ODrive 0");

  
  Serial.println("Waiting for ODrive 1...");
  while (!odrv1_user_data.received_heartbeat) {
    pumpEvents(can_intf);
    delay(100);
  }
  Serial.println("found ODrive 1");
   
  Serial.println("Waiting for ODrive 2...");
  while (!odrv2_user_data.received_heartbeat) {
    pumpEvents(can_intf);
    delay(100);
  }
  Serial.println("found ODrive 2");


  Serial.println("Waiting for ODrive 3...");
  while (!odrv3_user_data.received_heartbeat) {
    pumpEvents(can_intf);
    delay(100);
  }
  Serial.println("found ODrive 3");



    odrv0.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);
    odrv1.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);
    odrv2.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);
    odrv3.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);

    for (int i = 0; i < 15; ++i) {
      delay(10);
      pumpEvents(can_intf);
    }
}


void loop() {

  currentMillis = millis();
  if (currentMillis - previousMillis >= 10) {  // start timed event    
        previousMillis = currentMillis;

        IBus.loop();
      
        ch1 = IBus.readChannel(0); // get latest value for servo channel 0
        ch2 = IBus.readChannel(1); // get latest value for servo channel 1
        ch3 = IBus.readChannel(2); // get latest value for servo channel 3
        ch4 = IBus.readChannel(3); // get latest value for servo channel 4
        ch5 = IBus.readChannel(4); // get latest value for servo channel 5
        ch6 = IBus.readChannel(5); // get latest value for servo channel 6
        ch7 = IBus.readChannel(6); // get latest value for servo channel 7
        ch8 = IBus.readChannel(7); // get latest value for servo channel 8
        ch9 = IBus.readChannel(8); // get latest value for servo channel 9
        ch10 = IBus.readChannel(9); // get latest value for servo channel 10
      
        LFB = ch1;
        RFB = ch4;
        RLR = ch2;
        RT = ch6;
        LLR = ch3;
        LT = ch5;
        
        // *** threshold sticks ***
        RFBa = thresholdStick(RFB);
        RLRa = thresholdStick(RLR);
        RTa = thresholdStick(RT);
        LFBa = thresholdStick(LFB);
        LLRa = thresholdStick(LLR);
        LTa = thresholdStick(LT);
      
        // *** filter sticks ***
        RFBFiltered = filter(RFBa, RFBFiltered,20);
        RLRFiltered = filter(RLRa, RLRFiltered,20);
        RTFiltered = filter(RTa, RTFiltered,20);
        LFBFiltered = filter(LFBa, LFBFiltered,20);
        LLRFiltered = filter(LLRa, LLRFiltered,20);
        LTFiltered = filter(LTa, LTFiltered,20);    
      
        if (ch7 > 1350) {  // actuator mode - only works if the switch is on
      
          actuator1 = RLRa*-1;    // move actuators with sticks
          actuator2 = RLRa*-1;
          actuator3 = LLRa;
          actuator4 = LLRa;       
        }

        if (ch9 > 1400 && clFlag == 0) {          // Init Odrives

            Serial.println("Enabling closed loop control...");
              while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
              odrv0.clearErrors();
              delay(1);
              odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            }
          
            Serial.println("Enabling closed loop control...");
            while (odrv1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
              odrv1.clearErrors();
              delay(1);
              odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            }
          
            Serial.println("Enabling closed loop control...");
            while (odrv2_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
              odrv2.clearErrors();
              delay(1);
              odrv2.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            }
          
            Serial.println("Enabling closed loop control...");
            while (odrv3_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
              odrv3.clearErrors();
              delay(1);
              odrv3.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            }

            clFlag = 1;          // reset flag
        }

        else if (ch9 < 1200) {      // make sure we only power on ODrives once per button press
          clFlag = 0;
        }

        // actuator1
        if (actuator1 > 0) {
          analogWrite(0, actuator1);
          analogWrite(1, 0);
        }
        else if (actuator1 < 0) {
          actuator1a = abs(actuator1);
          analogWrite(1, actuator1a);
          analogWrite(0, 0);
        }
        else {
          analogWrite(1, 0);
          analogWrite(0, 0);
        }
        
        // actuator2
        if (actuator2 > 0) {
          analogWrite(2, actuator2);
          analogWrite(3, 0);
        }
        else if (actuator2 < 0) {
          actuator2a = abs(actuator2);
          analogWrite(3,actuator2a);
          analogWrite(2, 0);
        }
        else {
          analogWrite(2, 0);
          analogWrite(3, 0);
        }

        // actuator3
        if (actuator3 > 0) {
          analogWrite(4, actuator3);
          analogWrite(5, 0);
        }
        else if (actuator3 < 0) {
          actuator3a = abs(actuator3);
          analogWrite(5, actuator3a);
          analogWrite(4, 0);
        }
        else {
          analogWrite(4, 0);
          analogWrite(5, 0);
        }
        
        // actuator4
        if (actuator4 > 0) {
          analogWrite(6, actuator4);
          analogWrite(7, 0);
        }
        else if (actuator4 < 0) {
          actuator4a = abs(actuator4);
          analogWrite(7, actuator4a);
          analogWrite(6, 0);
        }
        else {
          analogWrite(6, 0);
          analogWrite(7, 0);
        }

        //*** sort out switch logics that share a channel  ***

        if (ch8 > 1400 && ch8 < 1500) {
          ch8a = 1;
          ch8b = 0;
        }

        else if (ch8 > 1950 && ch8 < 2050) {
          ch8a = 1;
          ch8b = 1;
        }

        else if (ch8 > 1700 && ch8 < 1800) {
          ch8a = 0;
          ch8b = 1;
        }

        else {
          ch8a = 0;
          ch8b = 0;
        }

        // *** driving wheels & steering ***

        forward = RFBFiltered/-3;
        diff = LTFiltered/-20;

        if (forward > 1) {
          right = forward + diff;
          left = forward;
        
          right = constrain(right,0,80);
          left = constrain(left,0,80);
        }

        else if (forward < -1) {
          diff = diff *-1;
          right = forward;
          left = forward + diff;
        
          right = constrain(right,-80,0);
          left = constrain(left,-80,0);        
        }

        else  {
          right = 0;
          left = 0; 
        }      

  } // end of timed loop

  
  
  pumpEvents(can_intf); // This is required on some platforms to handle incoming feedback CAN messages

  if (ch8a == 1) {     // motor enable

    if (ch8b == 1) {                      // driving with front wheel steering only
        odrv0.setVelocity(forward*-1);
        odrv1.setVelocity(left*-1);
        odrv2.setVelocity(right);
        odrv3.setVelocity(forward);        
    }

    else {                                // driving with four wheel steering
        odrv0.setVelocity(left*-1);   // +ve forward      *** right side
        odrv1.setVelocity(left*-1);   // +ve forward    
        odrv2.setVelocity(right);   // -ve forward      *** left side
        odrv3.setVelocity(right);   // -ve forward
    }
    
  }

  else if (ch8a == 0) {                   // disable motors
    odrv0.setVelocity(0);   // +ve forward      *** right side
    odrv1.setVelocity(0);   // +ve forward    
    odrv2.setVelocity(0);   // -ve forward      *** left side
    odrv3.setVelocity(0);   // -ve forward
  }

  

}
