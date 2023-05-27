#include <TimerOne.h>
//canbus master
#include <SPI.h>
#include <mcp2515.h>
#include <PS4BT.h> // PS4BT 
#include <usbhub.h> // dont forget to comment for ps4 usb
// #include <PS4USB.h>      //PS4 Usb 

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

//flags
bool f_flag = false;
bool b_flag = false;
bool l_flag = false;
bool r_flag = false;
bool s_flag = false;
bool c_flag = false;
bool a_flag = false;

//cytron - shooter
int m1_dir = 29;  //right motor with behind the shooter - pov
int m2_dir = 27;  //left motor with behind the shooter - pov
int m1_pwm = 8;
int m2_pwm = 4;

//y-axis of shooter
//int enA = 10;
//int in1 = 41;
//int in2 = 39;

long long currT = 0;

//slot sensor - shooter
int ir_1 = 18;  //leftTimer1 motor
int ir_2 = 19;  //right motor

int ctr1 = 0;    //to calculate ticks of motor1
int ctr2 = 0;    //to calculate ticks of motor2
int speed1 = 0;  //to store ticks of motor1
int speed2 = 0;  //to store ticks of motor2
int target = 0;  //Target ticks
int diff = 0;
int factor = 2;
int flag = 0;

//shooter - motor's pwm
int pwm1 = 0;
int pwm2 = 0;

//piston - shooter
int relay = 2;

// servos for the claw
Servo servo1;
Servo servo2;
int s1 = 13;
int s2 = 12;

// cascading lift motor using BTS7960 pins to be changed
int L_EN = 47;
int R_EN = 49;
int L_PWM = 7;
int R_PWM = 8;

// flipping motor using l298n 
int inf1 = 35;
int inf2 = 37;
int enbf = 4;

// relay claw
int relay_pin = 47;

//Use this for PS4 USB and comment PS4 BT
// USB Usb;
// PS4USB PS4(&Usb);

// Use this (USB Usb to PS4BT PS4(&Btd, PAIR)) for PS4 BT and comment PS4 USB
USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb);  // You have to create the Bluetooth Dongle instance like so

// You will need to hold down the PS and Share button at the same time, the PS4 controller will then start to blink rapidly indicating that it is in pairing mode
PS4BT PS4(&Btd, PAIR);




struct can_frame canMsg1;
struct can_frame canMsg2;
// Define the CS pin each CAN bus shield/module
MCP2515 mcp2515(49);

// // Define the CAN message IDs for each board
// #define MESSAGE_ID_MEGA 0x01
// #define MESSAGE_ID_NANO_1 0x02
// #define MESSAGE_ID_NANO_2 0x03

// // Create MCP_CAN objects for each CAN bus shield/module
// MCP_CAN can_mega(CS_PIN_MEGA);
// MCP_CAN can_nano_1(CS_PIN_NANO_1);
// MCP_CAN can_nano_2(CS_PIN_NANO_2);

//flags
bool f_flag = false;
bool b_flag = false;
bool l_flag = false;
bool r_flag = false;
bool s_flag = false;
bool c_flag = false;
bool a_flag = false;

USB Usb;
// PS4USB PS4(&Usb);

BTD Btd(&Usb);
PS4BT PS4(&Btd, PAIR);

byte mega_data;
void interrupt_routine1()  //slot sensor - shooter
{
  ctr1++;
}

void interrupt_routine2()  //slot sensor - shooter
{
  ctr2++;
}

void readmotor()  // shooter
{
  //    Serial.print("Time");
  //    Serial.println(millis());
  speed1 = ctr1;
  speed2 = ctr2;
  Serial.print("Motor1 ");
  Serial.print(speed1);
  Serial.print("  PWM1: ");
  Serial.println(pwm1);
  Serial.print("Motor2 ");
  Serial.print(speed2);
  Serial.print("  PWM2: ");
  Serial.println(pwm2);

  set_pwm1();
  set_pwm2();

  analogWrite(m1_pwm, pwm1);
  analogWrite(m2_pwm, pwm2);
  ctr1 = 0;
  ctr2 = 0;
}

int find_factor1()  //shooter
{
  diff = abs(ctr1 - target);
  if (diff >= 15) {
    return 30;
  } else if (diff >= 10) {
    return 20;
  } else if (diff >= 5) {
    return 10;
  } else if (diff >= 3) {
    return 5;
  } else if (diff >= 2) {
    return 3;
  } else if (diff >= 1) {
    return 1;
  } else {
    return 0;
  }
}

int find_factor2()  //shooter
{
  diff = abs(ctr2 - target);
  if (diff >= 15) {
    return 30;
  } else if (diff >= 10) {
    return 20;
  } else if (diff >= 5) {
    return 10;
  } else if (diff >= 3) {
    return 5;
  } else if (diff >= 2) {
    return 3;
  } else if (diff >= 1) {
    return 1;
  } else {
    return 0;
  }
}

void set_pwm1()  //shooter adjusting pwm of motor1

{
  factor = find_factor1();
  if (speed1 < target - 1 && pwm1 + factor <= 250) {
    pwm1 += factor;
  } else if (speed1 > target + 1 && pwm1 - factor >= 0) {
    Serial.println("Bye");
    pwm1 -= factor;
  } else if (target == 0) {
    pwm1 = 0;
  } else {
    pwm1 = pwm1;
  }
}

void set_pwm2()  // shooter adjusting pwm of motor2
{
  factor = find_factor2();
  if (speed2 < target - 1 && pwm2 + factor <= 250) {
    pwm2 += factor;
  } else if (speed2 > target + 1 && pwm2 - factor >= 0) {
    Serial.println("I'm dying");
    pwm2 -= factor;
  } else if (target == 0) {
    pwm2 = 0;
  } else {
    pwm2 = pwm2;
  }
}

void cascading_lift_clock()  // Downward Motion
{
  analogWrite(enac, 255);
  digitalWrite(inc1, HIGH);
  digitalWrite(inc2, LOW);
}
void cascading_lift_anticlock()  // upward motion
{
  analogWrite(enac, 255);
  digitalWrite(inc1, LOW);
  digitalWrite(inc2, HIGH);
}

void flip_anticlock() {
  analogWrite(enbf, 255);
  digitalWrite(inf1, LOW);
  digitalWrite(inf2, HIGH);
}
void flip_clock() {
  analogWrite(enbf, 255);
  digitalWrite(inf1, HIGH);
  digitalWrite(inf2, LOW);
}




void setup() {
  //slot sensors for shooter
  pinMode(ir_1, INPUT);
  pinMode(ir_2, INPUT);
  //cytron-shooter
  pinMode(m1_dir, OUTPUT);
  pinMode(m1_pwm, OUTPUT);
  pinMode(m2_dir, OUTPUT);
  pinMode(m2_pwm, OUTPUT);
  digitalWrite(m1_dir, HIGH);
  digitalWrite(m2_dir, HIGH);
  analogWrite(m1_pwm, 0);
  analogWrite(m2_pwm, 0);
  //piston shooter
  pinMode(relay, OUTPUT);

  Timer1.initialize();
  Timer1.attachInterrupt(readmotor);
  //interrupt fo slot sensor - shooter
  attachInterrupt(digitalPinToInterrupt(ir_1), interrupt_routine1, FALLING);
  attachInterrupt(digitalPinToInterrupt(ir_2), interrupt_routine2, FALLING);

  //L298N - yaxis
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(enA, HIGH);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 255);

  //l298n - claw
  pinMode(inc1, OUTPUT);
  pinMode(inc2, OUTPUT);
  pinMode(enac, OUTPUT);
  digitalWrite(inc1, LOW);
  digitalWrite(inc2, LOW);
  analogWrite(enac, 255);

  //l298n - flipping
    pinMode(inf1, OUTPUT);
    pinMode(inf2, OUTPUT);
    pinMode(enbf, OUTPUT);
    digitalWrite(inf1, LOW);
    digitalWrite(inf2, LOW);
    analogWrite(enbf, 255);

  // limit switches for flipping
   pinMode(limit_switch1, INPUT);
   pinMode(limit_switch2, INPUT);
   // limit switch for cascading lift
   pinMode(limit_switch3, INPUT);

  //relay for piston
  pinMode(relay_pin, OUTPUT);
  
  
  
  Serial.begin(115200);

  canMsg1.can_id  = 0x0F6;
  canMsg1.can_dlc = 1;
  // canMsg1.data[0] = 0x8E;

  canMsg2.can_id  = 0x036;
  canMsg2.can_dlc = 1;
  // canMsg2.data[0] = 0x0E;

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

#if !defined(MIPSEL)
  while (!Serial)
    ;  // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1)
      ;  // Halt
  }
  Serial.println(F("\r\nPS4 BT Library Started"));
}


void loop() {
  Usb.Task();

  if (PS4.connected()) {
    int right_x = map(PS4.getAnalogHat(RightHatX), 0, 255, -255, 255);
    int right_y = map(PS4.getAnalogHat(RightHatY), 0, 255, 255, -255);
    if (PS4.getButtonPress(L3) && flag == 0)  //start flywheel
    {
      flag = 1;
      Serial.print("Start Flywheel");
      target = 45;
    }

    if (PS4.getButtonPress(R3))  //shoot the ring using piston
    {
      Serial.print("Shoot Ring");
      digitalWrite(relay, HIGH);
      Serial.println("SHOT!!");

    } else  //retract piston
    {
      Serial.println("Retract Piston");
      digitalWrite(relay, LOW);
    }
      if (PS4.getButtonPress(R2))  //servo
     {
       servo1.attach(s1);
       servo2.attach(s2);
       for (int i = 0; i <= 65; i++) {
         Serial.println(i);
         servo1.writeMicroseconds(2000);
         Serial.println(i);
         servo2.writeMicroseconds(1000);
       }
       servo1.detach();
       servo2.detach();
     }
    if (PS4.getButtonPress(CROSS))  //stop the flywheel
    {
      Serial.println("Shooter stop");
      flag = 0;
      target = 0;
      pwm1 = 0;
      pwm2 = 0;
      analogWrite(m1_pwm, 0);
      analogWrite(m2_pwm, 0);
    }

    //claw
    if (PS4.getButtonPress(CIRCLE))  //open claw
    {
      Serial.print("Open Claw");
      digitalWrite(relay_pin, LOW);
    } else  //close claw
    {
      Serial.println("Close Claw");
      digitalWrite(relay_pin, HIGH);
    }
     if (PS4.getButtonPress(L2) && PS4.getButtonPress(TRIANGLE))  //lift down
    {
      Serial.println("lift claw down");
      cascading_lift_clock();
    } else if (PS4.getButtonPress(TRIANGLE))  //lift up
    {
      Serial.print("Claw down");
      cascading_lift_anticlock();
            if (digitalRead(limit_switch3) == HIGH)  // shut the motor when cascading lift reaches  max hieght
            {
              digitalWrite(inc1, LOW);
              digitalWrite(inc2, LOW);
            }
    } else {
      // Serial.println("Stop Lift");
      digitalWrite(inc1, LOW);
      digitalWrite(inc2, LOW);
    }
        //flip the claw
        if (PS4.getButtonPress(L2) && PS4.getButtonPress(SQUARE))  //clock
        {
          Serial.println("flip claw anticlockwise");
          flip_clock();
        }
    
        else if (PS4.getButtonPress(SQUARE))  //anticlock
        {
          Serial.println("Flip Claw Clockwise");
          flip_anticlock();
                if (digitalRead(limit_switch2) == HIGH and digitalRead(limit_switch1 == HIGH))  // run the following when claw completes 180 degrees
                {
                  digitalWrite(inf1, LOW);  // shut the flipping motor
                  digitalWrite(inf2, LOW);
                }
        } else {
          // Serial.println("Stop Claw");
          digitalWrite(inf1, LOW);
          digitalWrite(inf2, LOW);
        }
     
    
    



    // canbus communication starts here


    
    // Send a message from the Mega to the Nano boards

    // currT1 = micros();
    // currT2 = micros();
    //Serial.println("In");
    if (PS4.getButtonPress(L2) && PS4.getButtonPress(UP)) {
      // Serial.println("UP");
      // Serial2.write("f");
      // delay(100);
      if (!f_flag) {  //send only once string f
        Serial.println("forward target : 25");
        mega_data = 'F';
        canMsg1.data[0] = mega_data;

        mcp2515.sendMessage(&canMsg1);

        f_flag = true;
        //delay(100);
      }

      b_flag = false;
      l_flag = false;
      r_flag = false;
      s_flag = false;
      c_flag = false;
      a_flag = false;
    }

    else if (PS4.getButtonPress(L2) && PS4.getButtonPress(DOWN)) {
      Serial.println("backward target 25");
      // Serial2.write("b");
      // delay(100);
      if (!b_flag) {
        // Serial.println("DOWN");
        mega_data = 'B';

        canMsg1.data[0] = mega_data;

        mcp2515.sendMessage(&canMsg1);

        b_flag = true;
        //delay(100);
      }
      f_flag = false;
      l_flag = false;
      r_flag = false;
      s_flag = false;
      c_flag = false;
      a_flag = false;
    }

    else if (PS4.getButtonPress(L2) && PS4.getButtonPress(LEFT)) {
      Serial.println("LEFT target 25");
      // Serial2.write("l");
      // delay(100);
      if (!l_flag) {
        // Serial.println("LEFT");
        mega_data = 'L';

        canMsg1.data[0] = mega_data;

        mcp2515.sendMessage(&canMsg1);
        l_flag = true;
        //delay(100);
      }

      f_flag = false;
      b_flag = false;
      r_flag = false;
      s_flag = false;
      c_flag = false;
      a_flag = false;
    }

    else if (PS4.getButtonPress(L2) && PS4.getButtonPress(RIGHT)) {
      Serial.println("RIGHT target 25");
      // Serial2.write("r");
      // delay(100);
      if (!r_flag) {
        // Serial.println("RIGHT");
        mega_data = 'r';

        canMsg1.data[0] = mega_data;

        mcp2515.sendMessage(&canMsg1);
        r_flag = true;
        //delay(100);
      }

      f_flag = false;
      b_flag = false;
      l_flag = false;
      s_flag = false;
      c_flag = false;
      a_flag = false;
    }

    else if(PS4.getButtonPress(L2) && PS4.getButtonPress(L1)){
      Serial.println("Anticlockwise: : target 15");
      if(!a_flag){
        mega_data = 'a';
        canMsg1.data[0] = mega_data;
        mcp2515.sendMessage(&canMsg1);
        a_flag = true;
        

      }
        f_flag = false;
        b_flag = false;
        l_flag = false;
        r_flag = false;
        s_flag = false;
        c_flag = false;
        
        
    }
    else if(PS4.getButtonPress(L2) && PS4.getButtonPress(R1)){
      Serial.println("clockwise: : target 15");
      if(!a_flag){
        mega_data = 'a';
        canMsg1.data[0] = mega_data;
        mcp2515.sendMessage(&canMsg1);
        a_flag = true;
        

      }
        f_flag = false;
        b_flag = false;
        l_flag = false;
        r_flag = false;
        s_flag = false;
        c_flag = false;
        
        
      }
    if (PS4.getButtonPress(UP)) {
          // Serial.println("UP");
          // Serial2.write("f");
          // delay(100);
          if (!f_flag) {  //send only once string f
            Serial.println("forward target : 20");
            mega_data = 'f';
            canMsg1.data[0] = mega_data;
    
            mcp2515.sendMessage(&canMsg1);
    
            f_flag = true;
            //delay(100);
          }
    
          b_flag = false;
          l_flag = false;
          r_flag = false;
          s_flag = false;
          c_flag = false;
          a_flag = false;
        }

    else if (PS4.getButtonPress(DOWN)) {
      Serial.println("backward target 20");
      // Serial2.write("b");
      // delay(100);
      if (!b_flag) {
        // Serial.println("DOWN");
        mega_data = 'b';

        canMsg1.data[0] = mega_data;

        mcp2515.sendMessage(&canMsg1);

        b_flag = true;
        //delay(100);
      }
      f_flag = false;
      l_flag = false;
      r_flag = false;
      s_flag = false;
      c_flag = false;
      a_flag = false;
    }

    else if (PS4.getButtonPress(LEFT)) {
      Serial.println("LEFT target 20");
      // Serial2.write("l");
      // delay(100);
      if (!l_flag) {
        // Serial.println("LEFT");
        mega_data = 'l';

        canMsg1.data[0] = mega_data;

        mcp2515.sendMessage(&canMsg1);
        l_flag = true;
        //delay(100);
      }

      f_flag = false;
      b_flag = false;
      r_flag = false;
      s_flag = false;
      c_flag = false;
      a_flag = false;
    }

    else if (PS4.getButtonPress(RIGHT)) {
      Serial.println("RIGHT target 20");
      // Serial2.write("r");
      // delay(100);
      if (!r_flag) {
        // Serial.println("RIGHT");
        mega_data = 'r';

        canMsg1.data[0] = mega_data;

        mcp2515.sendMessage(&canMsg1);
        r_flag = true;
        //delay(100);
      }

      f_flag = false;
      b_flag = false;
      l_flag = false;
      s_flag = false;
      c_flag = false;
      a_flag = false;
    }

 else {
      Serial.println("Stop");
      // Serial2.write("s");
      // delay(100);
      if (!s_flag) {
        // Serial.println("Stop");
        mega_data = 's';

        canMsg1.data[0] = mega_data;

        mcp2515.sendMessage(&canMsg1);
        s_flag = true;
        //delay(100);
      }
      f_flag = false;
      b_flag = false;
      l_flag = false;
      r_flag = false;
      c_flag = false;
      a_flag = false;
    }
  }

  // Check for incoming messages on the Nano boards
  // checkIncomingMessages(&can_nano_1, MESSAGE_ID_NANO_1);
  // checkIncomingMessages(&can_nano_2, MESSAGE_ID_NANO_2);

  // delay(1000);
}

// void checkIncomingMessages(MCP_CAN *can, int message_id) {
//   if (can->checkReceive() == CAN_MSGAVAIL) {
//     byte nano_data[8];
//     byte nano_data_length = 0;
//     unsigned long nano_message_id = 0;

//     can->readMsgBuf(&nano_message_id, &nano_data_length, nano_data);

//     if (nano_message_id == message_id) {
//       Serial.print("Message received on ");
//       Serial.print(can->getCanName());
//       Serial.print(": ");

//       for (int i = 0; i < nano_data_length; i++) {
//         Serial.print(nano_data[i], HEX);
//         Serial.print(" ");
//       }

//       Serial.println();
//     }
//   }
// }
