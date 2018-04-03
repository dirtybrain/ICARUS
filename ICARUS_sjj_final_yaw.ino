//#include <SoftwareSerial.h>
//SoftwareSerial DLSerial(6,7);
//unsigned char imu[9]={0x75,0x65,0x0C,0x03,0x03,0x01,0x10,0xFD,0xF4};//imu
unsigned char imu[9]={0x75,0x65,0x0C,0x03,0x03,0x03,0x05,0xF4,0xED};//filter; change data length!

/* ==============Pin Setup===============================
 * 2  3  4  5  6  7   Input w/ interupt   Transmitter signal in, 2:Thro 3:Alie 4:Elev 5:Rudd 6:Gear 7:Aux1
 * 8  9  10 11 12     Output              Motro and Servos, Thro:8 LF:9 LR:10 RF:11 RR:12 
 * 13                 Output              LED control
 */

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 0.36;               //Gain setting for the roll P-controller 0.5
float pid_i_gain_roll = 0.002;              //Gain setting for the roll I-controller 0.0005
float pid_d_gain_roll = 0.0;                //Gain setting for the roll D-controller 0.0
int   pid_max_roll = 600;                  //Maximum output of the PID-controller (+/-)
int   pid_i_max_roll = 250;

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int   pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)
int   pid_i_max_pitch = pid_i_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 0.36;  //Gain setting for the pitch P-controller.
float pid_i_gain_yaw = 0.0;  //Gain setting for the pitch I-controller.
float pid_d_gain_yaw = 0.0;  //Gain setting for the pitch D-controller.
int   pid_max_yaw = 600;          //Maximum output of the PID-controller (+/-)
int   pid_i_max_yaw = 250;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_lift = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_lift = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_lift = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_lift = 400;                     //Maximum output of the PID-controller (+/-)

boolean auto_level = true;                 //Auto level on (true) or off (false)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//====Receiver Signal Variables======
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4, last_channel_5, last_channel_6;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4, receiver_input_channel_5, receiver_input_channel_6;  //1:Roll 2:Pitch 3: Throttle 4:Yaw
unsigned long timer_1, timer_2, timer_3, timer_4, timer_5, timer_6, current_time;

//====Control Signal Variables======
int esc;
int servo[4]; //0-->LF 1-->RF 2-->RR 3-->LR
int throttle, lift; // ignore battery compensation for now
unsigned long timer_channel_0, timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, motor_loop_timer;

int cal_int = 0, start; //cal_int is used in gyro calibrate;  start is used for checking RX value before take off.
unsigned long loop_timer = 0;
double gyro_pitch, gyro_roll, gyro_yaw, acc_z, acc_y, acc_x;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal, gyro_roll_int, gyro_pitch_int, gyro_yaw_int, acc_z_cal, acc_y_cal, acc_x_cal, acc_total_vector, angle_roll_acc_int, angle_pitch_acc_int;

float roll_level_adjust, pitch_level_adjust, yaw_level_adjust;

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;

float pid_i_mem_lift, pid_lift_setpoint, acc_z_input, pid_output_lift, pid_last_lift_d_error;

//====Servo Pulse Mid and Limits========
int flight_adjust[4] = {120, 118, 180, 185};
int servo_lower[4] = {1300, 1180, 1300, 1300};//Servo pulse lower limit, this is maximum pitch //0-->RR 1-->LR 2-->LF 3-->RF
int servo_upper[4] = {1770, 1700, 1800, 1770};//Servo pulse upper limit, this is minimum pitch
int servo_mid[4] = {1600, 1400, 1505, 1450};//Servo Pulse mid value, this is zero pitch
int servo_flight[4]; //Servo Pulse with flight offset, this is for taking off
int servo_val[4];

float angle_level_scale = 10;

bool FirstLoop;
//float Gyro_compensation_rate = 0.1;

//int timeconst = 1;

int samplenum = 200;

boolean gyro_angles_set;

unsigned long t;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){

  Serial.begin(115200);
  Serial.println("Begin!");
  
  DDRB |= B00111111 ;                                           //Configure digital poort 8 9 10 11 12 and 13 as output.
  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.
  
  //Use the led on the Arduino for startup indication
  digitalWrite(13, HIGH);                                       //Turn on the warning led.

  gyro_roll_cal = 0;
  gyro_pitch_cal = 0;
  gyro_yaw_cal = 0;
  gyro_roll_int = 0;
  gyro_pitch_int = 0;
  gyro_yaw_int = 0;
  delayMicroseconds(1000);                                                 //Wait 1 second before continuing
  
  Serial.println("IMU Calibration...");
  delay(2);
  //Take multiple gyro data samples to determine the average gyro offset (calibration).
  while (cal_int < samplenum){              //Take 2000 readings for calibration.
    gyro_signalen();                                           //Read the gyro output.
    if(cal_int % (samplenum/4) == 0)digitalWrite(13, !digitalRead(13));   //Change the led status to indicate calibration.
    //delay(2);
    if(cal_int % (samplenum/10) == 0)Serial.println(".");
    //delay(2);
    gyro_roll_cal += gyro_roll;                                //Add roll value to gyro_roll_cal.
    gyro_pitch_cal += gyro_pitch;                              //Add pitch value to gyro_pitch_cal.
    gyro_yaw_cal += gyro_yaw;                                  //Add yaw value to gyro_yaw_cal.
    //==================================================================================
    //We don't want the esc to be beeping annoyingly. So give it a 1000us puls while calibrating the gyro, and the servos need to be keeped in a position
    
    PORTB |= B00111111;                                        //Set digital poort 8 9 10 11 12 and 13 high.
    delayMicroseconds(1000);                                   //Wait 1000us.
    PORTB &= B11100000;                                        //Set digital poort 8 9 10 11 and 12 low.
        
    cal_int++; 
    }
  delay(3);                                                  //Wait 3 milliseconds before the next loop.  
    
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  gyro_roll_int = gyro_roll_cal / samplenum;                                       //Divide the roll total by 2000.
  gyro_pitch_int = gyro_pitch_cal / samplenum;                                      //Divide the pitch total by 2000.
  gyro_yaw_int = gyro_yaw_cal / samplenum;                                        //Divide the yaw total by 2000.

  Serial.println(".");
  Serial.println("Gyro calibration Done!");
  //=========================================================================================================================================================

  //=============Interrupts===================================================================================================================================
  PCICR |= (1 << PCIE2);                                        //Set PCIE2 to enable PCMSK2 scan.
  PCMSK2 |= (1 << PCINT18);                                     //Set PCINT18 (digital input 2) to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT19);                                     //Set PCINT19 (digital input 3) to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT20);                                     //Set PCINT20 (digital input 4) to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT21);                                     //Set PCINT21 (digital input 5) to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT22);                                     //Set PCINT22 (digital input 6) to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT23);                                     //Set PCINT23 (digital input 7) to trigger an interrupt on state change.
  //=========================================================================================================================================================

  //Wait until the receiver is active and the throttle is set to the lower position.
  while(receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1250){  
    Serial.println("Waiting for Transmitor.") ;
    start ++;                                                  //While waiting increment start whith every loop.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
    PORTB |= B00000001;                                        //Set digital poort 8 9 10 11 and 12 high.
    delayMicroseconds(1000);                                   //Wait 1000us.
    PORTB &= B11100000;                                        //Set digital poort 8 9 10 11 and 12 low.
    delay(3);                                                  //Wait 3 milliseconds before the next loop.
    if(start == 125){                                          //Every 125 loops (500ms).
      digitalWrite(13, !digitalRead(13));                      //Change the led status.
      start = 0;                                               //Start again at 0.
    }
  }
  
  start = 0;                                                   //Set start back to 0.
  
  //Load the battery voltage to the battery_voltage variable.
  //65 is the voltage compensation for the diode.
  //12.6V equals ~5V @ Analog 0.
  //12.6V equals 1023 analogRead(0).
  //1260 / 1023 = 1.2317.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  //battery_voltage = (analogRead(0) + 65) * 1.2317;
  
  //When everything is done, turn off the led.
  //Reset Gyro and Acc input value
  //gyro_roll_input = 0;
  //gyro_pitch_input = 0;
  //acc_z_input = 0;

  for(int i=0;i<4;i++)
  {servo_flight[i] = servo_lower[i];}
  
  digitalWrite(13,LOW);                                        //Turn off the warning led.
  FirstLoop = 1;
  gyro_angles_set = 0;

  Serial.println("Fly!");
  
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  
  gyro_signalen();
  Serial.println(gyro_yaw);
  //Serial.print(" ");
  //Serial.println(gyro_roll);
  //Serial.println(receiver_input_channel_1);
  //Serial.print(receiver_input_channel_2);
  //Serial.print(receiver_input_channel_3);
  //Serial.print(receiver_input_channel_4);
  //Serial.print(receiver_input_channel_5);
  //Serial.println(receiver_input_channel_6);

  /*if (!gyro_angles_set){
    angle_pitch = angle_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    gyro_angles_set = true;                                                 //Set the IMU started flag.
  }*/
  
  //angle_pitch = angle_pitch * (1-Gyro_compensation_rate) - angle_pitch_acc * Gyro_compensation_rate;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  //angle_roll = angle_roll * (1-Gyro_compensation_rate) + angle_roll_acc * Gyro_compensation_rate;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.
  
  pitch_level_adjust = gyro_pitch * angle_level_scale;                                    //Zoom pitch level to the scale of the transmitter signal
  roll_level_adjust = gyro_roll * angle_level_scale;                                      //Zoom pitch level to the scale of the transmitter signal
  yaw_level_adjust = gyro_yaw * angle_level_scale;                                      //Zoom pitch level to the scale of the transmitter signal

  //For starting the motors: throttle low and yaw left (step 1).
  if(receiver_input_channel_3 < 1100 && receiver_input_channel_4 < 1100)start = 1;
  //Serial.println(start);
  //When yaw stick is back in the center position start the motors (step 2).
  if(start == 1 && receiver_input_channel_3 < 1100 && receiver_input_channel_4 > 1450){
    start = 2;

    gyro_pitch = 0;
    gyro_roll = 0;
    gyro_yaw = 0;
    
    //Reset the pid controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
    pid_i_mem_lift = 0;
    pid_last_lift_d_error = 0;
    pid_roll_setpoint = 0;
    pid_pitch_setpoint = 0;
    pid_yaw_setpoint = 0;
  }
  
  //Stopping the motors: throttle low and yaw right.
  if(start == 2 && receiver_input_channel_3 < 1100 && receiver_input_channel_4 > 1900)start = 0;
  
  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).

  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_2 > 1508) pid_roll_setpoint = receiver_input_channel_2 - 1508;
  else if(receiver_input_channel_2 < 1492)pid_roll_setpoint = receiver_input_channel_2 - 1492;

  if(receiver_input_channel_1 > 1508)pid_pitch_setpoint = 1508 - receiver_input_channel_1;
  else if(receiver_input_channel_1 < 1492)pid_pitch_setpoint =1492 - receiver_input_channel_1;
  
  
  //PID inputs are known. So we can calculate the pid output.
  //Serial.println(pid_roll_setpoint);
  calculate_pid();
  //Serial.print(pid_output_roll);
  //Serial.print(" ");
  //Serial.println(pid_output_pitch);
  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //0.09853 = 0.08 * 1.2317.
  //battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;
  
  //Turn on the led if battery voltage is to low.
  //if(battery_voltage < 1050 && battery_voltage > 600)digitalWrite(12, HIGH);
  
  throttle = receiver_input_channel_3;                                      //Throttle signal for ESC

  //start = 2;
  if (start == 2){                                                          //The motors are started.
    //if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    if (receiver_input_channel_6 > 1600)                                    //Use Aux to select flight mode or idling.
    {
      for(int i=0;i<4;i++)
      servo_val[i] = servo_flight[i];               
    }
    else 
    {
      for(int i=0;i<4;i++)
      servo_val[i] = servo_mid[i];                
    }
    //lift = receiver_input_channel_4;                                      //Get the overall lift signal
    esc = throttle;                                                         //Calculate the pulse for ESC
    
    servo[0] = servo_val[0] - pid_output_pitch - pid_output_roll + pid_output_yaw;           //Calculate the pulse for servo 1 (RR)
    servo[1] = servo_val[1] + pid_output_pitch - pid_output_roll - pid_output_yaw;           //Calculate the pulse for servo 2 (LR)
    servo[2] = servo_val[2] - pid_output_pitch + pid_output_roll - pid_output_yaw;           //Calculate the pulse for servo 3 (LF)
    servo[3] = servo_val[3] + pid_output_pitch + pid_output_roll + pid_output_yaw;           //Calculate the pulse for servo 4 (RF)
    //Serial.println(pitch_level_adjust);
    /*
    servo[0] = servo_val[0] + pid_output_pitch;           //Calculate the pulse for servo 1 (RR)
    servo[1] = servo_val[1] + pid_output_pitch;           //Calculate the pulse for servo 2 (LR)
    servo[2] = servo_val[2] - pid_output_pitch;           //Calculate the pulse for servo 3 (LF)
    servo[3] = servo_val[3] - pid_output_pitch;           //Calculate the pulse for servo 4 (RF)
    */

/*Battery Compensation
    if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
    } 
*/  
    if (esc < 1100) esc = 1050;                                                 //Keep the motor running.  
    for(int i=0;i<4;i++) 
    {
      if (servo[i] < servo_lower[i]) servo[i] = servo_lower[i];                 //Keep the servos above limits
      if (servo[i] > servo_mid[i]) servo[i] = servo_mid[i];                 //Keep the servos below limits
      //if (servo[i] > servo_upper[i]) servo[i] = servo_upper[i];                 //Keep the servos below limits
    }                                     //Keep the servos above limits.
    if (esc > 2000) esc = 2000;                                                  //Keep the motor running.  
  }
  else{
    esc = 1000;                                                               //If start is not 2 keep a 1000us pulse for esc.
    for(int i=0;i<4;i++) 
      {
        servo[i]  = servo_mid[i];                                               //If start is not 2 keep a 1000us pulse for servo-1.
      } 
  }
  
  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need their pulse every 4ms.
  
  if(FirstLoop)
  FirstLoop = 0;
  else if((micros() - loop_timer) > 10000)
  digitalWrite(13,HIGH);
  else
  digitalWrite(13,LOW);

  //digitalWrite(13,LOW);
  while(micros() - loop_timer < 10000){};                                     //We wait until 4000us are passed
  //Serial.println(micros() - loop_timer);
  loop_timer = micros();                                                    //Set the timer for the next loop.

  
  PORTB |= B00011111;                                                       //Set digital poort 8 9 10 11 and 12 high.
  timer_channel_0 = esc + loop_timer;                                       //Calculate the time of the faling edge of the esc
  timer_channel_1 = servo[0] + loop_timer;                                   //Calculate the time of the faling edge of the servo-1 pulse.
  timer_channel_2 = servo[1] + loop_timer;                                   //Calculate the time of the faling edge of the servo-2 pulse.
  timer_channel_3 = servo[2] + loop_timer;                                   //Calculate the time of the faling edge of the servo-3 pulse.
  timer_channel_4 = servo[3] + loop_timer;                                   //Calculate the time of the faling edge of the servo-4 pulse.
  
  while(PORTB >= 1){                                                        //Stay in this loop until output 8 9 10 11 and 12 are low.
    //Serial.println(servo[0]);
    motor_loop_timer = micros();                                            //Read the current time.
    if(timer_channel_0 <= motor_loop_timer)PORTB &= B11111110;              //Set digital output 8  to low if the time is expired.
    if(timer_channel_1 <= motor_loop_timer)PORTB &= B11111101;              //Set digital output 9  to low if the time is expired.
    if(timer_channel_2 <= motor_loop_timer)PORTB &= B11111011;              //Set digital output 10 to low if the time is expired.
    if(timer_channel_3 <= motor_loop_timer)PORTB &= B11110111;              //Set digital output 11 to low if the time is expired.
    if(timer_channel_4 <= motor_loop_timer)PORTB &= B11101111;              //Set digital output 12 to low if the time is expired
  }

  //PORTB |= B00000010;                                        //Set digital poort 8 9 10 11 12 and 13 high.
  //delayMicroseconds(1800+gyro_roll*3);                                   //Wait 1000us.
  //delayMicroseconds(servo[1]);
  //PORTB &= B11100000;
  
  //Serial.println(micros()-t);
}
    

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This routine is called every time input 2 3 4 5 6 or 7 changed state
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(PCINT2_vect){
  current_time = micros();
  //Channe 1 THRO=========================================
  if(PIND & B00000100){                                        //Is input 2 high?
    if(last_channel_1 == 0){                                   //Input 2 changed from 0 to 1
      last_channel_1 = 1;                                      //Remember current input state
      timer_1 = current_time;                                  //Set timer_1 to current_time
    }
  }
  else if(last_channel_1 == 1){                                //Input 2 is not high and changed from 1 to 0
    last_channel_1 = 0;                                        //Remember current input state
    receiver_input_channel_1 = current_time - timer_1;         //Channel 1 is current_time - timer_1
  }
  //Channe 2 AILE=========================================
  if(PIND & B00001000 ){                                       //Is input 3 high?
    if(last_channel_2 == 0){                                   //Input 3 changed from 0 to 1
      last_channel_2 = 1;                                      //Remember current input state
      timer_2 = current_time;                                  //Set timer_2 to current_time
    }
  }
  else if(last_channel_2 == 1){                                //Input 3 is not high and changed from 1 to 0
    last_channel_2 = 0;                                        //Remember current input state
    receiver_input_channel_2 = current_time - timer_2;         //Channel 2 is current_time - timer_2
  }
  //Channel 3 ELEV=========================================
  if(PIND & B00010000 ){                                       //Is input 4 high?
    if(last_channel_3 == 0){                                   //Input 4 changed from 0 to 1
      last_channel_3 = 1;                                      //Remember current input state
      timer_3 = current_time;                                  //Set timer_3 to current_time
    }
  }
  else if(last_channel_3 == 1){                                //Input 4 is not high and changed from 1 to 0
    last_channel_3 = 0;                                        //Remember current input state
    receiver_input_channel_3 = current_time - timer_3;         //Channel 3 is current_time - timer_3

  }
  //Channel 4 RUDD=========================================
  if(PIND & B00100000 ){                                       //Is input 5 high?
    if(last_channel_4 == 0){                                   //Input 5 changed from 0 to 1
      last_channel_4 = 1;                                      //Remember current input state
      timer_4 = current_time;                                  //Set timer_4 to current_time
    }
  }
  else if(last_channel_4 == 1){                                //Input 5 is not high and changed from 1 to 0
    last_channel_4 = 0;                                        //Remember current input state
    receiver_input_channel_4 = current_time - timer_4;         //Channel 4 is current_time - timer_4
  }
  //Channel 5 GEAR=========================================
  if(PIND & B01000000 ){                                       //Is input 6 high?
    if(last_channel_5 == 0){                                   //Input 6 changed from 0 to 1
      last_channel_5 = 1;                                      //Remember current input state
      timer_5 = current_time;                                  //Set timer_5 to current_time
    }
  }
  else if(last_channel_5 == 1){                                //Input 6 is not high and changed from 1 to 0
    last_channel_5 = 0;                                        //Remember current input state
    receiver_input_channel_5 = current_time - timer_5;         //Channel 5 is current_time - timer_5
  }
  //Channel 6 AUX1=========================================
  if(PIND & B10000000 ){                                       //Is input 7 high?
    if(last_channel_6 == 0){                                   //Input 7 changed from 0 to 1
      last_channel_6 = 1;                                      //Remember current input state
      timer_6 = current_time;                                  //Set timer_6 to current_time
    }
  }
  else if(last_channel_6 == 1){                                //Input 7 is not high and changed from 1 to 0
    last_channel_6 = 0;                                        //Remember current input state
    receiver_input_channel_6 = current_time - timer_6;         //Channel 6 is current_time - timer_6
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for reading the gyro
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void gyro_signalen(){
  
  int j = 0;
  unsigned char data[22] = "";//22 for filter; 20 for imu
  Serial.write(imu,9);

  while (Serial.available() == 0){}
  
  while (Serial.available() > 0)
  {
    data[j] = Serial.read();
    j++;
    delayMicroseconds(80);
  }
  
  if (j == 22)
  {
    gyro_roll = floattodecimal(data[6],data[7],data[8],data[9]) * 57.3 - gyro_roll_int;
    gyro_pitch = floattodecimal(data[10],data[11],data[12],data[13]) * 57.3 - gyro_pitch_int;
    gyro_yaw = floattodecimal(data[14],data[15],data[16],data[17]) * 57.3 - gyro_yaw_int;
    //acc_x = floattodecimal(data[6],data[7],data[8],data[9]);
    //acc_y = floattodecimal(data[10],data[11],data[12],data[13]);
    //acc_z = floattodecimal(data[14],data[15],data[16],data[17]);
    //gyro_roll = floattodecimal(data[20],data[21],data[22],data[23]) - gyro_roll_int;
    //gyro_pitch = floattodecimal(data[24],data[25],data[26],data[27]) - gyro_pitch_int;
    //gyro_yaw = floattodecimal(data[28],data[29],data[30],data[31]) - gyro_yaw_int;
    //Serial.print(gyro_roll);
    //Serial.print(" ");
    //Serial.println(gyro_pitch);
  }
}

typedef union
{
  unsigned char u8[4];
  float f32;
}t_F32;

float floattodecimal(char byte0, char byte1, char byte2, char byte3)
{
  t_F32 t;
  t.u8[0] = byte3;
  t.u8[1] = byte2;
  t.u8[2] = byte1;
  t.u8[3] = byte0;
  return t.f32;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void calculate_pid(){

  //Roll calculations
  pid_error_temp = roll_level_adjust - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_i_max_roll)pid_i_mem_roll = pid_i_max_roll;
  else if(pid_i_mem_roll < pid_i_max_roll * -1)pid_i_mem_roll = pid_i_max_roll * -1;
  
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;
  
  pid_last_roll_d_error = pid_error_temp;
 
  //Pitch calculations
  pid_error_temp = pitch_level_adjust - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_i_max_pitch)pid_i_mem_pitch = pid_i_max_pitch;
  else if(pid_i_mem_pitch < pid_i_max_pitch * -1)pid_i_mem_pitch = pid_i_max_pitch * -1;
  
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;
    
  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = yaw_level_adjust - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_i_max_yaw)pid_i_mem_yaw = pid_i_max_yaw;
  else if(pid_i_mem_yaw < pid_i_max_yaw * -1)pid_i_mem_yaw = pid_i_max_yaw * -1;
  
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
    
  pid_last_yaw_d_error = pid_error_temp;

  /*Lift calculations
  //Lift calculations
  pid_error_temp = acc_z_input - pid_lift_setpoint;
  pid_i_mem_lift += pid_i_gain_lift * pid_error_temp;
  if(pid_i_mem_lift > pid_max_lift)pid_i_mem_lift = pid_max_lift;
  else if(pid_i_mem_lift < pid_max_lift * -1)pid_i_mem_lift = pid_max_lift * -1;
  
  pid_output_lift = pid_p_gain_lift * pid_error_temp + pid_i_mem_lift + pid_d_gain_lift * (pid_error_temp - pid_last_lift_d_error);
  if(pid_output_lift > pid_max_lift)pid_output_lift = pid_max_lift;
  else if(pid_output_lift < pid_max_lift * -1)pid_output_lift = pid_max_lift * -1;
    
  pid_last_lift_d_error = pid_error_temp;
  */
}


