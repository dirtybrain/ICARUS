/* ==============Pin Setup===============================
 * 2  3  4  5  6  7   Input w/ interupt   Transmitter signal in, 2:Thro 3:Alie 4:Elev 5:Rudd 6:Gear 7:Aux1
 * 8  9  10 11 12     Output              Motro and Servos, Thro:8 LF:9 LR:10 RF:11 RR:12 
 * 13                 Output              LED control
 */

#include <Wire.h> //Include the Wire.h library so we can communicate with the gyro.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 0.05;               //Gain setting for the roll P-controller 0.45
float pid_i_gain_roll = 0.01;              //Gain setting for the roll I-controller 
float pid_d_gain_roll = 0.000;                //Gain setting for the roll D-controller 
int   pid_max_roll = 600;                  //Maximum output of the PID-controller (+/-)
int   pid_i_max_roll = 250;

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int   pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)
int   pid_i_max_pitch = pid_i_max_roll;          //Maximum output of the PID-controller (+/-)

// Ignore Yaw for now
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

int cal_int, start; //cal_int is used in gyro calibrate;  start is used for checking RX value before take off.
unsigned long loop_timer = 0;
double gyro_pitch, gyro_roll, gyro_yaw, acc_z, acc_y, acc_x;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal, acc_z_cal, acc_y_cal, acc_x_cal, acc_total_vector, angle_roll_acc_int, angle_pitch_acc_int;
byte highByte, lowByte;

float roll_level_adjust, pitch_level_adjust;

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;

float pid_i_mem_lift, pid_lift_setpoint, acc_z_input, pid_output_lift, pid_last_lift_d_error;

//====Servo Pulse Mid and Limits========
int flight_adjust[4] = {120, 118, 180, 185};
int servo_lower[4] = {1150, 1140, 1180, 1190};//Servo pulse lower limit, this is maximum pitch //0-->RR 1-->LR 2-->LF 3-->RF
int servo_upper[4] = {1690, 1700, 1800, 1770};//Servo pulse upper limit, this is minimum pitch
int servo_mid[4] = {1400, 1400, 1505, 1520};//Servo Pulse mid value, this is zero pitch
int servo_flight[4]; //Servo Pulse with flight offset, this is for taking off
int servo_val[4]; 

int angel_level_scale = 10;

int i;

bool FirstLoop;
float Gyro_compensation_rate = 0.1;

int timeconst = 1;

boolean gyro_angles_set;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){

  Wire.begin();                                                //Start the I2C as master.
  TWBR = 12;                                                                //Set the I2C clock speed to 400kHz.

  
  Serial.begin(9600);
  Serial.println("Begin!");
  
  
  DDRB |= B00111111;                                           //Configure digital poort 8 9 10 11 12 and 13 as output.
  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.
  
  //Use the led on the Arduino for startup indication
  digitalWrite(13,HIGH);                                       //Turn on the warning led.
  delay(1000);                                                 //Wait 1 second befor continuing.

  IMU_Setup();

  gyro_roll_cal = 0;
  gyro_pitch_cal = 0;
  gyro_yaw_cal = 0;
  
  Serial.print("IMU Calibration...");
  //Take multiple gyro data samples to determine the average gyro offset (calibration).
  for (cal_int = 0; cal_int < 2000 ; cal_int ++){              //Take 2000 readings for calibration.
    if(cal_int % 500 == 0)digitalWrite(13, !digitalRead(13));   //Change the led status to indicate calibration.
    gyro_signalen();                                           //Read the gyro output.
    if(cal_int % 200 == 0)Serial.print(".");
    gyro_roll_cal += gyro_roll;                                //Add roll value to gyro_roll_cal.
    gyro_pitch_cal += gyro_pitch;                              //Add pitch value to gyro_pitch_cal.
    gyro_yaw_cal += gyro_yaw;                                  //Add yaw value to gyro_yaw_cal.
    acc_z_cal += acc_z;                                        //Add z acc value to acc_z_cal.
    acc_x_cal += acc_x;                                        //Add z acc value to acc_z_cal.
    acc_y_cal += acc_y;                                        //Add z acc value to acc_z_cal.
    
    //==================================================================================
    //We don't want the esc to be beeping annoyingly. So give it a 1000us puls while calibrating the gyro, and the servos need to be keeped in a position

    PORTB |= B00100001;                                        //Set digital poort 8 9 10 11 12 and 13 high.
    delayMicroseconds(1000);                                   //Wait 1000us.
    PORTD &= B11100000;                                        //Set digital poort 8 9 10 11 and 12 low.
    
    //==================================================================================
    
    
    
    delay(3);                                                  //Wait 3 milliseconds before the next loop.
  }
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  gyro_roll_cal /= 2000;                                       //Divide the roll total by 2000.
  gyro_pitch_cal /= 2000;                                      //Divide the pitch total by 2000.
  gyro_yaw_cal /= 2000;                                        //Divide the yaw total by 2000.
  acc_z_cal /= 2000;                                           //Divide the z_acc total by 2000.
  acc_x_cal /= 2000;                                           //Divide the x_acc total by 2000.
  acc_y_cal /= 2000;                                           //Divide the y_acc total by 2000.

  acc_total_vector = sqrt((acc_x_cal*acc_x_cal)+(acc_y_cal*acc_y_cal)+(acc_z_cal*acc_z_cal));       //Calculate the total accelerometer vector.
  angle_roll_acc_int =  asin((float)acc_y_cal/acc_total_vector)* 57.296;          //Calculate the pitch angle.
  angle_pitch_acc_int =  asin((float)acc_x_cal/acc_total_vector)* 57.296;          //Calculate the roll angle.

  angle_pitch = 0;
  angle_roll  = 0;

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

  //Wait until the receiver is active and the throtle is set to the lower position.
  while(receiver_input_channel_1 < 990 || receiver_input_channel_1 > 1250 || receiver_input_channel_4 < 1400){  
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
  gyro_roll_input = 0;
  gyro_pitch_input = 0;
  acc_z_input = 0;


  for(i=0;i<4;i++)
  servo_flight[i] = servo_lower[i]+flight_adjust[i];
  
  digitalWrite(13,LOW);                                        //Turn off the warning led.
  FirstLoop = 1;
  gyro_angles_set = 0;

  
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  
  //Let's get the current gyro data and scale it to degrees per second for the pid calculations.
  gyro_signalen();

  //gyro_pitch_input = gyro_pitch_input * 0.2 + gyro_pitch * 0.8;
  
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_pitch * 0.0000611 * timeconst;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += gyro_roll * 0.0000611 * timeconst;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066 * timeconst);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066 * timeconst);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       //Calculate the total accelerometer vector.
  
  angle_pitch_acc = asin((float)acc_x/acc_total_vector)* 57.296 - angle_roll_acc_int;          //Calculate the pitch angle.

  angle_roll_acc = asin((float)acc_y/acc_total_vector)* 57.296 - angle_pitch_acc_int;          //Calculate the roll angle.

  if (!gyro_angles_set){
    angle_pitch = angle_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    gyro_angles_set = true;                                                 //Set the IMU started flag.
  }
  angle_pitch = angle_pitch * (1-Gyro_compensation_rate) - angle_pitch_acc * Gyro_compensation_rate;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * (1-Gyro_compensation_rate) + angle_roll_acc * Gyro_compensation_rate;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = angle_pitch * angel_level_scale;                                    //Zoom pitch level to the scale of the transmitter signal
  roll_level_adjust = angle_roll * angel_level_scale;                                      //Zoom pitch level to the scale of the transmitter signal


  //For starting the motors: throttle low and yaw left (step 1).
  if(receiver_input_channel_1 < 1300 && receiver_input_channel_4 < 1200)start = 1;
  //When yaw stick is back in the center position start the motors (step 2).
  if(start == 1 && receiver_input_channel_1 < 1300 && receiver_input_channel_4 > 1450){
    start = 2;

    angle_pitch = 0;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = 0;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.

    
    //Reset the pid controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_lift = 0;
    pid_last_lift_d_error = 0;
  }
  //Stopping the motors: throttle low and yaw right.
  if(start == 2 && receiver_input_channel_1 < 1300 && receiver_input_channel_4 > 1900)start = 0;
  
  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).

  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_2 > 1508) pid_roll_setpoint = receiver_input_channel_2 - 1508;
  else if(receiver_input_channel_2 < 1492)pid_roll_setpoint = receiver_input_channel_2 - 1492;

  //pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
  //pid_roll_setpoint /= 2.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.



  if(receiver_input_channel_3 > 1508)pid_pitch_setpoint = receiver_input_channel_3 - 1508;
  else if(receiver_input_channel_3 < 1492)pid_pitch_setpoint = receiver_input_channel_3 - 1492;

  //pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
  //pid_pitch_setpoint /= 2.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.
  //pid_pitch_setpoint = - pid_pitch_setpoint;

/*
  pid_lift_setpoint = 0;
   if(receiver_input_channel_1 > 1050){ //Do not lift when turning off the motors.
    if(receiver_input_channel_4 > 1508)pid_lift_setpoint = (receiver_input_channel_4 - 1508)/3.0;
    else if(receiver_input_channel_4 < 1492)pid_lift_setpoint = (receiver_input_channel_4 - 1492)/3.0;
   }
   */


  
  //PID inputs are known. So we can calculate the pid output.
  calculate_pid();
  
  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //0.09853 = 0.08 * 1.2317.
  //battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;
  
  //Turn on the led if battery voltage is to low.
  //if(battery_voltage < 1050 && battery_voltage > 600)digitalWrite(12, HIGH);
  
  throttle = receiver_input_channel_1;                                      //Throttle signal for ESC
  
  if (start == 2){                                                          //The motors are started.
    //if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    if (receiver_input_channel_6 > 1600)                                    //Use Aux to select flight mode or idling.
    {
      for(i=0;i<4;i++)
      servo_val[i] = servo_flight[i];               
    }
    else 
    {
      for(i=0;i<4;i++)
      servo_val[i] = servo_mid[i];                
    }
    //lift = receiver_input_channel_4;                                      //Get the overall lift signal
    esc = throttle;                                                         //Calculate the pulse for ESC
/*
    servo[0] =  servo_val[0] + pid_output_pitch - pid_output_roll;           //Calculate the pulse for servo 1 (RR)
    servo[1] =  servo_val[1] + pid_output_pitch + pid_output_roll;           //Calculate the pulse for servo 2 (LR)
    servo[2] =  servo_val[2] - pid_output_pitch + pid_output_roll;           //Calculate the pulse for servo 3 (LF)
    servo[3] =  servo_val[3] - pid_output_pitch - pid_output_roll;           //Calculate the pulse for servo 4 (RF)
*/
    servo[0] =  servo_val[0] + pid_output_pitch;           //Calculate the pulse for servo 1 (RR)
    servo[1] =  servo_val[1] + pid_output_pitch;           //Calculate the pulse for servo 2 (LR)
    servo[2] =  servo_val[2] - pid_output_pitch;           //Calculate the pulse for servo 3 (LF)
    servo[3] =  servo_val[3] - pid_output_pitch;           //Calculate the pulse for servo 4 (RF)


/*Battery Compensation
    if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
    } 
*/  
    if (esc < 1100) esc = 1050;                                                 //Keep the motor running.  
    for(i=0;i<4;i++) 
    {
      if (servo[i] < servo_lower[i]) servo[i] = servo_lower[i];                 //Keep the servos above limits
      if (servo[i] > servo_mid[i]) servo[i] = servo_mid[i];                 //Keep the servos below limits
      //if (servo[i] > servo_upper[i]) servo[i] = servo_upper[i];                 //Keep the servos below limits
    }                                     //Keep the servos above limits.

    if (esc > 2000) esc = 2000;                                                  //Keep the motor running.  
  }
  
  else{
    esc = 1000;                                                               //If start is not 2 keep a 1000us pulse for esc.
      for(i=0;i<4;i++) 
      {
        servo[i]  = servo_mid[i];                                               //If start is not 2 keep a 1000us pulse for servo-1.
      } 
    }
  
  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need their pulse every 4ms.


  
  if(FirstLoop)
  FirstLoop = 0;
  else if((micros() - loop_timer) > 4000 * timeconst)
  digitalWrite(13,HIGH);
  else
  digitalWrite(13,LOW);;
  
  while(micros() - loop_timer < 4000 * timeconst);                                      //We wait until 4000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTB |= B00011111;                                                       //Set digital poort 8 9 10 11 and 12 high.
  timer_channel_0 = esc + loop_timer;                                       //Calculate the time of the faling edge of the esc
  timer_channel_1 = servo[0] + loop_timer;                                   //Calculate the time of the faling edge of the servo-1 pulse.
  timer_channel_2 = servo[1] + loop_timer;                                   //Calculate the time of the faling edge of the servo-2 pulse.
  timer_channel_3 = servo[2] + loop_timer;                                   //Calculate the time of the faling edge of the servo-3 pulse.
  timer_channel_4 = servo[3] + loop_timer;                                   //Calculate the time of the faling edge of the servo-4 pulse.

  while(PORTB >= 1){                                                        //Stay in this loop until output 8 9 10 11 and 12 are low.
    motor_loop_timer = micros();                                            //Read the current time.
    if(timer_channel_0 <= motor_loop_timer)PORTB &= B11111110;              //Set digital output 8  to low if the time is expired.
    if(timer_channel_1 <= motor_loop_timer)PORTB &= B11111101;              //Set digital output 9  to low if the time is expired.
    if(timer_channel_2 <= motor_loop_timer)PORTB &= B11111011;              //Set digital output 10 to low if the time is expired.
    if(timer_channel_3 <= motor_loop_timer)PORTB &= B11110111;              //Set digital output 11 to low if the time is expired.
    if(timer_channel_4 <= motor_loop_timer)PORTB &= B11101111;              //Set digital output 12 to low if the time is expired.
  }

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
  //Read from the gyro
  Wire.beginTransmission(107);                                 //Start communication with the gyro (adress 1101011)
  Wire.write(168);                                             //Start reading @ register 28h and auto increment with every read
  Wire.endTransmission();                                      //End the transmission
  Wire.requestFrom(107, 6);                                    //Request 6 bytes from the gyro
  while(Wire.available() < 6);                                 //Wait until the 6 bytes are received
  
  lowByte = Wire.read();                                       //First received byte is the low part of the angular data
  highByte = Wire.read();                                      //Second received byte is the high part of the angular data
  gyro_roll = ((highByte<<8)|lowByte);                         //Multiply highByte by 256 (shift left by 8) and ad lowByte
  if(cal_int == 2000)gyro_roll -= gyro_roll_cal;               //Only compensate after the calibration
  lowByte = Wire.read();                                       //First received byte is the low part of the angular data
  highByte = Wire.read();                                      //Second received byte is the high part of the angular data
  gyro_pitch = ((highByte<<8)|lowByte);                        //Multiply highByte by 256 (shift left by 8) and ad lowByte
  gyro_pitch *= 1;                                            //Invert axis
  if(cal_int == 2000)gyro_pitch -= gyro_pitch_cal;             //Only compensate after the calibration
  lowByte = Wire.read();                                       //First received byte is the low part of the angular data
  highByte = Wire.read();                                      //Second received byte is the high part of the angular data
  gyro_yaw = ((highByte<<8)|lowByte);                          //Multiply highByte by 256 (shift left by 8) and ad lowByte
  gyro_yaw *= -1;                                              //Invert axis
  if(cal_int == 2000)gyro_yaw -= gyro_yaw_cal;                 //Only compensate after the calibration


  
  //Read from the accelerometer
  Wire.beginTransmission(25);                                  //Tell the accelerometer that the master is requesting data (adress 0011001 SAD)
  Wire.write(168);                                            ///Start reading @ register 28h and auto increment with every read
  Wire.endTransmission();                                      //End the transmission 
  Wire.requestFrom(25,6);                                      //Request 2 bytes from the gyro



  while(Wire.available()<6);                                   //Wait until the 2 bytes are received
   
  lowByte = Wire.read();                                       //Get the low part of the acc data
  highByte = Wire.read();                                      //Get the high part of the acc data
  acc_x = ((highByte<<8)|lowByte);                             //Multiply highByte by 256 and ad lowByte
  
  
  lowByte = Wire.read();                                       //Get the low part of the acc data
  highByte = Wire.read();                                      //Get the high part of the acc data
  acc_y = ((highByte<<8)|lowByte);                             //Multiply highByte by 256 and ad lowByte
  

  lowByte = Wire.read();                                       //Get the low part of the acc data
  highByte = Wire.read();                                      //Get the high part of the acc data
  acc_z = ((highByte<<8)|lowByte);                             //Multiply highByte by 256 and ad lowByte
  
  
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

void IMU_Setup(){
  
  //===================Gyro Setup============================================================================================================================
  //---------Gyro Setup--------
  //Serial.println("Setting Gyro...");
  Wire.beginTransmission(107);                                 //Start communication with the gyro (adress 1101011)
  Wire.write(0x20);                                            //We want to write to register 1 (20 hex)
  Wire.write(0x0F);                                            //Set the register bits as 00001111 (Turn on the gyro and enable all axis)
  Wire.endTransmission();                                      //End the transmission with the gyro

  Wire.beginTransmission(107);                                 //Start communication with the gyro (adress 1101011)
  Wire.write(0x23);                                            //We want to write to register 4 (23 hex)
  Wire.write(0x90);                                            //Set the register bits as 10010000 (Block Data Update active & 500dps full scale)
  Wire.endTransmission();                                      //End the transmission with the gyro
  //Serial.println("Gyro setting Done! Begin calibration...");
  
  //--------Accelerometer Setup--------
  //The accelerometer is disabled by default and needs to be started while set the ODR
  Wire.beginTransmission(25);  //Start writing to the accelerometer (adress 0011001)
  Wire.write(0x20);            //Write to register 20, CTRL_REG1_A
  Wire.write(0x67);            //Set the register bits as 01100111 (Turn on the gyro and enabe all axis with ODR 200Hz)
  Wire.endTransmission();      //End the transmission with the accelerometer
  //Setup BDU
  Wire.beginTransmission(25); //Start communication with the accelerometer (adress 0011001)
  Wire.write(0x23);            //Write to register 23, CTRL_REG4_A
  Wire.write(0x90);            //Set the register bits as 10010000 (output registers not updated until MSb and LSb reading, +/- 4G)
  Wire.endTransmission();      //End the transmission with the accelerometer

  delay(250);                  //Give the gyro time to start.
  
}


