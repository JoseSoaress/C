// Spongbob project 
// This was a university project
// The idea of this project was to controlle a wooden box that would behave like a inverted pendulum
// In this project was used the following hardware: Arduino Mega 2560,2 12V DC motors, 1 H bridge,
// ultra-sound sensor, bluetooth module HC-06, Giroscope and accelarometer module  MPU650, and 2 bateries
// A small app was developed that would send moving signals by bluetooth

// This was done for learning purposes only


#include "TimerOne.h"
#include <Wire.h>
#define PWM1 7 //PWM motor 1
#define DIR1 6 // Direction of Motor 1
#define PWM2 8 // PWM motor 2
#define DIR2 9 // Direction of Motor 2
#define PIN_TRIG 36
#define PIN_ECHO 34
#define MAX_ADC 1023 //max 10 bit value of the ADC
#define sensor A0 // Sharp IR GP2Y0A41SK0F (4-30cm, analog)

int counter_encoder=0;//variable from the encoder
unsigned char previous_byte;//variable for the bluetooth
unsigned char current_value; //variable for the bluetooth
unsigned char x_value;//variable for the bluetooth
unsigned char valor_y;//variable for the bluetooth
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long loop_timer;

//anles, and accelarations
float angle_pitch, angle_roll;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;

// variables for the controlo
int reference_zero = 30; // value in degres from when the robot is leveled
float reference_angle;
float reference_encoder=0;
volatile float Cn = 0.0; //controler
volatile float Cn_1 = 0.0;
volatile float Cn2 = 0.0; //controler
volatile float Cn_12 = 0.0;
volatile float Cn_enc= 0.0;
volatile float Cn_enc_1 = 0.0;
volatile float U1 = 0.0; //variable to control motor 1
volatile float U2 = 0.0; //variable to control motor 2

//Gains from our sistem
float kp; //gain from the proporcional controler 
float ki;//gain from the integrative controloador 
float kd;//ganhodo derivative controler 
float kp_enc; //gain from the proporcional controler 
float ki_enc;//gain from the integrative controloador 
float kd_enc;//gain from the derivative controler 

//Error variables
float error = 0.0; //error between the reference and the real value
float prev_error_1 = 0.0;
float prev_error_2 = 0.0;
float error2 = 0.0; //error2 between the reference and the real value 
float prev_error_12 = 0.0;
float prev_error_22 = 0.0;
float erro_enc = 0.0; ////error between the reference and the real value (encoder)
float prev_enconder_error_1 = 0.0;
float prev_enconder_error_2 = 0.0;

//variables for the distance sensor
float alpha_repulsor = 0; //Works but then if falls
//float alpha_repulsor = 0.25;//Works but not perfecly, needs tuning
float base_distance = 250.0;
float sensor_value_ant = 0;
float sensor_value = 0;

//Joystick variables
float x_velocity=0;
float y_velocity=0;
float vel_gain_app_x=0.008;
float vel_gain_app_y=0.18;
volatile char flag_joy=0;
float tolerance=0.75;
int ki_error=ki_error+error;
int prev_kd_error=error;

//***************************Defenicoes de variables atribuicao de portos e setup dos registos************************
void setup() {
setup_mpu_6050_registers(); //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro
TCCR4B = TCCR4B & 0b11111000 | 0b1; //Chang PWM frequency 
pinMode(PWM1, OUTPUT);// define pin as an output
pinMode(DIR1, OUTPUT);// define port as an output
pinMode(PWM2, OUTPUT);// define pin as an output
pinMode(DIR2, OUTPUT);// define port as an output
Wire.begin(); //Start I2C as master
Serial.begin(115200); //Use only for debugging
Serial1.begin(9600);
digitalWrite(2, HIGH); // turn on pullup resistors
digitalWrite(3, HIGH); // turn on pullup resistors
digitalWrite(DIR1, HIGH); //ESCREVE 1 nos motores por defeito
digitalWrite(DIR2, HIGH); 
Timer1.initialize(4005); // Initialize o Timer1 and configure for a 0,5 periode
Timer1.attachInterrupt(callback); //Setting up interrupt
//A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.~
attachInterrupt(0, ai0, RISING);
//B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
attachInterrupt(1, ai1, RISING);
kp = 7;
ki = 1.20;
kd = 0.0630;
kp_enc = 0.261;
ki_enc = 0.330;
kd_enc = 0.00560;
reference_angle = 4.29;
}
//**************Interruptions to verify the encoder (blue wheel) **********************
void ai0() {// Canal A
// ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
// Check pin 3 to determine the direction
if (digitalRead(3) == LOW) {
counter_encoder++;
} else {
counter_encoder--;
}
}
void ai1() {//Canal B
// ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
// Check with pin 2 to determine the direction
if (digitalRead(2) == LOW) {
counter_encoder--;
} else {
counter_encoder++;
}
}
//*********************Interruption timer to read giroscopic and accelarometer
//*******************************4 in 4 ms***********************************************
void callback()
{
//Gyro angle calculations
//0.0000611 = 1 / (250Hz / 65.5) // we are working in the 250HZ frequency
angle_pitch += gyro_x * 0.0000611; //Calculate the traveled pitch angle and add this to the angle_pitch variable
angle_roll += gyro_y * 0.0000611; //Calculate the traveled roll angle and add this to the angle_roll variable
//0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
angle_pitch += angle_roll * sin(gyro_z * 0.000001066); //If the IMU has yawed transfer the roll angle to the pitch angel
angle_roll -= angle_pitch * sin(gyro_z * 0.000001066); //If the IMU has yawed transfer the pitch angle to the roll angel
//Accelerometer angle calculations
acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); //Calculate the total accelerometer vector
//57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296; //Calculate the pitch angle
angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296; //Calculate the roll angle
//Place the MPU-6050 spirit level and note the values in the following two lines for calibration
angle_pitch_acc -= 0.0; //Accelerometer calibration value for pitch
angle_roll_acc -= 0.0; //Accelerometer calibration value for roll
if (set_gyro_angles) { //If the IMU is already started
angle_pitch = angle_pitch * 0.9900 + angle_pitch_acc * 0.01; //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
angle_roll = angle_roll * 0.9900 + angle_roll_acc * 0.01;
}
else { //At first start
angle_pitch = angle_pitch_acc; //Set the gyro pitch angle equal to the accelerometer pitch angle
angle_roll = angle_roll_acc; //Set the gyro roll angle equal to the accelerometer roll angle
set_gyro_angles = true; //Set the IMU started flag
}
//To dampen the pitch and roll angles a complementary filter is used
angle_pitch_output = angle_pitch_output * 0 + angle_pitch * 1; //Take 90% of the output pitch value and add 10% of the raw pitch value
angle_roll_output = angle_roll_output * 0 + angle_roll * 1; //Take 90% of the output roll value and add 10% of the raw roll value
}

//******************** MAIN ***************************
void loop() {
static double spins=0.0;
float repulse_force = 0;
float repulse_force_u1 = 0;
float repulse_force_u2 = 0;
float sensor_filtrado = 0;
read_mpu_6050_data(); //Read the raw acc and gyro data from the MPU-6050
//spins=counter_encoder/50;//Divide the wheel in 8
spins=counter_encoder/101.5; //Divide the wheel in 4
erro_enc = reference_encoder + spins; // error = reference + sensor_value
// **********************************************************************************************
// ***** Low pass filter to to attenuate and adjust the values ​​read from the IR sensor **********
sensor_value = analogRead(sensor);
sensor_filtrado = sensor_value * 0.01 + sensor_value_ant * 0.99;
sensor_value_ant = sensor_value;
// **********************************************************************************************
//****If the robot is not in the initial position, it changes the energy values ​​to head *********
//****towards the origin Change the angle of inclination to move forward without tipping over ***
// *********************************************************************************************

//****************************** If it encounters an obstacle, the robot moves back *************
error = (reference_angle) - angle_pitch_output;
if (sensor_value >= 300)
{
error = (reference_angle-0.332) - angle_pitch_output;
error2 = error;//The error is equal to both engines because we want to controlle them together
repulse_force = -10;//increse velocity backwards
}
else{
// This part commented below is related to position control
/* repulse_force=0; // se nao encontrar obstaculo nao temos forca repulsao
if(Cn_enc<0)
{
error = (reference_angle+0.332) - angle_pitch_output; // error = reference- sensor_value
error2 = error;//The error is equal to both engines because we want to controlle them together
repulse_force=0;
repulse_force_u1= 0;
repulse_force_u2= 0;
}
else
{
error = (reference_angle-0.121) - angle_pitch_output; // error = reference- sensor_value
error2 = error;//error do motor e igual ao 1, pois sao para controlar os 2 ao mesmo tempo
repulse_force=0;
repulse_force_u1= 0;
repulse_force_u2= 0;
}
*/
//****************************** If there is commands to move, from the app **********+
if(flag_joy==1) //(error<tolerance||error<-tolerance)&&
{
if(x_velocity==0){
repulse_force_u1= 0;
repulse_force_u2= 0;
}
if(y_velocity==0){
repulse_force_u1= 0;
repulse_force_u2= 0;
}
if(y_velocity>0){
error = (reference_angle+0.831) - angle_pitch_output;//Incline the robo
error2 = error;//The error is equal to both engines because we want to controlle them together
repulse_force = -y_velocity;//increse velocity foward
repulse_force_u1= 0;
repulse_force_u2= 0;
}
if(y_velocity<0){
error = (reference_angle-0.221) - angle_pitch_output;//Incline the robo
error2 = error;//The error is equal to both engines because we want to controlle them together
repulse_force = y_velocity;//increse velocity backwards
repulse_force_u1= 0;
repulse_force_u2= 0;
}
if(x_velocity>0 && y_velocity>0) {
repulse_force_u2= -x_velocity;
}//increse velocity backwards
if(x_velocity<0 && y_velocity>0){
repulse_force_u1= x_velocity;
}
if(x_velocity>0 && y_velocity<0) {~
repulse_force_u2= x_velocity;
}//increse velocity backwards
if(x_velocity<0 && y_velocity<0){
repulse_force_u1= -x_velocity;
}
}

//****************************************************************************************************
//*****************************************PID SPEED to calculate the force for the position *******************************
Cn_enc=Cn_enc_1+(kp_enc * (erro_enc - prev_enconder_error_1)) + (ki_enc * erro_enc) + (kd_enc * (erro_enc - (2 * prev_enconder_error_1) + prev_enconder_error_2)); //formula
PID para controlo de velocity
//********************************************************************************************************************************************
//*****************************************PID SPEED to calculate the force for the angle of inclination *********************
Cn = Cn_1+(kp * (error - prev_error_1)) + (ki * error) + (kd * (error - (2 * prev_error_1) + prev_error_2));//+repulse_force;//formula PID to controle velocity
Cn2 = Cn_12+(kp * (error2 - prev_error_12)) + (ki * error2) + (kd * (error2 - (2 * prev_error_12) + prev_error_22));//+repulse_force;//formula PID to controle velocity
//******************************************************************************************************+*************************************
//**************************Checking the maximum allowed in variables********************
//*************************If they are reached, we enforce the maximum value permited ***
if (Cn >= 254) Cn = 254;
if (Cn <= -254) Cn = -254;
if (Cn2 >= 254) Cn2 = 254;
if (Cn2 <= -254) Cn2 = -254;
if (Cn_enc > 1) Cn_enc = 1;
if (Cn_enc <= -1) Cn_enc = -1;
//**************************************************************************************
//**************************************************************************************
// ****************************Update the variables ******************************
Cn_1=Cn;
Cn_12=Cn2;
prev_error_2 = prev_error_1; //Update the values
prev_error_22 = prev_error_12; //Update the values
prev_error_1 = error; //reload the value of the previous error, which is now the current one
prev_error_12 = error2; //reload the value of the previous error, which is now the current one
Cn_enc_1=Cn_enc;
prev_enconder_error_2 = prev_enconder_error_1;
prev_enconder_error_1 = erro_enc;
//*****************************************************************************************
//******************************
U1=Cn+repulse_force+repulse_force_u1;//+Cn_enc;
U2=Cn2+repulse_force+repulse_force_u2;//+Cn_enc;
if (U1 >= 254) U1 = 254;
if (U1 <= -254) U1 = -254;
if (U2 >= 254) U2 = 254;
if (U2 <= -254) U2 = -254;
// **********************************************************************************************
// ***Checks if the 2 engines receive the same signal and treats the data accordingly**********
// **********************************************************************************************
//Serial.println("U1");
//Serial.println(U1);
//Serial.println(U2);
if ( U1 < 0 && U2 < 0)
{
digitalWrite(DIR1, HIGH); //WRITE 0 on the H bridge so the motors run in one direction
digitalWrite(DIR2, HIGH);//After checking the engine rotation we can change this variable
U1 = U1 * -1;
U2 = U2 * -1;
}
else if(U1>=0 && U2>=0 ){
digitalWrite(DIR1, LOW); //If the error is positive, the motors run towards the other side
digitalWrite(DIR2, LOW);
}
else if ( U1 < 0 && U2 > 0)
{
digitalWrite(DIR1, HIGH); //WRITE 0 on the H bridge so the motors run in one direction
digitalWrite(DIR2, LOW);// After checking the engine rotation we can change this variable
U1 = U1 * -1;
}
else if(U2 < 0 && U1>0 ){
digitalWrite(DIR1, LOW); //If the error is positive, the motors run towards the other side
digitalWrite(DIR2, HIGH);
U2 = U2 * -1;
}
// **********************************************************************************************
// ********************** Send the calculeted values to the PWM *************************************
analogWrite (PWM1, U1); //pwm1 right motor (seeing from behind1)
analogWrite (PWM2, U2);//pwm2 left motor (seeing from behind1)
// **********************************************************************************************
// *****************Check if we receive anything via Bluetooth ********************
if (Serial1.available()) {
current_value = Serial1.read();
Serial.print("Recived=");
Serial.println(current_value);
//*************If we receive an X char the next value will be the X
if(previous_byte == 88)//88 Recived X
{
x_value = current_value;
}
//*************If we receive an Y char the next value will be the Y
if(previous_byte == 89)//89 Recived Y
{
valor_y = current_value;
}
if((previous_byte != 88)||(previous_byte != 89)) previous_byte = current_value;// If not X or Y update
flag_joy=1;
joystick();
}
}

void read_mpu_6050_data() { //Subroutine for reading the raw gyro and accelerometer data
Wire.beginTransmission(0x68); //Start communicating with the MPU-6050
Wire.write(0x3B); //Send the requested starting register
Wire.endTransmission(); //End the transmission
Wire.requestFrom(0x68, 14); //Request 14 bytes from the MPU-6050
while (Wire.available() < 14); //Wait until all the bytes are received
acc_x = Wire.read() << 8 | Wire.read(); //Add the low and high byte to the acc_x variable
acc_y = Wire.read() << 8 | Wire.read(); //Add the low and high byte to the acc_y variable
acc_z = Wire.read() << 8 | Wire.read(); //Add the low and high byte to the acc_z variable
temperature = Wire.read() << 8 | Wire.read(); //Add the low and high byte to the temperature variable
gyro_x = Wire.read() << 8 | Wire.read(); //Add the low and high byte to the gyro_x variable
gyro_y = Wire.read() << 8 | Wire.read(); //Add the low and high byte to the gyro_y variable
gyro_z = Wire.read() << 8 | Wire.read(); //Add the low and high byte to the gyro_z variable
}
void setup_mpu_6050_registers() {
//Activate the MPU-6050
Wire.beginTransmission(0x68); //Start communicating with the MPU-6050
Wire.write(0x6B); //Send the requested starting register
Wire.write(0x00); //Set the requested starting register
Wire.endTransmission(); //End the transmission
//Configure the accelerometer (+/-8g)
Wire.beginTransmission(0x68); //Start communicating with the MPU-6050
Wire.write(0x1C); //Send the requested starting register
Wire.write(0x10); //Set the requested starting register
Wire.endTransmission(); //End the transmission
//Configure the gyro (500dps full scale)
Wire.beginTransmission(0x68); //Start communicating with the MPU-6050
Wire.write(0x1B); //Send the requested starting register
Wire.write(0x08); //Set the requested starting register
Wire.endTransmission(); //End the transmission
}
void joystick() {
//********************************************
//function that determines the direction the user wants for the vehicle
if(x_value==100||x_value==0)// In this values, X is 0
{
x_velocity=0;
}
if(valor_y==100||valor_y==0)// In this values, Y is 0
{
y_velocity=0;
}
//******************************************
//Different values ​​correspond to different directions
//below is the map of values ​​for directions
if(x_value>100)//Go left
x_velocity=(x_value-100) * vel_gain_app_x;//This function regulated the velocity desired by the user in addition to the direction
if(x_value>0&&x_value<100)//esquerda
x_velocity=(x_value-100)*vel_gain_app_x;
//x_velocity=-99;
if(valor_y>0&&valor_y<100)//go (front/up)
y_velocity=(valor_y-100)*(-vel_gain_app_y);
//y_velocity=99;
if(valor_y>100)//go (back/down)
y_velocity=(valor_y-100)*(-vel_gain_app_y);
//y_velocity=-99;
//Serial.println(y_velocity);
}
