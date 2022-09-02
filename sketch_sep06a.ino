//Line Following Robot using PD Controller (Digital)


//Initialization
int initial_motor_speed = 200;        //can change the initial speed that you prefer
int maxSpeed = 0;                 //if the motor is too fast, decrease the max speed and vice versa
int lsense = 0, csense = 0, rsense = 0;
int clsense = 0, crsense = 0;
float Kp = 0;       //will be change using potentiometer
float Kd = 0;       //will be change using potettiometer
float Ki = 0;
float P = 0, D = 0, PD_value = 0, I = 0;
float error = 0, previous_error = 0;
float m1error = 0;
float m2error = 0;

//Pin declaration for SHIELD-2AMOTOR
int RightEn = 10;
int LeftEn = 9;

void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void);

void setup()
{
  Serial.begin(9600);       //Enable Serial Communications
  //Motor Driver Pin Setup
  pinMode(RightEn, OUTPUT);
  pinMode(LeftEn, OUTPUT);
  pinMode(3, INPUT); //leftsensor
  pinMode(4, INPUT); //centre sensor
  pinMode(5, INPUT); //right sensor
  pinMode(6, INPUT); //centre n left mid sensor
  pinMode(7, INPUT); //centre n right mid sensor


}

void loop()
{
  read_sensor_values();
  calculate_pid();
  motor_control();
}

void read_sensor_values()
{
  lsense = digitalRead(3);
  csense = digitalRead(4);
  rsense = digitalRead(5);
  clsense = digitalRead(6);
  crsense = digitalRead(7);
  Serial.print(lsense);
  Serial.print("\t");
  Serial.print(clsense);
  Serial.print("\t");
  Serial.print(csense);
  Serial.print("\t");
  Serial.print(crsense);
  Serial.print("\t");
  Serial.print(rsense);
  Serial.print("\t");




  if ((lsense == 1) && (clsense == 1) && (csense == 0) && (crsense == 1) && (rsense == 1)) //11011   0
    error = 0;
  else if ((lsense == 0) && (clsense == 0) && (csense == 0) && (crsense == 0) && (rsense == 0))//00000   0
    error = 0;
  else if ((lsense == 1) && (clsense == 1) && (csense == 0) && (crsense == 0) && (rsense == 1))//11001   1
    error = 1;
  else if ((lsense == 1) && (clsense == 1) && (csense == 1) && (crsense == 0) && (rsense == 1))//11101   2
    error = 2;
  else if ((lsense == 1) && (clsense == 1) && (csense == 1) && (crsense == 0) && (rsense == 0))//11100   4
    error = 4;
  else if ((lsense == 1) && (clsense == 1) && (csense == 1) && (crsense == 1) && (rsense == 0))//11110   6
    error = 6;
  else if ((lsense == 1) && (clsense == 1) && (csense == 0) && (crsense == 0) && (rsense == 0))//11000   6
    error = 6;
  else if ((lsense == 1) && (clsense == 0) && (csense == 0) && (crsense == 1) && (rsense == 1)) //10011    -1
    error = -1;
  else if ((lsense == 1) && (clsense == 0) && (csense == 1) && (crsense == 1) && (rsense == 1)) //10111    -2
    error = -2;
  else if ((lsense == 0) && (clsense == 0) && (csense == 1) && (crsense == 1) && (rsense == 1))//00111   -4
    error = -4;
  else if ((lsense == 0) && (clsense == 1) && (csense == 1) && (crsense == 1) && (rsense == 1))//01111   -6
    error = -6;
  else if ((lsense == 0) && (clsense == 0) && (csense == 0) && (crsense == 1) && (rsense == 1))//00011   -6
    error = -6;
  //  else if ((lsense == 1) && (clsense == 0) && (csense == 0) && (crsense == 1) && (rsense == 1))//10011
  //  {
  //    if ((lsense == 1) && (clsense == 1) && (csense == 0) && (crsense == 1) && (rsense == 1))
  //    { if ((lsense == 1) && (clsense == 1) && (csense == 1) && (crsense == 1) && (rsense == 1))
  //        error = -6;
  //    }
  //  }
  //  else if ((lsense == 1) && (clsense == 1) && (csense == 0) && (crsense == 0) && (rsense == 1))//11001
  //  {
  //    if ((lsense == 1) && (clsense == 1) && (csense == 0) && (crsense == 1) && (rsense == 1))
  //    { if ((lsense == 1) && (clsense == 1) && (csense == 1) && (crsense == 1) && (rsense == 1))
  //        error = 6;
  //    }
  //  }
  else if ((lsense == 1) && (clsense == 1) && (csense == 1) && (crsense == 1) && (rsense == 1))
  {
    if (error == 0) {
      if (m2error == -1 && m1error == 0)
        error = -6;
      else if (m2error == 1 && m1error == 0 )
        error = 6;
      else
        error = 0;
    }
    else if (error == 2) error = 2;
    else if (error == -2) error = -2;
    else if (error == 4) error = 4;
    else if (error == -4) error = -4;
    else if (error == -6) error = -6;
    else if (error == 6) error = 6;
    else if (error == 5) error = 5;
    else error = -5;
  }


  Serial.print(error);
  Serial.print("\t");

  if (error != m1error)
  {
    m2error = m1error;
    m1error = error;
  }
  Serial.print(m1error);
  Serial.print("\t");
  Serial.print(m2error);
  Serial.print("\t");
}

void calculate_pid()
{
  P = error;
  D = error - previous_error;
  //I = I + error;

  Kp = analogRead(A3);
  Kp = Kp / 10;
  Serial.print(Kp);
  Serial.print("\t");

  Kd = analogRead(A4);
  Kd = Kd / 20;
  Serial.print(Kd);
  Serial.print("\t");

  // Ki = analogRead(A2);
  // Ki = Ki / 100;
  //Serial.print(Ki);
  //Serial.print("\t");


  PD_value = (Kp * P) + (Kd * D) + (Ki * I);
  Serial.print(PD_value);
  Serial.print("\t");
  previous_error = error;
}

void motor_control()
{
  // Calculating the effective motor speed:
  int left_motor_speed = initial_motor_speed + PD_value;
  int right_motor_speed = initial_motor_speed - PD_value;
  int tempSpeed = analogRead(A5);
  maxSpeed = map(tempSpeed, 0, 1023, 0, 255);
  maxSpeed = constrain(maxSpeed, 0, 255);
  Serial.print(maxSpeed);
  Serial.print("\t");

  // The motor speed should not exceed the max PWM value
  left_motor_speed = constrain(left_motor_speed, 0, maxSpeed);
  right_motor_speed = constrain(right_motor_speed, 0, maxSpeed);

  // right_motor_speed = right_motor_speed + 5 ; //this line is needed if your motor didn't have same speed

  //open the Serial Monitor to see the speed of each motor
  //Serial.print ("right = ");
  Serial.print(right_motor_speed);
  Serial.print("\t");
  //Serial.print("left = ");
  Serial.println(left_motor_speed);
  Serial.print("\t");

  analogWrite(LeftEn, left_motor_speed);  //Left Motor Speed
  analogWrite(RightEn, right_motor_speed); //Right Motor Speed
}
