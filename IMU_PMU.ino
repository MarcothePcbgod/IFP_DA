/*
   Author: Marco Stunder & Daniel Stojicic
   Date: 06.09.2020
   Testcode fuer die Validierung des Algorithmuses
*/
/* Algorithmus



*/
#include <Adafruit_INA260.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>




Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_INA260 ina260 = Adafruit_INA260();

//PMU
#define Min_current 0    // 0 Amper
#define Max_current 2000 // 2 Amper

//Motor
#define RPWM_pin 5
#define LPWM_pin 6

//Encoder
#define CPR_gear_shaft 979
#define CHA 2
#define CHB 3
int A_count = 0;
bool new_angle = false;
double Encoder_Angular[2] = {0, 0};
double Angular_start_E = 0;
double Angular_end_E = 0;
double Angular_E = 0;
unsigned long time_start_E = 0;
unsigned long time_end_E = 0;

//IMU
double IMU_Angular[2] = {0, 0};
double Angular_start = 0;
double Angular_end = 0;
double Angular = 0;
unsigned long time_start = 0;
unsigned long time_end = 0;

class Matrix
{
  public:
    int rows;
    int columns;
    double *data;
    Matrix()
    {
      rows = 0;
      columns = 0;
      data = nullptr;
    }
    Matrix(int init_row, int init_columns)
    {
      rows = init_row;
      columns = init_columns;
      data = (double *)malloc(sizeof(double[rows * columns]));
      for (int i = 0; i < rows * columns; i++)
      {
        *(data + i) = i;
      }
    }
    Matrix(int init_row, int init_columns, double *array)
    {
      rows = init_row;
      columns = init_columns;
      data = (double *)malloc(sizeof(double[rows * columns]));
      for (int i = 0; i < rows * columns; i++)
      {
        *(data + i) = array[i];
      }
    }

    Matrix operator*(Matrix &b)
    {
      Matrix temp(this->rows, b.columns);
      double wert = 0;
      if (this->columns == b.rows)
      {
        for (int j2 = 0; j2 < b.columns; j2++)
        {
          for (int j = 0; j < this->rows; j++)
          {
            for (int i = 0; i < this->rows; i++)
            {
              wert += *(this->data + (j * this->columns + i)) * (*(b.data + (i * b.columns + j2)));
            }
            *(temp.data + (j * b.columns + j2)) = wert;
            wert = 0;
          }
        }
      }
      else
      {
        Serial.println("ERROR n!=m");
      }
      return temp;
    }
    Matrix operator=(Matrix a)
    {
      this->columns = a.columns;
      this->rows = a.rows;
      this->data = (double *)malloc(sizeof(double[this->columns * this->rows]));
      for (int i = 0; i < this->rows * this->columns; i++)
      {
        *(this->data + i) = *(a.data + i);
      }
      return *this;
    }
    Matrix operator+(Matrix &a)
    {
      Matrix temp(this->rows, this->columns);
      if (this->columns == a.columns && this->rows == a.rows)
      {
        for (int i = 0; i < this->rows * this->columns; i++)
        {
          *(temp.data + i) = *(this->data + i) + *(a.data + i);
        }
      }
      else
      {
        Serial.println("No mxn + mxn matrix addition");
      }
      return temp;
    }
};


void print_matrix(Matrix &a)
{
  for (int i = 0; i < a.rows; i++)
  {
    for (int j = 0; j < a.columns; j++)
    {
      Serial.print(*(a.data + (i * a.columns + j)));
      Serial.print(" ");
    }
    Serial.println("");
  }
}

void A_routine()
{
  if (digitalRead(CHB) == HIGH)
  {
    A_count++;
  }
  else
  {
    A_count--;
  }
  if ((A_count) >= 240 || A_count <= -240)
  {
    A_count = 0;
  }
}

void matrix_skalar_multiplication(Matrix &a, double skalar)
{
  for (int i = 0; i < a.rows * a.columns; i++)
  {
    if (*(a.data + i) != 0)
    {
      *(a.data + i) = *(a.data + i) * skalar;
    }
  }
}

float Get_angle_motor()
{
  return float(A_count) * 360 / 240;
}

void Motor_current_stabelazation(int current, bool direction)
{
  int set_value_current = 0;
  //grobes mapen von gewünschtem strom zu ungefährem duty cycle welcher diesen strom erzeugen soll
  set_value_current = map(current, Min_current, Max_current, 0, 255);
  // je nach richtungsvorgabe (das wäre + oder - beim im ALGORITHMUS berechneten strom) wird RPWM oder LPWM benutzt
  if (direction == true)
  {
    analogWrite(RPWM_pin, set_value_current);
  }
  else
  {
    analogWrite(LPWM_pin, set_value_current);
  }
  //kurz warten und dann echten wert einlesen
  delay(10);
  int  real_value_current = 0; //Read current wobei dieser in mA zurückgegeben werden musss
  // runden von z.B. 2543 auf 25 um eine endliche abfrage zu haben weil echter strom und gewünschter strom wsl nie 100% gleich sind
  while (real_value_current / 100 != current / 100)
  {
    // je nachdem ob zu hoch oder niedrieg den duty cycle verändern
    if (real_value_current > current)
    {
      set_value_current--;
    }
    else
    {
      set_value_current++;
    }
    // neue Spannung anlegen
    if (direction == true)
    {
      analogWrite(RPWM_pin, set_value_current);
    }
    else
    {
      analogWrite(LPWM_pin, set_value_current);
    }
    // nochmal einlesen
    delay(10);
    real_value_current = 0;//Read current wobei dieser in mA zurückgegeben werden muss
  }
}

double *getAngular_Velocity_IMU()
{
  //get Angular
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  //gets X from the angular vector
  Angular_start = euler.y();
  Angular_end = Angular_start;
  //start of timer
  time_start = millis();
  //waits for Angular change
  while (Angular_end == Angular_start)
  {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Angular_end = euler.y();
    Angular = Angular_end;
  }
  IMU_Angular[0] = Angular;
  //if angular changed delta Angular and delta time is calculated
  if (Angular_start != Angular_end)
  {
    time_end = millis();
    time_end -= time_start;
    Angular_end -= Angular_start;
  }
  // Angular_Velocity is calculated
  float Angular_Velocity = Angular_end / time_end;
  IMU_Angular[1] = Angular_Velocity;
  time_start = 0;
  time_end = 0;
  /*
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print("  ");

  Serial.print("Y: ");
  Serial.print(euler.y());
  Serial.print("  ");

  Serial.print("Z: ");
  Serial.print(euler.z());
  Serial.println("  ");
  */
  return IMU_Angular;
}

double *getAngular_Velocity_Encoder()
{
  Angular_start_E = Get_angle_motor();
  Angular_end_E = Angular_start_E;
  //start of timer
  time_start_E = millis();
  //waits for Angular change
  while (Angular_end_E == Angular_start_E)
  {
    Angular_end_E = Get_angle_motor();
    Angular_E = Angular_end_E;
  }
  Encoder_Angular[0] = Angular_E;
  //if angular changed delta Angular and delta time is calculated
  if (Angular_start_E != Angular_end_E)
  {
    time_end_E = millis();
    time_end_E -= time_start_E;
    Angular_end_E -= Angular_start_E;
  }
  // Angular_Velocity is calculated
  float Angular_Velocity_E = Angular_end_E / time_end_E;
  Encoder_Angular[1] = Angular_Velocity_E;
  time_start_E = 0;
  time_end_E = 0;
  return Encoder_Angular;
}

double Drehmoment = 0;
double Sollcurrent = 0;
double K_DATA[4] = { -154.2282, -9.8961, 10, 11.3146};
Matrix K(1, 4, K_DATA);
double Km = 25.1 * 0.001;
double Data_Matrix[4] = {0,0,0,0};

void setup() {
  pinMode(CHA, INPUT);
  pinMode(CHB, INPUT);
  attachInterrupt(digitalPinToInterrupt(CHA), A_routine, RISING);
  Serial.begin(115200);
  Serial.println("IMU Test");
  if (!bno.begin())
  {
    Serial.print("Couldn't find BNO055 chip");
    while (1);
  }
  Serial.println("Found BNO055 chip");

  /*
  Serial.println("Adafruit INA260 Test");
  if (!ina260.begin())
  {
    Serial.println("Couldn't find INA260 chip");
    while (1);
  }
  Serial.println("Found INA260 chip");
  */
  delay(1000);
}

void loop() {

  getAngular_Velocity_IMU();
  getAngular_Velocity_Encoder(); 

  Data_Matrix[0] = IMU_Angular[0];
  Data_Matrix[1] = IMU_Angular[1];
  Data_Matrix[2] = Encoder_Angular[2];
  Data_Matrix[3] = Encoder_Angular[4];

  matrix_skalar_multiplication(K, Data_Matrix[4]);
  matrix_skalar_multiplication(K, -1);
  Drehmoment = K;
  Sollcurrent = Drehmoment/Km;

  Serial.print("Sollcurrent: ");
  Serial.println(Sollcurrent);

  
  /*
  // Display the data
  Serial.print("IMU_Angular: ");
  Serial.print(IMU_Angular[0]);
  Serial.print("  ");
  Serial.print(" IMU_Angular_Velocity: ");
  Serial.println(IMU_Angular[1]);

  Serial.print("Encoder_Angular: ");
  Serial.print(Encoder_Angular[0]);
  Serial.print("  ");
  Serial.print("Encoder_Velocity: ");
  Serial.println(Encoder_Angular[1]);

  Serial.print("Current: ");
  Serial.print(ina260.readCurrent());
  Serial.println(" mA");
  */
}
