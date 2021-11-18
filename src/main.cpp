//**************************************************************************************************************
// ***************************** SECTION INCLUDE DE LIBRAIRIES *************************************************
//**************************************************************************************************************
#include <LibRobus.h>
#include <QTRSensors.h>
#include <Adafruit_TCs34725.h>

//**************************************************************************************************************
// ***************************** SECTION DEFINE ET VARIABLES GLOBALES ******************************************
//**************************************************************************************************************
// DÉPLACEMENT DU ROBOT
#define M_GAUCHE 0
#define M_DROIT 1
#define TOUR_CM 23.939
#define PULSE_PER_TURN 3200
#define HALF_CIRCLE_IN_DEG 180
#define WHEELS_RADIUS_45 9.6
#define WHEELS_RADIUS_90 9.6
#define KP_PID 0.0025
//**************************************************************************************************************

//**************************************************************************************************************
// MAPPING DES LED
#define BLUE_LED_PIN 42
#define YELLOW_LED_PIN 48
#define GREEN_LED_PIN 40
#define RED_LED_PIN 46
//**************************************************************************************************************

//**************************************************************************************************************
// ETAPE1 - DÉTECTEUR DE SIFFLET
#define SOUND_THRESHOLD 100
#define PIN_AMBIANT_SOUND A8
#define PIN_5KHZ_SOUND A11
#define SOUND_COUNT 10
int count_5khz = 0;
int soundDiff = 0;
//**************************************************************************************************************

//**************************************************************************************************************
// SUIVEUR DE LIGNE
#define KP 0.002
#define KD 0 //0.003
#define Centre 3500
#define VITESSE_MAX 0.3
#define BLANC 100
#define NOIR 800
#define NOIR2 400
#define DelaisLent 1000
#define DelaisRapide 100
#define Delais500 750
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
double ErreurPrecedente = 0;
int Etape = 1;
//**************************************************************************************************************

//**************************************************************************************************************
// Servomoteurs
#define ANGLE_HAUT 30
#define ANGLE_BAS 75
#define SERVO_J6 0
#define SERVO_J7 1

//**************************************************************************************************************

//**************************************************************************************************************
// Detecteur quille etape 2
#define FRONT_IR 0
#define RIGHT_IR 3
#define VOLT_TO_DISTANCE_1 25603.238227
#define VOLT_TO_DISTANCE_2 1.277397873
#define MAX_DISTANCE 40
#define TURN_SPEED 0.1
#define LEFT 0
#define RIGHT 1
#define TURN_PULSE 3200 // à corriger
#define MOVE_SPEED 0.3
#define PULSE_PER_CM 327
#define angleDroit 90
#define SPEED_SMASH_QUILLE 0.5

float distance = 0;
int initialDistance = 0;
int pinIR = 0;
int pinDetected = 0;
int pinAligned = 0;
int pinSmashed = 0;
unsigned long distanceVirage = 0;
unsigned long distanceParcourueQuille = 0;
int distanceWindow = 0;
//**************************************************************************************************************

//**************************************************************************************************************
// ETAPE 7 - DETECTION COULEUR
#define COLOR_THRESHOLD 0x50
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
typedef enum t_color
{
  NONE,
  RED,
  YELLOW,
  BLUE
} e_color;
e_color colorDetected = NONE;
//**************************************************************************************************************

typedef enum t_state
{
  DETECT_5KHZ,
  DETECT_QUILLE,
  QUILLE_TO_LINE,
  DETECT_T,
  TURN_TO_COLOR,
  T_TO_COLOR,
  DETECT_COLOR,
  COLOR_TO_BALL,
  TAKE_BALL,
  BALL_TO_COLOR,
  DROP_BALL,
  COLOR_TO_LINE,
  LAST_TURN
} e_state;

//e_state state = DETECT_T;
e_state state = DETECT_5KHZ;

//**************************************************************************************************************
// ***************************** SECTION FONCTIONS *************************************************************
//**************************************************************************************************************
// Fonctions de conversion
unsigned long DistanceToPulses(int distance)
{
  unsigned long returnValue = (((float)distance * PULSE_PER_TURN) / TOUR_CM);

  return returnValue;
}

unsigned long AngleToPulses(int angle)
{
  float wheelsRadius = 0;
  if (abs(angle) < 90)
  {
    wheelsRadius = WHEELS_RADIUS_45;
  }
  else
  {
    wheelsRadius = WHEELS_RADIUS_90;
  }
  unsigned long returnValue = floor(((float)angle * PI * wheelsRadius * PULSE_PER_TURN) / (HALF_CIRCLE_IN_DEG * TOUR_CM));

  return returnValue;
}
//**************************************************************************************************************

//**************************************************************************************************************
// Fonctions de déplacements
void EncodersReset()
{
  ENCODER_Reset(M_GAUCHE);
  ENCODER_Reset(M_DROIT);
}

void MotorsOff()
{
  MOTOR_SetSpeed(M_GAUCHE, 0.0);
  MOTOR_SetSpeed(M_DROIT, 0.0);
}

float MotorsPid()
{
  long error = abs(ENCODER_Read(M_GAUCHE)) - abs(ENCODER_Read(M_DROIT));
  return (error * KP_PID);
}

void SetMotorsSpeed(float speed)
{
  MOTOR_SetSpeed(M_GAUCHE, speed);
  MOTOR_SetSpeed(M_DROIT, speed + MotorsPid());
}

void TurnLeft(int degree)
{
  unsigned long degreeInPulses = AngleToPulses(degree);
  unsigned long degreeParcourue = 0;

  EncodersReset();
  while (abs(degreeParcourue) < degreeInPulses)
  {
    MOTOR_SetSpeed(M_GAUCHE, -0.3);
    MOTOR_SetSpeed(M_DROIT, 0.3);
    degreeParcourue = ENCODER_Read(M_DROIT);
  }
  MotorsOff();
  delay(100);
  EncodersReset();
}

void TurnRight(int degree)
{
  unsigned long degreeInPulses = AngleToPulses(degree);
  unsigned long degreeParcourue = 0;

  EncodersReset();
  while (degreeParcourue < degreeInPulses)
  {
    MOTOR_SetSpeed(M_GAUCHE, 0.3);
    MOTOR_SetSpeed(M_DROIT, -0.3);
    degreeParcourue = ENCODER_Read(LEFT);
  }
  MotorsOff();
  delay(100);
  EncodersReset();
}

void GoForward(int distance)
{
  unsigned long distanceInPulses = DistanceToPulses(distance);
  unsigned long distanceParcourue = 0;

  EncodersReset();
  while (distanceParcourue < distanceInPulses)
  {

    SetMotorsSpeed(0.3);
    distanceParcourue = ENCODER_Read(LEFT);
  }
  MotorsOff();
  delay(100);
  EncodersReset();
}
//*************************************************************************************************************

//**************************************************************************************************************
// Fonctions suiveur de ligne
void CalibrationSuiveurLigne()
{
  MotorsOff();

  // Initialisation Suiveur de ligne
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7, A6, A5, A4, A3, A2, A1, A0}, SensorCount);
  qtr.setEmitterPin(2);

  delay(Delais500);

  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
}

void SuiveurLigneLoop()
{
  // Trouver la position de la ligne
  uint16_t position = qtr.readLineWhite(sensorValues);

  int erreur = Centre - position;
  int PID = KP * erreur + KD * (erreur - ErreurPrecedente);
  ErreurPrecedente = erreur;

  MOTOR_SetSpeed(M_GAUCHE, constrain(VITESSE_MAX - PID, 0, VITESSE_MAX));
  MOTOR_SetSpeed(M_DROIT, constrain(VITESSE_MAX + PID, 0, VITESSE_MAX));
}

void SuiveurLigneHoraire()
{
  uint16_t position = qtr.readLineWhite(sensorValues);
  Serial.print("POSITION : ");
  Serial.println(position);
  while (position > 4000 && position < 3000)
  {
    MOTOR_SetSpeed(M_DROIT, -0.2);
    MOTOR_SetSpeed(M_GAUCHE, 0.2);
    delay(DelaisRapide);
  }
}

void SuiveurLigneAntiHoraire()
{
  uint16_t position = qtr.readLineWhite(sensorValues);

  while (position > 4000 && position < 3000)
  {
    MOTOR_SetSpeed(M_DROIT, 0.2);
    MOTOR_SetSpeed(M_GAUCHE, -0.2);
    delay(DelaisRapide);
  }
  MotorsOff();
}

void DirectionCouleur()
{
  uint16_t position = qtr.readLineWhite(sensorValues);
  MOTOR_SetSpeed(M_DROIT, 0.2);
  MOTOR_SetSpeed(M_GAUCHE, 0.2);
  delay(500);
  MOTOR_SetSpeed(M_DROIT, 0.2);
  MOTOR_SetSpeed(M_GAUCHE, -0.2);
  delay(DelaisLent);
  SuiveurLigneAntiHoraire();
}

void DirectionDroit()
{
  //SetMotorsSpeed(0.3);
  MOTOR_SetSpeed(M_DROIT, 0.3);
  MOTOR_SetSpeed(M_GAUCHE, 0.3);
}
//**************************************************************************************************************

//**************************************************************************************************************
// Fonctions sons
int SoundDifference()
{
  int ambiantSoundValue;
  int filteredSoundValue;
  int soundDiff;

  ambiantSoundValue = analogRead(PIN_AMBIANT_SOUND);
  filteredSoundValue = analogRead(PIN_5KHZ_SOUND);
  soundDiff = filteredSoundValue - ambiantSoundValue;

  return soundDiff;
}
//*************************************salut*************************************************************************

//**************************************************************************************************************
// Fonctions quille
float getDistance(int pinIR)
{
  int rawIR = ROBUS_ReadIR(pinIR);
  float distance = VOLT_TO_DISTANCE_1 * pow(rawIR, -VOLT_TO_DISTANCE_2);
  return distance;
}

bool pinDetection(int selectIr)
{
  distance = getDistance(selectIr); //distance du capteur sur cote droit
  if (distance <= MAX_DISTANCE)
  {
    distanceWindow++;
  }
  else
  {
    distanceWindow = 0;
  }
  if (distanceWindow >= 5)
  {
    return true;
  }
  return false;
}

void TurnRightQuille()
{
  EncodersReset();
  distanceVirage = 0;

  while (distanceVirage <= AngleToPulses(angleDroit))
  {
    distanceVirage = ENCODER_Read(M_GAUCHE);
    MOTOR_SetSpeed(LEFT, TURN_SPEED); //TO DO FIX THIS JACOB
    MOTOR_SetSpeed(RIGHT, -TURN_SPEED);
  }
  MotorsOff();
}

int DistanceRetour = 0;
unsigned long DistanceParcourueQuille = 0;
#define SPEED_SMASH_QUILLE 0.5

void PinSmash()
{
  EncodersReset();
  distance = getDistance(FRONT_IR); //distance capteur en avant

  while (DistanceParcourueQuille < DistanceToPulses(distance))
  {
    SetMotorsSpeed(SPEED_SMASH_QUILLE);
    DistanceParcourueQuille = ENCODER_Read(M_GAUCHE);
    delay(100);
  }
  DistanceRetour = ENCODER_Read(M_GAUCHE);
  TurnRight(540);
  delay(1000);
  EncodersReset();
  while (ENCODER_Read(M_GAUCHE) < DistanceRetour)
  {
    SetMotorsSpeed(SPEED_SMASH_QUILLE - 0.2);
    delay(100);
  }
  MotorsOff();
}
//**************************************************************************************************************

//**************************************************************************************************************
// Fonctions Servomoteurs
void Set_angle_servos(int angle)
{
  SERVO_SetAngle(SERVO_J6, 180 - angle + 5);
  SERVO_SetAngle(SERVO_J7, angle);
}

void Enable_servos()
{
  SERVO_Enable(SERVO_J6);
  SERVO_Enable(SERVO_J7);
  Set_angle_servos(ANGLE_HAUT);
}

void Disable_servos()
{
  SERVO_Disable(SERVO_J6);
  SERVO_Disable(SERVO_J7);
}

//**************************************************************************************************************
// ***************************** SECTION SETUP *****************************************************************
//**************************************************************************************************************
void setup()
{
  BoardInit();
  Enable_servos();
  Set_angle_servos(ANGLE_BAS);
  CalibrationSuiveurLigne();

  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
}

//**************************************************************************************************************
// ***************************** SECTION LOOP ******************************************************************
//**************************************************************************************************************
void loop()
{
  uint16_t position = qtr.readLineWhite(sensorValues);
  //LOCAL VARIABLES
  uint16_t clear, red, green, blue;
  uint32_t sum;
  float r, g, b;
  Serial.println(position);

  switch (state)
  {
  case DETECT_5KHZ:

    SuiveurLigneLoop();
    soundDiff = SoundDifference();

    if (soundDiff >= SOUND_THRESHOLD)
    {
      count_5khz += 1;

      if (count_5khz > SOUND_COUNT) // Delai 10 ms, boucle 100 fois, donc son 5khz détecté durant 1000ms
      {
        MotorsOff();
        digitalWrite(YELLOW_LED_PIN, HIGH);
        digitalWrite(RED_LED_PIN, HIGH);
        digitalWrite(BLUE_LED_PIN, HIGH);
        state = DETECT_QUILLE;
      }
    }
    else
    {
      count_5khz = 0;
    }
    delay(10);
    break;

  case DETECT_QUILLE:

    //IMPLIMENTER INFRA_ROUGE AVANT

    SuiveurLigneLoop();

    if (pinDetection(RIGHT_IR) == true)
    {
      TurnRightQuille();
      digitalWrite(GREEN_LED_PIN, HIGH);
      digitalWrite(YELLOW_LED_PIN, LOW);
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(BLUE_LED_PIN, LOW);
      state = QUILLE_TO_LINE;
    }
    break;

  case QUILLE_TO_LINE:

    //DirectionDroit(); // Retour vers la ligne blanche

    MOTOR_SetSpeed(M_DROIT, 0.3);
    MOTOR_SetSpeed(M_GAUCHE, 0.3);

    if ((sensorValues[0] < NOIR2 && sensorValues[2] < NOIR2) || (sensorValues[6] < NOIR2 && sensorValues[7] < NOIR2)) // On a retouché la ligne
    {

      /*for (int i = 0; i < 8; i++)
      {
        Serial.print(i);
        Serial.print(" : ");
        Serial.println(sensorValues[i]);
      }

      Serial.println("BLANC");*/
      MotorsOff();
      delay(100);
      digitalWrite(GREEN_LED_PIN, LOW);
      TurnRight(50);
      state = DETECT_T;
    }
    break;

  case DETECT_T:

    SuiveurLigneLoop(); // suit la ligne

    if (sensorValues[0] < BLANC && sensorValues[1] < BLANC && sensorValues[2] < BLANC && sensorValues[3] < BLANC) // si rendu au ''T'' vers carton de couleur
    {
      MotorsOff();
      state = TURN_TO_COLOR;
    }
    break;

  case TURN_TO_COLOR:
    state = T_TO_COLOR;
    DirectionCouleur();
    delay(500);
 
    break;

  //Audrey : Qu'est ce qu'on fait avec le "T"?
  //François : Bah on le met dans une théière pis on l'infuse !!! 4:52am

  case T_TO_COLOR:
    SuiveurLigneLoop();
    if (sensorValues[2] < 200 && sensorValues[3] < 200 && sensorValues[4] < 200 && sensorValues[5] < 200)
    {
      GoForward(8); // ADDED THIS
      MotorsOff();
      delay(DelaisLent);
      state = DETECT_COLOR;
    }
    break;

  case DETECT_COLOR:
    delay(100);
    // Lecture des filtres de couleur
    tcs.setInterrupt(false); // turn on LED
    delay(60);               // takes 50ms to read
    tcs.getRawData(&red, &green, &blue, &clear);
    tcs.setInterrupt(true); // turn off LED
    sum = clear;
    r = red;
    r /= sum;
    g = green;
    g /= sum;
    b = blue;
    b /= sum;
    r *= 0xFF;
    g *= 0xFF;
    b *= 0xFF;
    
    if ((int)g < 0x50)
    {
      colorDetected = RED;
      digitalWrite(RED_LED_PIN, HIGH);
    }
    else if ((int)b < 0x50)
    {
      colorDetected = YELLOW;
      digitalWrite(YELLOW_LED_PIN, HIGH);
    }
    else
    {
      colorDetected = BLUE;
      digitalWrite(BLUE_LED_PIN, HIGH);
    }

    Set_angle_servos(ANGLE_HAUT);
    state = COLOR_TO_BALL;

    break;
  case COLOR_TO_BALL:

    GoForward(32);
    state = TAKE_BALL;
    break;

  case TAKE_BALL:

    Set_angle_servos(ANGLE_BAS);
    state = BALL_TO_COLOR;
    break;

  case BALL_TO_COLOR:

    if (colorDetected == BLUE)
    {

      //90 deg a gauche

      TurnLeft(90);

      //avance 45 cm
      GoForward(45);

      //90 deg a droite
      TurnRight(90);

      //avance 225 cm
      GoForward(225);

      //stop
    }
    else if (colorDetected == RED)
    {

      //avance 225 cm
      GoForward(225);

      //stop
    }

    else if (colorDetected == YELLOW)

    {
      //90 deg a droite
      TurnRight(90);

      //avance 45 cm
      GoForward(45);

      //90 deg a gauche
      TurnLeft(90);

      //avance 225 cm
      GoForward(225);

      //stop
    }
    state = DROP_BALL;
    break;

  // ajouter un arrêt pour laisser la balle en place
  case DROP_BALL:
    delay(500);
    Set_angle_servos(ANGLE_HAUT);
    state = COLOR_TO_LINE;
    break;

  case COLOR_TO_LINE:

    if (colorDetected == BLUE)
    {
      //180 deg
      TurnLeft(180);

      //avance 225 cm
      GoForward(225);

      //90 deg a gauche

      TurnLeft(90);

      //avance 45 cm
      GoForward(45);

      //90 deg a droite
      TurnRight(90);

      //avance 100 cm
      GoForward(100);

      //stop
    }
    else if (colorDetected == RED)
    {
      //180 deg
      TurnLeft(168);

      //avance 225 cm
      GoForward(225);

      //avance 100 cm
      GoForward(100);

      //stop
    }

    else if (colorDetected == YELLOW)

    {

      //180 deg
      TurnLeft(180);

      //avance 225 cm
      GoForward(225);

      //90 deg a droite
      TurnRight(90);

      //avance 45 cm
      GoForward(45);

      //90 deg a gauche
      TurnLeft(90);

      //avance 100 cm
      GoForward(100);

      //stop
    }
    state = LAST_TURN;
    break;

  case LAST_TURN:

    // retour  au cercle^^

    break;
  };
}
