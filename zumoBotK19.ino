#include <Arduino.h>
#include "zumoBotLib.h"

#define SPEED 350
#define SPEED_TURN 300
#define NB_SENSOR 6

#define SENSOR_NO_LINE -1
#define SENSOR_CROSS_T -2
#define SENSOR_CROSS_RIGHT -3
#define SENSOR_CROSS_LEFT -4

//obj
ZumoBot *zumo = 0;

//variables
int tabsensorvalues[NB_SENSOR] = {0, 0, 0, 0, 0, 0};
int toutbanc = 0;

//
void testMotors()
{
  zumo->setMotorSpeeds(SPEED, SPEED, 2000);

  zumo->setMotorSpeeds(SPEED, -SPEED, 500);

  zumo->setMotorSpeeds(-SPEED, SPEED, 500);

  zumo->setMotorSpeeds(-SPEED, -SPEED, 2000);
}
//
void testLed()
{
  zumo->setLed(1);
  delay(1000);
  zumo->setLed(0);
  delay(500);
}
//
void testSensor()
{
  zumo->sensorRead();

  zumo->sensorPrintToSerial();

  delay(1000);
}
//
void calculBlanc()
{
  int val = 0;
  zumo->sensorRead();

  val = tabsensorvalues[0];
  val += tabsensorvalues[5];

  //moyenne
  toutbanc = val >> 1;
  //
  Serial.println("**********");
  Serial.print("blanc= ");
  Serial.println(toutbanc);
}
//
void suivrePisteNoire(int capteur)
{
  int kp = 10;
  int kd = 5;

  static int err_n = 0;
  static int err_n_1 = 0;
  static int speedLeft = 0;
  static int speedRight = 0;
  //
  err_n = 20 - (10 * capteur);

  speedLeft = SPEED - (kp * err_n);
  speedLeft -= kd * (err_n - err_n_1);

  speedRight = SPEED + (kp * err_n);
  speedRight += kd * (err_n - err_n_1);

  //saturation
  if (speedLeft > 400)
    speedLeft = 400;

  if (speedRight > 400)
    speedRight = 400;

  if (speedLeft < 0)
    speedLeft = 0;

  if (speedRight < 0)
    speedRight = 0;

  //mem
  err_n_1 = err_n;

  zumo->setMotorSpeeds(speedLeft, speedRight, 0);
}
//
int traitementCapteur()
{
  int res = 2;
  int tmp = 0;
  int valeurnumerique = 0;
  int seuil = toutbanc >> 1;

  zumo->sensorRead();
  //zumo->sensorPrintToSerial();
  //
  for (int i = 0; i < NB_SENSOR; i++)
  {
    tmp = tabsensorvalues[i];
    tabsensorvalues[i] = tmp - toutbanc;
  }
  //
  for (int i = 0; i < NB_SENSOR; i++)
  {
    if (tabsensorvalues[i] < seuil)
    {
      tabsensorvalues[i] = 0;
    }
    else
    {
      tabsensorvalues[i] = 1;
    }
  }
  //
  //zumo->sensorPrintToSerial();
  //
  valeurnumerique = tabsensorvalues[NB_SENSOR - 1];
  for (int i = NB_SENSOR - 2; i >= 0; i--)
  {
    valeurnumerique = valeurnumerique << 1;
    valeurnumerique += tabsensorvalues[i];
  }
  //case NO_LINE
  if (tabsensorvalues[0] == 0 && tabsensorvalues[3] == 0 && tabsensorvalues[5] == 0 && valeurnumerique < 1)
  {
    res = SENSOR_NO_LINE;
    // Serial.println("NO_LINE");
    return res;
  }
  //case CROSS_T
  if (tabsensorvalues[0] == 1 && tabsensorvalues[1] == 1 && tabsensorvalues[2] == 1
      && tabsensorvalues[3] == 1 && tabsensorvalues[4] == 1 && tabsensorvalues[5] == 1)
  {
    res = SENSOR_CROSS_T;
    //Serial.println("CROSS_T");
    return res;
  }
  //case CROSS_RIGHT
  if (tabsensorvalues[0] == 0 && tabsensorvalues[1] == 0 && tabsensorvalues[3] == 1 && tabsensorvalues[4] == 1 && tabsensorvalues[5] == 1)
  {
    res = SENSOR_CROSS_RIGHT;
    // Serial.println("CROSS_RIGHT");
    return res;
  }
  //case CROSS_LEFT
  if (tabsensorvalues[4] == 0 && tabsensorvalues[5] == 0 && tabsensorvalues[0] == 1 && tabsensorvalues[1] == 1 && tabsensorvalues[2] == 1)
  {
    res = SENSOR_CROSS_LEFT;
    // Serial.println("CROSS_LEFT");
    return res;
  }
  //case normal
  for (int i = 0; i < NB_SENSOR; i++)
  {
    if (tabsensorvalues[i] == 1)
    {
      res = i;
      //Serial.println(res);
      return res;
    }
  }

  return res;
}
//
void action()
{
  int capteur = traitementCapteur();

  if (capteur >= 0)
  {
    suivrePisteNoire(capteur);
  }
  else
  {
    zumo->setMotorStop(0);
    //
    switch (capteur)
    {
      //CROSS_LEFT
      case SENSOR_CROSS_LEFT:
        zumo->setMotorSpeeds(SPEED_TURN, SPEED_TURN, 100);
        capteur = -1;
        while (capteur < 0)
        {
          zumo->setMotorSpeeds(-SPEED_TURN, SPEED_TURN, 100);
          capteur = traitementCapteur();
        }
        zumo->setMotorStop(0);
        break;

      //CROSS_RIGHT
      case SENSOR_CROSS_RIGHT:
        zumo->setMotorSpeeds(SPEED_TURN, SPEED_TURN, 100);
        capteur = -1;
        while (capteur < 0)
        {
          zumo->setMotorSpeeds(SPEED_TURN, -SPEED_TURN, 100);
          capteur = traitementCapteur();
        }
        zumo->setMotorStop(0);
        break;

      //NO_LINE
      case SENSOR_NO_LINE:
        capteur = -1;
        while (capteur < 0)
        {
          zumo->setMotorSpeeds(-SPEED_TURN, SPEED_TURN, 100);
          capteur = traitementCapteur();
        }
        zumo->setMotorStop(0);
        break;

      //
      default:
        zumo->setMotorSpeeds(SPEED_TURN, SPEED_TURN, 100);
        zumo->setMotorStop(0);
        break;
    }
  }
}
//
void setup()
{
  //Serial.begin(9600);
  //
  zumo = ZumoBot::getInstance();
  //
  zumo->begin(tabsensorvalues);
  //
  zumo->buzzer(330, 300);
  //

  zumo->waitButtonPressAndReleased();
  //
  calculBlanc();
  //
  zumo->buzzer(1000, 300);
  //
  zumo->setMotorSpeeds(SPEED, SPEED, 0);

}

//
void loop()
{
  action();
}
