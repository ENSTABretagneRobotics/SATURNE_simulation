byte FORWARD_REVERSE_PIN = 2;
byte LEFT_RIGHT_PIN = 3;
byte SPEED_PIN = 4;
byte SWITCH_PIN = 5;

int FORWARD_REVERSE_PWM;
int LEFT_RIGHT_PWM;
int FORWARD_REVERSE_PWM_CAL = 0;
int LEFT_RIGHT_PWM_CAL = 0;
int SPEED_PWM;
int SWITCH_PWM;

int FORWARD_REVERSE_VALUE;
int LEFT_RIGHT_VALUE;
int SPEED_VALUE;
int SWITCH_VALUE;

int MIN_PWM = 1063;
int MAX_PWM = 1957;
int DIFF_PWM = MAX_PWM - MIN_PWM;
int NEUTRAL_PWM = (MAX_PWM+MIN_PWM)/2;

 
void setup() {
  pinMode(SWITCH_PIN, INPUT);
  pinMode(SPEED_PIN, INPUT);
  pinMode(LEFT_RIGHT_PIN, INPUT);
  pinMode(FORWARD_REVERSE_PIN, INPUT);
  pinMode(53,OUTPUT);
  Serial.begin(57600);
}

String checksum(String data)
{
    int checksum_value = 0;

    for (long i = 1; i < data.length(); i++) 
    {
        checksum_value ^= data[i];
    }

    String str = String(checksum_value,HEX);
    str.toUpperCase();
    if(str.length() == 1)
    {
      str = "0"+str;
    }
    return str;
}

 
void loop() {
  FORWARD_REVERSE_PWM = pulseIn(FORWARD_REVERSE_PIN, HIGH);
  delay(5);
  LEFT_RIGHT_PWM = pulseIn(LEFT_RIGHT_PIN, HIGH);
  delay(5);
  SPEED_PWM = pulseIn(SPEED_PIN, HIGH);
  delay(5);
  SWITCH_PWM = pulseIn(SWITCH_PIN, HIGH);
  delay(5);

  FORWARD_REVERSE_VALUE = 100*(1. - 2.*min(1.,max(0.,((float)FORWARD_REVERSE_PWM - (float)MIN_PWM - (float) FORWARD_REVERSE_PWM_CAL) / (float)DIFF_PWM)));
  LEFT_RIGHT_VALUE = 100*(-1. + 2.*min(1.,max(0.,((float)LEFT_RIGHT_PWM - (float)MIN_PWM - (float)LEFT_RIGHT_PWM_CAL) / (float)DIFF_PWM)));
  SPEED_VALUE = 100*(min(1.,max(0.,((float)SPEED_PWM - (float)MIN_PWM) / (float)DIFF_PWM)));
  if(SWITCH_PWM > 1000 and SWITCH_PWM < 1100)
  {
    SWITCH_VALUE = 1;
  }
  else
  {
    SWITCH_VALUE = 0;
  }

  if(SWITCH_VALUE == 1 and SPEED_VALUE < 4 and FORWARD_REVERSE_PWM > 1200 and LEFT_RIGHT_PWM > 1200 and FORWARD_REVERSE_PWM < 2000 and  LEFT_RIGHT_PWM > 2000)// BUG ?
  {
    FORWARD_REVERSE_PWM_CAL = FORWARD_REVERSE_PWM - NEUTRAL_PWM;
    LEFT_RIGHT_PWM_CAL = LEFT_RIGHT_PWM - NEUTRAL_PWM;
  }

  if(abs(FORWARD_REVERSE_VALUE) < 4)
  {
    FORWARD_REVERSE_VALUE = 0;
  }

  if(abs(LEFT_RIGHT_VALUE) < 4)
  {
    LEFT_RIGHT_VALUE = 0;
  }
  /*
  Serial.print("FORWARD_REVERSE : ");
  Serial.println(FORWARD_REVERSE_VALUE);
  Serial.print("LEFT_RIGHT : ");
  Serial.println(LEFT_RIGHT_VALUE);
  Serial.print("SPEED : ");
  Serial.println(SPEED_VALUE);
  Serial.print("SWTICH : ");
  Serial.println(SWITCH_VALUE);
  */
  String str = "$PARDN,CMD_RC,"+String(SWITCH_VALUE)+","+String(FORWARD_REVERSE_VALUE)+","+String(LEFT_RIGHT_VALUE)+","+String(SPEED_VALUE);
  String checksum_str = checksum(str);
  str = str + "*" + checksum_str; 
  Serial.println(str);
  
}
