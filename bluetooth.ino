void bluetooth() {
  if (Serial.available()) {  // data received from smartphone
    delay(2);
    cmd[0] =  Serial.read();
    if (cmd[0] == STX) {
      int i = 1;
      while (Serial.available()) {
        cmd[i] = Serial.read();
        if (cmd[i] > 127 || i > 7)                   break; // Communication error
        if ((cmd[i] == ETX) && (i == 2 || i == 7))   break; // Button or Joystick data
        i++;
      }
      if      (i == 2)  getButtonState(cmd[1]);    // 3 Bytes  ex: < STX "C" ETX >
      else if (i == 7)  getJoystickState(cmd);     // 6 Bytes  ex: < STX "200" "180" ETX >
    }
  }
}

String getButtonStatusString()  {
  String bStatus = "";
  for (int i = 0; i < 6; i++)  {
    if (buttonStatus & (B100000 >> i))      bStatus += "1";
    else                                    bStatus += "0";
  }
  return bStatus;
}

void getJoystickState(byte data[8])    {
  joyX = (data[1] - 48) * 100 + (data[2] - 48) * 10 + (data[3] - 48); // obtain the Int from the ASCII representation
  joyY = (data[4] - 48) * 100 + (data[5] - 48) * 10 + (data[6] - 48);
  joyX = joyX - 200;                                                  // Offset to avoid
  joyY = joyY - 200;                                                  // transmitting negative numbers

  if (joyX < -100 || joyX > 100 || joyY < -100 || joyY > 100)         // commmunication error
    return;
  if (joyY < -10 || joyY > 10)
    move_Speed = 0.015 * resp_rate * joyY;
  else 
    move_Speed = 0;  
  if (joyX < -10 || joyX > 10)
    rot_Speed = - 0.015 * joyX;
  else
    rot_Speed = 0;
}

void getButtonState(int bStatus)  {
  switch (bStatus) {
    // -----------------  BUTTON #1  -----------------------
    case 'A':
      buttonStatus |= B000001;        // ON
      //power = true;
      break;
    case 'B':
      buttonStatus &= B111110;        // OFF
      //power = false;
      break;

    // -----------------  BUTTON #2  -----------------------
    case 'C':
      buttonStatus |= B000010;        // ON
      resp_rate = 2;
      break;
    case 'D':
      buttonStatus &= B111101;        // OFF
      resp_rate = 1;
      break;

  }
}


