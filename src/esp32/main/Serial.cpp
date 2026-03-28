#include "Serial.h"

void send_msg(String message){
  Serial.println(message);
}

String receiveNprint_msg(TFT_COLUMN column, int line_number, int textsize, uint16_t text_colour, bool clearDisplay){
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (line_number){
      String msg = "Pi: " + command;
      if (column == TFT_LEFT_CLN){
        tft.clearln(TFT_LEFT_CLN, line_number);
        tft.clearln(TFT_RIGHT_CLN, line_number);
        tft.displayLeftln(line_number, textsize, msg.c_str(), text_colour, clearDisplay);
      } else{
        tft.clearln(TFT_LEFT_CLN, line_number);
        tft.clearln(TFT_RIGHT_CLN, line_number);
        tft.displayRightln(line_number, textsize, msg.c_str(), text_colour, clearDisplay);
      }
    }
    return command;
  } else return "";
}

