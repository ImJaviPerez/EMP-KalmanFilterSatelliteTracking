// Messages definition
#ifndef ST_USER_MESSAGES_H
  #define ST_USER_MESSAGES_H

  // Error messages
  const char MSG_ERROR_400[] = "400: Invalid request or text area empty. Go to root page.";
  const char MSG_ERROR_404[] = "404: Not found. Go to root page.";
  const char MSG_ERROR_TLE_LINE_NUMBER[] = "Error. TLE line number different from 0, 1 or 2.";

  // Common messages
  const char MSG_ESP5266_WS[] = "ESP8266 Web Server";
  const char MSG_WRITE_TLE[] = "Write (copy/paste) TLE satellite list:";
  const char MSG_TLE_SUCCESS[] = "TLE satellite list successfully sent!";
  const char MSG_SEND_BUTTON[] = "Send";
#endif
