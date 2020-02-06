// Messages definition
#ifndef ST_USER_MESSAGES_H
  #define ST_USER_MESSAGES_H
  
  // Error messages
  const char MSG_ERROR_400[] = "400: Solicitud no valida o area de texto vacia. Regresar a la pagina principal.";
  const char MSG_ERROR_404[] = "404: No encontrado. Regresar a la pagina principal.";
  const char MSG_ERROR_TLE_LINE_NUMBER[] = "Error. Numero de linea TLE diferente de 0, 1 o 2.";

  // Common messages
  const char MSG_ESP5266_WS[] = "Servidor Web ESP8266";
  const char MSG_WRITE_TLE[] = "Escribir lista de satelites TLE (copiar/pegar):";
  const char MSG_TLE_SUCCESS[] = "Envio satisfactorio de lista TLE!";
  const char MSG_SEND_BUTTON[] = "Enviar";
#endif
