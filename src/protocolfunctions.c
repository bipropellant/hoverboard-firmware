#include "config.h"
#include "protocol.h"
#include "protocolfunctions.h"
#include "bldc.h"
#include "softwareserial.h"
#ifdef FLASH_STORAGE
    #include "flashcontent.h"
    #include "flashaccess.h"
#endif
#include "comms.h"
#include "defines.h"

#ifdef INCLUDE_PROTOCOL

#ifdef SOFTWARE_SERIAL
    PROTOCOL_STAT sSoftwareSerial;
#endif
#if defined(SERIAL_USART2_IT) && !defined(READ_SENSOR)
    PROTOCOL_STAT sUSART2;
#endif
#if defined(SERIAL_USART3_IT) && !defined(READ_SENSOR)
    PROTOCOL_STAT sUSART3;
#endif

#ifdef FLASH_STORAGE
////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x80 to 0xA0 FlashContent

// from main.c
extern void change_PID_constants();
extern void init_PID_control();

extern volatile ELECTRICAL_PARAMS electrical_measurements;

void fn_FlashContentMagic ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {
    switch (fn_type) {
        case FN_TYPE_POST_WRITE:
            if (FlashContent.magic != CURRENT_MAGIC){
                char temp[128];
                sprintf(temp, "incorrect magic %d, should be %d\r\nFlash not written\r\n", FlashContent.magic, CURRENT_MAGIC);
                consoleLog(temp);
                FlashContent.magic = CURRENT_MAGIC;
                return;
            }
            writeFlash( (unsigned char *)&FlashContent, sizeof(FlashContent) );
            consoleLog("wrote flash\r\n");
            break;
    }
}

void fn_FlashContentPID ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {
    switch (fn_type) {
        case FN_TYPE_POST_WRITE:
            change_PID_constants();
            break;
    }
}

void fn_FlashContentMaxCurrLim ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {
    switch (fn_type) {
        case FN_TYPE_POST_WRITE:
            electrical_measurements.dcCurLim = MIN(DC_CUR_LIMIT, FlashContent.MaxCurrLim / 100);
            break;
    }
}

#endif











////////////////////////////////////////////////////////////////////////////////////////////
// initialize protocol and register functions
int setup_protocol() {

    int errors = 0;


    #ifdef SOFTWARE_SERIAL

      errors += protocol_init(&sSoftwareSerial);

      sSoftwareSerial.send_serial_data=softwareserial_Send;
      sSoftwareSerial.send_serial_data_wait=softwareserial_Send_Wait;
      sSoftwareSerial.timeout1 = 500;
      sSoftwareSerial.timeout2 = 100;
      sSoftwareSerial.allow_ascii = 1;

    #endif

    #if defined(SERIAL_USART2_IT) && !defined(READ_SENSOR)

      extern int USART2_IT_send(unsigned char *data, int len);

      errors += protocol_init(&sUSART2);

      sUSART2.send_serial_data=USART2_IT_send;
      sUSART2.send_serial_data_wait=USART2_IT_send;
      sUSART2.timeout1 = 500;
      sUSART2.timeout2 = 100;
      sUSART2.allow_ascii = 1;

    #endif

    #if defined(SERIAL_USART3_IT) && !defined(READ_SENSOR)

      extern int USART3_IT_send(unsigned char *data, int len);

      errors += protocol_init(&sUSART3);

      sUSART3.send_serial_data=USART3_IT_send;
      sUSART3.send_serial_data_wait=USART3_IT_send;
      sUSART3.timeout1 = 500;
      sUSART3.timeout2 = 100;
      sUSART3.allow_ascii = 1;

    #endif


    #ifdef FLASH_STORAGE

        errors += setParamVariable( 0x80, UI_SHORT, &FlashContent.magic,                  sizeof(short), PARAM_RW);
        setParamHandler(0x80, fn_FlashContentMagic);

        errors += setParamVariable( 0x81, UI_SHORT, &FlashContent.PositionKpx100,         sizeof(short), PARAM_RW);
        setParamHandler(0x81, fn_FlashContentPID);

        errors += setParamVariable( 0x82, UI_SHORT, &FlashContent.PositionKix100,         sizeof(short), PARAM_RW);
        setParamHandler(0x82, fn_FlashContentPID);

        errors += setParamVariable( 0x83, UI_SHORT, &FlashContent.PositionKdx100,         sizeof(short), PARAM_RW);
        setParamHandler(0x83, fn_FlashContentPID);

        errors += setParamVariable( 0x84, UI_SHORT, &FlashContent.PositionPWMLimit,       sizeof(short), PARAM_RW);
        setParamHandler(0x84, fn_FlashContentPID);

        errors += setParamVariable( 0x85, UI_SHORT, &FlashContent.SpeedKpx100,            sizeof(short), PARAM_RW);
        setParamHandler(0x85, fn_FlashContentPID);

        errors += setParamVariable( 0x86, UI_SHORT, &FlashContent.SpeedKix100,            sizeof(short), PARAM_RW);
        setParamHandler(0x86, fn_FlashContentPID);

        errors += setParamVariable( 0x87, UI_SHORT, &FlashContent.SpeedKdx100,            sizeof(short), PARAM_RW);
        setParamHandler(0x87, fn_FlashContentPID);

        errors += setParamVariable( 0x88, UI_SHORT, &FlashContent.SpeedPWMIncrementLimit, sizeof(short), PARAM_RW);
        setParamHandler(0x88, fn_FlashContentPID);

        errors += setParamVariable( 0x89, UI_SHORT, &FlashContent.MaxCurrLim,             sizeof(short), PARAM_RW);
        setParamHandler(0x89, fn_FlashContentMaxCurrLim);

        errors += setParamVariable( 0x89, UI_SHORT, &FlashContent.HoverboardEnable,       sizeof(short), PARAM_RW);


    #endif



    return errors;

}

#endif