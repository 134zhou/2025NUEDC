#include <Arduino.h>

#ifndef HMISerial
#define HMISerial Serial2
#endif

#define HMI_RESP_INVALID_CMD 0x00
#define HMI_RESP_SUCCESS 0x01
#define HMI_RESP_INVALID_CTRL_ID 0x02
#define HMI_RESP_INVALID_PAGE_ID 0x03
#define HMI_RESP_INVALID_PIC_ID 0x04
#define HMI_RESP_INVALID_ZI_ID 0x05
#define HMI_RESP_FILE_OPERATION_FAILED 0x06
#define HMI_RESP_CRC_CHECK_FALIED 0x09
#define HMI_RESP_BAUD_RATE_SET_FALIED 0x11
#define HMI_RESP_INVALID_CURVE_ID 0x12
#define HMI_RESP_INVALID_VARIABLE_NAME 0x1A
#define HMI_RESP_INVALID_VARIABLE_OPERATION 0x1B
#define HMI_RESP_ASSIGNMENT_FALIED 0x1C
#define HMI_RESP_EEPROM_FALIED 0x1D
#define HMI_RESP_INVALID_PARA_NUM 0x1E
#define HMI_RESP_INVALID_GPIO_OPERATION 0x1F
#define HMI_RESP_ESCAPE_CHAR_WRONG 0x20
#define HMI_RESP_VARIABLE_NAME_TOO_LONG 0x23

#define HMI_RESP_ADDT_DONE 0xFD
#define HMI_RESP_ADDT_READY 0xFE

#define HMI_RESP_NO_RESPONSE 0xFF

class HMI {
  private:
    uint8_t checkresp();
    struct curve_data
    {
      uint32_t w;
      uint8_t objid;
      uint8_t ch;
      uint8_t y_now;
      float y_now_f;
      bool empty;
    } curve;
    
  public:
    void init(uint32_t baud = 115200);
    uint8_t settxt(uint8_t num, String inputString);
    uint8_t setnumber(uint8_t num, uint32_t val);
    uint8_t setfloat(uint8_t num, float val, uint8_t vvs1 = 2);
    uint8_t setfloat(uint8_t num, double val, uint8_t vvs1 = 2);
    uint8_t setfloat(uint8_t num, uint32_t nval);
    void curve_init(uint32_t w, uint8_t objid = 1, uint8_t ch = 0);
    uint8_t curve_begin_transmission(uint8_t qyt);
    uint8_t curve_add_point(float y);
    uint8_t curve_add_point(uint8_t y);
    uint8_t curve_add_point(uint32_t x, float y);
    uint8_t curve_add_point(uint32_t x, uint8_t y);
    uint8_t curve_add_point(uint8_t *y, uint32_t size);
    uint8_t curve_add_point(uint8_t *y, uint32_t size, uint8_t x_interval);
    uint8_t curve_clear();
};