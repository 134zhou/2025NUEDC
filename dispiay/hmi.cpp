#include "hmi.h"

void HMI::init(uint32_t baud) {
  HMISerial.begin(baud);
}

uint8_t HMI::checkresp() {
  uint8_t resp;
  uint16_t count = 0;
  while (!HMISerial.available()) {
    delay(1);
    count++;
    if(count > 1000) return HMI_RESP_NO_RESPONSE;
  }
  while (HMISerial.available())
  {
    char inchar = HMISerial.read();
    if(inchar != 0xFF) resp = inchar;
  }
  return resp;
}

uint8_t HMI::settxt(uint8_t num, String inputString) {
  char str[12];
  sprintf(str, "t%d.txt=\"", num);
  HMISerial.print(str);
  HMISerial.print(inputString + "\"\xff\xff\xff");
  return checkresp();
}

uint8_t HMI::setnumber(uint8_t num, uint32_t val)
{
  char str[16];
  sprintf(str, "n%d.val=%d\xff\xff\xff", num, val);
  HMISerial.print(str);
  return checkresp();
}

uint8_t HMI::setfloat(uint8_t num, float val, uint8_t vvs1)
{
  return setfloat(num, (double)val, vvs1);
}

uint8_t HMI::setfloat(uint8_t num, double val, uint8_t vvs1)
{
  uint32_t nval = val * pow(10, vvs1);
  return setfloat(num, nval);
}

uint8_t HMI::setfloat(uint8_t num, uint32_t nval)
{
  char str[16];
  sprintf(str, "x%d.val=%d\xff\xff\xff", num, nval);
  HMISerial.print(str);
  return checkresp();
}

void HMI::curve_init(uint32_t w, uint8_t objid, uint8_t ch)
{
  curve.w = w;
  curve.objid = objid;
  curve.ch = ch;
  curve.y_now = 0;
  curve.y_now_f = 0.0F;
  curve.empty = true;
}

uint8_t HMI::curve_begin_transmission(uint8_t qyt)
{
  char str[20];
  sprintf(str, "addt %d,%d,%d\xff\xff\xff", curve.objid, curve.ch, qyt);
  HMISerial.print(str);
  return checkresp();
}

uint8_t HMI::curve_add_point(float y)
{
  if(y < 0) y = 0;
  if(y > 1) y = 1;
  uint8_t ny = round(y * 250.0F);
  char str[20];
  sprintf(str, "add %d,%d,%d\xff\xff\xff", curve.objid, curve.ch, ny);
  HMISerial.print(str);
  curve.y_now = ny;
  curve.y_now_f = y;
  return checkresp();
}

uint8_t HMI::curve_add_point(uint8_t y)
{
  char str[20];
  sprintf(str, "add %d,%d,%d\xff\xff\xff", curve.objid, curve.ch, y);
  HMISerial.print(str);
  curve.y_now = y;
  curve.y_now_f = (float)y / 250.0F;
  return checkresp();
}

uint8_t HMI::curve_add_point(uint32_t x, float y)
{
  if(y < 0) y = 0;
  if(y > 1) y = 1;

  if(curve.empty) {
    curve.empty = false;
    return curve_add_point(y);
  } else {
    float y_delta = y - curve.y_now_f;
    float y_slope = y_delta / (float)x;
    if(curve_begin_transmission(x) == HMI_RESP_ADDT_READY) {
      for(uint8_t i = 0; i < x; i++) {
        uint8_t ny = round((y_slope * (i + 1) + curve.y_now_f) * 250.0F);
        HMISerial.write(ny); //写入第i目标点的y整数坐标
      }
    }
    curve.y_now = round(y * 250.0F);
    curve.y_now_f = y;
    return checkresp();
  }
}

uint8_t HMI::curve_add_point(uint32_t x, uint8_t y)
{
  if(curve.empty) {
    curve.empty = false;
    return curve_add_point(y);
  } else {
    float y_delta = ((float)y - (float)curve.y_now) / 250.0F;
    float y_slope = y_delta / (float)x;
    if(curve_begin_transmission(x) == HMI_RESP_ADDT_READY) {
      for(uint8_t i = 0; i < x; i++) {
        uint8_t ny = round((y_slope * (i + 1) + (float)curve.y_now / 250.0F) * 250.0F);
        HMISerial.write(ny); //写入第i目标点的y整数坐标
      }
    }
    curve.y_now = y;
    curve.y_now_f = (float)y / 250.0F;
    return checkresp();
  }
}

uint8_t HMI::curve_add_point(uint8_t *y, uint32_t size)
{
  if(size > 255) size = 255;
  if(curve_begin_transmission(size) == HMI_RESP_ADDT_READY) {
    HMISerial.write(y, size);
  }
  curve.y_now = y[size - 1];
  curve.y_now_f = (float)y[size - 1] / 250.0F;
  return checkresp();
}

uint8_t HMI::curve_clear()
{
  char str[16];
  sprintf(str, "cle %d,%d\xff\xff\xff", curve.objid, curve.ch);
  HMISerial.print(str);
  curve.empty = true;
  curve.y_now = 0;
  curve.y_now_f = 0.0F;
  return checkresp();
}
