
#include <stdio.h>
//#include "sc8815.h"
#include "sc8815.c"

int main(void){
  uint16_t rv=calc_ref_voltage(49,3<<6,VBUS_RATIO);
  uint16_t ib=calc_current_limit(128,IBUS_RATIO,5);
  uint32_t cc=calc_adc_voltage(49,3<<6,VBUS_RATIO);
  uint8_t v1,v2;

  split_ref_voltage(5000,v1,v2,VBUS_RATIO);

  printf("%d %d %d\n",rv,v1,v2);
  return 0;
}
