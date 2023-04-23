#include <FlexCAN_T4.h>
#include <circular_buffer.h>
#include <imxrt_flexcan.h>
#include <isotp.h>
#include <isotp_server.h>
#include <kinetis_flexcan.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

void setup() {
  // put your setup code here, to run once:
  can1.setBaudRate(1000000);
  can2.setBaudRate(1000000);
}

void loop() {
  // put your main code here, to run repeatedly:

}
