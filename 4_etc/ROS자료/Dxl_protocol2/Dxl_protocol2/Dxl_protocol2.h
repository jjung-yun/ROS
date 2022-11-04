#ifndef Dxl_protocol2
#define Dxl_protocol2

#include "Arduino.h"
#include "avr/io.h"

/** H1 : Heaeer1
 *  H2 : H2
 *  H3 : H3
 *  R  : Reserved
 */
#define H1 0xFF
#define H2 0xFF
#define H3 0xFD
#define R 0x00

/** Control Table **/

/** EEPROM Area Address **/
#define ID1 1
#define ID2 2
#define ID3 3
#define Broadcast_ID 0xFE
#define Operating_Mode 11
#define Secondary_ID 12
#define Moving_threshold 24
#define PWM_lim 36
#define Current_lim 38
#define Accel_lim 40
#define Vel_lim 44
#define Max_pos_lim 48
#define Min_pos_lim 52


/** RAM Area Address (Low byte) **/
#define Torque_en 0x40
#define Status_return 0x44
#define Goal_pwm 0x64
#define Goal_vel 0x68
#define Goal_pos 0x74
#define Pres_vel 0x80
#define Pres_pos 0x84
#define Moving 0x7A

/** Communication **/
#define Rx 15
#define Tx 14

/* Instruction commend */
#define Ping 0x01
#define Read 0x02
#define Write 0x03
#define RegWrite 0x04
#define Action 0x05
#define FacReset 0x06
#define Reboot 0x08
#define Status 0x55
#define SyncRead 0x82
#define SyncWrite 0x83
#define BulkRead 0x92
#define BulkWrite 0x93

class dxl_protocol2{
  public:
  /** Functions for Setting **/
  void init_port();
  void wait();
  void factory_reset(byte id);
  void op_mode(byte id,int mode);
  void status_return_level(byte id,int mode);
  
  /** Functions to activate motors **/
  void torque_en(byte id,int input);
  void pos_move(byte id,float x);
  void sync_pos_move2(byte id1,float x1, byte id2, float x2);
  void sync_pos_move3(byte id1,float x1, byte id2, float x2, byte id3, float x3);

  void moving_status(byte id);
  void pos_read_data(byte id);
  void sync_read_data3(byte id1, byte id2, byte id3);
  void read_present_vel(byte id);
  void read_byte(int n);

  
  unsigned short update_crc(unsigned short crc_accum, unsigned short *data_blk_ptr, unsigned short data_blk_size);
  private:
  unsigned short crc_accum;
};
#endif

