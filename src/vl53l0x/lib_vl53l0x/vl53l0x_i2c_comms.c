#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_def.h"

extern struct i2c_client *vl53l0x_client;

int VL53L0X_i2c_init(void) {
  printk(KERN_INFO "init i2c,i2c addr = %d\n",vl53l0x_client->addr);
  return VL53L0X_ERROR_NONE;
}

int VL53L0X_write_multi(uint8_t deviceAddress, uint8_t index, uint8_t *pdata, uint32_t count) {
  deviceAddress = deviceAddress;
  if(i2c_smbus_write_i2c_block_data(vl53l0x_client,index,count,pdata)){
    printk(KERN_ALERT "Smbus send bytes failed\n");
  }
  return VL53L0X_ERROR_NONE;
}

int VL53L0X_write_byte(uint8_t deviceAddress, uint8_t index, uint8_t data) {
  // return VL53L0X_write_multi(deviceAddress, index, &data, 1);
  if(i2c_smbus_write_byte_data(vl53l0x_client,index,data)){
    printk(KERN_ALERT "Smbus send byte failed\n");
  }
  return VL53L0X_ERROR_NONE;
}

int VL53L0X_write_word(uint8_t deviceAddress, uint8_t index, uint16_t data) {

  uint8_t buff[2];
  buff[1] = data & 0xFF;
  buff[0] = data >> 8;
  return VL53L0X_write_multi(deviceAddress, index, buff, 2);
}

int VL53L0X_write_dword(uint8_t deviceAddress, uint8_t index, uint32_t data) {
  uint8_t buff[4];

  buff[3] = data & 0xFF;
  buff[2] = data >> 8;
  buff[1] = data >> 16;
  buff[0] = data >> 24;

  return VL53L0X_write_multi(deviceAddress, index, buff, 4);
}

int VL53L0X_read_multi(uint8_t deviceAddress, uint8_t index, uint8_t *pdata, uint32_t count) {
 if(i2c_smbus_read_i2c_block_data(vl53l0x_client,index,count,pdata) < 0 ){
    printk(KERN_ALERT "Smbus read mutil failed\n");
    return VL53L0X_ERROR_UNDEFINED;
  }
  return VL53L0X_ERROR_NONE;
}

int VL53L0X_read_byte(uint8_t deviceAddress, uint8_t index, uint8_t *data) {
  return VL53L0X_read_multi(0, index, data, 1);
  // uint8_t read_data = 0;
  // read_data = i2c_smbus_read_byte_data(vl53l0x_client,index);
  // if(read_data < 0){
  //   printk(KERN_ALERT "Smbus read byte failed\n");
  // }
  // *data = read_data ;
  // return VL53L0X_ERROR_NONE;
}

int VL53L0X_read_word(uint8_t deviceAddress, uint8_t index, uint16_t *data) {
  uint8_t buff[2];
  int r = VL53L0X_read_multi(deviceAddress, index, buff, 2);

  uint16_t tmp;
  tmp = buff[0];
  tmp <<= 8;
  tmp |= buff[1];
  *data = tmp;

  return r;
}

int VL53L0X_read_dword(uint8_t deviceAddress, uint8_t index, uint32_t *data) {
  uint8_t buff[4];
  int r = VL53L0X_read_multi(deviceAddress, index, buff, 4);

  uint32_t tmp;
  tmp = buff[0];
  tmp <<= 8;
  tmp |= buff[1];
  tmp <<= 8;
  tmp |= buff[2];
  tmp <<= 8;
  tmp |= buff[3];

  *data = tmp;

  return r;
}
