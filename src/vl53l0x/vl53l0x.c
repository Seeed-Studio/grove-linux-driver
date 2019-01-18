/*
 * vl53l0x.c
 * Library for ranging sensor.
 *
 * Copyright (c) 2018 seeed technology inc.
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/kdev_t.h>
#include <linux/sysfs.h>

#include "lib_vl53l0x/vl53l0x_api.h"
#include "lib_vl53l0x/vl53l0x_platform.h"
#include "lib_vl53l0x/required_version.h"


/**************************************************************************************************************/

static VL53L0X_Dev_t MyDevice = {
    .I2cDevAddr = 0x29,
    .comms_type = 1,
    .comms_speed_khz = 400,
};


static VL53L0X_Dev_t *pMyDevice = &MyDevice;
static VL53L0X_Version_t version;
static VL53L0X_Version_t* pVersion = &version;

static VL53L0X_Error check_version(void)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	int32_t status_int;

    status_int = VL53L0X_GetVersion(pVersion);
    if (status_int != 0){
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
        return Status;
    }
    
    if( pVersion->major != VERSION_REQUIRED_MAJOR ||
        pVersion->minor != VERSION_REQUIRED_MINOR ||
        pVersion->build != VERSION_REQUIRED_BUILD ){
    }
    return Status;
}

static VL53L0X_Error VL53L0X_calibration_oprt(void)
{
    uint32_t refSpadCount;
	uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    Status = VL53L0X_PerformRefSpadManagement(pMyDevice,
            &refSpadCount, &isApertureSpads); 
    if(VL53L0X_ERROR_NONE!=Status) return Status;

    Status = VL53L0X_PerformRefCalibration(pMyDevice,
            &VhvSettings, &PhaseCal); 
    if(VL53L0X_ERROR_NONE!=Status) return Status;

    return Status;
}

static VL53L0X_Error VL53L0X_set_limit_param(void)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
            VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    if(VL53L0X_ERROR_NONE!=Status) return Status;

    Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
            VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    if(VL53L0X_ERROR_NONE!=Status) return Status;        

    Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
            VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
    if(VL53L0X_ERROR_NONE!=Status) return Status;

    Status = VL53L0X_SetLimitCheckValue(pMyDevice,
            VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
            (FixPoint1616_t)(2261));
    if(VL53L0X_ERROR_NONE!=Status) return Status;

    return Status;
}


static int32_t common_init(void)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    Status=check_version();
    if(VL53L0X_ERROR_NONE!=Status) return Status;
    Status = VL53L0X_DataInit(&MyDevice); 
    if(VL53L0X_ERROR_NONE!=Status) return Status;

    Status = VL53L0X_StaticInit(pMyDevice); 
    if(VL53L0X_ERROR_NONE!=Status) return Status;

    Status=VL53L0X_calibration_oprt();
    if(VL53L0X_ERROR_NONE!=Status) return Status;
    Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
    if(VL53L0X_ERROR_NONE!=Status) return Status;
    Status = VL53L0X_set_limit_param();
	if(VL53L0X_ERROR_NONE!=Status) return Status;
    Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    if(VL53L0X_ERROR_NONE!=Status) return Status;
    return Status;
}


VL53L0X_Error VL53L0X_single_ranging_init(void)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
    if(VL53L0X_ERROR_NONE!=Status) return Status;

	Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
            VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    if(VL53L0X_ERROR_NONE!=Status) return Status;

    Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
            VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    if(VL53L0X_ERROR_NONE!=Status) return Status;        

    Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
            VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
    if(VL53L0X_ERROR_NONE!=Status) return Status;

    Status = VL53L0X_SetLimitCheckValue(pMyDevice,
            VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
            (FixPoint1616_t)2261);
    if(VL53L0X_ERROR_NONE!=Status) return Status;
    
    return Status;
}


static VL53L0X_Error PerformSingleRangingMeasurement(VL53L0X_RangingMeasurementData_t* RangingMeasurementData)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	if(Status == VL53L0X_ERROR_NONE){
        Status = VL53L0X_PerformSingleRangingMeasurement(pMyDevice,
             		RangingMeasurementData);
        return Status;
    }
    return Status;
}

static uint32_t read_distance(void)
{
    VL53L0X_RangingMeasurementData_t RangingMeasurementData;
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    memset(&RangingMeasurementData,0,sizeof(VL53L0X_RangingMeasurementData_t));
    Status = PerformSingleRangingMeasurement(&RangingMeasurementData);
    if(VL53L0X_ERROR_NONE == Status){
        return RangingMeasurementData.RangeMilliMeter;
    }
    else{
        return -1;
    }
}

/**************************************************************************************************************/

static ssize_t vl53l0x_distance_show(struct kobject* kobjs,struct kobj_attribute *attr,char *buf)
{
    uint32_t distance = 0;
    distance = read_distance();
    return sprintf(buf,"The distance = %d\n",distance);
}
static struct kobj_attribute status_attr = __ATTR_RO(vl53l0x_distance);


/**************************************************************************************************************/

static const struct i2c_device_id vl53l0x_drv_id_table[] = {
    {"of_vl53l0x",0},
    {},
};


static int major;
static struct class *vl53l0x_i2c_cls;
static struct device *vl53l0x_i2c_dev;
static const char* CLASS_NAME = "vl53l0x_cls";
static const char* DEVICE_NAME = "vl53l0x_dev";

struct i2c_client *vl53l0x_client;

static int vl53l0x_i2c_open(struct inode *node, struct file *file)
{
    return 0;
}

static ssize_t vl53l0x_i2c_read(struct file *file,char *buf, size_t len,loff_t *offset)
{
    int cnt = 0;
    uint32_t buf_len = 0;
    uint32_t distance = 0;
    uint8_t dis_buf[50] = {0};
    distance = read_distance();
    if(distance < 0){
        sprintf(dis_buf,"Read sensor failed\n");
    }
    else if(distance >= 2000){
        sprintf(dis_buf,"Object Distance out of range\n");
    }
    else{
        sprintf(dis_buf,"Object Distance = %d mm\n",distance);
    }
    buf_len = strlen(dis_buf);
    cnt = copy_to_user(buf,dis_buf,strlen(dis_buf));
    if(cnt){
        printk(KERN_INFO "copy to user failed\n");
        return -ENOMEM;
    }
    return buf_len;
}


static struct file_operations file_oprts = {
    .open = vl53l0x_i2c_open,
    .read = vl53l0x_i2c_read,
};


static int vl53l0x_drv_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    vl53l0x_client = client;

    Status = common_init();
    if(VL53L0X_ERROR_NONE != Status){
        printk(KERN_ALERT "Init sensor failed!\n");
        return -ENODEV;
    }
    Status = VL53L0X_single_ranging_init();
    if(VL53L0X_ERROR_NONE != Status){
        printk(KERN_ALERT "single ranging init failed!\n");
        return -ENODEV;
    }

    if(sysfs_create_file(&vl53l0x_client->dev.kobj, &status_attr.attr) < 0){
        printk(KERN_ALERT "Fail to create sys file\n");
        return -ENOMEM;
    }

    major = register_chrdev(0,DEVICE_NAME,&file_oprts);
    if(major < 0 ){
        sysfs_remove_file(&vl53l0x_client->dev.kobj, &status_attr.attr);
        printk(KERN_ALERT "Register failed!!\r\n");
        return major;
    }
    printk(KERN_ALERT "Registe success,major number = %d\r\n",major);

    vl53l0x_i2c_cls = class_create(THIS_MODULE,CLASS_NAME);
    if(IS_ERR(vl53l0x_i2c_cls)){
        sysfs_remove_file(&vl53l0x_client->dev.kobj, &status_attr.attr);
        unregister_chrdev(major,DEVICE_NAME);
        return PTR_ERR(vl53l0x_i2c_cls);
    }

    vl53l0x_i2c_dev = device_create(vl53l0x_i2c_cls,NULL,MKDEV(major,0),NULL,DEVICE_NAME);
    if(IS_ERR(vl53l0x_i2c_dev)){
        sysfs_remove_file(&vl53l0x_client->dev.kobj, &status_attr.attr);
        class_destroy(vl53l0x_i2c_cls);
        unregister_chrdev(major,DEVICE_NAME);
        return PTR_ERR(vl53l0x_i2c_dev);
    }
    printk(KERN_ALERT "vl53l0x_i2c device init success!!\r\n");
    

    return 0;
}

static int vl53l0x_drv_remove(struct i2c_client *client)
{
    sysfs_remove_file(&vl53l0x_client->dev.kobj, &status_attr.attr);
    device_destroy(vl53l0x_i2c_cls,MKDEV(major,0));
    class_unregister(vl53l0x_i2c_cls);
    class_destroy(vl53l0x_i2c_cls);
    unregister_chrdev(major,DEVICE_NAME);

    return 0;
}



static const struct of_device_id vl53l0x_match_table[] = {
	{.compatible = "of_vl53l0x"},
	{},
};

static struct i2c_driver vl53l0x_drv = {
    .driver = {
        .name = "of_vl53l0x",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(vl53l0x_match_table),
    },
    .probe = vl53l0x_drv_probe,
    .remove = vl53l0x_drv_remove,
    .id_table = vl53l0x_drv_id_table,
};

MODULE_DEVICE_TABLE(of,vl53l0x_match_table);


int drv_init(void)
{
    int ret = 0;
    ret  = i2c_add_driver(&vl53l0x_drv);
    if(ret){
        printk(KERN_ALERT "add driver failed!!!\n");
        return -ENODEV;
    }
    return 0;
}


void drv_exit(void)
{
    i2c_del_driver(&vl53l0x_drv);
    return ;
}


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("This module is for Seeed vl53l0x ranging sensor.");
MODULE_AUTHOR("Downey");

module_init(drv_init);
module_exit(drv_exit);
