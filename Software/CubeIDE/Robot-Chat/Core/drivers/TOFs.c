/*
 * TOF_Control.c
 *
 *  Created on: Dec 3, 2025
 *      Author: david
 */

#include "TOFs.h"

VL53L0X_Dev_t dev;
VL53L0X_DeviceInfo_t DeviceInfo;
VL53L0X_RangingMeasurementData_t RangingData;

i2c_mux_t mux ={
	.hi2c = &hi2c3,
	.rst_port = NULL,
	.rst_pin = 0,
	.addr_offset = 0
};

void configure_TOF(uint8_t addr){
    VL53L0X_Error status;

    // Attendre que le capteur ait fini de booter
    status = VL53L0X_WaitDeviceBooted(&dev);
    if (status != VL53L0X_ERROR_NONE)
    	printf("Error booting\r\n");

    status = VL53L0X_DataInit(&dev);
    if (status != VL53L0X_ERROR_NONE)
    	printf("Error DataInit\r\n");

    status = VL53L0X_StaticInit(&dev);
    if (status != VL53L0X_ERROR_NONE)
    	printf("Error StaticInit\r\n");

    status = VL53L0X_SetDeviceMode(&dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    if (status != VL53L0X_ERROR_NONE)
    	printf("Error setting Device\r\n");

    // ⚠️ Minimum ~20 ms
    status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&dev, 20000);
    if (status != VL53L0X_ERROR_NONE)
    	printf("Error Timing Budget\r\n");

    status = VL53L0X_SetInterMeasurementPeriodMilliSeconds(&dev, 20);
    if (status != VL53L0X_ERROR_NONE)
    	printf("Error Inter Measurement\r\n");
	VL53L0X_SetLimitCheckEnable(&dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckEnable(&dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckValue(&dev,
	    VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
	    (FixPoint1616_t)(0.1 * 65536)); // 0.1 Mcps
	VL53L0X_SetLimitCheckValue(&dev,
	    VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
	    (FixPoint1616_t)(60 * 65536)); // sigma 60 mm
	VL53L0X_StartMeasurement(&dev);          // Start ranging

}


int TOF_Init(){
    VL53L0X_Error status;
	dev.I2cHandle = &hi2c3;                       // ton handle I2C
	dev.I2cDevAddr = (uint8_t)(VL53L0X_DEFAULT_ADDRESS << 1); // HAL wants 8-bit addr
	dev.comms_type = 1;
	dev.comms_speed_khz = 400;

	i2c_mux_select_multi(&mux,CHANNEL_0);
	HAL_Delay(5);
	status = VL53L0X_GetDeviceInfo(&dev, &DeviceInfo);
	if (status != VL53L0X_ERROR_NONE) {
	    printf("VL53L0X_GetDeviceInfo failed: %d\r\n", status);
	    i2c_mux_select_multi(&mux, 0);

	    return 0;
	}
	configure_TOF(VL53L0X_DEFAULT_ADDRESS);

	i2c_mux_select_multi(&mux,CHANNEL_1);
	HAL_Delay(5);
		status = VL53L0X_GetDeviceInfo(&dev, &DeviceInfo);
		if (status != VL53L0X_ERROR_NONE) {
		    printf("VL53L0X_GetDeviceInfo failed: %d\r\n", status);
		    i2c_mux_select_multi(&mux, 0);
		    return 0;
	}
	configure_TOF(VL53L0X_DEFAULT_ADDRESS);

	i2c_mux_select_multi(&mux,CHANNEL_2);
	HAL_Delay(5);
		status = VL53L0X_GetDeviceInfo(&dev, &DeviceInfo);
		if (status != VL53L0X_ERROR_NONE) {
			printf("VL53L0X_GetDeviceInfo failed: %d\r\n", status);
			i2c_mux_select_multi(&mux, 0);
			return 0;
	}
	configure_TOF(VL53L0X_DEFAULT_ADDRESS);

	i2c_mux_select_multi(&mux,CHANNEL_3);
	HAL_Delay(5);
		status = VL53L0X_GetDeviceInfo(&dev, &DeviceInfo);
		if (status != VL53L0X_ERROR_NONE) {
		    printf("VL53L0X_GetDeviceInfo failed: %d\r\n", status);
		    i2c_mux_select_multi(&mux, 0);
		    return 0;
	}
	configure_TOF(VL53L0X_DEFAULT_ADDRESS);


	printf("ModelID: %s, Name: %s, Type: %s\r\n",
	       DeviceInfo.ProductId, DeviceInfo.Name, DeviceInfo.Type);



	i2c_mux_select_multi(&mux, 0);
	return 1;
}



int data_read_TOF(uint8_t addr,int ch){

	i2c_mux_select_multi(&mux, ch);
	HAL_Delay(2);

	VL53L0X_GetRangingMeasurementData(&dev, &RangingData);
	//printf("Distance = %u mm on channel %d\r\n", RangingData.RangeMilliMeter,ch);
	if (RangingData.RangeMilliMeter > 300 || RangingData.RangeStatus != 0 || RangingData.SignalRateRtnMegaCps < (0.5 * 65536)){
		printf("Void detected on channel %d\r\n",ch);
	}
	i2c_mux_select_multi(&mux, 0);
	return 1;
}
