/*
 * TOF_Control.c
 *
 *  Created on: Dec 3, 2025
 *      Author: david
 */

#include "TOFs.h"

VL53L0X_Dev_t dev;

VL53L0X_Dev_t dev_list[4]; // Crée un tableau de 4 structures
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


int TOF_Init() {
    VL53L0X_Error status;
    uint8_t channels[4] = {CHANNEL_0, CHANNEL_1, CHANNEL_2, CHANNEL_3};

    printf("[TOF] --- Debut de l'initialisation sequencelle ---\r\n");

    for (int i = 0; i < 4; i++) {
    	printf("[TOF] Configuration Capteur %d (Mux Channel: 0x%02X)...\r\n", i, channels[i]);

    	    // 1. FERMER TOUS LES CANAUX d'abord pour éviter les collisions d'adresse 0x52
    	    i2c_mux_select_multi(&mux, 0);
    	    HAL_Delay(5);

    	    // 2. OUVRIR LE CANAL CIBLE
    	    i2c_mux_select_multi(&mux, channels[i]);
    	    HAL_Delay(15); //on laisse le bus se stabiliser physiquement

    	    // 3. INITIALISATION
    	    dev_list[i].I2cHandle = &hi2c3;
    	    dev_list[i].I2cDevAddr = (uint8_t)(VL53L0X_DEFAULT_ADDRESS << 1);
    	    dev_list[i].comms_type = 1;
    	    dev_list[i].comms_speed_khz = 400;

    	    printf("[TOF]  -> DataInit %d... ", i);
    	    status = VL53L0X_DataInit(&dev_list[i]);
    	    if (status != VL53L0X_ERROR_NONE) {
    	        // TENTATIVE DE RECOUVREMENT : On réessaie une fois après un délai
    	    	HAL_Delay(10);
    	        status = VL53L0X_DataInit(&dev_list[i]);
    	        if (status != VL53L0X_ERROR_NONE) {
    	            printf("ECHEC PERSISTANT (Code: %d)\r\n", status);
    	            return 0;
    	        }
    	    }
    	    printf("OK\r\n");

        // 3. StaticInit : Prepare le capteur (chargement du firmware interne)
        printf("[TOF]  -> StaticInit %d... ", i);
        status = VL53L0X_StaticInit(&dev_list[i]);
        if (status != VL53L0X_ERROR_NONE) {
            printf("ECHEC (Code: %d)\r\n", status);
            return 0;
        }
        printf("OK\r\n");

        // 4. Configuration du mode
        status = VL53L0X_SetDeviceMode(&dev_list[i], VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
        if (status != VL53L0X_ERROR_NONE) {
            printf("[TOF]  -> Erreur SetDeviceMode %d\r\n", i);
            return 0;
        }

        // 5. Demarrage de la mesure
        status = VL53L0X_StartMeasurement(&dev_list[i]);
        if (status != VL53L0X_ERROR_NONE) {
            printf("[TOF]  -> Erreur StartMeasurement %d\r\n", i);
            return 0;
        }

        printf("[TOF] Capteur %d initialise et pret.\r\n", i);
    }

    // Deselection de tous les canaux du MUX pour eviter les conflits futurs
    i2c_mux_select_multi(&mux, 0);
    printf("[TOF] --- Initialisation terminee avec succes ---\r\n");

    return 1;
}

// --- Lecture ---
int data_read_TOF(uint8_t addr, int ch_idx) {
    uint8_t channels[4] = {CHANNEL_0, CHANNEL_1, CHANNEL_2, CHANNEL_3};
    i2c_mux_select_multi(&mux, channels[ch_idx]);

    VL53L0X_GetRangingMeasurementData(&dev_list[ch_idx], &RangingData);

    int is_void = 0;

    // On ne considère que c'est du vide QUE si :
    // 1. La distance est vraiment grande (ex: > 400mm)
    // 2. OU le status est 4 (Phase Fail) ou 5 (Hardware Fail)
    // Le status 0, 1, 2 sont souvent acceptables sur une table.
    if (RangingData.RangeMilliMeter > 400 || RangingData.RangeStatus == 4) {
        is_void = 1;
    }

    i2c_mux_select_multi(&mux, 0);
    return is_void;
}


