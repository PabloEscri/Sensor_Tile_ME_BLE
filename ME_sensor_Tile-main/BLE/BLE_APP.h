/*
 * BLE_APP.h
 *
 *  Created on: 3 mar. 2021
 *      Author: pablo
 */

#ifndef BLE_APP_H_
#define BLE_APP_H_
#include "sensor_service.h"
#include "bluenrg_utils.h"
#include "bluenrg_types.h"
#include "bluenrg_gatt_server.h"
#include "bluenrg_gap.h"
#include "string.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "hci_const.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_aci_const.h"
#include "hci.h"
#include "hci_le.h"
#include "sm.h"

typedef enum
{
  CENTRAL = 0,
  PERIPHERAL = 1
} BLE_Estados_t;

void Init_BlueNRG_Stack(BLE_Estados_t estado_BLE);
void Init_BlueNRG_Custom_Services(BLE_Estados_t estado_BLE);
void  StartBeaconing(void);

#endif /* BLE_APP_H_ */
