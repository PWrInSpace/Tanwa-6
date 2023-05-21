/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.7 */

#ifndef PB_LORA_PB_H_INCLUDED
#define PB_LORA_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _LoRaFrameTanwa {
    uint32_t tanWaState;
    uint32_t pressureSensor;
    uint32_t solenoid_fill;
    uint32_t solenoid_depr;
    bool abortButton;
    bool igniterContinouity_1;
    bool igniterContinouity_2;
    uint32_t hxRequest_RCK; /* arduino string */
    uint32_t hxRequest_TANK; /* arduino string */
    float vbat;
    uint32_t motorState_1;
    uint32_t motorState_2;
    uint32_t motorState_3;
    uint32_t motorState_4;
    float rocketWeight_temp;
    float tankWeight_temp;
    float rocketWeight_val;
    float tankWeight_val;
    uint32_t rocketWeightRaw_val;
    uint32_t tankWeightRaw_val;
    bool interface_rck;
    bool interface_tank;
    bool interface_mcu;
} LoRaFrameTanwa;

typedef struct _LoRaCommandTanwa {
    uint32_t lora_dev_id;
    uint32_t sys_dev_id;
    uint32_t command;
    int32_t payload;
} LoRaCommandTanwa;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define LoRaFrameTanwa_init_default              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define LoRaCommandTanwa_init_default            {0, 0, 0, 0}
#define LoRaFrameTanwa_init_zero                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define LoRaCommandTanwa_init_zero               {0, 0, 0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define LoRaFrameTanwa_tanWaState_tag            1
#define LoRaFrameTanwa_pressureSensor_tag        2
#define LoRaFrameTanwa_solenoid_fill_tag         3
#define LoRaFrameTanwa_solenoid_depr_tag         4
#define LoRaFrameTanwa_abortButton_tag           5
#define LoRaFrameTanwa_igniterContinouity_1_tag  6
#define LoRaFrameTanwa_igniterContinouity_2_tag  7
#define LoRaFrameTanwa_hxRequest_RCK_tag         8
#define LoRaFrameTanwa_hxRequest_TANK_tag        9
#define LoRaFrameTanwa_vbat_tag                  10
#define LoRaFrameTanwa_motorState_1_tag          11
#define LoRaFrameTanwa_motorState_2_tag          12
#define LoRaFrameTanwa_motorState_3_tag          13
#define LoRaFrameTanwa_motorState_4_tag          14
#define LoRaFrameTanwa_rocketWeight_temp_tag     15
#define LoRaFrameTanwa_tankWeight_temp_tag       16
#define LoRaFrameTanwa_rocketWeight_val_tag      17
#define LoRaFrameTanwa_tankWeight_val_tag        18
#define LoRaFrameTanwa_rocketWeightRaw_val_tag   19
#define LoRaFrameTanwa_tankWeightRaw_val_tag     20
#define LoRaFrameTanwa_interface_rck_tag         21
#define LoRaFrameTanwa_interface_tank_tag        22
#define LoRaFrameTanwa_interface_mcu_tag         23
#define LoRaCommandTanwa_lora_dev_id_tag         1
#define LoRaCommandTanwa_sys_dev_id_tag          2
#define LoRaCommandTanwa_command_tag             3
#define LoRaCommandTanwa_payload_tag             4

/* Struct field encoding specification for nanopb */
#define LoRaFrameTanwa_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, UINT32,   tanWaState,        1) \
X(a, STATIC,   REQUIRED, UINT32,   pressureSensor,    2) \
X(a, STATIC,   REQUIRED, UINT32,   solenoid_fill,     3) \
X(a, STATIC,   REQUIRED, UINT32,   solenoid_depr,     4) \
X(a, STATIC,   REQUIRED, BOOL,     abortButton,       5) \
X(a, STATIC,   REQUIRED, BOOL,     igniterContinouity_1,   6) \
X(a, STATIC,   REQUIRED, BOOL,     igniterContinouity_2,   7) \
X(a, STATIC,   REQUIRED, UINT32,   hxRequest_RCK,     8) \
X(a, STATIC,   REQUIRED, UINT32,   hxRequest_TANK,    9) \
X(a, STATIC,   REQUIRED, FLOAT,    vbat,             10) \
X(a, STATIC,   REQUIRED, UINT32,   motorState_1,     11) \
X(a, STATIC,   REQUIRED, UINT32,   motorState_2,     12) \
X(a, STATIC,   REQUIRED, UINT32,   motorState_3,     13) \
X(a, STATIC,   REQUIRED, UINT32,   motorState_4,     14) \
X(a, STATIC,   REQUIRED, FLOAT,    rocketWeight_temp,  15) \
X(a, STATIC,   REQUIRED, FLOAT,    tankWeight_temp,  16) \
X(a, STATIC,   REQUIRED, FLOAT,    rocketWeight_val,  17) \
X(a, STATIC,   REQUIRED, FLOAT,    tankWeight_val,   18) \
X(a, STATIC,   REQUIRED, UINT32,   rocketWeightRaw_val,  19) \
X(a, STATIC,   REQUIRED, UINT32,   tankWeightRaw_val,  20) \
X(a, STATIC,   REQUIRED, BOOL,     interface_rck,    21) \
X(a, STATIC,   REQUIRED, BOOL,     interface_tank,   22) \
X(a, STATIC,   REQUIRED, BOOL,     interface_mcu,    23)
#define LoRaFrameTanwa_CALLBACK NULL
#define LoRaFrameTanwa_DEFAULT NULL

#define LoRaCommandTanwa_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, UINT32,   lora_dev_id,       1) \
X(a, STATIC,   REQUIRED, UINT32,   sys_dev_id,        2) \
X(a, STATIC,   REQUIRED, UINT32,   command,           3) \
X(a, STATIC,   REQUIRED, INT32,    payload,           4)
#define LoRaCommandTanwa_CALLBACK NULL
#define LoRaCommandTanwa_DEFAULT NULL

extern const pb_msgdesc_t LoRaFrameTanwa_msg;
extern const pb_msgdesc_t LoRaCommandTanwa_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define LoRaFrameTanwa_fields &LoRaFrameTanwa_msg
#define LoRaCommandTanwa_fields &LoRaCommandTanwa_msg

/* Maximum encoded size of messages (where known) */
#define LoRaCommandTanwa_size                    29
#define LoRaFrameTanwa_size                      117

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
