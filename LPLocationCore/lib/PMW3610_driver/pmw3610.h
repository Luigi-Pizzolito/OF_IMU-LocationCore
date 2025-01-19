#pragma once

// Hard defines for PMW3610
// Mostly taken from ZMK's PMW3610 driver
// as datasheet does not provide much information

/* Timing specifications */
// Taken from ZMK's PMW3610 driver and ADNS-5050 datasheet
#define PMW3610_RESET_HOLD_TIME_US   1      // Reset hold time (T_RSTH)
#define PMW3610_WAKEUP_TIME_MS       10     // Post-reset wait time (T_WKUP)
#define PMW3610_NCS_SETUP_TIME_US    1      // 3-wire SPI chip select setup time (T_NCSS)
#define PMW3610_HANDOVER_TIME_US     4      // 3-wire SPI handover time (T_SRAD)
#define PMW3610_HOLD_TIME_US         1      // 3-wire SPI hold time after handoff (T_HOLD)
#define PMW3610_INTERREAD_TIME_US    1      // 3-wire SPI hold time between consecutive read commands (T_SRX)
#define PMW3610_INTERWRITE_TIME_US   30     // 3-wire SPI hold time between consecutive write/read commands (T_SWX)

#define PMW3610_INIT_SELF_TEST_MS    50     // Time to wait after self-test initialization

/* Sensor registers (addresses) */
#define PMW3610_REG_PRODUCT_ID       0x00
#define PMW3610_REG_REVISION_ID      0x01
#define PMW3610_REG_MOTION           0x02
#define PMW3610_REG_DELTA_X_L        0x03
#define PMW3610_REG_DELTA_Y_L        0x04
#define PMW3610_REG_DELTA_XY_H       0x05
#define PMW3610_REG_SQUAL            0x06
#define PMW3610_REG_SHUTTER_HIGHER   0x07
#define PMW3610_REG_SHUTTER_LOWER    0x08
#define PMW3610_REG_PIX_MAX          0x09
#define PMW3610_REG_PIX_AVG          0x0A
#define PMW3610_REG_PIX_MIN          0x0B

#define PMW3610_REG_CRC0             0x0C
#define PMW3610_REG_CRC1             0x0D
#define PMW3610_REG_CRC2             0x0E
#define PMW3610_REG_CRC3             0x0F
#define PMW3610_REG_SELF_TEST        0x10

#define PMW3610_REG_PERFORMANCE      0x11
#define PMW3610_REG_MOTION_BURST     0x12

#define PMW3610_REG_RUN_DOWNSHIFT    0x1B
#define PMW3610_REG_REST1_PERIOD     0x1C
#define PMW3610_REG_REST1_DOWNSHIFT  0x1D
#define PMW3610_REG_REST2_PERIOD     0x1E
#define PMW3610_REG_REST2_DOWNSHIFT  0x1F
#define PMW3610_REG_REST3_PERIOD     0x20
#define PMW3610_REG_OBSERVATION      0x2D

#define PMW3610_REG_PIXEL_GRAB       0x35
#define PMW3610_REG_FRAME_GRAB       0x36

#define PMW3610_REG_POWER_UP_RESET   0x3A
#define PMW3610_REG_SHUTDOWN         0x3B

#define PMW3610_REG_SPI_CLK_ON_REQ   0x41
#define PMW3610_REG_RES_STEP         0x85

#define PMW3610_REG_NOT_REV_ID       0x3E
#define PMW3610_REG_NOT_PROD_ID      0x3F

#define PMW3610_REG_PRBS_TEST_CTL    0x47
#define PMW3610_REG_SPI_PAGE0        0x7F
#define PMW3610_REG_VCSEL_CTL        0x9E
#define PMW3610_REG_LSR_CONTROL      0x9F
#define PMW3610_REG_SPI_PAGE1        0xFF

/* Sensor identification values */
#define PMW3610_PRODUCT_ID           0x3E
#define PMW3610_REVISION_ID          0x01
#define PMW3610_POWERUP_CMD_RESET    0x5A