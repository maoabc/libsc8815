
#ifndef __SC8815_H_
#define __SC8815_H_

#include <stdbool.h>
#include <stdint.h>

#define SC8815_DEFAULT_ADDR (0x74 << 1)

#ifndef CONFIG_BAT_SENSE_RESISTOR
#define CONFIG_BAT_SENSE_RESISTOR 5 /*mR*/
#endif

#ifndef CONFIG_BUS_SENSE_RESISTOR
#define CONFIG_BUS_SENSE_RESISTOR 5 /*mR*/
#endif

#ifndef CONFIG_EXTERNAL_VBAT
#define CONFIG_EXTERNAL_VBAT 0
#endif

#ifndef CONFIG_BATTERY_CELL_COUNT
#define CONFIG_BATTERY_CELL_COUNT 3 /*4s*/
#endif

#ifndef CONFIG_BATTERY_VOLTAGE
#define CONFIG_BATTERY_VOLTAGE 1 /*4.2V*/
#endif

#ifndef CONFIG_BUS_VOLTAGE_RATIO
#define CONFIG_BUS_VOLTAGE_RATIO 0 /*12.5x*/
#endif

#ifndef CONFIG_BAT_VOLTAGE_RATIO
#define CONFIG_BAT_VOLTAGE_RATIO 0 /*12.5x*/
#endif

#ifndef CONFIG_BUS_CURRENT_RATIO
#define CONFIG_BUS_CURRENT_RATIO 2 /*3x*/
#endif

#ifndef CONFIG_BAT_CURRENT_RATIO
#define CONFIG_BAT_CURRENT_RATIO 0 /*6x*/
#endif

#ifndef CONFIG_VINREG_RATIO
#define CONFIG_VINREG_RATIO 0 /*100x*/
#endif

#ifndef CONFIG_EXTERNAL_VBUS
#define CONFIG_EXTERNAL_VBUS 0
#endif


#if(CONFIG_EXTERNAL_VBUS==1)

#ifndef CONFIG_FB_RESISTOR_UP
#define CONFIG_FB_RESISTOR_UP 1000 /*R*/
#endif

#ifndef CONFIG_FB_RESISTOR_DOWN
#define CONFIG_FB_RESISTOR_DOWN 1000 /*R*/
#endif

#endif


typedef enum {
    SC8815_LOW,
    SC8815_HIGH,
} sc8815_io_state;

/*SC8815 CE PSTOP I2C operations*/
typedef struct sc8815_ops {
    int (*i2c_read)(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

    int (*i2c_write)(uint8_t addr, uint8_t reg, const uint8_t *buf, uint16_t len);

    void (*pstop_switch)(sc8815_io_state state);

    void (*chip_enable)(sc8815_io_state state);

    void (*delay_ms)(uint32_t ms);
} sc8815_ops;

typedef struct sc8815_chip {
    uint8_t slave_addr;
    const struct sc8815_ops *ops;

} sc8815_chip;

typedef enum {
    REG_VBAT_SET = 0x00,
    REG_VBUSREF_I_SET = 0x01,
    REG_VBUSREF_I_SET2 = 0x02,
    REG_VBUSREF_E_SET = 0x03,
    REG_VBUSREF_E_SET2 = 0x04,
    REG_IBUS_LIM_SET = 0x05,
    REG_IBAT_LIM_SET = 0x06,
    REG_VINREG_SET = 0x07,
    REG_RATIO = 0x08,
    REG_CTRL0_SET = 0x09,
    REG_CTRL1_SET = 0x0a,
    REG_CTRL2_SET = 0x0b,
    REG_CTRL3_SET = 0x0c,
    REG_VBUS_FB_VALUE = 0x0d,
    REG_VBUS_FB_VALUE2 = 0x0e,
    REG_VBAT_FB_VALUE = 0x0f,
    REG_VBAT_FB_VALUE2 = 0x10,
    REG_IBUS_VALUE = 0x11,
    REG_IBUS_VALUE2 = 0x12,
    REG_IBAT_VALUE = 0x13,
    REG_IBAT_VALUE2 = 0x14,
    REG_ADIN_VALUE = 0x15,
    REG_ADIN_VALUE2 = 0x16,
    REG_STATUS = 0x17,
    REG_MASK = 0x19,

} sc8815_regs;

typedef enum {
    BAT_IR_0_mR = 0,
    BAT_IR_20_mR = 1,
    BAT_IR_40_mR = 2,
    BAT_IR_80_mR = 3,
} ir_compensation;

typedef enum {
    CELL_1S = 0,
    CELL_2S = 1,
    CELL_3S = 2,
    CELL_4S = 3
} battery_cell;

typedef enum {
    VOLTAGE_4_10 = 0,
    VOLTAGE_4_20 = 1,
    VOLTAGE_4_25 = 2,
    VOLTAGE_4_30 = 3,
    VOLTAGE_4_35 = 4,
    VOLTAGE_4_40 = 4,
    VOLTAGE_4_45 = 6,
    VOLTAGE_4_50 = 7,
} battery_voltage;

typedef union {
    uint8_t val;
    struct {
        uint8_t vcell_set: 3;
        uint8_t csel: 2;
        uint8_t vbat_sel: 1;
        uint8_t ircomp: 2;
    };
} vbat_set_st;

typedef union {
    uint8_t val;
    struct {
        uint8_t vbus_ratio: 1;
        uint8_t vbat_mon_ratio: 1;
        uint8_t ibus_ratio: 2;
        uint8_t ibat_ratio: 1;
        uint8_t reserved0: 1;
        uint8_t reserved1: 2;
    };
} ratio_st;

typedef union {
    uint8_t val;
    struct {
        uint8_t dead_time: 2;
        uint8_t freq: 2;
        uint8_t vinreg_ratio: 1;
        uint8_t reserved0: 1;
        uint8_t reserved1: 1;
        uint8_t en_otg: 1;
    };
} ctrl0_st;

typedef union {
    uint8_t val;
    struct {
        uint8_t reserved0: 1;
        uint8_t reserved1: 1;
        uint8_t dis_ovp: 1;
        uint8_t trickle_set: 1;
        uint8_t fb_sel: 1;
        uint8_t dis_term: 1;
        uint8_t dis_trickle: 1;
        uint8_t ichar_sel: 1;
    };
} ctrl1_st;

typedef union {
    uint8_t val;
    struct {
        uint8_t slew_set: 2;
        uint8_t en_dither: 1;
        uint8_t factory: 1;
        uint8_t reserved0: 4;
    };
} ctrl2_st;

typedef union {
    uint8_t val;
    struct {
        uint8_t en_pfm: 1;
        uint8_t eoc_set: 1;
        uint8_t dis_short_fold_back: 1;
        uint8_t loop_set: 1;
        uint8_t ilim_bw_sel: 1;
        uint8_t ad_start: 1;
        uint8_t gpo_ctrl: 1;
        uint8_t en_pgate: 1;
    };
} ctrl3_st;


typedef union {
    uint8_t val;
    struct {
        uint8_t reserved0: 1;
        uint8_t eoc: 1;
        uint8_t otp: 1;
        uint8_t vbus_short: 1;
        uint8_t reserved1: 1;
        uint8_t indet: 1;
        uint8_t ac_ok: 1;
        uint8_t reserved2: 1;
    };
} status_st;

typedef union {
    uint8_t val;
    struct {
        uint8_t reserved0: 1;
        uint8_t eoc_mask: 1;
        uint8_t otp_mask: 1;
        uint8_t vbus_short_mask: 1;
        uint8_t reserved1: 1;
        uint8_t indet_mask: 1;
        uint8_t ac_ok_mask: 1;
        uint8_t reserved2: 1;
    };
} mask_st;

/**
 *
 * @param chip
 * @param ops
 * @param slave_addr
 */
void sc8815_init(sc8815_chip *chip,const sc8815_ops *ops, uint8_t slave_addr);

int sc8815_hw_config(sc8815_chip *chip);

int sc8815_read_reg(sc8815_chip *chip, uint8_t reg, uint8_t *val);

int sc8815_write_reg(sc8815_chip *chip, uint8_t reg, uint8_t val);

int sc8815_battery_setup(sc8815_chip *chip, ir_compensation ircomp,
                         bool external, battery_cell cell,
                         battery_voltage voltage);


int sc8815_get_bus_out_voltage(sc8815_chip *chip, uint16_t *vol);

int sc8815_set_bus_out_voltage(sc8815_chip *chip, uint16_t vol);


int sc8815_get_vinreg_voltage(sc8815_chip *chip, uint16_t *vol);

int sc8815_set_vinreg_voltage(sc8815_chip *chip, uint16_t vol);


int sc8815_get_bat_current_limit(sc8815_chip *chip, uint16_t *cur);

int sc8815_set_bat_current_limit(sc8815_chip *chip, uint16_t cur);

int sc8815_get_bus_current_limit(sc8815_chip *chip, uint16_t *cur);

/**
 *
 * @param chip
 * @param cur  mA
 * @return
 */
int sc8815_set_bus_current_limit(sc8815_chip *chip, uint16_t cur);

uint16_t sc8815_read_bat_voltage(sc8815_chip *chip);

uint16_t sc8815_read_bat_current(sc8815_chip *chip);

uint16_t sc8815_read_bus_voltage(sc8815_chip *chip);

uint16_t sc8815_read_bus_current(sc8815_chip *chip);

uint16_t sc8815_read_adin_voltage(sc8815_chip *chip);


void sc8815_enable(sc8815_chip *chip, bool enable);

void sc8815_power_switch(sc8815_chip *chip, bool on);

void sc8815_otg_enable(sc8815_chip *chip, bool discharge);

void sc8815_pfm_enable(sc8815_chip *chip,bool enable);

void sc8815_adc_start(sc8815_chip *chip, bool start);

void sc8815_gpo_switch(sc8815_chip *chip, bool on);

void sc8815_pgate_switch(sc8815_chip *chip, bool on);

void sc8815_ovp_enable(sc8815_chip *chip, bool enable);

#endif
