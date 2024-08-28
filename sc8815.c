

#include "sc8815.h"

#if (CONFIG_BUS_CURRENT_RATIO == 1)
#define IBUS_RATIO 6
#elif (CONFIG_BUS_CURRENT_RATIO == 2)
#define IBUS_RATIO 3
#else
#error "Unkown ibus ratio."
#endif

#if (CONFIG_BAT_CURRENT_RATIO == 0)
#define IBAT_RATIO 6
#elif (CONFIG_BAT_CURRENT_RATIO == 1)
#define IBAT_RATIO 12
#else
#error "Unkown ibat ratio."
#endif

#if (CONFIG_EXTERNAL_VBUS == 1)

#define FB_SEL 1
#define VBUS_RATIO (1 + CONFIG_FB_RESISTOR_UP / CONFIG_FB_RESISTOR_DOWN)

#elif (CONFIG_EXTERNAL_VBUS == 0)

#define FB_SEL 0

#if (CONFIG_BUS_VOLTAGE_RATIO == 0)
#define VBUS_RATIO 12.5
#elif (CONFIG_BUS_VOLTAGE_RATIO == 1)
#define VBUS_RATIO 5
#else
#error "Unkown vbus ratio."
#endif

#else
#error "Unkown external vbus setting."
#endif

#if (CONFIG_BAT_VOLTAGE_RATIO == 0)
#define VBAT_RATIO 12.5
#elif (CONFIG_BAT_VOLTAGE_RATIO == 1)
#define VBAT_RATIO 5
#else
#error "Unkown vbat ratio."
#endif

#if (CONFIG_VINREG_RATIO == 0)
#define VINREG_RATIO 100
#elif (CONFIG_VINREG_RATIO == 1)
#define VINREG_RATIO 40
#else
#error "Unkown vinreg ratio."
#endif

static inline int sc8815_reg_mask_value(sc8815_chip *chip, uint8_t reg,
                                        uint8_t mask, uint8_t val) {
  int ret;
  uint8_t rv = 0;
  if ((ret = chip->ops->i2c_read(chip->slave_addr, reg, &rv, 1)) != 0) {
    return ret;
  }

  rv &= ~mask;
  rv |= val;

  if ((ret = chip->ops->i2c_write(chip->slave_addr, reg, &rv, 1)) != 0) {
    return ret;
  }
  return ret;
}

#define sc8815_clear_bits(_chip, _reg, _mask)                                  \
  sc8815_reg_mask_value(_chip, _reg, _mask, 0)

#define sc8815_set_bits(_chip, _reg, _bits)                                    \
  sc8815_reg_mask_value(_chip, _reg, 0, _bits)

int sc8815_hw_config(sc8815_chip *chip) {
  int ret = 0;
  sc8815_enable(chip, true);
  sc8815_power_switch(chip, false);

  ret |=
      sc8815_battery_setup(chip, BAT_IR_0_mR, true, CONFIG_BATTERY_CELL_COUNT,
                           CONFIG_BATTERY_VOLTAGE);

  ctrl0_st c0_mask = {.vinreg_ratio = 1};
  ctrl0_st c0 = {.vinreg_ratio = CONFIG_VINREG_RATIO};
  ret |= sc8815_reg_mask_value(chip, REG_CTRL0_SET, c0_mask.val, c0.val);

  // fb_sel set

  // clear bits
  ctrl1_st c1_mask = {.dis_ovp = 1,
                      .trickle_set = 1,
                      .fb_sel = 1,
                      .dis_term = 1,
                      .dis_trickle = 1,
                      .ichar_sel = 1};
  // set fb_sel bit
  ctrl1_st c1 = {.dis_ovp = 0,
                 .trickle_set = 0,
                 .fb_sel = FB_SEL,
                 .dis_term = 0,
                 .dis_trickle = 0,
                 .ichar_sel = 0};
  ret |= sc8815_reg_mask_value(chip, REG_CTRL1_SET, c1_mask.val, c1.val);

  ratio_st ratio_mask = {
      .vbus_ratio = 1, .vbat_mon_ratio = 1, .ibus_ratio = 3, .ibat_ratio = 1};

  ratio_st ratio = {
      .vbus_ratio = CONFIG_BUS_VOLTAGE_RATIO,
      .vbat_mon_ratio = CONFIG_BAT_VOLTAGE_RATIO,
      .ibus_ratio = CONFIG_BUS_CURRENT_RATIO,
      .ibat_ratio = CONFIG_BAT_CURRENT_RATIO,
  };

  ret |= sc8815_reg_mask_value(chip, REG_RATIO, ratio_mask.val, ratio.val);

  mask_st mask = {
      .ac_ok_mask = 1,
      .indet_mask = 1,
  };
  ret |= sc8815_set_bits(chip, REG_MASK, mask.val);

  return ret;
}

void sc8815_init(sc8815_chip *chip, const sc8815_ops *ops, uint8_t slave_addr) {
  chip->ops = ops;
  chip->slave_addr = slave_addr;
}

int sc8815_read_reg(sc8815_chip *chip, uint8_t reg, uint8_t *val) {
  return chip->ops->i2c_read(chip->slave_addr, reg, val, 1);
}

int sc8815_write_reg(sc8815_chip *chip, uint8_t reg, uint8_t val) {
  return chip->ops->i2c_write(chip->slave_addr, reg, &val, 1);
}

int sc8815_battery_setup(sc8815_chip *chip, ir_compensation ircomp,
                         bool internal, battery_cell cell,
                         battery_voltage voltage) {
  vbat_set_st bat = {
      .vcell_set = voltage,
      .csel = cell,
      .vbat_sel = internal ? 0 : 1,
      .ircomp = ircomp,
  };
  return sc8815_write_reg(chip, REG_VBAT_SET, bat.val);
}

void sc8815_enable(sc8815_chip *chip, bool enable) {
  chip->ops->chip_enable(enable ? SC8815_LOW : SC8815_HIGH);
  chip->ops->delay_ms(5);
}

void sc8815_power_switch(sc8815_chip *chip, bool on) {
  chip->ops->pstop_switch(on ? SC8815_LOW : SC8815_HIGH);
  chip->ops->delay_ms(5);
}

#define calc_ref_voltage(_v1, _v2, _ratio)                                     \
  (uint32_t)((4 * (_v1) + ((_v2) >> 6) + 1) * 2 * (_ratio))

static inline int sc8815_get_internal_bus_ref_voltage(sc8815_chip *chip,
                                                      uint16_t *vol) {
  uint8_t v1, v2;
  int ret;

  *vol = 0;
  if ((ret = chip->ops->i2c_read(chip->slave_addr, REG_VBUSREF_I_SET, &v1,
                                 1)) != 0) {
    return ret;
  }
  if ((ret = chip->ops->i2c_read(chip->slave_addr, REG_VBUSREF_I_SET2, &v2,
                                 1)) != 0) {
    return ret;
  }

  if (v1 != 0 || v2 != 0) {
    *vol = calc_ref_voltage(v1, v2, VBUS_RATIO);
  }
  return 0;
}

#define split_ref_voltage(_v, _v1, _v2, _ratio)                                \
  {                                                                            \
    uint16_t _tmp = ((_v) / (2 * (_ratio))) - 1;                               \
    (_v1) = _tmp >> 2;                                                         \
    (_v2) = (_tmp & 0x3) << 6;                                                 \
  }

static inline int sc8815_set_internal_bus_ref_voltage(sc8815_chip *chip,
                                                      uint16_t vol) {
  uint8_t v1, v2;
  int ret;

  split_ref_voltage(vol, v1, v2, VBUS_RATIO);

  if ((ret = chip->ops->i2c_write(chip->slave_addr, REG_VBUSREF_I_SET, &v1,
                                  1)) != 0) {
    return ret;
  }

  if ((ret = chip->ops->i2c_write(chip->slave_addr, REG_VBUSREF_I_SET2, &v2,
                                  1)) != 0) {
    return ret;
  }
  return 0;
}

// fb_sel=1
static inline int sc8815_get_external_bus_ref_voltage(sc8815_chip *chip,
                                                      uint16_t *vol) {

  uint8_t v1, v2;
  int ret;

  *vol = 0;
  if ((ret = chip->ops->i2c_read(chip->slave_addr, REG_VBUSREF_E_SET, &v1,
                                 1)) != 0) {
    return ret;
  }
  if ((ret = chip->ops->i2c_read(chip->slave_addr, REG_VBUSREF_E_SET2, &v2,
                                 1)) != 0) {
    return ret;
  }

  if (v1 != 0 || v2 != 0) {
    *vol = calc_ref_voltage(v1, v2, VBUS_RATIO);
  }
  return 0;
}

static inline int sc8815_set_external_bus_ref_voltage(sc8815_chip *chip,
                                                      uint16_t vol) {
  uint8_t v1, v2;
  int ret;

  split_ref_voltage(vol, v1, v2, VBUS_RATIO);

  if ((ret = chip->ops->i2c_write(chip->slave_addr, REG_VBUSREF_E_SET, &v1,
                                  1)) != 0) {
    return ret;
  }

  if ((ret = chip->ops->i2c_write(chip->slave_addr, REG_VBUSREF_E_SET2, &v2,
                                  1)) != 0) {
    return ret;
  }
  return 0;
}

int sc8815_get_bus_out_voltage(sc8815_chip *chip, uint16_t *vol) {
#if (CONFIG_EXTERNAL_VBUS == 0)
  return sc8815_get_internal_bus_ref_voltage(chip, vol);
#else
  return sc8815_get_external_bus_ref_voltage(chip, vol);
#endif
}

int sc8815_set_bus_out_voltage(sc8815_chip *chip, uint16_t vol) {
#if (CONFIG_EXTERNAL_VBUS == 0)
  return sc8815_set_internal_bus_ref_voltage(chip, vol);
#else
  return sc8815_set_external_bus_ref_voltage(chip, vol);
#endif
}

int sc8815_get_vinreg_voltage(sc8815_chip *chip, uint16_t *vol) {
  uint8_t v;
  int ret;

  *vol = 0;

  if ((ret = chip->ops->i2c_read(chip->slave_addr, REG_VINREG_SET, &v, 1)) !=
      0) {
    return ret;
  }
  *vol = (v + 1) * VINREG_RATIO;
  return 0;
}

int sc8815_set_vinreg_voltage(sc8815_chip *chip, uint16_t vol) {
  uint8_t v;
  int ret;

  v = (vol / VINREG_RATIO) - 1;

  if ((ret = chip->ops->i2c_write(chip->slave_addr, REG_VINREG_SET, &v, 1)) !=
      0) {
    return ret;
  }
  return 0;
}

#define calc_current_limit(_i1, _ratio, _sense_res)                            \
  (uint32_t)((((_i1) + 1) / 256.0f) * ((_ratio) * (10 / (_sense_res))) * 1000)

int sc8815_get_bus_current_limit(sc8815_chip *chip, uint16_t *cur) {
  uint8_t i;
  int ret;
  *cur = 0;

  if ((ret = chip->ops->i2c_read(chip->slave_addr, REG_IBUS_LIM_SET, &i, 1)) !=
      0) {
    return ret;
  }
  *cur = calc_current_limit(i, IBUS_RATIO, CONFIG_BUS_SENSE_RESISTOR);

  return 0;
}

#define convert_current_limit(_cur, _ratio, _sense_res)                        \
  (uint8_t)((_cur) / (((_ratio) * (10 / (_sense_res)) * 1000.0f) * 256) - 1)

int sc8815_set_bus_current_limit(sc8815_chip *chip, uint16_t cur) {
  uint8_t i;
  int ret;
  if (cur < 500) {
    cur = 500;
  }

  i = convert_current_limit(cur, IBUS_RATIO, CONFIG_BUS_SENSE_RESISTOR);

  if ((ret = chip->ops->i2c_write(chip->slave_addr, REG_IBUS_LIM_SET, &i, 1)) !=
      0) {
    return ret;
  }

  return 0;
}

int sc8815_get_bat_current_limit(sc8815_chip *chip, uint16_t *cur) {
  uint8_t i;
  int ret;

  *cur = 0;

  if ((ret = chip->ops->i2c_read(chip->slave_addr, REG_IBAT_LIM_SET, &i, 1)) !=
      0) {
    return ret;
  }
  *cur = calc_current_limit(i, IBAT_RATIO, CONFIG_BAT_SENSE_RESISTOR);

  return 0;
}

int sc8815_set_bat_current_limit(sc8815_chip *chip, uint16_t cur) {
  uint8_t i;
  int ret;

  if (cur < 500) {
    cur = 500;
  }

  i = convert_current_limit(cur, IBAT_RATIO, CONFIG_BAT_SENSE_RESISTOR);

  if ((ret = chip->ops->i2c_write(chip->slave_addr, REG_IBAT_LIM_SET, &i, 1)) !=
      0) {
    return ret;
  }

  return 0;
}

#define calc_adc_voltage(_v1, _v2, _ratio)                                     \
  (uint32_t)((4 * (_v1) + 1 + ((_v2) >> 6)) * (_ratio) * 2)

uint16_t sc8815_read_bus_voltage(sc8815_chip *chip) {
  uint8_t v1, v2;

  if (chip->ops->i2c_read(chip->slave_addr, REG_VBUS_FB_VALUE, &v1, 1) != 0) {
    return 0;
  }
  if (chip->ops->i2c_read(chip->slave_addr, REG_VBUS_FB_VALUE2, &v2, 1) != 0) {
    return 0;
  }
  if (v1 != 0 || v2 != 0) {
    return calc_adc_voltage(v1, v2, VBUS_RATIO);
  }
  return 0;
}

uint16_t sc8815_read_bat_voltage(sc8815_chip *chip) {
  uint8_t v1, v2;

  if (chip->ops->i2c_read(chip->slave_addr, REG_VBAT_FB_VALUE, &v1, 1) != 0) {
    return 0;
  }
  if (chip->ops->i2c_read(chip->slave_addr, REG_VBAT_FB_VALUE2, &v2, 1) != 0) {
    return 0;
  }
  if (v1 != 0 || v2 != 0) {
    return calc_adc_voltage(v1, v2, VBAT_RATIO);
  }

  return 0;
}

#define calc_adc_current(_i1, _i2, _ratio, _sense_res)                         \
  (uint32_t)((((4 * (_i1) + ((_i2) >> 6) + 1) * 2 * 1000) / 1200.0f) *         \
             ((_ratio) * (10 / (_sense_res))))

uint16_t sc8815_read_bus_current(sc8815_chip *chip) {
  uint8_t i1, i2;
  if (chip->ops->i2c_read(chip->slave_addr, REG_IBUS_VALUE, &i1, 1) != 0) {
    return 0;
  }

  if (chip->ops->i2c_read(chip->slave_addr, REG_IBUS_VALUE2, &i2, 1) != 0) {
    return 0;
  }
  if (i1 != 0 || i2 != 0) {
    return calc_adc_current(i1, i2, IBUS_RATIO, CONFIG_BUS_SENSE_RESISTOR);
  }
  return 0;
}

uint16_t sc8815_read_bat_current(sc8815_chip *chip) {
  uint8_t i1, i2;
  if (chip->ops->i2c_read(chip->slave_addr, REG_IBAT_VALUE, &i1, 1) != 0) {
    return 0;
  }

  if (chip->ops->i2c_read(chip->slave_addr, REG_IBAT_VALUE2, &i2, 1) != 0) {
    return 0;
  }
  if (i1 != 0 || i2 != 0) {
    return calc_adc_current(i1, i2, IBAT_RATIO, CONFIG_BAT_SENSE_RESISTOR);
  }

  return 0;
}

#define calc_adin_voltage(_v1, _v2)                                            \
  (uint32_t)((4 * (_v1) + ((_v2) >> 6) + 1) * 2)

uint16_t sc8815_read_adin_voltage(sc8815_chip *chip) {
  uint8_t v1, v2;

  if (chip->ops->i2c_read(chip->slave_addr, REG_ADIN_VALUE, &v1, 1) != 0) {
    return 0;
  }
  if (chip->ops->i2c_read(chip->slave_addr, REG_ADIN_VALUE2, &v2, 1) != 0) {
    return 0;
  }
  if (v1 != 0 || v2 != 0) {
    return calc_adin_voltage(v1, v2);
  }
  return 0;
}

void sc8815_otg_enable(sc8815_chip *chip, bool discharge) {
  ctrl0_st c0 = {.en_otg = 1};
  if (discharge) {
    sc8815_set_bits(chip, REG_CTRL0_SET, c0.val);
  } else {
    sc8815_clear_bits(chip, REG_CTRL0_SET, c0.val);
  }
}

void sc8815_pfm_enable(sc8815_chip *chip, bool enable) {
  ctrl3_st c3 = {.en_pfm = 1};
  if (enable) {
    sc8815_set_bits(chip, REG_CTRL3_SET, c3.val);
  } else {
    sc8815_clear_bits(chip, REG_CTRL3_SET, c3.val);
  }
}

void sc8815_adc_start(sc8815_chip *chip, bool start) {
  ctrl3_st c3 = {.ad_start = 1};
  if (start) {
    sc8815_set_bits(chip, REG_CTRL3_SET, c3.val);
  } else {
    sc8815_clear_bits(chip, REG_CTRL3_SET, c3.val);
  }
}

void sc8815_gpo_switch(sc8815_chip *chip, bool on) {
  ctrl3_st c3 = {.gpo_ctrl = 1};
  if (on) {
    sc8815_set_bits(chip, REG_CTRL3_SET, c3.val);
  } else {
    sc8815_clear_bits(chip, REG_CTRL3_SET, c3.val);
  }
}

void sc8815_pgate_switch(sc8815_chip *chip, bool on) {
  ctrl3_st c3 = {.en_pgate = 1};
  if (on) {
    sc8815_set_bits(chip, REG_CTRL3_SET, c3.val);
  } else {
    sc8815_clear_bits(chip, REG_CTRL3_SET, c3.val);
  }
}

void sc8815_ovp_enable(sc8815_chip *chip, bool enable) {
  ctrl1_st c1 = {.dis_ovp = 1};
  if (enable) {
    sc8815_clear_bits(chip, REG_CTRL3_SET, c1.val);
  } else {
    sc8815_set_bits(chip, REG_CTRL3_SET, c1.val);
  }
}
