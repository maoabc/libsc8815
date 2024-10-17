

#include "sc8815.h"
#include <string.h>

#define GET_IBUS_RATIO(_conf) (_conf).bus.ibus_ratio == 1 ? 6 : 3

#define GET_IBAT_RATIO(_conf) (_conf).bat.ibat_ratio ? 12 : 6

#define GET_VBAT_MON_RATIO(_conf) (_conf).bat.vbat_mon_ratio ? 5 : 12.5

#define GET_BAT_SENSE_RESISTOR(_conf) (_conf).bat.sense_resistor

#define GET_BUS_SENSE_RESISTOR(_conf) (_conf).bus.sense_resistor

#define GET_BUS_VOLTAGE_RATIO(_conf)                                           \
  ((_conf).bus.external_vbus) ? (_conf).bus.external_vbus_ratio                \
  : (_conf).bus.vbus_ratio    ? 5                                              \
                              : 12.5

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

static int sc8815_battery_setup(sc8815_chip *chip, ir_compensation_e ircomp,
                                uint8_t external, battery_cell_e cell,
                                battery_voltage_e voltage) {
  vbat_set_st bat = {
      .vcell_set = voltage,
      .csel = cell,
      .vbat_sel = external ? 1 : 0,
      .ircomp = ircomp,
  };
  return sc8815_write_reg(chip, REG_VBAT_SET, bat.val);
}

#define sc8815_clear_bits(_chip, _reg, _mask)                                  \
  sc8815_reg_mask_value(_chip, _reg, _mask, 0)

#define sc8815_set_bits(_chip, _reg, _bits)                                    \
  sc8815_reg_mask_value(_chip, _reg, 0, _bits)

int sc8815_hw_config(sc8815_chip *chip) {
  int ret = 0;
  sc8815_enable(chip, true);
  sc8815_power_switch(chip, false);

  ret |= sc8815_battery_setup(
      chip, chip->hw_config.bat.ir_comp, chip->hw_config.bat.external_vbat,
      chip->hw_config.bat.cell_number, chip->hw_config.bat.cell_voltage);

  ctrl0_st c0_mask = {
      .vinreg_ratio = 1,
      .freq = 3,
  };
  ctrl0_st c0 = {
      .vinreg_ratio = chip->hw_config.vinreg_ratio,
      .freq = 1, // 300kHz
  };
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
                 .fb_sel = chip->hw_config.bus.external_vbus,
                 .dis_term = 0,
                 .dis_trickle = 1,
                 .ichar_sel = 0};
  ret |= sc8815_reg_mask_value(chip, REG_CTRL1_SET, c1_mask.val, c1.val);

  ctrl2_st c2_mask = {.slew_set = 3, .en_dither = 1};
  ctrl2_st c2 = {.slew_set = 0, // 1mV/us
                 .en_dither = 0};
  ret |= sc8815_reg_mask_value(chip, REG_CTRL2_SET, c2_mask.val, c2.val);

  ratio_st ratio_mask = {
      .vbus_ratio = 1, .vbat_mon_ratio = 1, .ibus_ratio = 3, .ibat_ratio = 1};

  ratio_st ratio = {
      .vbus_ratio = chip->hw_config.bus.vbus_ratio,
      .vbat_mon_ratio = chip->hw_config.bat.vbat_mon_ratio,
      .ibus_ratio = chip->hw_config.bus.ibus_ratio,
      .ibat_ratio = chip->hw_config.bat.ibat_ratio,
  };

  ret |= sc8815_reg_mask_value(chip, REG_RATIO, ratio_mask.val, ratio.val);

  mask_st mask = {
      .ac_ok_mask = 1,
      .indet_mask = 1,
  };
  ret |= sc8815_set_bits(chip, REG_MASK, mask.val);

  return ret;
}

void sc8815_init(sc8815_chip *chip, const sc8815_ops *ops, uint8_t slave_addr,
                 const sc8815_config_st *conf) {
  chip->ops = ops;
  chip->slave_addr = slave_addr;
  memcpy(&chip->hw_config, conf, sizeof(sc8815_config_st));
}

int sc8815_read_reg(sc8815_chip *chip, uint8_t reg, uint8_t *val) {
  return chip->ops->i2c_read(chip->slave_addr, reg, val, 1);
}

int sc8815_write_reg(sc8815_chip *chip, uint8_t reg, uint8_t val) {
  return chip->ops->i2c_write(chip->slave_addr, reg, &val, 1);
}

void sc8815_enable(sc8815_chip *chip, bool enable) {
  chip->ops->chip_enable(enable ? SC8815_LOW : SC8815_HIGH);
  chip->ops->delay_ms(5);
}

bool sc8815_is_enable(sc8815_chip *chip) {
  sc8815_io_state state = chip->ops->get_chip_enable_state();
  return state == SC8815_LOW ? true : false;
}

void sc8815_power_switch(sc8815_chip *chip, bool on) {
  chip->ops->pstop_switch(on ? SC8815_LOW : SC8815_HIGH);
  chip->ops->delay_ms(5);
}

#define calc_ref_voltage(_v1, _v2, _ratio)                                     \
  (uint32_t)((4 * (_v1) + ((_v2) >> 6) + 1) * (2 * (_ratio)))

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
    *vol = calc_ref_voltage(v1, v2, GET_BUS_VOLTAGE_RATIO(chip->hw_config));
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

  split_ref_voltage(vol, v1, v2, GET_BUS_VOLTAGE_RATIO(chip->hw_config));

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
    *vol = calc_ref_voltage(v1, v2, GET_BUS_VOLTAGE_RATIO(chip->hw_config));
  }
  return 0;
}

static inline int sc8815_set_external_bus_ref_voltage(sc8815_chip *chip,
                                                      uint16_t vol) {
  uint8_t v1, v2;
  int ret;

  split_ref_voltage(vol, v1, v2, GET_BUS_VOLTAGE_RATIO(chip->hw_config));

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
  if (chip->hw_config.bus.external_vbus) {
    return sc8815_get_external_bus_ref_voltage(chip, vol);
  } else {
    return sc8815_get_internal_bus_ref_voltage(chip, vol);
  }
}

int sc8815_set_bus_out_voltage(sc8815_chip *chip, uint16_t vol) {
  if (chip->hw_config.bus.external_vbus) {
    return sc8815_set_external_bus_ref_voltage(chip, vol);
  } else {
    return sc8815_set_internal_bus_ref_voltage(chip, vol);
  }
}

#define convert_vinreg_ratio(_vinreg) ((_vinreg) == 0 ? 100 : 40)

int sc8815_get_vinreg_voltage(sc8815_chip *chip, uint16_t *vol) {
  uint8_t v;
  int ret;

  *vol = 0;

  if ((ret = chip->ops->i2c_read(chip->slave_addr, REG_VINREG_SET, &v, 1)) !=
      0) {
    return ret;
  }
  *vol = (v + 1) * convert_vinreg_ratio(chip->hw_config.vinreg_ratio);
  return 0;
}

int sc8815_set_vinreg_voltage(sc8815_chip *chip, uint16_t vol) {
  uint8_t v;
  int ret;

  uint8_t new_vinreg = chip->hw_config.vinreg_ratio;
  if (vol < 1200 && new_vinreg != 1) {
    new_vinreg = 1;
  } else if (new_vinreg != 0) { // Greater then 12v vinreg_ratio_dyn must be 1
    new_vinreg = 0;
  }
  if (new_vinreg != chip->hw_config.vinreg_ratio) {
    ctrl0_st c0_mask = {.vinreg_ratio = 1};
    ctrl0_st c0 = {.vinreg_ratio = new_vinreg};
    if (sc8815_reg_mask_value(chip, REG_CTRL0_SET, c0_mask.val, c0.val) == 0) {
      chip->hw_config.vinreg_ratio = new_vinreg;
    }
  }

  v = (vol / convert_vinreg_ratio(chip->hw_config.vinreg_ratio)) - 1;

  if ((ret = chip->ops->i2c_write(chip->slave_addr, REG_VINREG_SET, &v, 1)) !=
      0) {
    return ret;
  }
  return 0;
}

#define calc_current_limit(_i, _ratio, _sense_res)                             \
  (uint32_t)(((((_i) + 1) / 256.0f) * ((_ratio) * (10 / (_sense_res)))) * 1000)

int sc8815_get_bus_current_limit(sc8815_chip *chip, uint16_t *cur) {
  uint8_t i;
  int ret;
  *cur = 0;

  if ((ret = chip->ops->i2c_read(chip->slave_addr, REG_IBUS_LIM_SET, &i, 1)) !=
      0) {
    return ret;
  }
  *cur = calc_current_limit(i, GET_IBUS_RATIO(chip->hw_config),
                            GET_BUS_SENSE_RESISTOR(chip->hw_config));

  return 0;
}

#define convert_current_limit(_cur, _ratio, _sense_res)                        \
  (uint8_t)(((((_cur) / 1000.0f) / ((_ratio) * (10 / (_sense_res)))) * 256) - 1)

int sc8815_set_bus_current_limit(sc8815_chip *chip, uint16_t cur) {
  uint8_t i;
  int ret;
  if (cur < 300) {
    cur = 300;
  }

  i = convert_current_limit(cur, GET_IBUS_RATIO(chip->hw_config),
                            GET_BUS_SENSE_RESISTOR(chip->hw_config));

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
  *cur = calc_current_limit(i, GET_IBAT_RATIO(chip->hw_config),
                            GET_BAT_SENSE_RESISTOR(chip->hw_config));

  return 0;
}

int sc8815_set_bat_current_limit(sc8815_chip *chip, uint16_t cur) {
  uint8_t i;
  int ret;

  if (cur < 300) {
    cur = 300;
  }

  i = convert_current_limit(cur, GET_IBAT_RATIO(chip->hw_config),
                            GET_BAT_SENSE_RESISTOR(chip->hw_config));

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
    return calc_adc_voltage(v1, v2, GET_BUS_VOLTAGE_RATIO(chip->hw_config));
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
    return calc_adc_voltage(v1, v2, GET_VBAT_MON_RATIO(chip->hw_config));
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
    return calc_adc_current(i1, i2, GET_IBUS_RATIO(chip->hw_config),
                            GET_BUS_SENSE_RESISTOR(chip->hw_config));
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
    return calc_adc_current(i1, i2, GET_IBAT_RATIO(chip->hw_config),
                            GET_BAT_SENSE_RESISTOR(chip->hw_config));
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
  chip->ops->delay_ms(1);
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

bool sc8815_is_active(sc8815_chip *chip) {
  sc8815_io_state ce = chip->ops->get_chip_enable_state();
  sc8815_io_state pstop = chip->ops->get_pstop_state();
  return ce == SC8815_LOW && pstop == SC8815_LOW ? true : false;
}

bool sc8815_is_otg_enable(sc8815_chip *chip) {
  ctrl0_st c0 = {.val = 0};
  sc8815_read_reg(chip, REG_CTRL0_SET, &c0.val);
  return c0.en_otg == 1 ? true : false;
}

void sc8815_set_factory(sc8815_chip *chip) {
  ctrl2_st c2 = {.factory = 1};
  sc8815_set_bits(chip, REG_CTRL2_SET, c2.val);
}

uint8_t sc8815_get_status(sc8815_chip *chip) {
  uint8_t status = 0;
  sc8815_read_reg(chip, REG_STATUS, &status);
  return status;
}
