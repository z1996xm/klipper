# TMC5262 configuration
#
# Copyright (C) 2018-2023  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2023  Alex Voinea <voinea.dragos.alexandru@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
from . import tmc, tmc2130

# Internal 16 MHz oscillator; CLK must be tied to GND or VCC_IO.
TMC_FREQUENCY = 16000000.
PLL_CONFIG = 0x000001ec  # divider=15, internal clock, clk_sys_sel/clk_fsm_ena
PLL_START = PLL_CONFIG | 0x1001
PLL_CLEAR_FLAGS = PLL_CONFIG | 0x6000
PLL_CONFIG_MASK = 0x000003ee
PLL_ERROR_MASK = 0x00007000
PLL_TIMEOUT = 1.0
PLL_RETRIES = 3

Registers = {
    "GCONF":                    0x00,
    "GSTAT":                    0x01,
    "DO_CONF":                  0x02,
    "DO_SCOPE_CONF":            0x03,
    "IOIN":                     0x04,
    "X_COMPARE":                0x05,
    "X_COMPARE_REPEAT":         0x06,
    "DRV_CONF":                 0x0A,
    "PLL":                      0x0B,
    "IHOLD_IRUN":               0x10,
    "TPOWERDOWN":               0x11,
    "TSTEP":                    0x12,
    "TPWMTHRS":                 0x13,
    "TCOOLTHRS":                0x14,
    "THIGH":                    0x15,
    "TSGP_LOW_VEL_THRS":        0x16,
    "T_RCOIL_MEAS":             0x17,
    "TUDCSTEP":                 0x18,
    "UDC_CONF":                 0x19,
    "STEPS_LOST":               0x1A,
    "XTARGET":                  0x2D,
    "SW_MODE":                  0x34,
    "RAMP_STAT":                0x35,
    "XLATCH":                   0x36,
    "ENCMODE":                  0x38,
    "X_ENC":                    0x39,
    "ENC_CONST":                0x3A,
    "ENC_STATUS":               0x3B,
    "ENC_LATCH":                0x3C,
    "ENC_DEVIATION":            0x3D,
    "VIRTUAL_STOP_L":           0x3E,
    "VIRTUAL_STOP_R":           0x3F,
    "CURRENT_PI_REG":           0x40,

    "ANGLE_PI_REG":             0x41,
    "CUR_ANGLE_LIMIT":          0x42,
    "ANGLE_LOWER_LIMIT":        0x43,
    "CUR_ANGLE_MEAS":           0x44,
    "PI_RESULTS":               0x45,
    "COIL_INDUCT":              0x46,
    "R_COIL":                   0x47,

    "R_COIL_USER":              0x48,
    "SGP_CONF":                 0x49,
    "SGP_IND_2_3":              0x4A,
    "SGP_IND_0_1":              0x4B,

    "INDUCTANCE_VOLTAGE":       0x4C,
    "SGP_BEMF":                 0x4D,
    "COOLSTEPPLUS_CONF":        0x4E,
    "COOLSTEPPLUS_PI_REG":      0x4F,
    "COOLSTEPPLUS_PI_DOWN":     0x50,

    "COOLSTEPPLUS_RESERVE_CONF":0x51,
    "COOLSTEPPLUS_LOAD_RESERVE":0x52,

    "TSTEP_VELOCITY":           0x53,
    "ADC_VSUPPLY_TEMP":         0x58,
    "ADC_I":                    0x59,
    "OTW_OV_VTH":               0x5A,
    "MSLUT0":                   0x60,
    "MSLUT1":                   0x61,
    "MSLUT2":                   0x62,
    "MSLUT3":                   0x63,
    "MSLUT4":                   0x64,
    "MSLUT5":                   0x65,
    "MSLUT6":                   0x66,
    "MSLUT7":                   0x67,

    "MSLUTSEL":                 0x68,
    "MSLUTSTART":               0x69,
    "MSCNT":                    0x6A,
    "MSCURACT":                 0x6B,

    "CHOPCONF":                 0x6C,
    "COOLCONF":                 0x6D,
    "DRV_STATUS":               0x6F,
    "PWMCONF":                  0x70,
}

ReadRegisters = [
    "GCONF", "GSTAT", "DO_CONF", "DO_SCOPE_CONF", "IOIN", "DRV_CONF", "PLL",
    "IHOLD_IRUN", "TPOWERDOWN", "TSTEP", "TPWMTHRS", "TCOOLTHRS", "THIGH",
    "TSGP_LOW_VEL_THRS", "T_RCOIL_MEAS", "TUDCSTEP", "UDC_CONF", "STEPS_LOST",
    "SW_MODE", "CURRENT_PI_REG", "ANGLE_PI_REG", "CUR_ANGLE_LIMIT",
    "ANGLE_LOWER_LIMIT", "CUR_ANGLE_MEAS", "PI_RESULTS", "COIL_INDUCT",
    "R_COIL", "R_COIL_USER", "SGP_CONF", "COOLSTEPPLUS_CONF",
    "COOLSTEPPLUS_PI_REG", "COOLSTEPPLUS_PI_DOWN", "COOLSTEPPLUS_RESERVE_CONF",
    "COOLSTEPPLUS_LOAD_RESERVE", "TSTEP_VELOCITY", "ADC_VSUPPLY_TEMP", "ADC_I",
    "OTW_OV_VTH", "MSLUTSEL", "MSLUTSTART", "MSCNT", "MSCURACT", "CHOPCONF",
    "COOLCONF", "DRV_STATUS", "PWMCONF",
]

TMC5262RegisterDefaults = [
    ("GCONF", 0x80000016),
    ("DO_CONF", 0xa0000000),
    ("DO_SCOPE_CONF", 0x00000000),
    ("DRV_CONF", 0x0000003e),
    ("PLL", 0x000001ec),
    ("IHOLD_IRUN", 0x0407a08f),
    ("TPOWERDOWN", 0x0000000a),
    ("TPWMTHRS", 0x00000000),
    ("TCOOLTHRS", 0x00001388),
    ("THIGH", 0x00000000),
    ("TSGP_LOW_VEL_THRS", 0x00001000),
    ("T_RCOIL_MEAS", 0x00000eb3),
    ("TUDCSTEP", 0x00000000),
    ("UDC_CONF", 0x000000ce),
    ("XTARGET", 0x00000000),
    ("SW_MODE", 0x00008000),
    ("ENCMODE", 0x00000000),
    ("X_ENC", 0x00000000),
    ("ENC_CONST", 0x00010000),
    ("CURRENT_PI_REG", 0x00110043),
    ("ANGLE_PI_REG", 0x00140032),
    ("CUR_ANGLE_LIMIT", 0x0fff0100),
    ("ANGLE_LOWER_LIMIT", 0x00000100),
    ("COIL_INDUCT", 0x00000320),
    ("R_COIL_USER", 0x00000000),
    ("SGP_CONF", 0x00004000),
    ("COOLSTEPPLUS_CONF", 0x00000012),
    ("COOLSTEPPLUS_PI_REG", 0x00100080),
    ("COOLSTEPPLUS_PI_DOWN", 0x0040064b),
    ("COOLSTEPPLUS_RESERVE_CONF", 0x64dc3296),
    ("OTW_OV_VTH", 0x01ff01ff),
    ("MSLUT0", 0xaaaab554),
    ("MSLUT1", 0x4a9554aa),
    ("MSLUT2", 0x24492929),
    ("MSLUT3", 0x10104222),
    ("MSLUT4", 0xfbffffff),
    ("MSLUT5", 0xb5bb777d),
    ("MSLUT6", 0x49295556),
    ("MSLUT7", 0x00404222),
    ("MSLUTSEL", 0xffff8056),
    ("MSLUTSTART", 0x00f70000),
    ("CHOPCONF", 0x13410153),
    ("COOLCONF", 0x00000000),
    ("PWMCONF", 0x000fe000),
]

Fields = {}

Fields["GCONF"] = {
    "fast_standstill":                      0x01 << 0,
    "en_pwm_mode":                          0x01 << 1,      #replace en_stealthchop
    "multistep_filt":                       0x01 << 2,
    "shaft":                                0x01 << 3,
    "small_hysteresis":                     0x01 << 4,
    "stop_enable":                          0x01 << 5,
    "direct_mode":                          0x01 << 6,
    "length_steppulse":                     0x0f << 8,
    "ov_nn":                                0x01 << 12,
    "step_dir":                             0x01 << 31
}

Fields["GSTAT"] = {
    "reset":                                0x01 << 0,
    "drv_err":                              0x01 << 1,
    "uv_cp":                                0x01 << 2,
    "register_reset":                       0x01 << 3,
    "vm_uvlo":                              0x01 << 4,
    "vccio_uv":                             0x01 << 5,
}

Fields["DO_CONF"] = {
    "diag0_error":                          0x01 << 0,
    "diag0_otpw":                           0x01 << 1,
    "diag0_stall":                          0x01 << 2,
    "diag0_index":                          0x01 << 3,
    "diag0_step":                           0x01 << 4,
    "diag0_dir":                            0x01 << 5,

    "diag0_xcomp":                          0x01 << 6,
    "diag0_ov":                             0x01 << 7,
    "diag0_dcustep":                        0x01 << 8,
    "diag0_ev_stop_ref":                    0x01 << 9,
    "diag0_ev_stop_sg":                     0x01 << 10,
    "diag0_ev_pos_reached":                 0x01 << 11,
    "diag0_ev_n_deviation":                 0x01 << 12,

    "diag1_error":                          0x01 << 13,
    "diag1_otpw":                           0x01 << 14,
    "diag1_stall":                          0x01 << 15,
    "diag1_index":                          0x01 << 16,
    "diag1_step":                           0x01 << 17,
    "diag1_dir":                            0x01 << 18,

    "diag1_xcomp":                          0x01 << 19,
    "diag1_ov":                             0x01 << 20,
    "diag1_dcustep":                        0x01 << 21,
    "diag1_ev_stop_ref":                    0x01 << 22,
    "diag1_ev_stop_sg":                     0x01 << 23,
    "diag1_ev_pos_reached":                 0x01 << 24,
    "diag1_ev_n_deviation":                 0x01 << 25,

    "diag0_nod_pp":                         0x01 << 28,
    "diag0_invpp":                          0x01 << 29,
    "diag1_nod_pp":                         0x01 << 30,
    "diag1_invpp":                          0x01 << 31,
}

Fields["IOIN"] = {
    "refl":                                 0x01 << 0,
    "refr":                                 0x01 << 1,
    "encb":                                 0x01 << 2,
    "enca":                                 0x01 << 3,
    "drv_enn":                              0x01 << 4,
    "encn":                                 0x01 << 5,
    "ext_res_det":                          0x01 << 13,
    "ext_clk":                              0x01 << 14,
    "silicon_rv":                           0x07 << 16
}

Fields["DRV_CONF"] = {
    "current_range":                        0x03 << 0,
    "current_range_scale":                  0x03 << 2,
    "slope_control":                        0x03 << 4,
}

Fields["PLL"] = {
    "commit":                               0x01 << 0,
    "ext_not_int":                          0x01 << 1,
    "clk_sys_sel":                          0x01 << 2,
    "clk_fsm_ena":                          0x01 << 3,
    "clock_divider":                        0x1F << 5,
    "clk_1m0_tmo":                          0x01 << 12,
    "clk_loss":                             0x01 << 13,
    "clk_is_stuck":                         0x01 << 14,
}

Fields["IHOLD_IRUN"] = {
    "ihold":                                0xff << 0,
    "irun":                                 0xff << 8,
    "iholddelay":                           0xff << 16,
    "irundelay":                            0x0f << 24
}

Fields["TPOWERDOWN"] = {
    "tpowerdown":                           0xff << 0,
}

Fields["TSTEP"] = {
    "tstep":                                0xfffff << 0,
}

Fields["TPWMTHRS"] = {
    "tpwmthrs":                             0xfffff << 0,
}

Fields["TCOOLTHRS"] = {
    "tcoolthrs":                            0xfffff << 0,
}

Fields["THIGH"] = {
    "thigh":                                0xfffff << 0,
}

Fields["TSGP_LOW_VEL_THRS"] = {
    "tsgp_low_vel_thrs":                    0xfffff << 0,
}

Fields["T_RCOIL_MEAS"] = {
    "t_rcoil_meas":                         0xfffff << 0,
}

Fields["UDC_CONF"] = {
    "decel_thrs":                           0x0f << 0,
    "accel_thrs":                           0x0f << 4,
    "udc_enable":                           0x01 << 8,
}

Fields["SW_MODE"] = {
    "stop_l_enable":                        0x01 << 0,
    "stop_r_enable":                        0x01 << 1,
    "pol_stop_l":                           0x01 << 2,
    "pol_stop_r":                           0x01 << 3,
    "swap_lr":                              0x01 << 4,
    "latch_l_active":                       0x01 << 5,
    "latch_l_inactive":                     0x01 << 6,
    "latch_r_active":                       0x01 << 7,
    "latch_r_inactive":                     0x01 << 8,
    "en_latch_encoder":                     0x01 << 9,
    "sg_stop":                              0x01 << 10,
    "en_softstop":                          0x01 << 11,
    "en_virtual_stop_l":                    0x01 << 12,
    "en_virtual_stop_r":                    0x01 << 13,
    "virtual_stop_enc":                     0x01 << 14,
    "hard_stop_clr_cur_int":                0x01 << 15
}

Fields["ENC_CONST"] = {
    "enc_const":                            0xffffffff << 0,
}

Fields["XLATCH"] = {
    "xlatch":                               0xffffffff << 0,
}

Fields["CURRENT_PI_REG"] = {
    "cur_p":                                0xfff << 0,
    "cur_i":                                0x3ff << 16,
}

Fields["ANGLE_PI_REG"] = {
    "angle_p":                              0xfff << 0,
    "angle_i":                              0x3ff << 16,
}

Fields["ANGLE_LOWER_LIMIT"] = {
    "angle_lower_i_limit":                  0x3ff << 0,
    "angle_error":                          0x3ff << 16,
}

Fields["COOLSTEPPLUS_PI_REG"] = {
    "coolstep_p":                           0xfff << 0,
    "coolstep_i":                           0x3ff << 16,
}

Fields["CUR_ANGLE_LIMIT"] = {
    "angle_pi_limit":                       0x3ff << 0,
    "angle_pi_int_pos_clip":                0x01 << 12,
    "angle_pi_int_neg_clip":                0x01 << 13,
    "angle_pi_pos_clip":                    0x01 << 14,
    "angle_pi_neg_clip":                    0x01 << 15,
    "cur_pi_limit":                         0xfff << 16,
    "cur_pi_int_pos_clip":                  0x01 << 28,
    "cur_pi_int_neg_clip":                  0x01 << 29,
    "cur_pi_pos_clip":                      0x01 << 30,
    "cur_pi_neg_clip":                      0x01 << 31,
}

Fields["COIL_INDUCT"] = {
    "coil_induct":                          0x7fff << 0,
    "rcoil_manual":                         0x01 << 16,
    "rcoil_thermal_coupling":               0x01 << 17,
}

Fields["COOLSTEPPLUS_PI_DOWN"] = {
    "cool_pi_down_limit":                   0xfff << 0,
    "cool_pi_off_speed":                    0xfff << 16,
}

Fields["COOLSTEPPLUS_RESERVE_CONF"] = {
    "cool_low_load_reserve":                0xff << 0,
    "cool_hi_load_reserve":                 0xff << 8,
    "cool_low_generatoric_reserve":         0xff << 16,
    "cool_hi_generatoric_reserve":          0xff << 24,
}

Fields["COOLSTEPPLUS_LOAD_RESERVE"] = {
    "sgp_result":                           0x3ff << 0,
    "coolstep_load_reserve":                0x1ff << 16,
}

Fields["R_COIL_USER"] = {
    "r_coil_user_b":                        0xfff << 0,
    "r_coil_user_a":                        0xfff << 16,
}

Fields["SGP_CONF"] = {
    "sgp_thrs":                             0x1ff << 0,
    "sgp_filt_en":                          0x01 << 12,
    "sgp_low_vel_freeze":                   0x01 << 13,
    "sgp_clear_cur_pi":                     0x01 << 14,
    "sgp_low_vel_slope":                    0xff << 16,    
    "sgp_low_vel_cnts":                     0x03 << 28, 
}

Fields["COOLSTEPPLUS_CONF"] = {
    "cool_cur_div":                         0xf << 0,
    "load_filt_en":                         0x01 << 4,
}

Fields["OTW_OV_VTH"] = {
    "overvoltage_vth":                      0x1ff << 0,
    "overtempprewarning_vth":               0x1ff << 16
}

Fields["MSLUT0"] = { "mslut0": 0xffffffff }
Fields["MSLUT1"] = { "mslut1": 0xffffffff }
Fields["MSLUT2"] = { "mslut2": 0xffffffff }
Fields["MSLUT3"] = { "mslut3": 0xffffffff }
Fields["MSLUT4"] = { "mslut4": 0xffffffff }
Fields["MSLUT5"] = { "mslut5": 0xffffffff }
Fields["MSLUT6"] = { "mslut6": 0xffffffff }
Fields["MSLUT7"] = { "mslut7": 0xffffffff }
Fields["MSLUTSEL"] = {
    "x3":                       0xFF << 24,
    "x2":                       0xFF << 16,
    "x1":                       0xFF << 8,
    "w3":                       0x03 << 6,
    "w2":                       0x03 << 4,
    "w1":                       0x03 << 2,
    "w0":                       0x03 << 0,
}

Fields["MSLUTSTART"] = {
    "start_sin":                0xFF << 0,
    "start_sin90":              0xFF << 16,
    "offset_sin90":             0xFF << 24,
}

Fields["MSCNT"] = {
    "mscnt":                                0x3ff << 0
}

Fields["MSCURACT"] = {
    "cur_a":                                0x1ff << 16,
    "cur_b":                                0x1ff << 0
}

Fields["CHOPCONF"] = {
    "toff":                                 0x0F << 0,
    "hstrt":                                0x07 << 4,
    "hend":                                 0x0F << 7,
    "fd3":                                  0x01 << 11,
    "disfdcc":                              0x01 << 12,
    "chm":                                  0x01 << 14,
    "tbl":                                  0x03 << 15,
    "tpfd":                                 0x0F << 20, # midrange resonances
    "mres":                                 0x0F << 24,
    "intpol":                               0x01 << 28,
    "dedge":                                0x01 << 29
}

Fields["COOLCONF"] = {
    "semin":                                0x0F << 0,
    "seup":                                 0x03 << 5,
    "semax":                                0x0F << 8,
    "sedn":                                 0x07 << 12,
    "seimin":                               0x01 << 15,
    "sgt":                                  0x7F << 16,
    "thigh_sg_off":                         0x01 << 23,
    "sfilt":                                0x01 << 24
}

Fields["DRV_STATUS"] = {
    "sg_result":                            0x3FF << 0,
    "seq_stopped":                          0x01 << 10,
    "ov":                                   0x01 << 11,
    "s2vsa":                                0x01 << 12,
    "s2vsb":                                0x01 << 13,
    "stealth":                              0x01 << 14,
    "cs_actual":                            0xff << 16,
    "stallguard":                           0x01 << 24,
    "ot":                                   0x01 << 25,
    "otpw":                                 0x01 << 26,
    "s2ga":                                 0x01 << 27,
    "s2gb":                                 0x01 << 28,
    "ola":                                  0x01 << 29,
    "olb":                                  0x01 << 30,
    "stst":                                 0x01 << 31
}

Fields["PWMCONF"] = {
    "pwm_freq":                             0x0F << 0,
    "freewheel":                            0x03 << 4,
    "ol_thrsh":                             0x03 << 6,
    "sd_on_meas_lo":                        0x0F << 12,
    "sd_on_meas_hi":                        0x0F << 16
}

Fields["TUDCSTEP"] = {"tudcstep": 0xfffff}
Fields["STEPS_LOST"] = {"steps_lost": 0xfffff}
Fields["TSTEP_VELOCITY"] = {"tstep_velocity": 0x7fffff}

SignedFields = ["cur_a", "cur_b", "sgt", "offset_sin90", "enc_const",
                "xlatch", "sgp_result"]

# Read-only status fields remain available to DUMP_TMC, never to initialization.
ReadOnlyFields = {
    "angle_error", "angle_pi_int_pos_clip", "angle_pi_int_neg_clip",
    "angle_pi_pos_clip", "angle_pi_neg_clip", "cur_pi_int_pos_clip",
    "cur_pi_int_neg_clip", "cur_pi_pos_clip", "cur_pi_neg_clip",
}
ReadOnlyRegisters = {
    "IOIN", "TSTEP", "STEPS_LOST", "XLATCH", "ENC_LATCH",
    "CUR_ANGLE_MEAS", "PI_RESULTS", "R_COIL", "SGP_IND_2_3", "SGP_IND_0_1",
    "INDUCTANCE_VOLTAGE", "SGP_BEMF", "COOLSTEPPLUS_LOAD_RESERVE",
    "TSTEP_VELOCITY", "ADC_VSUPPLY_TEMP", "ADC_I", "MSCNT", "MSCURACT",
    "DRV_STATUS",
}

FieldFormatters = {
    "shaft":            (lambda v: "1(Reverse)" if v else ""),
    "reset":            (lambda v: "1(Reset)" if v else ""),
    "drv_err":          (lambda v: "1(ErrorShutdown!)" if v else ""),
    "uv_cp":            (lambda v: "1(Undervoltage!)" if v else ""),
    "version":          (lambda v: "%#x" % v),
    "mres":             (lambda v: "%d(%dusteps)" % (v, 0x100 >> v)),
    "otpw":             (lambda v: "1(OvertempWarning!)" if v else ""),
    "ot":               (lambda v: "1(OvertempError!)" if v else ""),
    "s2ga":             (lambda v: "1(ShortToGND_A!)" if v else ""),
    "s2gb":             (lambda v: "1(ShortToGND_B!)" if v else ""),
    "ola":              (lambda v: "1(OpenLoad_A!)" if v else ""),
    "olb":              (lambda v: "1(OpenLoad_B!)" if v else ""),
    "cs_actual":        (lambda v: ("%d" % v) if v else "0(Reset?)"),
    "s2vsa":            (lambda v: "1(ShortToSupply_A!)" if v else ""),
    "s2vsb":            (lambda v: "1(ShortToSupply_B!)" if v else ""),
}

MAX_CURRENT = 4
CURRENT_RANGE_SCALE = [0.25, 0.50, 0.75, 1.0]
KIFS = [18000., 36000., 54000., 72000.]
WaveTableRegisters = ["MSLUT%d" % i for i in range(8)] + ["MSLUTSEL", "MSLUTSTART"]


class TMC5262FieldHelper(tmc.FieldHelper):
    def get_field(self, field_name, reg_value=None, reg_name=None):
        value = super().get_field(field_name, reg_value, reg_name)
        # DRV_STATUS.SG_RESULT is unsigned in SpreadCycle, but mirrors the
        # signed SGP_RESULT in StealthChop+ (datasheet p. 238).
        if field_name == "sg_result":
            if reg_value is None:
                reg_value = self.registers.get("DRV_STATUS", 0)
            if reg_value & Fields["DRV_STATUS"]["stealth"] and value & 0x200:
                value -= 0x400
        return value


class MCU_TMC5262_SPI(tmc2130.MCU_TMC_SPI):
    # The common SPI driver supplies two-frame reads, write-echo verification,
    # retries, MCU-time scheduling and per-device daisy-chain addressing.
    def get_mcu(self):
        return self.tmc_spi.spi.get_mcu()


class TMC5262CurrentHelper:
    def __init__(self, config, mcu_tmc):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.mcu_tmc = mcu_tmc
        self.fields = mcu_tmc.get_fields()
        self.Rref = 12000.
        run_current = config.getfloat('run_current',
                                      above=0., maxval=MAX_CURRENT)
        hold_current = config.getfloat('hold_current', MAX_CURRENT,
                                       above=0., maxval=MAX_CURRENT)
        self.req_hold_current = hold_current
        drv_current_range = self._calc_current_range(run_current)
        if drv_current_range == 0:
            drv_current_range_scale = self._calc_current_range_scale(run_current)
        else:
            drv_current_range_scale = 3
        self.fields.set_field("current_range", drv_current_range)
        self.fields.set_field("current_range_scale", drv_current_range_scale)
        irun, ihold = self._calc_current(run_current, hold_current,
                                         drv_current_range, drv_current_range_scale)
        self.fields.set_field("ihold", ihold)
        self.fields.set_field("irun", irun)
    def _get_ifs_rms(self, current_range, current_range_scale=3):
        return (KIFS[current_range] * CURRENT_RANGE_SCALE[current_range_scale]
                / math.sqrt(2.) / self.Rref)
    def _calc_current_range(self, current):
        for current_range in range(4):
            if current <= self._get_ifs_rms(current_range):
                break
        return current_range
    def _calc_current_range_scale(self, current):
        for current_range_scale in range(4):
            if current <= self._get_ifs_rms(0, current_range_scale):
                break
        return current_range_scale
    def _calc_current_bits(self, current, current_range, current_range_scale):
        bits = int((current * 4. * 4. * 250.)
                   / (4.24 * (current_range + 1)
                      * (current_range_scale + 1)))
        return min(max(0, bits), 250)
    def _calc_current(self, run_current, hold_current,
                      current_range, current_range_scale):
        irun = self._calc_current_bits(run_current, current_range,
                                       current_range_scale)
        ihold = self._calc_current_bits(min(hold_current, run_current),
                                        current_range, current_range_scale)
        return irun, ihold
    def _calc_current_from_field(self, field_name):
        bits = self.fields.get_field(field_name)
        current_range = self.fields.get_field("current_range")
        current_range_scale = self.fields.get_field("current_range_scale")
        current = (bits * 4.24 * (current_range + 1)
                   * (current_range_scale + 1)) / (4. * 4. * 250.)
        return current
    def get_current(self):
        run_current = self._calc_current_from_field("irun")
        hold_current = self._calc_current_from_field("ihold")
        current_range = self.fields.get_field("current_range")
        range_scale = self.fields.get_field("current_range_scale")
        max_current = min(MAX_CURRENT,
                          4.24 * (current_range + 1) * (range_scale + 1) / 16.)
        return run_current, hold_current, self.req_hold_current, max_current
    def set_current(self, run_current, hold_current, print_time):
        # Keep DRV_CONF's range fixed after initial configuration (pp. 47, 75).
        # A range change invalidates R_COIL and must not occur during motion.
        max_current = self.get_current()[3]
        if run_current < 0. or run_current > max_current:
            raise self.printer.command_error(
                "TMC %s current exceeds configured range (maximum %.3fA); "
                "change run_current in the config and restart Klipper"
                % (self.name, max_current))
        self.req_hold_current = hold_current
        drv_current_range = self.fields.get_field("current_range")
        drv_current_range_scale = self.fields.get_field("current_range_scale")
        irun, ihold = self._calc_current(run_current, hold_current,
                                         drv_current_range, drv_current_range_scale)
        self.fields.set_field("ihold", ihold)
        val = self.fields.set_field("irun", irun)
        self.mcu_tmc.set_register("IHOLD_IRUN", val, print_time)


class TMC5262CommandHelper(tmc.TMCCommandHelper):
    def _init_clock_pll(self, print_time=None):
        # Do not reset a healthy PLL on every stepper enable: disabling the PLL
        # resets control logic, including the sequencer used for phase tracking.
        reactor = self.printer.get_reactor()
        debug = self.printer.get_start_args().get('debugoutput') is not None
        if not debug:
            val = self.mcu_tmc.get_register("PLL")
            if (val & (PLL_CONFIG_MASK | PLL_ERROR_MASK | 1)) == PLL_CONFIG:
                return
        self.mcu_phase_offset = None
        for attempt in range(PLL_RETRIES):
            # Disable FSM before (re)starting, select internal 16 MHz, commit
            # and clear the old startup timeout (datasheet pp. 106-109).
            self.mcu_tmc.set_register("PLL", 0, print_time)
            self.mcu_tmc.set_register("PLL", PLL_START, print_time)
            # set_register waits for the SPI echo; delays below start AFTER
            # actual MCU execution, even when INIT_TMC supplied print_time.
            deadline = reactor.monotonic() + PLL_TIMEOUT
            while True:
                reactor.pause(reactor.monotonic() + 0.001)
                val = 0 if debug else self.mcu_tmc.get_register("PLL")
                if not val & 1 or reactor.monotonic() >= deadline:
                    break
            if not val & 1:
                self.mcu_tmc.set_register("PLL", PLL_CLEAR_FLAGS, print_time)
                reactor.pause(reactor.monotonic() + 0.010)
                val = PLL_CONFIG if debug else self.mcu_tmc.get_register("PLL")
                if (val & (PLL_CONFIG_MASK | PLL_ERROR_MASK | 1)) == PLL_CONFIG:
                    self.fields.registers["PLL"] = PLL_CONFIG
                    return
            logging.warning("TMC %s PLL start attempt %d failed: %s",
                            self.stepper_name, attempt + 1,
                            self.fields.pretty_format("PLL", val))
        self.mcu_tmc.set_register("PLL", 0, print_time)
        raise self.printer.command_error(
            "TMC %s internal clock PLL failed to start" % (self.stepper_name,))

    def _init_registers(self, print_time=None):
        self._init_clock_pll(print_time)
        debug = self.printer.get_start_args().get('debugoutput') is not None
        wave_updates = set(WaveTableRegisters)
        if not debug:
            # Re-enabling must not rewrite the wave table at an arbitrary
            # microstep. Unchanged hardware values need no write (p. 230).
            wave_updates = {
                reg for reg in WaveTableRegisters
                if self.mcu_tmc.get_register(reg) != self.fields.registers[reg]
            }
            if wave_updates and self._query_phase() != 0:
                raise self.printer.command_error(
                    "TMC %s wave table update requires MSCNT=0"
                    % (self.stepper_name,))
        # Keep GSTAT.reset available to Klipper's error checker so a chip reset
        # invalidates the stored phase offset. Its W1C flags are cleared there.
        for reg_name in list(self.fields.registers):
            if (reg_name != "PLL" and
                    (reg_name not in WaveTableRegisters or reg_name in wave_updates)):
                self.mcu_tmc.set_register(reg_name,
                                         self.fields.registers[reg_name],
                                         print_time)

    def cmd_INIT_TMC(self, gcmd):
        # Drain queued moves before inspecting MSCNT or restarting the PLL.
        self.printer.lookup_object('toolhead').wait_moves()
        super().cmd_INIT_TMC(gcmd)

    def cmd_SET_TMC_FIELD(self, gcmd):
        field = gcmd.get('FIELD').lower()
        reg = self.fields.lookup_register(field)
        if field in ReadOnlyFields or reg in ReadOnlyRegisters:
            raise gcmd.error("TMC5262 field '%s' is read-only" % (field,))
        # Klipper owns pulse generation and must remain in STEP/DIR mode.
        if field in ("step_dir", "direct_mode", "current_range",
                     "current_range_scale") or reg == "PLL":
            raise gcmd.error("TMC5262 field '%s' is managed by the driver"
                             % (field,))
        if reg in WaveTableRegisters:
            self.printer.lookup_object('toolhead').wait_moves()
            if self._query_phase() != 0:
                raise gcmd.error("TMC5262 wave table update requires MSCNT=0")
        super().cmd_SET_TMC_FIELD(gcmd)


def TMC5262RegisterDefaultsHelper(config, mcu_tmc):
    registers = mcu_tmc.get_fields().registers
    for reg_name, val in TMC5262RegisterDefaults:
        registers[reg_name] = val


def TMC5262StealthchopHelper(config, mcu_tmc):
    fields = mcu_tmc.get_fields()
    velocity = config.getfloat('stealthchop_threshold', None, minval=0.)
    if velocity is not None:
        tpwmthrs = tmc.TMCtstepHelper(mcu_tmc, velocity, config=config)
        fields.set_field("tpwmthrs", tpwmthrs)
        fields.set_field("en_pwm_mode", True)


def TMC5262VcoolthrsHelper(config, mcu_tmc):
    velocity = config.getfloat('coolstep_threshold', None, minval=0.)
    if velocity is not None:
        tcoolthrs = tmc.TMCtstepHelper(mcu_tmc, velocity, config=config)
        mcu_tmc.get_fields().set_field("tcoolthrs", tcoolthrs)


def TMC5262VhighHelper(config, mcu_tmc):
    velocity = config.getfloat('high_velocity_threshold', None, minval=0.)
    if velocity is not None:
        thigh = tmc.TMCtstepHelper(mcu_tmc, velocity, config=config)
        mcu_tmc.get_fields().set_field("thigh", thigh)


class TMC5262VirtualPinHelper(tmc.TMCVirtualPinHelper):
    def handle_homing_move_begin(self, hmove):
        if self.mcu_endstop not in hmove.get_mcu_endstops():
            return
        # TMC5262 routes diag stall selection through DO_CONF, not GCONF.
        self.en_pwm = self.fields.get_field("en_pwm_mode")
        val = self.fields.set_field("en_pwm_mode", 0)
        self.mcu_tmc.set_register("GCONF", val)
        val = self.fields.set_field(self.diag_pin_field, 1)
        self.mcu_tmc.set_register("DO_CONF", val)
        # Enable tcoolthrs (if not already)
        self.coolthrs = self.fields.get_field("tcoolthrs")
        if self.coolthrs == 0:
            tc_val = self.fields.set_field("tcoolthrs", 0xfffff)
            self.mcu_tmc.set_register("TCOOLTHRS", tc_val)
        # Disable thigh
        reg = self.fields.lookup_register("thigh", None)
        if reg is not None:
            self.thigh = self.fields.get_field("thigh")
            th_val = self.fields.set_field("thigh", 0)
            self.mcu_tmc.set_register(reg, th_val)

    def handle_homing_move_end(self, hmove):
        if self.mcu_endstop not in hmove.get_mcu_endstops():
            return
        # Restore stealthchop and diag routing.
        val = self.fields.set_field("en_pwm_mode", self.en_pwm)
        self.mcu_tmc.set_register("GCONF", val)
        val = self.fields.set_field(self.diag_pin_field, 0)
        self.mcu_tmc.set_register("DO_CONF", val)
        # Restore tcoolthrs
        tc_val = self.fields.set_field("tcoolthrs", self.coolthrs)
        self.mcu_tmc.set_register("TCOOLTHRS", tc_val)
        # Restore thigh
        reg = self.fields.lookup_register("thigh", None)
        if reg is not None:
            th_val = self.fields.set_field("thigh", self.thigh)
            self.mcu_tmc.set_register(reg, th_val)

######################################################################
# TMC5262 printer object
######################################################################
class TMC5262:
    def __init__(self, config):
        # Setup mcu communication
        self.fields = TMC5262FieldHelper(Fields, SignedFields, FieldFormatters)
        # STEP/DIR motion with SPI register configuration.
        self.mcu_tmc = MCU_TMC5262_SPI(config, Registers, self.fields,
                                       TMC_FREQUENCY)
        TMC5262RegisterDefaultsHelper(config, self.mcu_tmc)
        # Allow virtual pins to be created
        TMC5262VirtualPinHelper(config, self.mcu_tmc)

        # Register commands
        current_helper = TMC5262CurrentHelper(config, self.mcu_tmc)
        cmdhelper = TMC5262CommandHelper(config, self.mcu_tmc, current_helper)
        cmdhelper.setup_register_dump(ReadRegisters)
        self.get_phase_offset = cmdhelper.get_phase_offset
        self.get_status = cmdhelper.get_status
        # Setup basic register values
        tmc.TMCWaveTableHelper(config, self.mcu_tmc)
        TMC5262StealthchopHelper(config, self.mcu_tmc)
        TMC5262VcoolthrsHelper(config, self.mcu_tmc)
        TMC5262VhighHelper(config, self.mcu_tmc)
        set_config_field = self.fields.set_config_field

        # Allow other registers to be set from the config
        # GCONF: mandatory STEP/DIR mode; keep normal Klipper dir_pin control.
        self.fields.set_field("step_dir", 1)
        set_config_field(config, "fast_standstill", 0)
        set_config_field(config, "en_pwm_mode", 1)
        set_config_field(config, "multistep_filt", 1)
        set_config_field(config, "shaft", 0)
        set_config_field(config, "small_hysteresis", 1)

        # DO_CONF
        set_config_field(config, "diag0_nod_pp", 0)
        set_config_field(config, "diag0_invpp", 1)
        set_config_field(config, "diag1_nod_pp", 0)
        set_config_field(config, "diag1_invpp", 1)

        # DRV_CONF
        set_config_field(config, "slope_control", 3)

        # IHOLD_IRUN
        set_config_field(config, "iholddelay", 7)
        set_config_field(config, "irundelay", 4)

        # TPOWERDOWN
        set_config_field(config, "tpowerdown", 10)

        # Low velocity thresholds
        set_config_field(config, "tsgp_low_vel_thrs", 4096)
        set_config_field(config, "t_rcoil_meas", 3763)

        # UDC_CONF
        set_config_field(config, "decel_thrs", 14)
        set_config_field(config, "accel_thrs", 12)
        set_config_field(config, "udc_enable", 0)

        # SW_MODE
        set_config_field(config, "hard_stop_clr_cur_int", 1)

        # ENC_CONST
        set_config_field(config, "enc_const", 65536)

        # CURRENT_PI_REG
        set_config_field(config, "cur_p", 67)
        set_config_field(config, "cur_i", 17)

        # ANGLE_PI_REG
        set_config_field(config, "angle_p", 50)
        set_config_field(config, "angle_i", 20)

        # CUR_ANGLE_LIMIT
        set_config_field(config, "angle_pi_limit", 256)
        set_config_field(config, "cur_pi_limit", 4095)

        # ANGLE_LOWER_LIMIT
        set_config_field(config, "angle_lower_i_limit", 256)

        # COIL_INDUCT
        set_config_field(config, "coil_induct", 800)
        set_config_field(config, "rcoil_manual", 0)
        set_config_field(config, "rcoil_thermal_coupling", 0)

        # R_COIL_USER
        set_config_field(config, "r_coil_user_b", 0)
        set_config_field(config, "r_coil_user_a", 0)

        # SGP_CONF
        set_config_field(config, "sgp_clear_cur_pi", 1)

        # COOLSTEPPLUS_CONF
        set_config_field(config, "cool_cur_div", 2)
        set_config_field(config, "load_filt_en", 1)

        # COOLSTEPPLUS_PI_REG
        set_config_field(config, "coolstep_p", 128)
        set_config_field(config, "coolstep_i", 16)

        # COOLSTEPPLUS_PI_DOWN
        set_config_field(config, "cool_pi_down_limit", 1611)
        set_config_field(config, "cool_pi_off_speed", 64)

        # COOLSTEPPLUS_RESERVE_CONF
        set_config_field(config, "cool_low_load_reserve", 150)
        set_config_field(config, "cool_hi_load_reserve", 50)
        set_config_field(config, "cool_low_generatoric_reserve", 220)
        set_config_field(config, "cool_hi_generatoric_reserve", 100)

        # OTW_OV_VTH
        set_config_field(config, "overvoltage_vth", 511)
        set_config_field(config, "overtempprewarning_vth", 511)

        # 0x60(MSLUT[0])
        set_config_field(config, "mslut0", 2863314260)

        # 0x61(MSLUT[1])
        set_config_field(config, "mslut1", 1251300522)

        # 0x62(MSLUT[2])
        set_config_field(config, "mslut2", 608774441)

        # 0x63(MSLUT[3])
        set_config_field(config, "mslut3", 269500962)

        # 0x64(MSLUT[4])
        set_config_field(config, "mslut4", 4227858431)

        # 0x65(MSLUT[5])
        set_config_field(config, "mslut5", 3048961917)

        # 0x66(MSLUT[6])
        set_config_field(config, "mslut6", 1227445590)

        # 0x67(MSLUT[7])
        set_config_field(config, "mslut7", 4211234)

        # 0x68(MSLUTSEL)MSLUTSTART
        set_config_field(config, "w0", 2)
        set_config_field(config, "w1", 1)
        set_config_field(config, "w2", 1)
        set_config_field(config, "w3", 1)
        set_config_field(config, "x1", 128)
        set_config_field(config, "x2", 255)
        set_config_field(config, "x3", 255)

        # 0x69(MSLUTSTART)
        set_config_field(config, "start_sin", 0)
        set_config_field(config, "start_sin90", 247)
        set_config_field(config, "offset_sin90", 0)

        # CHOPCONF
        set_config_field(config, "toff", 3)
        set_config_field(config, "hstrt", 5)
        set_config_field(config, "hend", 2)
        set_config_field(config, "tbl", 2)
        set_config_field(config, "tpfd", 4)
        set_config_field(config, "dedge", 0)

        # COOLCONF
        set_config_field(config, "semin", 0)
        set_config_field(config, "seup", 0)
        set_config_field(config, "semax", 0)
        set_config_field(config, "sedn", 0)
        set_config_field(config, "seimin", 0)
        set_config_field(config, "sgt", 0)
        set_config_field(config, "thigh_sg_off", 0)
        set_config_field(config, "sfilt", 0)

        # PWMCONF
        set_config_field(config, "sd_on_meas_lo", 14)
        set_config_field(config, "sd_on_meas_hi", 15)


def load_config_prefix(config):
    return TMC5262(config)
