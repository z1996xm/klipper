# TMC2262 configuration
#
# Copyright (C) 2018-2023  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2023  Alex Voinea <voinea.dragos.alexandru@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
from . import bus, tmc, tmc2130

TMC_FREQUENCY=16000000.

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
    "COOLSTEPPLUS_PI_DOWN":     0x50,
    "SGP_IND_2_3":              0x4A,
    "SGP_IND_0_1":              0x4B,

    "INDUCTANCE_VOLTAGE":       0x4C,
    "SGP_BEMF":                 0x4D,
    "COOLSTEPPLUS_CONF":        0x4E,
    "COOLSTEPPLUS_PI_REG":      0x4F,

    "COOLSTEPPLUS_PI_DOW":      0x50,
    "COOLSTEPPLUS_RESERVE_CONF":0x51,
    "COOLSTEPPLUS_LOAD_RESERVE":0x52,

    "TSTEP_Velocity":           0x53,
    "ADC_VSUPPLY_TEMP":         0x58,
    "ADC_I":                    0x58,
    "OTW_OV_VTH":               0x59,
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
    "GCONF", "GSTAT", "DO_CONF", "IOIN", "PLL",
    "TPOWERDOWN", "TSTEP", "TPWMTHRS", "TCOOLTHRS", "THIGH", "TSTEP_Velocity", "ADC_VSUPPLY_TEMP", "OTW_OV_VTH", "MSLUTSEL",
    "MSLUTSTART", "MSCNT", "MSCURACT", "CHOPCONF", "COOLCONF", "DRV_STATUS",
    "PWMCONF"
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

    "diag0_nOD_PP":                         0x01 << 28,
    "diag0_invPP":                          0x01 << 29,
    "diag1_nOD_PP":                         0x01 << 30,
    "diag1_invPP":                          0x01 << 31,
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
    "silicon_rv":                           0x03 << 16
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
    "bit_3":                                0x01 << 3,
    "bit_4":                                0x01 << 4,
    "clock_divider":                        0x1F << 5,
    "bit_10":                               0x01 << 10,
    "bit_11":                               0x01 << 11,
    "clk_1mo_tmo":                          0x01 << 12,
    "clk_loss":                             0x01 << 13,
    "clk_is_stuck":                         0x01 << 14,
    "pll_lock_loss":                        0x01 << 15
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
    "enc_const":                            0xffffff << 0,
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
    "coolstep_i":                           0xfff << 16,
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
    "cool_pi_down_speed":                   0xfff << 16,
}

Fields["COOLSTEPPLUS_RESERVE_CONF"] = {
    "sgp_result":                           0x3ff << 0,
    "coolstep_load_reserve":                0xfff << 16,
}

Fields["R_COIL_USER"] = {
    "r_coil_user_b":                        0xfff << 0,
    "r_coil_user_a":                        0xfff << 16,
}

Fields["SGP_CONF"] = {
    "sgp_thrs":                             0xff << 0,
    "sgp_filt_en":                          0x01 << 12,
    "sgp_low_vel_freeze":                   0x01 << 13,
    "sgp_clear_cur_pi":                     0x01 << 14,
    "sgp_low_vel_slope":                    0xff << 16,    
    "sgp_low_vel_cnts":                     0x01 << 30, 
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
    "cur_a":                                0x1ff << 0,
    "cur_b":                                0x1ff << 16
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
    "sedn":                                 0x03 << 13,
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
    "stealth":                              0x1F << 14,
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

SignedFields = ["cur_a", "cur_b", "sgt", "offset_sin90"]

# FieldFormatters = dict(tmc2130.FieldFormatters)
# FieldFormatters.update({
#     "s2vsa":            (lambda v: "1(ShortToSupply_A!)" if v else ""),
#     "s2vsb":            (lambda v: "1(ShortToSupply_B!)" if v else ""),
#     # "adc_temp":         (lambda v: "0x%04x(%.1fC)" % (v, ((v - 2038) / 7.7))),
#     # "adc_vsupply":      (lambda v: "0x%04x(%.3fV)" % (v, v * 0.009732)),
#     # "adc_ain":          (lambda v: "0x%04x(%.3fmV)" % (v, v * 0.3052)),
# })

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

class TMC2262CurrentHelper:
    def __init__(self, config, mcu_tmc):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.mcu_tmc = mcu_tmc
        self.fields = mcu_tmc.get_fields()
        # self.Rref = config.getfloat('rref', 12000.,
        #                             minval=12000., maxval=60000.)
        self.Rref = 12000.
        self.crs = 1.
        # max_cur = self._get_ifs_rms(3)
        run_current = config.getfloat('run_current', above=0., maxval=MAX_CURRENT)
        hold_current = config.getfloat('hold_current',MAX_CURRENT,
                                       above=0., maxval=MAX_CURRENT)
        self.req_hold_current = hold_current
        drv_current_range , self.crs = self._calc_current_range(run_current)
        drv_current_range_scale = None
        if drv_current_range == 0:
            drv_current_range_scale , self.crs = self._calc_current_range_scale(run_current)
            self.fields.set_field("current_range_scale", drv_current_range_scale)
        else :
            drv_current_range_scale = 3
            self.fields.set_field("current_range_scale", drv_current_range_scale)
        self.fields.set_field("current_range", drv_current_range)
        irun, ihold = self._calc_current(run_current, hold_current,
                                         drv_current_range, drv_current_range_scale)
        self.fields.set_field("ihold", ihold)
        self.fields.set_field("irun", irun)
    def _get_ifs_rms(self, current_range):
        # if current_range is None:
        #     current_range = self.fields.get_field("current_range")
        KIFS = [18000., 36000., 54000., 72000.]
        return (KIFS[current_range] * self.crs / math.sqrt(2.) / self.Rref)     # Ifs = CRS*KIFS/Rref/1.414
    def _calc_current_range(self, current):
        crs = 1.
        for current_range in range(4):
            if current <= self._get_ifs_rms(current_range):
                break
        return current_range , crs
    def _calc_current_range_scale(self, current):
        ifs_rms = [0.266, 0.532, 0.798, 1.064]
        CRS = [0.25, 0.50, 0.75, 1.0]
        for current_range_scale in range(4):
            if current <= ifs_rms[current_range_scale]:
                crs = CRS[current_range_scale]
                break
        return current_range_scale , crs
    def _calc_current_bits(self, current, current_range, current_range_scale):
        # ifs_rms = self._get_ifs_rms(current_range)
        IRUN = int((current *4 *4 *250)/(4.24 *(current_range+1) *(current_range_scale+1)))
        return min(max(0, IRUN), 250)
    def _calc_current(self, run_current, hold_current ,current_range, current_range_scale):
        irun = self._calc_current_bits(run_current, current_range, current_range_scale)
        ihold = self._calc_current_bits(hold_current, current_range, current_range_scale)
        return irun, ihold
    def _calc_current_from_field(self, field_name):
        # ifs_rms = self._get_ifs_rms(current_range)
        bits = self.fields.get_field(field_name)
        current_range = self.fields.get_field("current_range")
        current_range_scale = self.fields.get_field("current_range_scale")
        current = (bits * (4.24 *(current_range+1) *(current_range_scale+1)))/(4 *4 *250)
        return current
    def get_current(self):
        run_current = self._calc_current_from_field("irun")
        hold_current = self._calc_current_from_field("ihold")
        return run_current, hold_current, self.req_hold_current, MAX_CURRENT
    def set_current(self, run_current, hold_current, print_time):
        self.req_hold_current = hold_current
        drv_current_range , self.crs = self._calc_current_range(run_current)
        drv_current_range_scale = None
        if drv_current_range == 0:
            drv_current_range_scale , self.crs = self._calc_current_range_scale(run_current)
            self.fields.set_field("current_range_scale", drv_current_range_scale)
        else :
            drv_current_range_scale = 3
            self.fields.set_field("current_range_scale", drv_current_range_scale)
        self.fields.set_field("current_range", drv_current_range)
        irun, ihold = self._calc_current(run_current, hold_current,
                                         drv_current_range, drv_current_range_scale)
        self.fields.set_field("ihold", ihold)
        val = self.fields.set_field("irun", irun)
        self.mcu_tmc.set_register("IHOLD_IRUN", val, print_time)

######################################################################
# TMC2240 printer object
######################################################################
class TMC2262:
    def __init__(self, config):
        # Setup mcu communication
        self.fields = tmc.FieldHelper(Fields, SignedFields, FieldFormatters)
        # if config.get("uart_pin", None) is not None:
        #     # use UART for communication
        #     self.mcu_tmc = tmc_uart.MCU_TMC_uart(config, Registers, self.fields,
        #                                          3, TMC_FREQUENCY)
        # else:
            # Use SPI bus for communication
        self.mcu_tmc = tmc2130.MCU_TMC_SPI(config, Registers, self.fields,
                                               TMC_FREQUENCY)
        # Allow virtual pins to be created
        tmc.TMCVirtualPinHelper(config, self.mcu_tmc)

        # Register commands
        current_helper = TMC2262CurrentHelper(config, self.mcu_tmc)
        cmdhelper = tmc.TMCCommandHelper(config, self.mcu_tmc, current_helper)
        cmdhelper.setup_register_dump(ReadRegisters)
        self.get_phase_offset = cmdhelper.get_phase_offset
        self.get_status = cmdhelper.get_status
        # Setup basic register values
        tmc.TMCWaveTableHelper(config, self.mcu_tmc)
        tmc.TMCStealthchopHelper(config, self.mcu_tmc)
        set_config_field = self.fields.set_config_field

        # #enable stealthchop+
        # # GCONF
        # set_config_field(config, "fast_standstill", 0)
        # set_config_field(config, "en_pwm_mode", 1)
        # set_config_field(config, "multistep_filt", 1)
        # set_config_field(config, "shaft", 0)
        # set_config_field(config, "small_hysteresis", 1)
        # set_config_field(config, "stop_enable", 0)
        # set_config_field(config, "direct_mode", 0)
        # # set_config_field(config, "mres", 4)
        # set_config_field(config, "length_steppulse", 0)
        # set_config_field(config, "ov_nn", 1)
        # set_config_field(config, "step_dir", 0)

        # set_config_field(config, "semin", 0)
        # set_config_field(config, "seup", 0)
        # set_config_field(config, "semax", 0)
        # set_config_field(config, "sedn", 0)
        # set_config_field(config, "seimin", 0)
        # set_config_field(config, "sgt", 0)
        # set_config_field(config, "thigh_sg_off", 0)
        # set_config_field(config, "sfilt", 0)

        # set_config_field(config, "toff", 3)
        # set_config_field(config, "hstrt", 5)
        # set_config_field(config, "hend", 2)
        # set_config_field(config, "fd3", 0)
        # set_config_field(config, "disfdcc", 0)
        # set_config_field(config, "chm", 0)
        # set_config_field(config, "tbl", 2)
        # set_config_field(config, "tpfd", 4)
        # # set_config_field(config, "mres", 4)
        # set_config_field(config, "intpol", 1)
        # set_config_field(config, "dedge", 0)

        # set_config_field(config, "pll_lock_loss", 0)

        #enable stealthchop+
        # # 0x00(GCONF)
        set_config_field(config, "fast_standstill", 0)
        set_config_field(config, "en_pwm_mode", 1)
        set_config_field(config, "multistep_filt", 1)
        set_config_field(config, "shaft", 0)
        set_config_field(config, "small_hysteresis", 1)

        # # 0x02(DO_CONF)
        set_config_field(config, "diag0_nOD_PP", 0)
        set_config_field(config, "diag0_invPP", 1)
        set_config_field(config, "diag1_nOD_PP", 0)
        set_config_field(config, "diag1_invPP", 1)

        # # # 0x03(DO_CONF)

        # # # 0x0A(DRV_CONF)
        # # set_config_field(config, "current_range", 2)
        # # set_config_field(config, "current_range_scale", 3)
        set_config_field(config, "slope_control", 3)

        # # # 0x0B(PLL)

        # # # 0x10(IHOLD_IRUN)
        # # set_config_field(config, "ihold", 182)
        # # set_config_field(config, "irun",  182)
        set_config_field(config, "iholddelay", 7)
        set_config_field(config, "irundelay", 4)

        # # # 0x11(TPOWERDOWN)
        set_config_field(config, "tpowerdown", 10)

        # # # 0x13(TPWMTHRS)
        set_config_field(config, "tpwmthrs", 5) #新增

        # # # 0x14(TCOOLTHRS)
        set_config_field(config, "tcoolthrs", 1471)

        # # # 0x15(THIGH)

        # # # 0x16(TSGP_LOW_VEL_THRS)
        set_config_field(config, "tsgp_low_vel_thrs", 4096)

        # # # 0x17(T_RCOIL_MEAS)
        set_config_field(config, "t_rcoil_meas", 4096)

        # # # 0x18(TUDCSTEP)

        # # # 0x19(UDC_CONF)
        set_config_field(config, "decel_thrs", 14)
        set_config_field(config, "accel_thrs", 12)
        set_config_field(config, "udc_enable", 0)

        # # # 0x1A(STEPS_LOST)

        # # # 0x34(SW_MODE)
        set_config_field(config, "hard_stop_clr_cur_int", 1)

        # # # 0x38(ENCMODE)

        # # # 0x39(X_ENC)

        # # # 0x3A(ENC_CONSTc)
        set_config_field(config, "enc_const", 1)

        # # # 0x40(CURRENT_PI_REG)
        set_config_field(config, "cur_p", 152)
        set_config_field(config, "cur_i", 76)

        # # # 0x41(ANGLE_PI_REG)
        set_config_field(config, "angle_p", 50)
        set_config_field(config, "angle_i", 20)

        # # # 0x42(CUR_ANGLE_LIMIT)
        set_config_field(config, "angle_pi_limit", 256)
        set_config_field(config, "cur_pi_limit", 4095)

        # # # 0x43(ANGLE_LOWER_LIMIT)
        set_config_field(config, "angle_lower_i_limit", 256)
        set_config_field(config, "angle_error", 0)

        # # 0x46(COIL_INDUCT)
        set_config_field(config, "coil_induct", 400)
        set_config_field(config, "rcoil_manual", 0)
        set_config_field(config, "rcoil_thermal_coupling", 0)

        # # 0x48(R_COIL_USER)
        set_config_field(config, "r_coil_user_b", 0)

        # # 0x49(SGP_CONF)
        set_config_field(config, "sgp_clear_cur_pi", 1)

        # # 0x4E(COOLSTEPPLUS_CONF)
        set_config_field(config, "cool_cur_div", 6)
        set_config_field(config, "load_filt_en", 1)

        # # 0x4F(COOLSTEPPLUS_PI_REG) 新增
        set_config_field(config, "coolstep_p", 128)
        set_config_field(config, "coolstep_i", 16)

        # # 0x50(COOLSTEPPLUS_PI_DOWN)
        set_config_field(config, "cool_pi_down_limit", 128)
        set_config_field(config, "cool_pi_down_speed", 64)

        # # 0x51(COOLSTEPPLUS_RESERVE_CONF)
        set_config_field(config, "sgp_result", 662)
        set_config_field(config, "coolstep_load_reserve", 1244)

        # # 0x5A(OTW_OV_VTH)
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
        set_config_field(config, "x3", 2555)

        # 0x69(MSLUTSTART)
        set_config_field(config, "start_sin", 0)
        set_config_field(config, "start_sin90", 247)
        set_config_field(config, "offset_sin90", 0)

        # # 0x6C(CHOPCONF)
        set_config_field(config, "toff", 3)
        set_config_field(config, "hstrt", 5)
        set_config_field(config, "hend", 2)
        set_config_field(config, "tbl", 2)
        set_config_field(config, "tpfd", 4)
        set_config_field(config, "dedge", 1)

        # # 0x6D(COOLCONF)
        set_config_field(config, "semin", 0)
        set_config_field(config, "seup", 0)
        set_config_field(config, "semax", 0)
        set_config_field(config, "sedn", 0)
        set_config_field(config, "seimin", 0)
        set_config_field(config, "sgt", 0)
        set_config_field(config, "thigh_sg_off", 0)
        set_config_field(config, "sfilt", 0)

        # # 0x70(PWMCONF)
        set_config_field(config, "sd_on_meas_lo", 14)
        set_config_field(config, "sd_on_meas_hi", 15)

        set_config_field(config, "pll_lock_loss", 0)
        #enable stealthchop+

def load_config_prefix(config):
    return TMC2262(config)
