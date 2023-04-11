import logging, time, collections, threading, multiprocessing, os
from . import bus, motion_report, adxl345

# LIS3DH registers
REG_LIS3DH_WHO_AM_I_ADDR = 0x0F
REG_LIS3DH_CTRL_REG1_ADDR = 0x20
REG_LIS3DH_CTRL_REG2_ADDR = 0x21
REG_LIS3DH_CTRL_REG3_ADDR = 0x22
REG_LIS3DH_CTRL_REG4_ADDR = 0x23
REG_LIS3DH_CTRL_REG5_ADDR = 0x24
REG_LIS3DH_CTRL_REG6_ADDR = 0x25
REG_LIS3DH_STATUS_REG_ADDR = 0x27
REG_LIS3DH_OUT_XL_ADDR = 0x28
REG_LIS3DH_OUT_XH_ADDR = 0x29
REG_LIS3DH_OUT_YL_ADDR = 0x2A
REG_LIS3DH_OUT_YH_ADDR = 0x2B
REG_LIS3DH_OUT_ZL_ADDR = 0x2C
REG_LIS3DH_OUT_ZH_ADDR = 0x2D
REG_LIS3DH_FIFO_CTRL = 0x2E
REG_MOD_READ = 0x80
REG_MOD_MULTI = 0x40


LIS3DH_XYZ_ENABLE      =          (0x07)
LIS3DH_BDU_NO_UPDATE	=           (0x1<<3)
LIS3DH_FILTER_BW_800   =          (0x00) 
LIS3DH_FULLSCALE_16    =          (0x40)      #/* 16 g */
LIS3DH_SELFTEST_NORMAL =          (0x00)
LIS3DH_SERIALINTERFACE_4WIRE    = (0x00)
LIS3DH_DATARATE_1600   =          (0x90)
LIS3DH_STREAM_MODE     =          (0x40)


QUERY_RATES = {
    25: 0x40, 50: 0x50, 100: 0x60,  400: 0x70,
    800: 0x80, 1600: 0x90,
}

LIS3DSH_DEV_ID = 0x33
SET_FIFO_CTL = 0x90

FREEFALL_ACCEL = 9.80665 
# SCALE_XY = 0.00048828125 * FREEFALL_ACCEL # 1 / 265 (at 3.3V) mg/LSB
# SCALE_Z  = SCALE_XY

SCALE = 12. * FREEFALL_ACCEL/16.

Accel_Measurement = collections.namedtuple(
    'Accel_Measurement', ('time', 'accel_x', 'accel_y', 'accel_z'))


MIN_MSG_TIME = 0.100

BYTES_PER_SAMPLE = 6
SAMPLES_PER_BLOCK = 8

# Printer class that controls LIS3DSH chip
class LIS3DSH:
    def __init__(self, config):
        self.printer = config.get_printer()
        adxl345.AccelCommandHelper(config, self)
        self.query_rate = 0
        am = {'x': (0, SCALE), 'y': (1, SCALE), 'z': (2, SCALE),
              '-x': (0, -SCALE), '-y': (1, -SCALE), '-z': (2, -SCALE)}
        axes_map = config.getlist('axes_map', ('x','y','z'), count=3)
        if any([a not in am for a in axes_map]):
            raise config.error("Invalid lis3dsh axes_map parameter")
        self.axes_map = [am[a.strip()] for a in axes_map]
        self.data_rate = config.getint('rate', 1600)
        if self.data_rate not in QUERY_RATES:
            raise config.error("Invalid rate parameter: %d" % (self.data_rate,))
        # Measurement storage (accessed from background thread)
        self.lock = threading.Lock()
        self.raw_samples = []
        # Setup mcu sensor_lis3dsh bulk query code
        self.spi = bus.MCU_SPI_from_config(config, 3, default_speed=3000000)
        self.mcu = mcu = self.spi.get_mcu()
        self.oid = oid = mcu.create_oid()
        self.query_lis3dsh_cmd = self.query_lis3dsh_end_cmd = None
        self.query_lis3dsh_status_cmd = None
        mcu.add_config_cmd("config_lis3dsh oid=%d spi_oid=%d"
                           % (oid, self.spi.get_oid()))
        mcu.add_config_cmd("query_lis3dsh oid=%d clock=0 rest_ticks=0"
                           % (oid,), on_restart=True)
        mcu.register_config_callback(self._build_config)
        mcu.register_response(self._handle_lis3dsh_data, "lis3dsh_data", oid)
        # Clock tracking
        self.last_sequence = self.max_query_duration = 0
        self.last_limit_count = self.last_error_count = 0
        self.clock_sync = adxl345.ClockSyncRegression(self.mcu, 640)
        # API server endpoints
        self.api_dump = motion_report.APIDumpHelper(
            self.printer, self._api_update, self._api_startstop, 0.100)
        self.name = config.get_name().split()[-1]
        wh = self.printer.lookup_object('webhooks')
        wh.register_mux_endpoint("lis3dsh/dump_lis3dsh", "sensor", self.name,
                                 self._handle_dump_lis3dsh)
    def _build_config(self):
        cmdqueue = self.spi.get_command_queue()
        self.query_lis3dsh_cmd = self.mcu.lookup_command(
            "query_lis3dsh oid=%c clock=%u rest_ticks=%u", cq=cmdqueue)
        self.query_lis3dsh_end_cmd = self.mcu.lookup_query_command(
            "query_lis3dsh oid=%c clock=%u rest_ticks=%u",
            "lis3dsh_status oid=%c clock=%u query_ticks=%u next_sequence=%hu"
            " buffered=%c fifo=%c limit_count=%hu", oid=self.oid, cq=cmdqueue)
        self.query_lis3dsh_status_cmd = self.mcu.lookup_query_command(
            "query_lis3dsh_status oid=%c",
            "lis3dsh_status oid=%c clock=%u query_ticks=%u next_sequence=%hu"
            " buffered=%c fifo=%c limit_count=%hu", oid=self.oid, cq=cmdqueue)
    def read_reg(self, reg):
        params = self.spi.spi_transfer([reg | REG_MOD_READ, 0x00])
        response = bytearray(params['response'])
        return response[1]
    def set_reg(self, reg, val, minclock=0):
        self.spi.spi_send([reg, val & 0xFF], minclock=minclock)
        stored_val = self.read_reg(reg)
        if stored_val != val:
            raise self.printer.command_error(
                    "Failed to set LIS3DSH register [0x%x] to 0x%x: got 0x%x. "
                    "This is generally indicative of connection problems "
                    "(e.g. faulty wiring) or a faulty lis3dsh chip." % (
                        reg, val, stored_val))
    # Measurement collection
    def is_measuring(self):
        return self.query_rate > 0
    def _handle_lis3dsh_data(self, params):
        with self.lock:
            self.raw_samples.append(params)
    def _extract_samples(self, raw_samples):
        # Load variables to optimize inner loop below
        (x_pos, x_scale), (y_pos, y_scale), (z_pos, z_scale) = self.axes_map
        last_sequence = self.last_sequence
        time_base, chip_base, inv_freq = self.clock_sync.get_time_translation()
        # Process every message in raw_samples
        count = seq = 0
        samples = [None] * (len(raw_samples) * SAMPLES_PER_BLOCK)
        for params in raw_samples:
            seq_diff = (last_sequence - params['sequence']) & 0xffff
            seq_diff -= (seq_diff & 0x8000) << 1
            seq = last_sequence - seq_diff
            d = bytearray(params['data'])
            msg_cdiff = seq * SAMPLES_PER_BLOCK - chip_base

            for i in range(len(d) // BYTES_PER_SAMPLE):
                d_xyz = d[i*BYTES_PER_SAMPLE:(i+1)*BYTES_PER_SAMPLE]
                xlow, xhigh, ylow, yhigh, zlow, zhigh = d_xyz
                # Merge and perform twos-complement

                rx = ((xhigh << 8) | xlow) - ((xhigh & 0x80) << 9)
                ry = ((yhigh << 8) | ylow) - ((yhigh & 0x80) << 9)
                rz = ((zhigh << 8) | zlow) - ((zhigh & 0x80) << 9)
                
                # rx = ((xhigh << 8) | xlow)
                # ry = ((yhigh << 8) | ylow)
                # rz = ((zhigh << 8) | zlow)

                raw_xyz = (rx, ry, rz)

                # x =round((raw_xyz[x_pos] /16)*12*9.80665, 6)
                # y =round((raw_xyz[y_pos] /16)*12*9.80665, 6)
                # z =round((raw_xyz[z_pos] /16)*12*9.80665, 6)

                x = round(raw_xyz[x_pos] * x_scale, 6)
                y = round(raw_xyz[y_pos] * y_scale, 6)
                z = round(raw_xyz[z_pos] * z_scale, 6)
                
                ptime = round(time_base + (msg_cdiff + i) * inv_freq, 6)
                samples[count] = (ptime, x, y, z)
                count += 1
        self.clock_sync.set_last_chip_clock(seq * SAMPLES_PER_BLOCK + i)
        del samples[count:]
        return samples
    def _update_clock(self, minclock=0):
        # Query current state
        for retry in range(5):
            params = self.query_lis3dsh_status_cmd.send([self.oid],
                                                        minclock=minclock)
            fifo = params['fifo'] & 0x7f
            if fifo <= 32:
                break
        else:
            raise self.printer.command_error("Unable to query lis3dsh fifo")
        mcu_clock = self.mcu.clock32_to_clock64(params['clock'])
        sequence = (self.last_sequence & ~0xffff) | params['next_sequence']
        if sequence < self.last_sequence:
            sequence += 0x10000
        self.last_sequence = sequence
        buffered = params['buffered']
        limit_count = (self.last_limit_count & ~0xffff) | params['limit_count']
        if limit_count < self.last_limit_count:
            limit_count += 0x10000
        self.last_limit_count = limit_count
        duration = params['query_ticks']
        if duration > self.max_query_duration:
            # Skip measurement as a high query time could skew clock tracking
            self.max_query_duration = max(2 * self.max_query_duration,
                                          self.mcu.seconds_to_clock(.000005))
            return
        self.max_query_duration = 2 * duration
        msg_count = (sequence * SAMPLES_PER_BLOCK
                     + buffered // BYTES_PER_SAMPLE + fifo)
        # The "chip clock" is the message counter plus .5 for average
        # inaccuracy of query responses and plus .5 for assumed offset
        # of lis3dsh hw processing time.
        chip_clock = msg_count + 1
        self.clock_sync.update(mcu_clock + duration // 2, chip_clock)
    def _update_clock(self, minclock=0):
        # Query current state
        for retry in range(5):
            params = self.query_lis3dsh_status_cmd.send([self.oid],
                                                        minclock=minclock)
            fifo = params['fifo'] & 0x7f
            if fifo <= 32:
                break
        else:
            raise self.printer.command_error("Unable to query lis3dsh fifo")
        mcu_clock = self.mcu.clock32_to_clock64(params['clock'])
        sequence = (self.last_sequence & ~0xffff) | params['next_sequence']
        if sequence < self.last_sequence:
            sequence += 0x10000
        self.last_sequence = sequence
        buffered = params['buffered']
        limit_count = (self.last_limit_count & ~0xffff) | params['limit_count']
        if limit_count < self.last_limit_count:
            limit_count += 0x10000
        self.last_limit_count = limit_count
        duration = params['query_ticks']
        if duration > self.max_query_duration:
            # Skip measurement as a high query time could skew clock tracking
            self.max_query_duration = max(2 * self.max_query_duration,
                                          self.mcu.seconds_to_clock(.000005))
            return
        self.max_query_duration = 2 * duration
        msg_count = (sequence * SAMPLES_PER_BLOCK
                     + buffered // BYTES_PER_SAMPLE + fifo)
        # The "chip clock" is the message counter plus .5 for average
        # inaccuracy of query responses and plus .5 for assumed offset
        # of lis3dsh hw processing time.
        chip_clock = msg_count + 1
        self.clock_sync.update(mcu_clock + duration // 2, chip_clock)
    def _start_measurements(self):
        if self.is_measuring():
            return
        # In case of miswiring, testing LIS3DSH device ID prevents treating
        # noise or wrong signal as a correctly initialized device
        dev_id = self.read_reg(REG_LIS3DH_WHO_AM_I_ADDR)
        logging.info("lis3dsh_dev_id: %x", dev_id)
        if dev_id != LIS3DSH_DEV_ID:
            raise self.printer.command_error(
                "Invalid lis3dsh id (got %x vs %x).\n"
                "This is generally indicative of connection problems\n"
                "(e.g. faulty wiring) or a faulty lis3dsh chip."
                % (dev_id, LIS3DSH_DEV_ID))
        # Setup chip in requested query rate
        dev_data =  0x47 
        self.set_reg(REG_LIS3DH_CTRL_REG1_ADDR &(~(0xC0)), dev_data)
        dev_data =  0x30
        self.set_reg(REG_LIS3DH_CTRL_REG4_ADDR & (~(0xC0)), dev_data)
        dev_data =  0x40
        self.set_reg(REG_LIS3DH_CTRL_REG5_ADDR & (~(0xC0)), dev_data)       #Stream Mode. If the FIFO is full the new sample overwrites the older one
        dev_data = 0x80
        self.set_reg(REG_LIS3DH_FIFO_CTRL & (~(0xC0)), dev_data) 
        # Setup samples
        with self.lock:
            self.raw_samples = []
        # Start bulk reading
        systime = self.printer.get_reactor().monotonic()
        print_time = self.mcu.estimated_print_time(systime) + MIN_MSG_TIME
        reqclock = self.mcu.print_time_to_clock(print_time)
        rest_ticks = self.mcu.seconds_to_clock(4. / self.data_rate)
        self.query_rate = self.data_rate
        self.query_lis3dsh_cmd.send([self.oid, reqclock, rest_ticks],
                                    reqclock=reqclock)
        logging.info("LIS3DSH starting '%s' measurements", self.name)
        # Initialize clock tracking
        self.last_sequence = 0
        self.last_limit_count = self.last_error_count = 0
        self.clock_sync.reset(reqclock, 0)
        self.max_query_duration = 1 << 31
        self._update_clock(minclock=reqclock)
        self.max_query_duration = 1 << 31
    def _finish_measurements(self):
        if not self.is_measuring():
            return
        # Halt bulk reading
        params = self.query_lis3dsh_end_cmd.send([self.oid, 0, 0])
        self.query_rate = 0
        with self.lock:
            self.raw_samples = []
        logging.info("LIS3DSH finished '%s' measurements", self.name)
    # API interface
    def _api_update(self, eventtime):
        self._update_clock()
        with self.lock:
            raw_samples = self.raw_samples
            self.raw_samples = []
        if not raw_samples:
            return {}
        samples = self._extract_samples(raw_samples)
        if not samples:
            return {}
        return {'data': samples, 'errors': self.last_error_count,
                'overflows': self.last_limit_count}
    def _api_startstop(self, is_start):
        if is_start:
            self._start_measurements()
        else:
            self._finish_measurements()
    def _handle_dump_lis3dsh(self, web_request):
        self.api_dump.add_client(web_request)
        hdr = ('time', 'x_acceleration', 'y_acceleration', 'z_acceleration')
        web_request.send({'header': hdr})
    def start_internal_client(self):
        cconn = self.api_dump.add_internal_client()
        return adxl345.AccelQueryHelper(self.printer, cconn)


def load_config(config):
    return LIS3DSH(config)

def load_config_prefix(config):
    return LIS3DSH(config)