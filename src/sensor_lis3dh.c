#include <string.h> // memcpy
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "spicmds.h" // spidev_transfer


struct lis3dh {
    struct timer timer;
    uint32_t rest_ticks;
    struct spidev_s *spi;
    uint16_t sequence, limit_count;
    uint8_t flags, data_count, fifo_disable;
    uint8_t data[48];
};

enum {
    LIS_HAVE_START = 1<<0, LIS_RUNNING = 1<<1, LIS_PENDING = 1<<2,
};

static struct task_wake lis3dh_wake;

// Event handler that wakes lis3dh_task() periodically
static uint_fast8_t
lis3dh_event(struct timer *timer)
{
    struct lis3dh *ax = container_of(timer, struct lis3dh, timer);
    ax->flags |= LIS_PENDING;
    sched_wake_task(&lis3dh_wake);
    return SF_DONE;
}


void
command_config_lis3dh(uint32_t *args)
{
    struct lis3dh *ax = oid_alloc(args[0], command_config_lis3dh
                                   , sizeof(*ax));
    ax->timer.func = lis3dh_event;
    ax->spi = spidev_oid_lookup(args[1]);
}
DECL_COMMAND(command_config_lis3dh, "config_lis3dh oid=%c spi_oid=%c");


// Report local measurement buffer
static void
lis3dh_report(struct lis3dh *ax, uint8_t oid)
{
    sendf("lis3dh_data oid=%c sequence=%hu data=%*s"
          , oid, ax->sequence, ax->data_count, ax->data);
    ax->data_count = 0;
    ax->sequence++;
}

// Report buffer and fifo status
static void
lis3dh_status(struct lis3dh *ax, uint_fast8_t oid
            , uint32_t time1, uint32_t time2, uint_fast8_t fifo)
{
    sendf("lis3dh_status oid=%c clock=%u query_ticks=%u next_sequence=%hu"
          " buffered=%c fifo=%c limit_count=%hu"
          , oid, time1, time2-time1, ax->sequence
          , ax->data_count, fifo, ax->limit_count);
}


// Helper code to reschedule the lis3dh_event() timer
static void
lis3dh_reschedule_timer(struct lis3dh *ax)
{
    irq_disable();
    ax->timer.waketime = timer_read_time() + ax->rest_ticks;
    sched_add_timer(&ax->timer);
    irq_enable();
}


// Chip registers

#define LIS_CTRL_REG5   0x24        //bit 6:FIFO EN
#define LIS_AR_DATAX0   0x28
#define LIS_FIFO_CTRL   0x2E
#define LIS_FIFO_SRC    0x2F
#define LIS_AM_READ     0x80
#define LIS_AM_MULTI    0x40


// Query accelerometer data
static void
lis3dh_query(struct lis3dh *ax, uint8_t oid)
{
    uint8_t msg[7] = {0};
    uint8_t fifo[2] = {LIS_FIFO_SRC| LIS_AM_READ , 0};
    uint8_t fifo_empty,fifo_ovrn = 0;

    msg[0] = LIS_AR_DATAX0 | LIS_AM_READ | LIS_AM_MULTI ;
    uint8_t *d = &ax->data[ax->data_count];

    spidev_transfer(ax->spi, 1, sizeof(msg), msg);

    spidev_transfer(ax->spi, 1, sizeof(fifo), fifo);
    fifo_empty = fifo[1]&0x20;
    fifo_ovrn = fifo[1]&0x40;

    d[0] = msg[1]; // x low bits
    d[1] = msg[2]; // x high bits
    d[2] = msg[3]; // y low bits
    d[3] = msg[4]; // y high bits
    d[4] = msg[5]; // z low bits
    d[5] = msg[6]; // z high bits

    ax->data_count += 6;
    if (ax->data_count + 6 > ARRAY_SIZE(ax->data))
        lis3dh_report(ax, oid);

    // Check fifo status
    if (fifo_ovrn)
        ax->limit_count++;

    // check if we need to run the task again (more packets in fifo?)
    if (!fifo_empty&&!(ax->fifo_disable)) {
        sched_wake_task(&lis3dh_wake); // More data in fifo - wake this task again
    } else if (ax->flags & LIS_RUNNING) {
        // Sleep until next check time
        sched_del_timer(&ax->timer);
        ax->flags &= ~LIS_PENDING;
        lis3dh_reschedule_timer(ax);
    }
}


// Startup measurements
static void
lis3dh_start(struct lis3dh *ax, uint8_t oid)
{
    sched_del_timer(&ax->timer);
    ax->flags = LIS_RUNNING;
    ax->fifo_disable = 0;
    uint8_t reg5[2] = {LIS_CTRL_REG5 , 0x40};
    uint8_t ctrl[2] = {LIS_FIFO_CTRL , 0x80};
    spidev_transfer(ax->spi, 0, sizeof(ctrl), ctrl);
    spidev_transfer(ax->spi, 0, sizeof(reg5), reg5); 
    lis3dh_reschedule_timer(ax);
}


// End measurements
static void
lis3dh_stop(struct lis3dh *ax, uint8_t oid)
{
    // Disable measurements
    sched_del_timer(&ax->timer);
    ax->flags = 0;
    // Drain any measurements still in fifo
    ax->fifo_disable = 1;
    lis3dh_query(ax, oid);

    uint8_t reg5[2] = {LIS_CTRL_REG5 , 0};
    uint8_t ctrl[2] = {LIS_FIFO_CTRL , 0};
    uint32_t end1_time = timer_read_time();
    spidev_transfer(ax->spi, 0, sizeof(reg5), reg5);
    spidev_transfer(ax->spi, 0, sizeof(ctrl), ctrl);
    uint32_t end2_time = timer_read_time();

    uint8_t msg[2] = { LIS_FIFO_SRC | LIS_AM_READ , 0};              
    spidev_transfer(ax->spi, 1, sizeof(msg), msg);
    uint8_t fifo_status = msg[1]&0x1f;

    //Report final data
    if (ax->data_count)
        lis3dh_report(ax, oid);
    lis3dh_status(ax, oid, end1_time, end2_time, fifo_status);
}


void
command_query_lis3dh(uint32_t *args)
{
    struct lis3dh *ax = oid_lookup(args[0], command_config_lis3dh);

    if (!args[2]) {
        // End measurements
        lis3dh_stop(ax, args[0]);
        return;
    }
    // Start new measurements query
    sched_del_timer(&ax->timer);
    ax->timer.waketime = args[1];
    ax->rest_ticks = args[2];
    ax->flags = LIS_HAVE_START;
    ax->sequence = ax->limit_count = 0;
    ax->data_count = 0;
    ax->fifo_disable = 0;
    sched_add_timer(&ax->timer);
}
DECL_COMMAND(command_query_lis3dh,
             "query_lis3dh oid=%c clock=%u rest_ticks=%u");

void
command_query_lis3dh_status(uint32_t *args)
{
    struct lis3dh *ax = oid_lookup(args[0], command_config_lis3dh);
    uint8_t msg[2] = { LIS_FIFO_SRC | LIS_AM_READ, 0x00 };
    uint32_t time1 = timer_read_time();
    spidev_transfer(ax->spi, 1, sizeof(msg), msg);
    uint32_t time2 = timer_read_time();
    lis3dh_status(ax, args[0], time1, time2, msg[1]&0x1f);
}
DECL_COMMAND(command_query_lis3dh_status, "query_lis3dh_status oid=%c");


void
lis3dh_task(void)
{
    if (!sched_check_wake(&lis3dh_wake))
        return;
    uint8_t oid;
    struct lis3dh *ax;
    foreach_oid(oid, ax, command_config_lis3dh) {
        uint_fast8_t flags = ax->flags;
        if (!(flags & LIS_PENDING))
            continue;
        if (flags & LIS_HAVE_START)
            lis3dh_start(ax, oid);
        else
            lis3dh_query(ax, oid);
    }
}
DECL_TASK(lis3dh_task);