#include <string.h> // memcpy
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "spicmds.h" // spidev_transfer


struct lis3dsh {
    struct timer timer;
    uint32_t rest_ticks;
    struct spidev_s *spi;
    uint16_t sequence, limit_count;
    uint8_t flags, data_count;
    uint8_t data[48];
};

enum {
    LIS_HAVE_START = 1<<0, LIS_RUNNING = 1<<1, LIS_PENDING = 1<<2,
};

static struct task_wake lis3dsh_wake;

// Event handler that wakes lis3dsh_task() periodically
static uint_fast8_t
lis3dsh_event(struct timer *timer)
{
    struct lis3dsh *ax = container_of(timer, struct lis3dsh, timer);
    ax->flags |= LIS_PENDING;
    sched_wake_task(&lis3dsh_wake);
    return SF_DONE;
}


void
command_config_lis3dsh(uint32_t *args)
{
    struct lis3dsh *ax = oid_alloc(args[0], command_config_lis3dsh
                                   , sizeof(*ax));
    ax->timer.func = lis3dsh_event;
    ax->spi = spidev_oid_lookup(args[1]);
}
DECL_COMMAND(command_config_lis3dsh, "config_lis3dsh oid=%c spi_oid=%c");


// Report local measurement buffer
static void
lis3dsh_report(struct lis3dsh *ax, uint8_t oid)
{
    sendf("lis3dsh_data oid=%c sequence=%hu data=%*s"
          , oid, ax->sequence, ax->data_count, ax->data);
    ax->data_count = 0;
    ax->sequence++;
}

// Report buffer and fifo status
static void
lis3dsh_status(struct lis3dsh *ax, uint_fast8_t oid
            , uint32_t time1, uint32_t time2, uint_fast8_t fifo)
{
    sendf("lis3dsh_status oid=%c clock=%u query_ticks=%u next_sequence=%hu"
          " buffered=%c fifo=%c limit_count=%hu"
          , oid, time1, time2-time1, ax->sequence
          , ax->data_count, fifo, ax->limit_count);
}


// Helper code to reschedule the lis3dsh_event() timer
static void
lis3dsh_reschedule_timer(struct lis3dsh *ax)
{
    irq_disable();
    ax->timer.waketime = timer_read_time() + ax->rest_ticks;
    sched_add_timer(&ax->timer);
    irq_enable();
}


// Chip registers

#define LIS_CTRL_REG6   0x25        //bit 6:FIFO EN
#define LIS_AR_DATAX0   0x28
#define LIS_FIFO_SRC    0x2F
#define LIS_AM_READ     0x80
#define LIS_AM_MULTI    0x40


// Query accelerometer data
static void
lis3dsh_query(struct lis3dsh *ax, uint8_t oid)
{
    // Read data
    uint8_t msg[9] = { LIS_AR_DATAX0 | LIS_AM_READ | LIS_AM_MULTI, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t *d = &ax->data[ax->data_count];
    spidev_transfer(ax->spi, 1, sizeof(msg), msg);

    uint8_t fifo_empty = msg[8] & 0x20;        //  0=FIFO not empty; !0=FIFO empty
    uint8_t fifo_ovrn = msg[8] & 0x40;         //  0=FIFO is not completely filled; !0=FIFO is completely filled
    // Extract x, y, z measurements   

    d[0] = msg[1]; // x low bits
    d[1] = msg[2]; // x high bits
    d[2] = msg[3]; // y low bits
    d[3] = msg[4]; // y high bits
    d[4] = msg[5]; // z low bits
    d[5] = msg[6]; // z high bits

    ax->data_count += 6;
    if (ax->data_count + 6 > ARRAY_SIZE(ax->data))
        lis3dsh_report(ax, oid);

    // Check fifo status
    if (fifo_ovrn)
        ax->limit_count++;

    // check if we need to run the task again (more packets in fifo?)
    if (!fifo_empty) {
        sched_wake_task(&lis3dsh_wake); // More data in fifo - wake this task again
    } else if (ax->flags & LIS_RUNNING) {
        // Sleep until next check time
        sched_del_timer(&ax->timer);
        ax->flags &= ~LIS_PENDING;
        lis3dsh_reschedule_timer(ax);
    }
}


// Startup measurements
static void
lis3dsh_start(struct lis3dsh *ax, uint8_t oid)
{
    sched_del_timer(&ax->timer);
    ax->flags = LIS_RUNNING;
    uint8_t msg_fifo[2] = { LIS_CTRL_REG6, 0x40 };                //ENABLE FIFO
    spidev_transfer(ax->spi, 0, sizeof(msg_fifo), msg_fifo);
    lis3dsh_reschedule_timer(ax);
}


// End measurements
static void
lis3dsh_stop(struct lis3dsh *ax, uint8_t oid)
{
    // Disable measurements
    sched_del_timer(&ax->timer);
    ax->flags = 0;

    uint8_t msg_fifo[2] = { LIS_CTRL_REG6, 0x00 };                //DISABLE FIFO
    uint32_t end1_time = timer_read_time();
    spidev_transfer(ax->spi, 0, sizeof(msg_fifo), msg_fifo);
    uint32_t end2_time = timer_read_time();

    // Drain any measurements still in fifo
    uint8_t msg[2] = {LIS_FIFO_SRC | LIS_AM_READ , 0x00 };              
    spidev_transfer(ax->spi, 1, sizeof(msg), msg);

    uint8_t fifo_status = msg[1] & 0x20;

    while (!fifo_status) {
        lis3dsh_query(ax, oid);
        msg[2] = {LIS_FIFO_SRC | LIS_AM_READ , 0x00 }; 
        spidev_transfer(ax->spi, 1, sizeof(msg), msg);
        fifo_status = msg[1] & 0x20;
    }
    // Report final data
    if (ax->data_count)
        lis3dsh_report(ax, oid);
    lis3dsh_status(ax, oid, end1_time, end2_time, fifo_status);
}


void
command_query_lis3dsh(uint32_t *args)
{
    struct lis3dsh *ax = oid_lookup(args[0], command_config_lis3dsh);

    if (!args[2]) {
        // End measurements
        lis3dsh_stop(ax, args[0]);
        return;
    }
    // Start new measurements query
    sched_del_timer(&ax->timer);
    ax->timer.waketime = args[1];
    ax->rest_ticks = args[2];
    ax->flags = LIS_HAVE_START;
    ax->sequence = ax->limit_count = 0;
    ax->data_count = 0;
    sched_add_timer(&ax->timer);
}
DECL_COMMAND(command_query_lis3dsh,
             "query_lis3dsh oid=%c clock=%u rest_ticks=%u");

void
command_query_lis3dsh_status(uint32_t *args)
{
    struct lis3dsh *ax = oid_lookup(args[0], command_config_lis3dsh);
    uint8_t msg[2] = { LIS_FIFO_SRC | LIS_AM_READ, 0x00 };
    uint32_t time1 = timer_read_time();
    spidev_transfer(ax->spi, 1, sizeof(msg), msg);
    uint32_t time2 = timer_read_time();
    lis3dsh_status(ax, args[0], time1, time2, msg[1]);
}
DECL_COMMAND(command_query_lis3dsh_status, "query_lis3dsh_status oid=%c");


void
lis3dsh_task(void)
{
    if (!sched_check_wake(&lis3dsh_wake))
        return;
    uint8_t oid;
    struct lis3dsh *ax;
    foreach_oid(oid, ax, command_config_lis3dsh) {
        uint_fast8_t flags = ax->flags;
        if (!(flags & LIS_PENDING))
            continue;
        if (flags & LIS_HAVE_START)
            lis3dsh_start(ax, oid);
        else
            lis3dsh_query(ax, oid);
    }
}
DECL_TASK(lis3dsh_task);