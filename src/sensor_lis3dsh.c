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
    uint8_t flags, data_count, fifo_disable;
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

#define LIS_CTRL_REG5   0x24        //bit 6:FIFO EN
#define LIS_AR_DATAX0   0x28
#define LIS_FIFO_CTRL   0x2E
#define LIS_FIFO_SRC    0x2F
#define LIS_AM_READ     0x80
#define LIS_AM_MULTI    0x40


// Query accelerometer data
static void
lis3dsh_query(struct lis3dsh *ax, uint8_t oid)
{
    // uint8_t msg[256] = {0};
    uint8_t msg[7] = {0};
    uint8_t fifo[2] = {LIS_FIFO_SRC| LIS_AM_READ , 0};
    // uint8_t reg5[2] = {LIS_CTRL_REG5 , 0};
    // uint8_t ctrl[2] = {LIS_FIFO_CTRL , 0};

    uint8_t fifo_empty,fifo_ovrn,fifo_num = 0;
    spidev_transfer(ax->spi, 1, sizeof(fifo), fifo);
    fifo_num = fifo[1]&0x1f;

    msg[0] = LIS_AR_DATAX0 | LIS_AM_READ | LIS_AM_MULTI ;
    uint8_t *d = &ax->data[ax->data_count];
    // spidev_transfer(ax->spi, 1, (fifo_num*6+1), msg);
    spidev_transfer(ax->spi, 1, sizeof(msg), msg);

    // spidev_transfer(ax->spi, 0, sizeof(reg5), reg5);
    // spidev_transfer(ax->spi, 0, sizeof(ctrl), ctrl);

    // d[0] = msg[fifo_num*5+1]; // x low bits
    // d[1] = msg[fifo_num*5+2]; // x high bits
    // d[2] = msg[fifo_num*5+3]; // y low bits
    // d[3] = msg[fifo_num*5+4]; // y high bits
    // d[4] = msg[fifo_num*5+5]; // z low bits
    // d[5] = msg[fifo_num*5+6]; // z high bits

    d[0] = msg[1]; // x low bits
    d[1] = msg[2]; // x high bits
    d[2] = msg[3]; // y low bits
    d[3] = msg[4]; // y high bits
    d[4] = msg[5]; // z low bits
    d[5] = msg[6]; // z high bits

    fifo[1]=0;
    spidev_transfer(ax->spi, 1, sizeof(fifo), fifo);
    fifo_empty = fifo[1]&0x20;
    fifo_ovrn = fifo[1]&0x40;

    ax->data_count += 6;
    if (ax->data_count + 6 > ARRAY_SIZE(ax->data))
        lis3dsh_report(ax, oid);

    // Check fifo status
    if (fifo_ovrn)
        ax->limit_count++;

    // check if we need to run the task again (more packets in fifo?)
    if (!fifo_empty&&!(ax->fifo_disable)) {
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
    ax->fifo_disable = 0;
    uint8_t reg5[2] = {LIS_CTRL_REG5 , 0x40};
    uint8_t ctrl[2] = {LIS_FIFO_CTRL , 0x80};
    spidev_transfer(ax->spi, 0, sizeof(ctrl), ctrl);
    spidev_transfer(ax->spi, 0, sizeof(reg5), reg5); 
    lis3dsh_reschedule_timer(ax);
}


// End measurements
static void
lis3dsh_stop(struct lis3dsh *ax, uint8_t oid)
{
    // Disable measurements
    sched_del_timer(&ax->timer);
    ax->flags = 0;
    // Drain any measurements still in fifo
    ax->fifo_disable = 1;
    lis3dsh_query(ax, oid);

    uint8_t reg5[2] = {LIS_CTRL_REG5 , 0};
    uint8_t ctrl[2] = {LIS_FIFO_CTRL , 0};
    uint32_t end1_time = timer_read_time();
    spidev_transfer(ax->spi, 0, sizeof(reg5), reg5);
    spidev_transfer(ax->spi, 0, sizeof(ctrl), ctrl);
    uint32_t end2_time = timer_read_time();

    uint8_t msg[2] = { LIS_FIFO_SRC | LIS_AM_READ , 0};              
    spidev_transfer(ax->spi, 1, sizeof(msg), msg);
    uint8_t fifo_status = msg[1];

    //Report final data
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
    ax->fifo_disable = 0;
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