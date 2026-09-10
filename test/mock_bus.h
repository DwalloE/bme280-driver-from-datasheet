/*
 * mock_bus.h - fault-injection I2C bus for host tests.
 *
 * A 256-byte register file plus a scripted fault: WHICH failure and on
 * WHICH bus operation it fires. That second knob is what lets the suite
 * walk a fault along every transaction the driver makes and prove each
 * call site's error branch executes - "handles errors" is a claim about
 * every read and write, not about the first one.
 */
#ifndef MOCK_BUS_H
#define MOCK_BUS_H

#include <stdint.h>
#include "../main/bme280.h"

typedef enum {
    MOCK_FAULT_NONE = 0,
    MOCK_FAULT_NACK_ADDR,   /* address byte not ACKed */
    MOCK_FAULT_NACK_DATA,   /* ACKs the address, dies mid-transfer */
    MOCK_FAULT_TIMEOUT,     /* slave stretches SCL past the master limit */
    MOCK_FAULT_BUSY,        /* bus owned elsewhere / arbitration lost */
    MOCK_FAULT_UNKNOWN,     /* an error code the driver has never met */
    MOCK_FAULT_STUCK_HIGH,  /* the trap: SDA stuck high, every read is
                               0xFF and the bus status is CLEAN */
} mock_fault_t;

typedef struct {
    uint8_t regs[256];

    mock_fault_t fault;
    int fault_on_op;     /* 0-based bus-operation index the fault fires on;
                            -1 = fire on every operation */
    int op_count;        /* operations performed so far */

    int status_busy_reads;   /* status (0xF3) reports im_update/measuring
                                set for this many reads, then clear */
    int forced_triggers;     /* ctrl_meas writes with mode==forced seen */
} mock_bus_t;

/* Reset the mock to a healthy sensor: chip id 0x60, a real device's
 * calibration image in both banks, plausible raw data registers. */
void mock_bus_init(mock_bus_t *m);

/* The bme280_bus_t the driver is handed. ctx must be a mock_bus_t*. */
extern const bme280_bus_t mock_bus_ops;

/* Helpers for scripting register content. */
void mock_set_calib_bank0(mock_bus_t *m, const uint8_t bank0[26]);
void mock_set_calib_bank1(mock_bus_t *m, const uint8_t bank1[7]);
void mock_set_raw(mock_bus_t *m, int32_t adc_T, int32_t adc_P, int32_t adc_H);

#endif /* MOCK_BUS_H */
