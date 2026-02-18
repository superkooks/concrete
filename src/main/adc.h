#include <inttypes.h>
#include "driver/spi_common.h"
#include "driver/spi_master.h"

void init_adcs();
void read_adcs(uint8_t buf[16]);
void read_adc(spi_device_handle_t adc, uint8_t buf[8]);

typedef enum {
    SAFE,
    TAMPER,
    TRIGGER
} SensorState;

typedef struct {
    int16_t addr;
    char *name;
    SensorState state;
} PhysSensor;
