#ifndef VIRT_SENSOR_H_
#define VIRT_SENSOR_H_

#include "conf_generated.h"

class VirtSensor {
    public:
    uint32_t sensor_addr;

    int32_t entry_delay;
    int32_t exit_delay;

    bool chime;
    bool tamper_armed;
    bool tamper_disarmed;

    VirtSensor(const VirtSensorConfig *config);
};

#endif
