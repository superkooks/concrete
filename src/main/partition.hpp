#ifndef PARTITION_H_
#define PARTITION_H_

#include <string>
#include <vector>
#include "conf_generated.h"
#include "virt_sensor.hpp"

class Partition {
    private:
    std::string name;
    bool armed;

    time_t armed_at;
    time_t alarm_at; // used for entry timers

    std::vector<VirtSensor> virt_sensors;
    // std::vector<Keypad> keypads;
    // std::vector<Output> outputs;

    public:
    Partition(const PartitionConfig *config);

    std::string& get_name();
    void disarm();
    void arm();
    // bool tick(std::vector<PhysSensor> phys_sensors);
};

#endif
