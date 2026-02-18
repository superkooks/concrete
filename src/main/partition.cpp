#include "partition.hpp"
#include "adc.h"

Partition::Partition(const PartitionConfig *config) {
    this->name = config->name()->str();
    for (const auto vs_conf : *config->virt_sensors()) {
        this->virt_sensors.push_back(VirtSensor(vs_conf));
    }

    this->armed = false;
    this->armed_at = 0;
    this->alarm_at = 0;
}

std::string& Partition::get_name() {
    return name;
}

void Partition::arm() {
    armed = true;
    armed_at = time(NULL);
    alarm_at = 0;
}

void Partition::disarm() {
    armed = false;
}

// bool Partition::tick(std::vector<PhysSensor> phys_sensors) {
//     time_t cur_time = time(NULL);

//     for (const VirtSensor& vs : virt_sensors) {
//         // Get the physical sensor state that the virtual sensor references
//         SensorState state = SAFE;
//         for (const PhysSensor& ps : phys_sensors) {
//             if (ps.addr == vs.sensor_addr) {
//                 state = ps.state;
//                 break;
//             }
//         }

//         if (state == TAMPER) {
//             return true;

//         } else if (state == TRIGGER) {
//             if (armed_at + vs.exit_delay > cur_time) {
//                 // Ignore because of exit delay
//                 continue;
//             }
            
//             // Check if this sensor should trigger alarm earlier
//             if (alarm_at == 0 || cur_time + vs.entry_delay < alarm_at) {
//                 // Trigger the alarm after the entry delay
//                 alarm_at = cur_time + vs.entry_delay;
//             }
//         }
//     }

//     if (alarm_at != 0 && alarm_at <= cur_time) {
//         return true;
//     }

//     return false;
// }
