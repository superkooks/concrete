#include "adc.h"
#include "partition.h"
#include "conf.pb-c.h"
#include <string.h>

Partition *new_partition_from_config(PartitionConfig *conf) {
    Partition *part = malloc(sizeof(Partition));
    
    part->name = malloc(strlen(conf->name) + 1);
    strcpy(part->name, conf->name);
    part->armed = false;

    part->armed_at = 0;
    part->alarm_at = 0;

    part->n_virt_sensors = conf->n_virt_sensors;
    part->virt_sensors = malloc(sizeof(VirtSensor*) * conf->n_virt_sensors);
    for (int i = 0; i < conf->n_virt_sensors; i++) {
        part->virt_sensors[i] = malloc(sizeof(VirtSensor));
        memcpy(part->virt_sensors[i], conf->virt_sensors[i], sizeof(VirtSensor));
    }

    part->n_keypads = conf->n_keypads;
    part->keypads = malloc(sizeof(Keypad*) * conf->n_virt_sensors);
    for (int i = 0; i < conf->n_keypads; i++) {
        part->keypads[i] = malloc(sizeof(Keypad));
        memcpy(part->keypads[i], conf->keypads[i], sizeof(Keypad));
    }

    part->n_outputs = conf->n_outputs;
    part->outputs = malloc(sizeof(Output*) * conf->n_outputs);
    for (int i = 0; i < conf->n_outputs; i++) {
        part->outputs[i] = malloc(sizeof(Output));
        memcpy(part->outputs[i], conf->outputs[i], sizeof(Output));
    }

    return part;
}

void free_partition(Partition *part) {
    free(part->name);

    for (int i = 0; i < part->n_virt_sensors; i++) {
        free(part->virt_sensors[i]);
    }
    free(part->virt_sensors);

    for (int i = 0; i < part->n_keypads; i++) {
        free(part->keypads[i]);
    }
    free(part->keypads);

    for (int i = 0; i < part->n_outputs; i++) {
        free(part->outputs[i]);
    }
    free(part->outputs);

    free(part);
}

void arm_partition(Partition *part) {
    time(&part->armed_at);
    part->alarm_at = 0;
    part->armed = true;
}

bool scan_partition(Partition *part, int n_phys_sensors, PhysSensor *sensors) {
    time_t cur_time;
    time(&cur_time);

    for (int i = 0; i < part->n_virt_sensors; i++) {
        VirtSensor *vsensor = part->virt_sensors[i];

        // Get the physical sensor state that the virtual sensor references
        SensorState state = SAFE;
        for (int j = 0; j < n_phys_sensors; j++) {
            if (sensors[j].addr == vsensor->sensor_addr) {
                state = sensors[j].state;
                break;
            }
        }

        if (state == TAMPER) {
            return true;

        } else if (state == TRIGGER) {
            if (part->armed_at + vsensor->exit_delay > cur_time) {
                // Ignore because of exit delay
                continue;
            }
            
            // Check if this sensor should trigger alarm earlier
            if (part->alarm_at == 0 || cur_time + vsensor->entry_delay < part->alarm_at) {
                // Trigger the alarm after the entry delay
                part->alarm_at = cur_time + vsensor->entry_delay;
            }
        }
    }

    if (part->alarm_at != 0 && part->alarm_at <= cur_time) {
        return true;
    }

    return false;
}
