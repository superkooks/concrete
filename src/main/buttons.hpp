#ifndef BUTTONS_H_
#define BUTTONS_H_

#include <stdint.h>

class Debounced {
    private:
    uint32_t delay;
    void (*callback)(void *);
    uint32_t last_call;
    bool cur_value;

    public:
    Debounced(int delay_ms, void (*callback)(void *));
    void update(bool next_value, void *arg);
};

void init_buttons(void (*callback)(void *));
void poll_buttons();

#endif
