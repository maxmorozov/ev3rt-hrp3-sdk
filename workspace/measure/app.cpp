#include "ev3api.h"
#include "app.h"

#include <chrono>
#include <sstream>
#include <cstdio>
#include <vector>
#include <cmath>

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

/**
 * Constants for the self-balance control algorithm.
 */
const uint32_t WAIT_TIME_MS = 4;

static const int ITERATIONS = 100000;

std::vector<std::chrono::nanoseconds> buffer;

//We use this clock to measure intervals, so we need monotonic clock
using clock_type = std::conditional<std::chrono::high_resolution_clock::is_steady,
    std::chrono::high_resolution_clock,
    std::chrono::steady_clock>::type;

static FILE *bt = NULL;


double percentile(const std::vector<std::chrono::nanoseconds>& data, double percentile) {
    double index = data.size()*percentile/100;
    size_t low = std::floor(index);
    size_t high = std::ceil(index);
    if (low == high) {
        return std::chrono::duration_cast<std::chrono::microseconds>(data.at(low)).count();
    }
    double value0 = std::chrono::duration_cast<std::chrono::microseconds>(data.at(low)).count()*(high - index);
    double value1 = std::chrono::duration_cast<std::chrono::microseconds>(data.at(high)).count()*(index - low);

    return value0 + value1;
}

void balance_task(intptr_t unused) {

    buffer.reserve(ITERATIONS);

    clock_type::time_point start = clock_type::now();

    /**
     * Main loop for the self-balance control algorithm
     */
    while(1) {
        std::chrono::nanoseconds time = clock_type::now() - start;
        buffer.emplace_back(time);
        
        if (buffer.size() == ITERATIONS) {
           std::ostringstream oss;

           oss << "min/50%/95%/99%/100% = "
               << std::chrono::duration_cast<std::chrono::microseconds>(buffer.front()).count() << "/"
               << percentile(buffer, 50) << "/"
               << percentile(buffer, 95) << "/"
               << percentile(buffer, 99) << "/"
               << std::chrono::duration_cast<std::chrono::microseconds>(buffer[buffer.size() - 1]).count() << " us\n";
           
			fprintf(bt, "%s\n", oss.str().c_str());

           break;
        }


        tslp_tsk(WAIT_TIME_MS * 1000U);
    }
}


void idle_task(intptr_t unused) {
    while(1) {
    	fprintf(bt, "Press 'h' for usage instructions.\n");
    	tslp_tsk(1000U * 1000U);
    }
}

void main_task(intptr_t unused) {
    // Open Bluetooth file
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    // Start task for self-balancing
    act_tsk(BALANCE_TASK);

    // Start task for printing message while idle
	act_tsk(IDLE_TASK);

    while(1) {
        while (!ev3_bluetooth_is_connected()) tslp_tsk(100U * 1000U);
    	uint8_t c = fgetc(bt);
    	sus_tsk(IDLE_TASK);
    	switch(c) {
    	case 'h':
    		fprintf(bt, "==========================\n");
    		fprintf(bt, "Usage:\n");
    		fprintf(bt, "Press 'i' for idle task\n");
    		fprintf(bt, "Press 'h' for this message\n");
    		fprintf(bt, "==========================\n");
    		break;

    	case 'i':
    		fprintf(bt, "Idle task started.\n");
    		rsm_tsk(IDLE_TASK);
    		break;

    	default:
    		fprintf(bt, "Unknown key '%c' pressed.\n", c);
    	}
    }
}
