#include "ev3api.h"
#include "app.h"

#include <inttypes.h>
#include <chrono>
#include <cstdio>
#include <array>
#include <cmath>
#include <algorithm>
#include <numeric>

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

static const int ITERATIONS = 100000;

//We use this clock to measure intervals, so we need monotonic clock
using clock_type = std::conditional<std::chrono::high_resolution_clock::is_steady,
    std::chrono::high_resolution_clock,
    std::chrono::steady_clock>::type;

// logger
static FILE *bt = NULL;

class stats {
    std::array<std::chrono::nanoseconds, ITERATIONS> buffer{};
    std::size_t buffer_size;
    clock_type::time_point start_time;

    double percentile(double percentile) {
        double index = buffer.size()*percentile/100;
        size_t low = std::floor(index);
        size_t high = std::ceil(index);
        if (low == high) {
            return std::chrono::duration_cast<std::chrono::microseconds>(buffer.at(low)).count();
        }
        double value0 = std::chrono::duration_cast<std::chrono::microseconds>(buffer.at(low)).count()*(high - index);
        double value1 = std::chrono::duration_cast<std::chrono::microseconds>(buffer.at(high)).count()*(index - low);

        return value0 + value1;
    }

    static double sqr(double v) {
        return v*v;
    }

public:
    void start() {
        buffer_size = 0;

        start_time = clock_type::now();
    }

    void sample() {
        clock_type::time_point now = clock_type::now();
        buffer[buffer_size] = now - start_time;
        start_time = now;
        ++buffer_size;
    }

    size_t size() const {
        return buffer_size;
    }

    void report() {
        std::make_heap(buffer.begin(), buffer.end());
        std::sort_heap(buffer.begin(), buffer.end());

        auto total = std::accumulate(buffer.begin(), buffer.end(), std::chrono::nanoseconds{0}, [](auto acc, auto v){return acc + v;});
        double mean = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(total).count())/buffer.size();

        double stddev = std::sqrt(std::accumulate(buffer.begin(), buffer.end(), 0.0, 
                [=](auto acc, auto v){return acc + sqr(std::chrono::duration_cast<std::chrono::microseconds>(v).count() - mean);})/(buffer.size() - 1));

       	fprintf(bt, "avg/stddev = %g/%g us\n", mean, stddev);

        double p50  = percentile(50);
        double p95  = percentile(95);
        double p99  = percentile(99);
        double p999 = percentile(99.9);

       	fprintf(bt, "min/50%%/95%%/99%%/99.9%%/100%% = %" PRId64 "/%g/%g/%g/%g/%" PRId64 " us\n", 
            std::chrono::duration_cast<std::chrono::microseconds>(buffer.front()).count(),
            p50, p95, p99, p999,
            std::chrono::duration_cast<std::chrono::microseconds>(buffer[buffer.size() - 1]).count()
        );
    }

};


stats data;

void balance_task(intptr_t unused) {
    while(data.size() < ITERATIONS) {
    	slp_tsk();

        data.sample();
    }
    data.report();

    SVC_PERROR(stp_cyc(CYC1));

	ext_tsk();
}


void idle_task(intptr_t unused) {
    while(1) {
    	fprintf(bt, "Press 'h' for usage instructions. Buffer size=%d\n", data.size());
    	tslp_tsk(1000U * 1000U);
    }
}

void main_task(intptr_t unused) {
    // Open Bluetooth file
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

	act_tsk(BALANCE_TASK);

    data.start();

    // Start task for self-balancing
    SVC_PERROR(sta_cyc(CYC1));

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
