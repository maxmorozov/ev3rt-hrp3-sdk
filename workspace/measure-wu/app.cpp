#include "ev3api.h"
#include "app.h"

#include <inttypes.h>
#include <chrono>
#include <sstream>
#include <cstdio>
#include <vector>
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

class stats {
    std::vector<std::chrono::nanoseconds> buffer;
    clock_type::time_point start_time;

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

    static double sqr(double v) {
        return v*v;
    }

public:
    void start() {
        buffer.clear();
        buffer.reserve(ITERATIONS);

        start_time = clock_type::now();
    }

    void sample() {
        clock_type::time_point now = clock_type::now();
        buffer.emplace_back(now - start_time);
        start_time = now;
    }

    size_t size() const {
        return buffer.size();
    }

    std::string report() {
        std::sort(buffer.begin(), buffer.end());

        auto total = std::accumulate(buffer.begin(), buffer.end(), std::chrono::nanoseconds{0}, [](auto acc, auto v){return acc + v;});
        double mean = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(total).count())/buffer.size();

        double stddev = std::sqrt(std::accumulate(buffer.begin(), buffer.end(), 0.0, 
                [=](auto acc, auto v){return acc + sqr(std::chrono::duration_cast<std::chrono::microseconds>(v).count() - mean);})/(buffer.size() - 1));

        std::ostringstream oss;

        oss << "avg/stddev = "
            << mean << "us/" << stddev << "us\n"
            << "min/50%/95%/99%/99.9%/100% = "
            << std::chrono::duration_cast<std::chrono::microseconds>(buffer.front()).count() << "/"
            << percentile(buffer, 50) << "/"
            << percentile(buffer, 95) << "/"
            << percentile(buffer, 99) << "/"
            << percentile(buffer, 99.9) << "/"
            << std::chrono::duration_cast<std::chrono::microseconds>(buffer[buffer.size() - 1]).count() << " us\n";

        return oss.str();
    }

};


stats data;

static FILE *bt = NULL;

void balance_task(intptr_t unused) {
    while(data.size() < ITERATIONS) {
    	slp_tsk();

        data.sample();
    }
   	fprintf(bt, "%s\n", data.report().c_str());

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
