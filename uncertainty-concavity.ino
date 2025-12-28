// Adapted from https://github.com/oamodular/uncertainty/blob/main/software/stable/uncertainty/uncertainty.ino
// Original license: https://github.com/oamodular/uncertainty/blob/main/LICENSE.md
// License for this code: https://creativecommons.org/licenses/by-sa/4.0/

#include <Arduino.h>
#include <vector>
#include <math.h>
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"

using namespace std;

// number of output gates
#define NUM_GATES 8
#define TARGET_SAMPLE_RATE (40000)
#define TIMER_INTERVAL ((int)(1000000.0/TARGET_SAMPLE_RATE))
#define SAMPLE_RATE (1000000.0/TIMER_INTERVAL)

// ADC input pin
int inputPin = 26;
// hold pins for gates
int gatePins[] = {27,28,29,0,3,4,2,1};

double debugVal = -50;

template <size_t SIZE>
class RingBuffer {
private:
  double _data[SIZE];
  size_t _start = 0;
  size_t _size = 0;

public:
  RingBuffer() {}

  double operator[](int index) const {
    return _data[(_start + index) % SIZE];
  }

  void put(const double& value) {
    if (_size < SIZE) {
      _data[(_start + _size) % SIZE] = value;
      _size += 1;
    }
    else {
      _data[_start] = value;
      _start = (_start + 1) % SIZE;
    }
  }

  double last() const {
    return this->operator[](_size-1);
  }

  double average() const {
    double sum = 0.0;
    for (int i=0; i<_size; i++) {
      sum += this->operator[](i);
    }
    return sum / _size;
  }

  int pos_count(const double epsilon, const int sampleCount) const {
    int count = 0;
    for (int i=max(0, _size - sampleCount); i<_size; i++) {
      if (this->operator[](i) > epsilon) {
        count++;
      }
    }
    return count;
  }

  int neg_count(double epsilon, const int sampleCount) const {
    int count = 0;
    for (int i=max(0, _size - sampleCount); i<_size; i++) {
      if (this->operator[](i) < -epsilon) {
        count++;
      }
    }
    return count;
  }

  size_t size() const {
    return _size;
  }

  bool is_full() const {
    return _size == SIZE;
  }

  void clear() {
    _size = 0;
  }
};

// filters perform both smoothing and derivative calculations
// a hann window is used as a smoothing filter (f0), and its derivatives are used to generate derivative filters (f1/f2/f3)
// this performed better than other methods I tried with smoothing and then performing derivative calculations

#define FILTER_SIZE_SMALL 11
#define FILTER_SIZE_LARGE 50

class FIRFilter {
private:
  int _length;
  double* _coefficients;

public:
  FIRFilter(const int length, const vector<double> coef) {
    _length = length;
    _coefficients = new double[_length];
    for (int i=0; i<length; i++) {
      _coefficients[i] = coef[i];
    }
  }

  double process(const RingBuffer<FILTER_SIZE_LARGE>& signal) {
    double value = 0.0;
    for (int i=0; i<_length; i++) {
      value += _coefficients[i] * signal[_length - i - 1];
    }
    return value;
  } 
};

// length 11
FIRFilter f0_small(FILTER_SIZE_SMALL, {
  0, 0.0493116, 0.17841104, 0.33798673, 0.46708618, 0.51639778, 0.46708618, 0.33798673, 0.17841104, 0.0493116, 0
});
FIRFilter f1_small(FILTER_SIZE_SMALL, {
  2.533706e-16, 0.26286556, 0.4253254, 0.4253254, 0.26286556, -0, -0.26286556, -0.4253254, -0.4253254, -0.26286556, -2.533706e-16
});
FIRFilter f2_small(FILTER_SIZE_SMALL, {
  0.40824829, 0.3302798, 0.12615566, -0.12615566, -0.3302798, -0.40824829, -0.3302798, -0.12615566, 0.12615566, 0.3302798, 0.40824829
});
FIRFilter f3_small(FILTER_SIZE_SMALL, {
  -2.533706e-16, -0.26286556, -0.4253254, -0.4253254, -0.26286556, 0, 0.26286556, 0.4253254, 0.4253254, 0.26286556, 2.533706e-16
});

// length 50
FIRFilter f0_large(FILTER_SIZE_LARGE, {
  0, 0.00095763223, 0.0038148046, 0.0085246025, 0.015009691, 0.023163586, 0.032852399, 0.043917041, 0.056175832, 0.069427481, 0.083454397, 0.098026259, 0.1129038, 0.12784272, 0.14259774, 0.15692657, 0.17059393, 0.18337541, 0.19506114, 0.20545923, 0.21439895, 0.22173351, 0.22734248, 0.23113375, 0.23304508, 0.23304508, 0.23113375, 0.22734248, 0.22173351, 0.21439895, 0.20545923, 0.19506114, 0.18337541, 0.17059393, 0.15692657, 0.14259774, 0.12784272, 0.1129038, 0.098026259, 0.083454397, 0.069427481, 0.056175832, 0.043917041, 0.032852399, 0.023163586, 0.015009691, 0.0085246025, 0.0038148046, 0.00095763223, 0
});
FIRFilter f1_large(FILTER_SIZE_LARGE, {
  2.4741602e-17, 0.025835088, 0.051245965, 0.075815384, 0.099139917, 0.12083657, 0.1405491, 0.15795381, 0.17276493, 0.18473925, 0.19368015, 0.19944084, 0.20192671, 0.20109695, 0.19696518, 0.18959925, 0.17912011, 0.16569982, 0.14955874, 0.13096191, 0.11021469, 0.087657753, 0.063661474, 0.038619875, 0.012944139, -0.012944139, -0.038619875, -0.063661474, -0.087657753, -0.11021469, -0.13096191, -0.14955874, -0.16569982, -0.17912011, -0.18959925, -0.19696518, -0.20109695, -0.20192671, -0.19944084, -0.19368015, -0.18473925, -0.17276493, -0.15795381, -0.1405491, -0.12083657, -0.099139917, -0.075815384, -0.051245965, -0.025835088, -2.4741602e-17
});
FIRFilter f2_large(FILTER_SIZE_LARGE, {
  0.19802951, 0.19640369, 0.19155293, 0.18355687, 0.17254681, 0.15870355, 0.14225437, 0.12346938, 0.10265703, 0.080159047, 0.056344858, 0.031605489, 0.0063471582, -0.019015393, -0.044065711, -0.068392472, -0.09159623, -0.11329598, -0.13313541, -0.15078877, -0.16596618, -0.17841842, -0.18794104, -0.19437768, -0.19762264, -0.19762264, -0.19437768, -0.18794104, -0.17841842, -0.16596618, -0.15078877, -0.13313541, -0.11329598, -0.09159623, -0.068392472, -0.044065711, -0.019015393, 0.0063471582, 0.031605489, 0.056344858, 0.080159047, 0.10265703, 0.12346938, 0.14225437, 0.15870355, 0.17254681, 0.18355687, 0.19155293, 0.19640369, 0.19802951
});
FIRFilter f3_large(FILTER_SIZE_LARGE, {
  -2.4741602e-17, -0.025835088, -0.051245965, -0.075815384, -0.099139917, -0.12083657, -0.1405491, -0.15795381, -0.17276493, -0.18473925, -0.19368015, -0.19944084, -0.20192671, -0.20109695, -0.19696518, -0.18959925, -0.17912011, -0.16569982, -0.14955874, -0.13096191, -0.11021469, -0.087657753, -0.063661474, -0.038619875, -0.012944139, 0.012944139, 0.038619875, 0.063661474, 0.087657753, 0.11021469, 0.13096191, 0.14955874, 0.16569982, 0.17912011, 0.18959925, 0.19696518, 0.20109695, 0.20192671, 0.19944084, 0.19368015, 0.18473925, 0.17276493, 0.15795381, 0.1405491, 0.12083657, 0.099139917, 0.075815384, 0.051245965, 0.025835088, 2.4741602e-17
});

RingBuffer<FILTER_SIZE_LARGE> filterSamples;

// min/max number of samples to use for stability calculations
#define STABILITY_MIN 1
#define STABILITY_MAX 128
// cutoff magnitude for positive or negative
#define EPSILON 0.000001

RingBuffer<STABILITY_MAX> d0;
RingBuffer<STABILITY_MAX> d1;
RingBuffer<STABILITY_MAX> d2;
RingBuffer<STABILITY_MAX> d3;
int d0_state = 0;
int d1_state = 0;
int d2_state = 0;
int d3_state = 0;

static double to_double(uint32_t value) {
  double center = 1 << 11;
  return (value - center) / center;
}

// only detect pos/neg when all samples agree (fully stable)
// only switch it off once none of the samples match the current state
static int update_state(const RingBuffer<STABILITY_MAX>& buffer, const int state, const int stabilityThreshold) {
  int posCount = buffer.pos_count(EPSILON, stabilityThreshold);
  int negCount = buffer.neg_count(EPSILON, stabilityThreshold);

  double downPercentage = stabilityThreshold / (4.0*STABILITY_MAX); // ~0 to 0.25
  double upPercentage = 1.0 - downPercentage;
  int stabilityDown = stabilityThreshold * downPercentage;
  int stabilityUp = max(1, stabilityThreshold * upPercentage);

  if (posCount >= stabilityUp) {
    return 1;
  }
  if (negCount >= stabilityUp) {
    return -1;
  }
  if (state == 1 && posCount <= stabilityDown) {
    return 0;
  }
  if (state == -1 && negCount <= stabilityDown) {
    return 0;
  }

  // have a mix of states, keep it as it is
  return state;
}

#define SIGNAL_CHANGE_SAMPLE_COUNT 8
#define SIGNAL_CHANGE_MIN 0.01
#define SIGNAL_CHANGE_MAX 0.05

static bool audioHandler(struct repeating_timer *t) {
  double sample = to_double(adc_read());

  filterSamples.put(sample);
  if (!filterSamples.is_full()) {
    return true;
  }

  // want to get some kind of estimate of the frequency of the input so we can adjust stability requirements for low frequencies
  // to achieve this, we're getting a total of the distances between the newest samples and several recent ones, which should
  // give us a good sense of how much the input is changing
  double totalChange = 0.0;
  for (int i=0; i<SIGNAL_CHANGE_SAMPLE_COUNT; i++) {
    totalChange += fabs(d0[STABILITY_MAX - i - 1] - d0[STABILITY_MAX - i - 2]);
  }
  double frequencyFactor = 1.0 - max(0.0, min(1.0, (totalChange - SIGNAL_CHANGE_MIN) / (SIGNAL_CHANGE_MAX - SIGNAL_CHANGE_MIN)));

  FIRFilter* f0 = &f0_small;
  FIRFilter* f1 = &f1_small;
  FIRFilter* f2 = &f2_small;
  FIRFilter* f3 = &f3_small;

  d0.put(f0->process(filterSamples));
  d1.put(f1->process(filterSamples));
  d2.put(-f2->process(filterSamples)); // not sure why, but negating this is necessary to give expected result for sine input
  d3.put(f3->process(filterSamples));

  debugVal = d1.last();
  int stabilityThreshold = STABILITY_MIN + (STABILITY_MAX - STABILITY_MIN) * frequencyFactor;

  // perform comparisons
  d0_state = update_state(d0, d0_state, stabilityThreshold);
  d1_state = update_state(d1, d1_state, stabilityThreshold);
  d2_state = update_state(d2, d2_state, stabilityThreshold);
  d3_state = update_state(d3, d3_state, stabilityThreshold);

  // output
  gpio_put(gatePins[0], d0_state == 1);
  gpio_put(gatePins[1], d0_state == -1);
  gpio_put(gatePins[2], d1_state == 1);
  gpio_put(gatePins[3], d1_state == -1);
  gpio_put(gatePins[4], d2_state == 1);
  gpio_put(gatePins[5], d2_state == -1);
  gpio_put(gatePins[6], d3_state == 1);
  gpio_put(gatePins[7], d3_state == -1);

  return true;
}

struct repeating_timer _timer_;

void setup() {
  // 2x overclock for MAX POWER
  set_sys_clock_khz(250000, true);

  // initialize ADC
  adc_init();
  adc_gpio_init(inputPin);
  adc_select_input(0);
  gpio_set_pulls(inputPin, false, false);

  // initialize gate out pins
  for (int i=0; i<NUM_GATES; i++) {
    int pin = gatePins[i];
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
  }

  // audio callback
  add_repeating_timer_us(-TIMER_INTERVAL, audioHandler, NULL, &_timer_);

  // init serial debugging
  Serial.begin(9600);
}

void loop() {
  Serial.println(debugVal, 6);
  delay(1);
}
