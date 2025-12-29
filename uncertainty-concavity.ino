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

// Using FIR filters for both smoothing and derivative calculations
// A gaussian function is used as a smoothing filter (f0), and its derivatives are used to generate derivative filters (f1/f2/f3)
// This performed better than smoothing and then performing derivative calculations using finite difference methods afterward
// see: filter_design.m
#define FILTER_SIZE 12

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

  double process(const RingBuffer<FILTER_SIZE>& signal) {
    double value = 0.0;
    for (int i=0; i<_length; i++) {
      value += _coefficients[i] * signal[_length - i - 1];
    }
    return value;
  }
};

// length 12
FIRFilter f0(FILTER_SIZE, {
  0.0061626445, 0.027278785, 0.089674993, 0.21893041, 0.39694401, 0.53449154, 0.53449154, 0.39694401, 0.21893041, 0.089674993, 0.027278785, 0.0061626445
});
FIRFilter f1(FILTER_SIZE, {
  0.026146575, 0.094694018, 0.24211651, 0.42221252, 0.4593096, 0.20615594, -0.20615594, -0.4593096, -0.42221252, -0.24211651, -0.094694018, -0.026146575
});
FIRFilter f2(FILTER_SIZE, {
  0.074103676, 0.21320372, 0.39892784, 0.42105646, 0.095137439, -0.3218225, -0.3218225, 0.095137439, 0.42105646, 0.39892784, 0.21320372, 0.074103676
});
FIRFilter f3(FILTER_SIZE, {
  0.16047688, 0.35063696, 0.4249255, 0.12421424, -0.3121926, -0.24051157, 0.24051157, 0.3121926, -0.12421424, -0.4249255, -0.35063696, -0.16047688
});

RingBuffer<FILTER_SIZE> filterSamples;

// min/max number of samples to use for stability calculations
#define STABILITY_MIN 4
#define STABILITY_MAX 128
// for estimating signal frequency
#define SIGNAL_CHANGE_SAMPLE_COUNT 8
#define SIGNAL_CHANGE_MIN 0.001
#define SIGNAL_CHANGE_MAX 0.1
// cutoff magnitude for positive or negative
#define EPSILON 0.00001

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

static bool audioHandler(struct repeating_timer *t) {
  double sample = to_double(adc_read());

  filterSamples.put(sample);
  if (!filterSamples.is_full()) {
    return true;
  }

  // Want to get some kind of estimate of the frequency of the input so we can adjust stability requirements for low frequencies.
  // To achieve this, we're getting a total of the distances between the newest samples and several recent ones, which should
  // give us a good sense of how much the input is changing.
  double totalChange = 0.0;
  for (int i=0; i<SIGNAL_CHANGE_SAMPLE_COUNT; i++) {
    totalChange += fabs(d0[STABILITY_MAX - i - 1] - d0[STABILITY_MAX - i - 2]);
  }
  double frequencyFactor = 1.0 - min(1.0, max(0.0, (totalChange - SIGNAL_CHANGE_MIN) / (SIGNAL_CHANGE_MAX - SIGNAL_CHANGE_MIN)));
  int stabilityThreshold = STABILITY_MIN + (STABILITY_MAX - STABILITY_MIN) * frequencyFactor;

  d0.put(f0.process(filterSamples));
  d1.put(f1.process(filterSamples));
  // not sure why, but negating these is necessary to give expected result for sine input
  d2.put(-f2.process(filterSamples));
  d3.put(-f3.process(filterSamples));

  debugVal = d1.last();

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
