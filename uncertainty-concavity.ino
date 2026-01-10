// Adapted from https://github.com/oamodular/uncertainty/blob/main/software/stable/uncertainty/uncertainty.ino
// Original license: https://github.com/oamodular/uncertainty/blob/main/LICENSE.md
// License for this code: https://creativecommons.org/licenses/by-sa/4.0/

#include <Arduino.h>
#include <vector>
#include <math.h>
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/adc.h"

using namespace std;

// number of output gates
#define NUM_GATES 8
#define MILLISECONDS_IN_SECOND 1000000.0
#define TARGET_SAMPLE_RATE 40000
#define TIMER_INTERVAL ((int)(MILLISECONDS_IN_SECOND/TARGET_SAMPLE_RATE))
#define SAMPLE_RATE (MILLISECONDS_IN_SECOND/TIMER_INTERVAL)

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
    return this->operator[](_size - 1);
  }

  double last(const int i) const {
    return this->operator[](_size - i - 1);
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
#define FILTER_SIZE_0 11
#define FILTER_SIZE_1 11
#define FILTER_SIZE_2 11
#define FILTER_SIZE_3 11
#define FILTER_BUFFER_SIZE FILTER_SIZE_3

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

  template <size_t S>
  double process(const RingBuffer<S>& signal) {
    static_assert(S >= FILTER_BUFFER_SIZE, "Invalid buffer size.");
    double value = 0.0;
    for (int i=0; i<_length; i++) {
      value += _coefficients[i] * signal.last(i);
    }
    return value;
  }
};

FIRFilter f0(FILTER_SIZE_0, {
  0.022794181, 0.088921617, 0.25634015, 0.54607443, 0.85963276, 1, 0.85963276, 0.54607443, 0.25634015, 0.088921617, 0.022794181
});
FIRFilter f1(FILTER_SIZE_1, {
  0.062683997, 0.19562756, 0.42296125, 0.60068187, 0.47279802, -0, -0.47279802, -0.60068187, -0.42296125, -0.19562756, -0.062683997
});
FIRFilter f2(FILTER_SIZE_2, {
  0.1609839, 0.38591982, 0.56971599, 0.38771284, -0.16977747, -0.5, -0.16977747, 0.38771284, 0.56971599, 0.38591982, 0.1609839
});
/*FIRFilter f3(FILTER_SIZE_3, {
  0.029605214, 0.19817002, 0.51869794, 0.12778035, -0.86275244, 0, 0.86275244, -0.12778035, -0.51869794, -0.19817002, -0.029605214
});*/

// samples of raw input
RingBuffer<FILTER_BUFFER_SIZE> filterSamples;
// samples with DC-offset removed, since the filter don't do a good job of that on their own
// and the derivatives should be operating independently of any offset
RingBuffer<FILTER_BUFFER_SIZE> acSamples;

// min/max number of samples to use for stability calculations
#define STABILITY_MIN 1
#define STABILITY_MAX 128
// for estimating signal frequency
#define FREQUENCY_LOW 0.1
#define FREQUENCY_HIGH 20.0
// slope cutoff for frequency cycle detection
#define FREQUENCY_EPSILON 0.01
// cutoff magnitude for positive or negative
#define EPSILON 0.0001

RingBuffer<STABILITY_MAX> d0;
RingBuffer<STABILITY_MAX> d1;
RingBuffer<STABILITY_MAX> d2;
//RingBuffer<STABILITY_MAX> d3;
int d0_state = 0;
int d1_state = 0;
int d2_state = 0;
//int d3_state = 0;

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

int stabilityThreshold = STABILITY_MAX;
double estimatedFrequency = FREQUENCY_LOW;

// used to determine when a cycle has completed
bool hasBeenNegative = false;
uint32_t lastTimeDiff = 0;
uint32_t lastNegToPosZeroCrossing = 0;

// degree of DC-removal -- closer to one means cleaner DC removal, but slower reaction time
#define DC_REMOVAL_STRENGTH 0.99999
// these are available elsewhere, but we're caching them as an optimization
double lastSample = 0.0;
double lastAcSample = 0.0;

void core1_entry() {
  // not sure why, but negating these is necessary to give expected result for sine input
  // NOTE: this is not related to running on second core and was present before that change
  d2.put(-f2.process(acSamples));

  d2_state = update_state(d2, d2_state, stabilityThreshold);

  bool equal = (d1_state != 0 && d2_state == d1_state);
  bool notEqual = (d1_state != 0 && d2_state != 0 && d2_state != d1_state);

  gpio_put(gatePins[4], d2_state == 1);
  gpio_put(gatePins[5], d2_state == -1);
  // In initial versions, the final two outputs corresponded to the third derivative of the signal,
  // but the filter for this was particularly inaccurate, often just tracking the
  // first derivative signal.
  //
  // Using comparisons of the first and second derivatives instead often yields much more
  // interesting outputs, both at slow rates (e.g. for rhythms) and at audio-rate.
  gpio_put(gatePins[6], equal);
  gpio_put(gatePins[7], notEqual);
}

static bool audioHandler(struct repeating_timer *t) {
  uint32_t time = micros();
  double sample = to_double(adc_read());

  filterSamples.put(sample);

  // simple DC-removal filter for derivative calculations
  double acSample = sample - lastSample + DC_REMOVAL_STRENGTH * lastAcSample;
  acSamples.put(acSample);
  lastSample = sample;
  lastAcSample = acSample;

  // run half of the filters on the other core
  multicore_reset_core1();
  multicore_launch_core1(core1_entry);

  d0.put(f0.process(filterSamples));
  d1.put(f1.process(acSamples));

  debugVal = d1.last();

  // perform comparisons
  d0_state = update_state(d0, d0_state, stabilityThreshold);
  d1_state = update_state(d1, d1_state, stabilityThreshold);

  // output
  gpio_put(gatePins[0], d0_state == 1);
  gpio_put(gatePins[1], d0_state == -1);
  gpio_put(gatePins[2], d1_state == 1);
  gpio_put(gatePins[3], d1_state == -1);

  // update frequency estimate
  //
  // using 1st derivative going from negative to positive as a sign that a cycle of some sort has completed
  // using derivative ensures that we're not susceptible to any DC offset
  // this should give a decent estimate of the highest frequency component that is large enough to not be smoothed out
  double lastSlope = d1.last();
  if (lastSlope > FREQUENCY_EPSILON && hasBeenNegative) {
    // assume time elapsed is one full period of the highest frequency component
    lastTimeDiff = time - lastNegToPosZeroCrossing;
    lastNegToPosZeroCrossing = time;
    estimatedFrequency = MILLISECONDS_IN_SECOND / lastTimeDiff;
    hasBeenNegative = false;
  }
  else {
    if (lastSlope < -FREQUENCY_EPSILON) {
      hasBeenNegative = true;
    }
    uint32_t timeDiff = time - lastNegToPosZeroCrossing;
    if (timeDiff > lastTimeDiff) {
      // it's been too long, update estimated frequency anyway
      estimatedFrequency = MILLISECONDS_IN_SECOND / timeDiff;
    }
  }

  double frequencyFactor = 1.0 - min(1.0, max(0.0, (estimatedFrequency - FREQUENCY_LOW) / (FREQUENCY_HIGH - FREQUENCY_LOW))); // frequency inverse-lerp
  double lerpFactor = frequencyFactor * frequencyFactor; // quadratic adjustment
  stabilityThreshold = STABILITY_MIN + (STABILITY_MAX - STABILITY_MIN) * lerpFactor; // stability count lerp

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
