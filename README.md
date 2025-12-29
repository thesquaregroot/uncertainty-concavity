# uncertainty-concavity

A signal derivative comparator for the Eurorack module
[Uncertainty](https://oamodular.org/products/uncertainty) by
[Olivia Artz Modular](https://github.com/oamodular).

## About this firmware

This firmware calculates the following from the input signal:

* Value (the signal itself, or "0th" derivative)
* Slope (rate of change, or first derivative)
* Concavity (rate of slope change, or second derivative)
* Jerk (rate of concavity change, or third derivative)

The outputs correspond two:

1. Positive (input is positive)
2. Negative (input is negative)
3. Increasing (first derivative is positive)
4. Decreasing (first derivative is negative)
5. Concave Up / Convex (second derivative is positive)
6. Concave Down / Concave (second derivative is negative)
7. Concavity Increasing (third derivative is positive)
8. Concavity Decreasing (third derivative is negative)

Whenever these conditions are met, the corresponding output will go high.  When
the condition is no longer met, the output will go low.  A value that is either
zero or indeterminate is ignored.  For example, an input signal that is a
constant 0V, or extremely noisy/high-frequency, may result in most or all of
the outputs staying low.

It is also worth noting that the pairs of outputs corresponding to the same
comparison signal (1/2,  3/4, 5/6, and 7/8) will never be "on" at the same time,
and will often appear as logical negations of one another.  As a result, each of
these pairs could be good candidates to use as a pseudo-stereo pair.  Though,
depending on the input signal, other pairs may be even better suited for this
purpose.

## Expected Range of Operation

Because derivative calculations inherently amplify noise, this firmware using
digital filters to smooth the data and perform the derivative calculations.
This inherently produces phase shift and input lag, which is most noticeable on
audio-rate signals.  As a result, the description above is mostly applicable for
LFO-rate signals.  However, ignoring phase-shift/lag, my testing suggests a
fairly stable output is possible for signals below ~1.5 kHz.

I also did my best to ensure that very slow LFOs also produce stable output,
but there may be a point with extremely slow signals where the derivatives are
no longer easily calculated and the bottom six outputs will either stop
responding to the signal, or produce unexpected results.

## Use Cases

### Comparator With Extras

Since the top two outputs are essentially just comparing the input signal
against 0V, an external offset can be used to move the signal up or down,
adjusting the effective comparator threshold.  The first output will act like a
standard comparator, going high when the value is above the threshold.  The
second output will go high when the value is below the threshold.  Just keep in
mind that a positive signal offset would be like having a negative threshold
value, and vice versa.

The derivative outputs shouldn't be affected by the offset (unless it's
changing), but you will continue to get these outputs even for an offset signal.

### Sine to Quadrature Square Waveshaper/Distortion



### Aliasing Noise Effect



## Installing

If you want to install this firmware as-is, you can download the .uf2 file (see
"Releases") and follow the instructions
[here](https://github.com/oamodular/uncertainty?tab=readme-ov-file#how-to-reinstall-the-default-firmware).

If you would like to modify the code and run it, you can do so using the Arduino
IDE, following the instructions
[here](https://wiki.seeedstudio.com/XIAO-RP2040-with-Arduino/).

