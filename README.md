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

1. Positive value
2. Negative value
3. Positive slope, increasing value
4. Negative slope, decreasing value
5. Positive concavity (convex), increasing slope
6. Negative concavity (concave), decreasing slope
7. Positive jerk, increasing concavity
8. Negative jerk, decreasing concavity

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

Because derivative calculations inherently amplify noise, this firmware uses
digital filters to smooth the data and perform the derivative calculations.
This inherently produces phase shift and input lag, which is most noticeable on
audio-rate signals.  As a result, the description above is mostly applicable for
LFO-rate signals.  However, ignoring phase-shift/lag, my testing suggests a
fairly stable output is generally possible up to about 1.5 kHz, after which the
results become increasingly wave-shape-dependent, though often up to about 3 kHz
is okay.

I also did my best to ensure that slower LFOs also produce stable output, but
there may be a point, below around 0.1 Hz, or 10 seconds per cycle, where the
derivative values because too small, and the bottom six outputs will either
stop responding to the signal, or produce unexpected results.

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

### Sine to Quadrature Square Waves

With a pure sine wave input, the first four outputs (and also the four
"positive" outputs, 1/3/5/7) will generate square waves at the input signal's
frequency, each 90 degrees out of phase from two others.  This is because the
successive derivatives of sine are: cosine, negative sine, and negative cosine.

At LFO rates, these could be used to generate rhythms, while at audio rates they
could be used as the start of a mono-to-stereo, or even mono-to-quadraphonic,
effect.

### Crunchy Distortion / Aliasing Noise Effect

As previously mentioned, audio-rate inputs will increasingly result in phase
shifted and delayed outputs, all pulse-like representations of some aspect of
the signal.  The fact that uncertainty only outputs pulse wave, along with the various
sources of noise and aliasing inherent in the module/firmware design, means that
as the prominent frequencies of the input signal increase, additional inharmonic
frequencies will be introduced and amplified.  This resulting in everything from
subtle metallic qualities to radio-transmission effects to crunchy distortion.

## Installing

If you want to install this firmware as-is, you can download the .uf2 file (see
"Releases") and follow the instructions
[here](https://github.com/oamodular/uncertainty?tab=readme-ov-file#how-to-reinstall-the-default-firmware).

If you would like to modify the code and run it, you can do so using the Arduino
IDE, following the instructions
[here](https://wiki.seeedstudio.com/XIAO-RP2040-with-Arduino/).

