# uncertainty-concavity

A signal derivative comparator for the Eurorack module
[Uncertainty](https://oamodular.org/products/uncertainty) by
[Olivia Artz Modular](https://github.com/oamodular).

## About this firmware

This firmware calculates the following from the input signal:

* Value (the signal itself, or "0th" derivative)
* Rate of Change (first derivative)
* Concavity (rate of rate of change, or second derivative)
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

Whenever (and for the duration that) these conditions are met, the corresponding
output will go high.  When the condition is not longer met, the output will go
low.  A value that is either zero or indeterminate is ignored.
For example, an input signal that is a constant 0V, or extremely
noisy/high-frequency, may result in most or all of the outputs staying low.

It is also worth noting that the pairs of outputs corresponding to the same
comparison signal (1/2,  3/4, 5/6, and 7/8) will never be "on" at the same time,
and will often appear as logical negations of one another.  As a result, each of
these pairs could be good candidates to use as a pseudo-stereo pair.  Though,
depending on the input signal, other pairs may be even better suited for this
purpose.

## Use Cases

### Comparator With Extras

### Sine to Quadrature Square Waveshaper

## Installing

If you want to install this firmware as-is, you can download the .uf2 file (see
"Releases") and follow the instructions
[here](https://github.com/oamodular/uncertainty?tab=readme-ov-file#how-to-reinstall-the-default-firmware).

If you would like to modify the code and run it, you can do so using the Arduino
IDE, following the instructions
[here](https://wiki.seeedstudio.com/XIAO-RP2040-with-Arduino/).

