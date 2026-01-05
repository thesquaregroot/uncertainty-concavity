pkg load signal

# for normalizing values into filter coefficients
normalize = @(v) ( v / norm(v) );

# for each of copy-pasting into C++ code
to_string = @(v) ( strcat(strjoin(arrayfun(@(x) num2str(x, 8),v,'UniformOutput',false),', '), "\n") );

# gaussian function and its derivates, calculated using wolfram alpha
#
# was previously using hann window, but was bothered by the fact that the 1st
# and 3rd derivate coefficients were just negations of each other
# this seems to work just as well for sinousoids but may produce more interesting
# (accurate?) results for other wave shapes
#
# this is essentially doing the same thing as the `gaussian` function,
# just keeping it more general so we can use it derivates as well
a = 0.5; # chosen semi-arbitrarily to give relatively even values in the output filter coefficients
f_g0 = @(x, L) ( exp( -(a*x)^2/2 ) );
f_g1 = @(x, L) ( -a*x * exp( -(a*x)^2/2) );
f_g2 = @(x, L) ( a * (a*x^2 - 1) * exp( -(a*x)^2/2) );
f_g3 = @(x, L) ( -a^2*x * (a*x^2 - 3) * exp( -(a*x)^2/2 ) );

size = 12
disp("")
values = linspace(-size/2, size/2, size);

# normalizing the output since we're going to end up doing comparisons on the
# filter outputs anyway, so this should hopefully resulting in more consistent
# scaling for said comparisons
f0 = normalize(arrayfun(f_g0, values, size));
f1 = normalize(arrayfun(f_g1, values, size));
f2 = normalize(arrayfun(f_g2, values, size));
f3 = normalize(arrayfun(f_g3, values, size));

#hold off
#plot(f0)
#hold on
#plot(f1)
#plot(f2)
#plot(f3)

# formatting for ease of copy-and-paste into c++ code
out0 = to_string(f0)
out1 = to_string(f1)
out2 = to_string(f2)
out3 = to_string(f3)
