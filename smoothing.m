pkg load signal

# for normalizing values into filter coefficients
normalize = @(v) ( v / norm(v) );

# for each of copy-pasting into C++ code
to_string = @(v) ( strcat(strjoin(arrayfun(@(x) num2str(x, 8),v,'UniformOutput',false),', '), "\n") );

# hann window and its derviatives, calculated using wolfram alpha
# discarding 1/L factors as we're going to normalize anyway
f_h0 = @(x, L) ( (1/2 + 1/2*cos(2*pi*x/L)) );
f_h1 = @(x, L) ( (-pi*sin(2*pi*x/L)) );
f_h2 = @(x, L) ( (-2*pi*pi*cos(2*pi*x/L)) );
f_h3 = @(x, L) ( (4*pi*pi*pi*sin(2*pi*x/L)) );

size = 11
disp("")
values = linspace(-size/2, size/2, size);

h0 = normalize(arrayfun(f_h0, values, size));
h1 = normalize(arrayfun(f_h1, values, size));
h2 = normalize(arrayfun(f_h2, values, size));
h3 = normalize(arrayfun(f_h3, values, size));

f0 = to_string(h0)
f1 = to_string(h1)
f2 = to_string(h2)
f3 = to_string(h3)
