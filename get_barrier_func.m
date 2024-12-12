function barrier_func = get_barrier_func()
% x: state vec 

h1 = @(x) (x(1) - 2.2)^2 + (x(3) - 1)^2 - 1 + (x(8) - 2.2)^2;
h2 = @(x) x(1)^2 + (x(2) + 0.2)^2 + x(3)^2 - 1;
h3 = @(x) (x(1) - 3)^2 + x(2)^2 + (x(3) - 0.5)^2 - 1;
h = @(x) h1(x) + h2(x) + h3(x);  % yields scalar
barrier_func = @(x)-log10(h(x));  % scalar

end