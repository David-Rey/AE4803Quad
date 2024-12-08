function [clipped_x] = clip(x, lb, ub)

clipped_x = min(max(x, lb), ub);

end