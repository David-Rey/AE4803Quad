function [c, c_term] = quad_cost(Q, R, Qf, target_state)
% yield cost functions for the quadrotor
% edited by Elliot for final project

% Q, R, and Qf are all square
% target_state: 1-by-9 target state vector (in HW2 this was a column
% vector)

    function [c, cx, cu, cxx, cxu, cuu] = costfn(x, u)
        err = (x - target_state);  % untransposed -alex
        c = 0.5 * (err.' * Q * err + u.' * R * u);
        cx = Q * err;
        cxx = Q;
        cxu = 0;
        cu = R * u;
        cuu = R;
    end

    function [c, cx, cxx] = term_costfn(x)
        err = (x - target_state);  % untransposed again
        c = 0.5 * (err' * Qf * err);
        cx = Qf * err;
        cxx = Qf;
    end

c = @costfn;
c_term = @term_costfn;
end

