function [c, c_term] = quad_cost_barrier(Q, R, Qf, target_state)
% yield cost functions for the quadrotor

% Q, R, and Qf are all square
% target_state: 1-by-12 target state vector (in HW2 this was a column
% vector)

    function [c, cx, cu, cxx, cxu, cuu] = costfn(x, u)
        % x: state vector with barrier state at end 
        w = x(13);  % get barrier state
        x = x(1:12);  % cut off barrier state for error calcs
        err = (x - target_state);
        % added +w^2 term to both fns
        c = 0.5 * (err.' * Q * err + u.' * R * u) + w^2;
        cx = Q * err;
        cxx = Q;
        cxu = 0;
        cu = R * u;
        cuu = R;
    end

    function [c, cx, cxx] = term_costfn(x)
        w = x(13);
        x = x(1:12);
        err = (x - target_state); 
        c = 0.5 * (err' * Qf * err) + w^2;
        cx = Qf * err;
        cxx = Qf;
    end

c = @costfn;
c_term = @term_costfn;
end
