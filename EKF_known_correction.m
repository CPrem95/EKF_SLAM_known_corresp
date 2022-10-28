function [mu ,sig] = EKF_known_correction(mu_bar, sig_bar, obs, Q, landm_no, ini_var)
%mu_bar = Predicted state estimation
%sig_bar = predicted state uncertainty (covariance)
%obs = NOT the all observations. Only the current state observation
%Q = observation uncertainty
%landm_n = number of TOTAL landmarks
%ini_var = initial landmark variance

% This function performs the correction/update step
%% Correction Steps
% Known data association
% c^i_t >> i-th measurement at time t observes the landmark with index j
% Initialize landmark if unobserved
% Compute the Expected observation
% Compute the Jacobian of h
% Proceed with computing the Kalman gain
for j = 1:landm_no
    %% Predicted observation
    if  obs(1,j,3) ~= 0
        r = obs(1,j,1);
        psi = obs(1,j,2);
        z = [r, psi]';

        if sig_bar(2 + 2*j, 2 + 2*j) >= ini_var && sig_bar(3 + 2*j, 3 + 2*j) >= ini_var
            % relative measurement
            % if the landmark has not been observed before,
            rel_meas = [r*cos(psi + mu_bar(3)) r*sin(psi + mu_bar(3))]';
            mu_bar(2 + 2*j:3 + 2*j) = mu_bar(1:2) + rel_meas;
%             disp('***')
%             disp(mu_bar(1:2) + rel_meas)
%             disp(j)
%             disp('***')
        end
        %% Expected observation
        % what was observed

        del = [mu_bar(2 + 2*j) - mu_bar(1), mu_bar(3 + 2*j) - mu_bar(2)]';
        q = del'*del;
        sq = sqrt(q);
        z0 = [sq, atan2(del(2), del(1)) - mu_bar(3)]';

        F_xj = [eye(3), zeros(3, 2*j - 2), zeros(3, 2), zeros(3, 2*landm_no - 2*j);
            zeros(2, 3), zeros(2, 2*j - 2), eye(2), zeros(2, 2*landm_no - 2*j)];
        
        h = (1/q)*[-del(1)*sq, -del(2)*sq, 0, del(1)*sq, del(2)*sq;
            del(2), -del(1), -q, -del(2), del(1)];

        H = h*F_xj;
        
        A = H*sig_bar*H' + Q;
%         dA = decomposition(A);
%         tf = isIllConditioned(dA);
%         rcon = rcond(dA);
%         disp(rcon)
%         if rcon < 0.2
%             disp('ill conditioned')
%             pause(5)
%         end
        K = sig_bar*H'/A;
        
        %**** keep this section to bypass the error section_1
        prev_xt = mu_bar(1);
        prev_yt = mu_bar(2);
        temp_mu_bar = mu_bar + K*(z - z0);
        new_xt = temp_mu_bar(1);
        new_yt = temp_mu_bar(2);

        if abs(new_yt - prev_yt) > 0.2 || abs(new_xt - prev_xt) > 0.2
            disp(j)
            disp('error')
        else
            mu_bar = mu_bar + K*(z - z0);
            sig_bar = (eye(3 + 2*landm_no) - K*H)*sig_bar;
        end
        %**** end of section_1
        
        %**** keep this section to KEEP the error section_2
%         mu_bar = mu_bar + K*(z - z0);
%         sig_bar = (eye(3 + 2*landm_no) - K*H)*sig_bar;
        %**** end of section_2
    end
end
mu = mu_bar;
sig = sig_bar;
end