function O = m3_observer_30Hz(q3, q3d, d4, d4d, tau3, f4, P, kO, Ts)
%M3_OBSERVER_30HZ  M3 30 Hz momentum observer (Branch B exact FOH, and KF-GMO) on given samples.
%
%   O = m3_observer_30Hz(q3, q3d, d4, d4d, tau3, f4, P, kO, Ts)
%
%   Inputs (vectors of the same length N, sampled every Ts seconds):
%     q3 [rad], q3d [rad/s], d4 [m], d4d [m/s]   joint position and velocity (NO acceleration)
%     tau3 [N m], f4 [N]                           measured drive torque / force
%     P   struct with Md, Izz, c, g, fv3, fv4, fc3, fc4, eps3, eps4
%     kO  observer gain [1/s] (M3: 20),  Ts  sample time [s] (M3: 1/30)
%
%   Output O.tau_hat_B  2xN  Branch B estimate of the generalized load [tau_L3; F_L4]
%          O.tau_hat_KF 2xN  3-state Kalman-filtered GMO estimate
%          O.sigma_KF   2xN  KF standard deviation of the load state
%          O.phi, O.beta0, O.beta1  FOH coefficients used
%
%   The update equations are copied line for line from
%   M3/scripts/build_M3_DiscreteObserver_30Hz.m (execute_discrete_30Hz_case),
%   so that this function IS the M3 observer, applied to data it did not generate.
%   Branch B leaves Coulomb friction out of h (friction-uncertain branch);
%   the KF-GMO uses h_A = h_B - Fc (friction-aware), exactly as in M3.
%   Sign convention (M3): tau_actuator = tau_model - tau_L, so tau_L equals the
%   generalized external load Q = J'F used in M2.
%
%   Project: IIT/SRIC/R/AEH/2026/104 (CMPDI MT-189). Written 2026-11-18.

    N = numel(q3);
    q3 = q3(:); q3d = q3d(:); d4 = d4(:); d4d = d4d(:); tau3 = tau3(:); f4 = f4(:);

    phi_d = exp(-kO * Ts);
    beta1 = (1.0 - (1.0 - phi_d) / (kO * Ts)) / kO;   % multiplies the NEW sample u_k
    beta0 = (1.0 - phi_d) / kO - beta1;               % multiplies the OLD sample u_{k-1}

    z_B = zeros(2, N); tau_hat_B = zeros(2, N);
    tau_hat_KF = zeros(2, N); sigma_kf = zeros(2, N);

    A_kf = [1.0, Ts, 0.5*Ts^2; 0.0, 1.0, Ts; 0.0, 0.0, 1.0];
    B_kf = [Ts; 0.0; 0.0];
    C_kf = [1.0, 0.0, 0.0];
    Q_kf3 = diag([10, (1000 * Ts)^2, (1.5e6 * Ts)^2]);
    Q_kf4 = diag([1, (200 * Ts)^2, (1.5e5 * Ts)^2]);
    P_kf3 = diag([1e4, 1e10, 1e12]);
    P_kf4 = diag([1e2, 1e8, 1e10]);
    x_kf3 = zeros(3, 1); x_kf4 = zeros(3, 1);

    p_prev = zeros(2, 1); h_A_prev = zeros(2, 1); h_B_prev = zeros(2, 1);

    for k = 1:N
        r_k  = d4(k) + P.c;
        D11  = P.Izz + P.Md * r_k^2;
        D22  = P.Md;
        p_k  = [D11 * q3d(k); D22 * d4d(k)];
        G_k  = [P.Md * P.g * r_k * cos(q3(k)); P.Md * P.g * sin(q3(k))];
        CT_k = [0.0; P.Md * r_k * (q3d(k)^2)];
        Fv_k = [P.fv3 * q3d(k); P.fv4 * d4d(k)];
        Fc_k = [P.fc3 * tanh(q3d(k) / P.eps3); P.fc4 * tanh(d4d(k) / P.eps4)];

        h_B_k = [tau3(k); f4(k)] - G_k - Fv_k + CT_k;
        h_A_k = h_B_k - Fc_k;

        if k == 1
            z_B(:, k) = p_k;
            x_kf3 = [p_k(1); 0.0; 0.0];
            x_kf4 = [p_k(2); 0.0; 0.0];
        else
            uB_prev = h_B_prev + kO * p_prev;
            uB_k    = h_B_k    + kO * p_k;
            z_B(:, k) = phi_d * z_B(:, k-1) + beta0 * uB_prev + beta1 * uB_k;

            R_kf3 = (D11 * 0.003)^2;
            R_kf4 = (D22 * 0.003)^2;
            x_pred3 = A_kf * x_kf3 + B_kf * (0.5 * (h_A_prev(1) + h_A_k(1)));
            P_pred3 = A_kf * P_kf3 * A_kf' + Q_kf3;
            K_gain3 = (P_pred3 * C_kf') / (C_kf * P_pred3 * C_kf' + R_kf3);
            x_kf3   = x_pred3 + K_gain3 * (p_k(1) - C_kf * x_pred3);
            P_kf3   = (eye(3) - K_gain3 * C_kf) * P_pred3;
            x_pred4 = A_kf * x_kf4 + B_kf * (0.5 * (h_A_prev(2) + h_A_k(2)));
            P_pred4 = A_kf * P_kf4 * A_kf' + Q_kf4;
            K_gain4 = (P_pred4 * C_kf') / (C_kf * P_pred4 * C_kf' + R_kf4);
            x_kf4   = x_pred4 + K_gain4 * (p_k(2) - C_kf * x_pred4);
            P_kf4   = (eye(3) - K_gain4 * C_kf) * P_pred4;
        end

        tau_hat_B(:, k)  = kO * (p_k - z_B(:, k));
        tau_hat_KF(:, k) = [x_kf3(2); x_kf4(2)];
        sigma_kf(:, k)   = [sqrt(P_kf3(2, 2)); sqrt(P_kf4(2, 2))];

        p_prev = p_k; h_A_prev = h_A_k; h_B_prev = h_B_k;
    end

    O = struct('tau_hat_B', tau_hat_B, 'tau_hat_KF', tau_hat_KF, 'sigma_KF', sigma_kf, ...
               'phi', phi_d, 'beta0', beta0, 'beta1', beta1, 'kO', kO, 'Ts', Ts);
end
