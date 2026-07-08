function P = shovel_params()
%SHOVEL_PARAMS  Single source of truth for the 2-DOF cable-shovel model.
%
%  Every consumer reads THIS file:
%   1) ShovelSimulator_v4.slx  -> Model Properties > Callbacks > InitFcn:
%           P = shovel_params;
%      then reference P.m, P.Izz, P.fv3_degs, ... inside the block dialogs
%      instead of typing numbers.
%   2) shovel_math MATLAB Function block -> declare m, c, Izz, fv3, fv4, g
%      as Parameter-scope arguments (Ports & Data Manager) bound to P.*
%   3) validate_and_identify_4.m -> P = shovel_params;
%
%  Rationale: v3 script comments contradicted the .slx (claimed Izz=839,000
%  and fv3=340 while the model held 287,900 and 3,800). Duplication caused
%  that drift; this file removes the duplication.
%
%  All values traceable to Rasuli, Tafazoli & Dunford (IEEE 2014), Table II
%  (dynamic identification case), except geometry offsets which are the v4
%  planar-model construction.

% ---- Gravity ----------------------------------------------------------
P.g        = 9.80665;      % m/s^2

% ---- Dipper handle body (Rasuli Table II) -----------------------------
P.m        = 54300;        % kg      Md, dipper+handle mass
P.L_COG    = 2.82;         % m       COM offset used in the CAD/Simscape body
P.x_ref    = 1.50;         % m       rigid transform offset along slide axis
P.c        = P.x_ref - P.L_COG;   % = -1.32 m  net in-plane lever offset
                           %          lever arm r(t) = d_4(t) + P.c

% ---- Rotational inertia -----------------------------------------------
% NOTE (open verification item): Rasuli Table II reports 287,900 kg*m^2 as
% "Izz3". Whether this is inertia about the COM or the grouped quantity
% (Izz3 + Md*L^2) about the pivot must be resolved against Rasuli Eq. 23/26
% before the 4-DOF derivation. The Simscape body currently uses it as the
% COM value on the axis parallel to the spin axis.
P.Izz      = 287900;       % kg*m^2

% ---- Joint friction (Rasuli Table II) ---------------------------------
P.fv3      = 3800;         % N*m*s/rad  revolute (saddle) viscous
P.fv3_degs = P.fv3*pi/180; % = 66.3225 N*m/(deg/s), the unit the Simscape
                           %   Revolute Joint dialog expects (VERIFIED in
                           %   .slx: DampingCoefficient = 66.322512)
P.fc3      = 340;          % N*m        revolute Coulomb  (NOT yet in model)
P.fv4      = 1473;         % N*s/m      prismatic (crowd) viscous (in model)
P.fc4      = 480;          % N          prismatic Coulomb (NOT yet in model)

% ---- Flags describing what the .slx actually contains -----------------
P.model_has_coulomb = false;   % flip to true once fc3/fc4 are wired in
end
