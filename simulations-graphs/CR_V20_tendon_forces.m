function res = CR_V20_tendon_forces(varargin)
%CR_V20_tendon_forces  Inverse statics: the tendon tensions that hold the V20
%                      tip at a target position, with optional tip payload.
%
%   CR_V20_tendon_forces                              % demo targets + plot + CSV
%   res = CR_V20_tendon_forces('targets', [60 -40 250]);
%   res = CR_V20_tendon_forces('targets', tg, 'tipLoadN', [0 0 -0.49]);  % 50 g
%
%   HOW THE FORCES ARE FOUND (pure statics, no dynamics)
%   ----------------------------------------------------
%   Each notch is treated as a torsion spring: bending joint i by the bend
%   vector b_i (rad) takes a moment K_i*b_i, with
%       K_i = E * I_i / L_neck,   I_i = pi/4 * (rNeck_i^4 - rHole^4)
%   the beam bending stiffness of the thin neck (annular cross-section; the
%   neck radius of every joint is measured from the STL). A cable with
%   tension T_j (constant along the cable - NO friction) passes joint i at
%   lever arm r_i, applying bending moment r_i*T_j toward the cable direction
%   u_j. Static moment balance at every joint:
%       K_i * b_i = r_i * w + m_ext,i     with    w = sum_j T_j * u_j
%   (m_ext,i = moment of the optional external tip force about joint i).
%   One 2-vector w ("bend effort", in N) therefore sets the whole shape. The
%   script searches w so the forward kinematics put the tip on the target
%   (fminsearch), then splits w into the two non-negative tensions of the
%   cables that bracket its direction (the third cable is slack, T = 0).
%
%   The reachable tip set is only a 2-DOF surface (bend direction x amount),
%   so an arbitrary xyz target is matched as closely as possible and the
%   residual err_mm is reported - check it before trusting the tensions.
%
%   ASSUMPTIONS - READ BEFORE TRUSTING THE NUMBERS
%   * E_MPa defaults to 26 N/mm^2 (typical TPU 95A datasheet value). Printed
%     TPU is softer and anisotropic: CALIBRATE by hanging a known weight on
%     the tip, measuring the sag, and scaling E until the model matches.
%   * neckLenMm (effective bending length of a notch, default 4 mm) is a
%     tuning parameter too; the measured flat length of each neck (~1.5 mm)
%     is printed for reference.
%   * No cable friction (cf. the octopus-reaching paper in this course:
%     friction in spiral channels is NOT small), no weight of the arm itself,
%     linear elasticity, small-angle cable routing - the same fidelity as
%     CR_V3_LogSpiral_sim.
%   * Tensions are meaningful only while no joint is clamped at thetaMaxDeg:
%     a fully closed notch also carries contact moment, so the reported
%     tension there is a lower bound. nSat in the output counts clamped
%     joints.
%
%   DATA OUTPUT
%   * CSV (csvFile, written next to this script): one row per target with
%     target, achieved tip, err_mm, tensions T1..T3 (N), cable pulls
%     d1..d3 (mm, <0 = pay out), max joint bend (deg) and nSat.
%   * res struct: all of the above plus per-joint z, neck radius, stiffness
%     K, lever arms, bend vectors and world joint positions of every pose.
%   * 3D plot of every solved pose against its target.

p = inputParser;
addParameter(p, 'stl', 'CR_3D_V20.STL');
addParameter(p, 'targets', []);           % n x 3 tip targets (mm); [] = demo set
addParameter(p, 'E_MPa', 26);             % TPU Young's modulus (N/mm^2) - CALIBRATE
addParameter(p, 'neckInnerR', 0.8);       % central hole radius in the neck (mm)
addParameter(p, 'neckLenMm', 4);          % effective bending length of a notch (mm) - TUNE
addParameter(p, 'thetaMaxDeg', 30);       % notch limit, as in the motion sim
addParameter(p, 'cableAngDeg', [32 152 272]);   % V20 channels (measured)
addParameter(p, 'cableZ', [3 23.4 64.9 103.2 137.2 167.8 194.5 218.3 239.4 258.5 275.1 289.9 303 314.3 321]);
addParameter(p, 'cableR', [33.93 32.17 29.24 26.74 23.93 21.29 18.87 16.63 14.58 12.67 11.03 9.53 8.26 7.18 6.57]);
addParameter(p, 'axisGuess', [35.31 38.51]);
addParameter(p, 'nJointsExpected', 13);
addParameter(p, 'tipLoadN', [0 0 0]);     % external tip force, world frame (N); [0 0 -0.49] = 50 g payload
addParameter(p, 'csvFile', 'CR_V20_tendon_forces.csv');
addParameter(p, 'graphics', true);
parse(p, varargin{:});
o = p.Results;

%% ------------------------------------------------- load STL, find geometry
T = readSTLtri(locateSTL(o.stl));
zTop = max(max(T(:, [3 6 9])));
[axy, zj, rNeck, flatLen] = findJoints(T, zTop, o.axisGuess);
nJ = numel(zj);
if nJ ~= o.nJointsExpected
    warning('Found %d joints (expected %d). Check the STL / detection settings.', nJ, o.nJointsExpected);
end
rLever = interp1(o.cableZ(:), o.cableR(:), zj, 'linear', 'extrap');  % cable lever arm (mm)
Ia = pi/4*(rNeck.^4 - o.neckInnerR^4);     % second moment of area of the neck (mm^4)
K  = o.E_MPa * Ia / o.neckLenMm;           % joint stiffness (N*mm/rad)
thMax = o.thetaMaxDeg*pi/180;
phi = o.cableAngDeg(:)'*pi/180;
U = [cos(phi); sin(phi)];                  % 2 x 3 unit vectors toward each cable
jp = [repmat(axy, nJ, 1), zj];             % joint points (on the axis)
tipRef = [axy, zTop];
Fext = o.tipLoadN(:);

fprintf('Axis at (%.2f, %.2f) mm, %d joints. E = %g MPa, neck length = %g mm, hole r = %g mm\n', ...
        axy(1), axy(2), nJ, o.E_MPa, o.neckLenMm, o.neckInnerR);
fprintf(' joint |  z (mm) | neck r (mm) | flat len (mm) | K (N*mm/rad) | lever r (mm)\n');
for i = 1:nJ
    fprintf(' %5d | %7.2f | %11.2f | %13.2f | %12.1f | %12.2f\n', ...
            i, zj(i), rNeck(i), flatLen(i), K(i), rLever(i));
end
mFull = max(K(:)*thMax ./ rLever(:));      % bend effort that clamps even the stiffest joint
fprintf('Bend effort |w| to clamp the first joint: %.2f N; to clamp all joints: %.2f N\n', ...
        min(K(:)*thMax ./ rLever(:)), mFull);
if any(Fext), fprintf('External tip load: [%g %g %g] N\n', Fext); end

%% ------------------------------------------------------------ demo targets
targets = o.targets;
if isempty(targets)
    targets = zeros(0, 3);
    for a = [45 165 285]*pi/180
        for m = [0.3 0.7]*mFull
            tip = forwardW(m*[cos(a); sin(a)], K, rLever, thMax, jp, tipRef, Fext);
            targets(end+1, :) = tip; %#ok<AGROW>
        end
    end
    fprintf('\nNo targets given: using %d demo targets on the reachable surface.\n', size(targets, 1));
end
nT = size(targets, 1);

%% --------------------------------------------- inverse statics per target
opt = optimset('TolX', 1e-7, 'TolFun', 1e-7, 'MaxFunEvals', 4000, 'MaxIter', 4000);
res = struct('targets', targets, 'tip', zeros(nT,3), 'err', zeros(nT,1), ...
             'T', zeros(nT,3), 'pulls', zeros(nT,3), 'w', zeros(nT,2), ...
             'b', zeros(2,nJ,nT), 'maxBendDeg', zeros(nT,1), 'nSat', zeros(nT,1), ...
             'joints', zj, 'rNeck', rNeck, 'K', K, 'rLever', rLever, 'axis', axy, ...
             'pw', zeros(nJ,3,nT), 'E_MPa', o.E_MPa, 'tipLoadN', Fext');
fprintf('\n tgt |   target (mm)        |  achieved tip (mm)   | err  |   T1    T2    T3 (N) |  pulls (mm)          | maxBend | sat\n');
for t = 1:nT
    tg = targets(t, :);
    psi0 = atan2(tg(2)-axy(2), tg(1)-axy(1));
    bw = []; be = Inf;
    for m0 = [0.1 0.3 0.6 0.9]*mFull           % multistart along the target azimuth
        f = @(w) norm(forwardW(w, K, rLever, thMax, jp, tipRef, Fext) - tg);
        [ws, fv] = fminsearch(f, m0*[cos(psi0); sin(psi0)], opt);
        if fv < be, be = fv; bw = ws; end
    end
    [tip, b, pw] = forwardW(bw, K, rLever, thMax, jp, tipRef, Fext);
    Tn = tensionsFromW(bw, U);
    d = sum((b'*U) .* repmat(rLever(:), 1, 3), 1);     % cable length change (mm)
    bend = sqrt(sum(b.^2, 1));
    nSat = sum(bend >= thMax - 1e-9);
    res.tip(t,:) = tip;  res.err(t) = be;  res.T(t,:) = Tn;  res.pulls(t,:) = d;
    res.w(t,:) = bw';  res.b(:,:,t) = b;  res.maxBendDeg(t) = max(bend)*180/pi;
    res.nSat(t) = nSat;  res.pw(:,:,t) = pw;
    fprintf(' %3d | %6.1f %6.1f %6.1f | %6.1f %6.1f %6.1f | %4.2f | %5.2f %5.2f %5.2f | %6.1f %6.1f %6.1f | %6.1f  | %2d\n', ...
            t, tg, tip, be, Tn, d, res.maxBendDeg(t), nSat);
    if be > 2
        fprintf('       -> target is %.1f mm off the reachable surface; nearest point used.\n', be);
    end
end

%% ------------------------------------------------------------- CSV output
csv = o.csvFile;
if isempty(fileparts(csv)), csv = fullfile(fileparts(mfilename('fullpath')), csv); end
fid = fopen(csv, 'w');
fprintf(fid, ['target_x_mm,target_y_mm,target_z_mm,tip_x_mm,tip_y_mm,tip_z_mm,err_mm,' ...
              'T1_N,T2_N,T3_N,pull1_mm,pull2_mm,pull3_mm,maxBend_deg,nSatJoints\n']);
for t = 1:nT
    fprintf(fid, '%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%d\n', ...
            targets(t,:), res.tip(t,:), res.err(t), res.T(t,:), res.pulls(t,:), ...
            res.maxBendDeg(t), res.nSat(t));
end
fclose(fid);
res.csv = csv;
fprintf('\nWrote %s\n', csv);

%% ------------------------------------------------------------------- plot
if ~o.graphics, return; end
figure('Color', 'w', 'Position', [60 60 760 700]); hold on;
plot3(axy(1)*[1 1], axy(2)*[1 1], [0 zTop], ':', 'Color', [0.5 0.5 0.5]);
cols = hsv(max(nT, 2))*0.85;
for t = 1:nT
    Pn = [axy 0; res.pw(:,:,t); res.tip(t,:)];
    plot3(Pn(:,1), Pn(:,2), Pn(:,3), '-o', 'Color', cols(t,:), ...
          'MarkerFaceColor', cols(t,:), 'MarkerSize', 4, 'LineWidth', 1.5);
    plot3(targets(t,1), targets(t,2), targets(t,3), 'x', 'Color', cols(t,:), ...
          'MarkerSize', 12, 'LineWidth', 2);
end
axis equal; grid on; box on; view(38, 22);
xlabel('x (mm)'); ylabel('y (mm)'); zlabel('z (mm)');
title(sprintf('Backbone poses (o-) and tip targets (x), E = %g MPa', o.E_MPa));
end

%% ======================================================================
%  Local functions
%% ======================================================================

function [tip, b, pw] = forwardW(w, K, r, thMax, jp, tipRef, Fext)
% Shape and tip for a given bend effort w (2x1, N): b_i = (r_i*w + m_ext,i)/K_i,
% clipped at the notch limit. With an external tip force the moment arm depends
% on the deformed shape, so iterate a few fixed-point passes.
nJ = numel(r);
b = zeros(2, nJ);
nit = 1 + 5*any(Fext ~= 0);
for it = 1:nit
    [pw, tip] = chainPts(b, jp, tipRef);
    for i = 1:nJ
        mext = [0; 0];
        if any(Fext)
            M = cross((tip - pw(i,:))', Fext);  % moment of the tip force about joint i
            mext = [M(2); -M(1)];               % its bending components (base frame)
        end
        bi = (r(i)*w(:) + mext) / K(i);
        nb = norm(bi);
        if nb > thMax, bi = bi*(thMax/nb); end
        b(:, i) = bi;
    end
end
[pw, tip] = chainPts(b, jp, tipRef);
end

function [pw, tip] = chainPts(b, jp, tipRef)
% World positions of the joints and the tip for bend vectors b.
nJ = size(jp, 1);
H = chainTransforms(b, jp, zeros(nJ, 1));
pw = zeros(nJ, 3);
for i = 1:nJ
    q = H(:,:,i)*[jp(i,:)'; 1];  pw(i,:) = q(1:3)';
end
q = H(:,:,nJ+1)*[tipRef'; 1];  tip = q(1:3)';
end

function Tn = tensionsFromW(w, U)
% Minimal non-negative cable tensions with sum_j T_j*u_j = w: the two cables
% bracketing w's direction pull, the third is slack.
Tn = zeros(1, 3);
if norm(w) < 1e-12, return; end
for a = 1:3
    for b = a+1:3
        c = [U(:,a) U(:,b)] \ w(:);
        if all(c >= -1e-9)
            Tn([a b]) = max(c', 0);
            return
        end
    end
end
end

function H = chainTransforms(b, jp, delta)
% H(:,:,i+1) = pose of the body above joint i (H(:,:,1) = fixed base).
nJ = size(jp, 1);
H = zeros(4, 4, nJ+1);
H(:,:,1) = eye(4);
for i = 1:nJ
    w = [-b(2,i); b(1,i); 0];               % rotation vector that tilts +z toward b
    Rm = rotvec(w);
    pi_ = jp(i,:)';
    Hi = [Rm, pi_ - Rm*pi_ + [0; 0; -delta(i)]; 0 0 0 1];
    H(:,:,i+1) = H(:,:,i)*Hi;
end
end

function Rm = rotvec(w)
th = norm(w);
if th < 1e-12, Rm = eye(3); return; end
a = w/th;
Kx = [0 -a(3) a(2); a(3) 0 -a(1); -a(2) a(1) 0];
Rm = eye(3) + sin(th)*Kx + (1 - cos(th))*(Kx*Kx);
end

function [axy, zj, rNeck, flatLen] = findJoints(T, zTop, axisGuess)
% Axis (x,y) and notch joints of the V20 STL: joints are the flat bottoms of
% local minima of the radial extent R(z); the minimum there is the neck radius.
tt = linspace(0, 1, 40);
P = zeros(0, 3);
for e = 1:3
    a = T(:, 3*(e-1)+(1:3));
    b = T(:, 3*mod(e,3)+(1:3));
    for t = tt
        P = [P; a + t*(b - a)]; %#ok<AGROW>
    end
end
axy = axisGuess;
for it = 1:4
    r = hypot(P(:,1)-axy(1), P(:,2)-axy(2));
    m = r < 1.9 & P(:,3) > zTop-30 & P(:,3) < zTop-17;   % central tube near the tip neck
    axy = mean(P(m, 1:2), 1);
end
r = hypot(P(:,1)-axy(1), P(:,2)-axy(2));
dz = 0.25; z0 = 0;
bin = floor((P(:,3) - z0)/dz) + 1;
ok = bin >= 1;
nb = max(bin);
R = accumarray(bin(ok), r(ok), [nb 1], @max, NaN);
zc = z0 + ((1:nb)' - 0.5)*dz;
w = round(6/dz); w2 = round(20/dz);
isN = false(nb, 1);
for i = 1:nb
    if isnan(R(i)), continue; end
    a = max(1, i-w); b = min(nb, i+w); a2 = max(1, i-w2); b2 = min(nb, i+w2);
    isN(i) = R(i) <= min(R(a:b)) + 0.1 && R(i) < 0.35*max(R(a2:b2));
end
d = diff([0; isN; 0]);
st = find(d == 1); en = find(d == -1) - 1;
zj = []; rNeck = []; flatLen = [];
for k = 1:numel(st)
    zs = zc(st(k)); ze = zc(en(k));
    if ze < zTop - 3                        % ignore the rounded tip apex
        zj(end+1) = (zs + ze)/2;            %#ok<AGROW>
        rNeck(end+1) = min(R(st(k):en(k))); %#ok<AGROW>
        flatLen(end+1) = ze - zs + dz;      %#ok<AGROW>
    end
end
zj = zj(:); rNeck = rNeck(:); flatLen = flatLen(:);
end

function fname = locateSTL(fname)
here = fileparts(mfilename('fullpath'));
cands = {fname, fullfile(here, fname)};
for i = 1:numel(cands)
    if exist(cands{i}, 'file') == 2, fname = cands{i}; return; end
end
[f, p] = uigetfile({'*.stl;*.STL', 'STL files'}, 'Select CR_3D_V20.STL');
if isequal(f, 0), error('No STL file selected.'); end
fname = fullfile(p, f);
end

function T = readSTLtri(fn)
% Binary or ASCII STL -> n x 9 triangle corner list (no toolbox needed).
fid = fopen(fn, 'r');
if fid < 0, error('Cannot open %s', fn); end
fseek(fid, 0, 'eof'); sz = ftell(fid); fseek(fid, 80, 'bof');
nT = fread(fid, 1, 'uint32');
if sz == 84 + 50*nT
    raw = fread(fid, [50 nT], 'uint8=>uint8');
    fclose(fid);
    f = double(typecast(reshape(raw(13:48,:), [], 1), 'single'));
    T = reshape(f, 9, nT)';
else
    fclose(fid);
    txt = fileread(fn);
    tok = regexp(txt, 'vertex\s+(\S+)\s+(\S+)\s+(\S+)', 'tokens');
    A = reshape(str2double([tok{:}]), 3, [])';
    T = [A(1:3:end,:) A(2:3:end,:) A(3:3:end,:)];
end
end