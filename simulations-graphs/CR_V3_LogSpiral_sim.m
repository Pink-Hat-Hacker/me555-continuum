function res = CR_V3_LogSpiral_sim(varargin)
%CR_V3_LogSpiral_sim  Tendon-driven motion simulation of the V3 log-spiral
%                     continuum robot, animated directly on the STL mesh.
%
%   CR_V3_LogSpiral_sim                      % run with defaults, play animation
%   CR_V3_LogSpiral_sim('stl','myfile.STL')  % choose another STL
%   CR_V3_LogSpiral_sim('thetaMaxDeg',45)    % change joint limit
%   res = CR_V3_LogSpiral_sim('graphics',false)  % numbers only, no figure
%
%   No toolboxes needed. Tested in Octave; written for MATLAB R2016b+.
%
%   MODEL (what is and is not simulated)
%   ------------------------------------
%   * The STL is cut at its 8 notch joints (found automatically from the
%     thin neck of the tube) into a fixed base + 8 rigid units.
%   * Each joint is a 2-DOF hinge (bend about any axis perpendicular to the
%     backbone, no twist) located on the backbone axis at the notch apex.
%     Body i+1 is rotated about joint i, and the rotations chain up the
%     robot (forward kinematics of a rigid-link serial chain).
%   * Three straight cable channels (30, 150, 270 deg about the axis,
%     radius 18.55 - 0.0845 z mm, read from the STL) act as tendons.
%     Pulling cable j by d_j mm shortens it by sum_i r_i * (b_i . u_j)
%     where b_i is joint i's bend vector (rad), r_i the cable's lever arm
%     at that joint, u_j the unit vector toward the cable.
%   * Joint bend is found by minimising the elastic energy
%     sum k_i |b_i|^2 / 2 subject to the cable-length constraints, with
%     k_i = r_i^kExp  (joint stiffness scales with unit size).  Joints
%     that reach the notch limit thetaMax are clamped and the rest keep
%     bending (active-set solve), so the robot curls like a spiral.
%   * Not modelled: TPU material nonlinearity, cable friction, gravity,
%     self-contact of the units, axial compression.  The mesh between
%     units is blended over +/-hw mm so the neck deforms smoothly.
%
%   Cable pull d_j > 0 means the cable is pulled in; d_j < 0 means it is paid
%   out (two-cable steering needs one cable to feed out while the other pulls).
%
%   OUTPUT res (struct): joints (z, mm), b (2 x nJ x nT, rad),
%   pulls (nT x 3, mm), tip (nT x 3, mm), labels.

p = inputParser;
addParameter(p, 'stl', 'CR_3D_V6.STL');
addParameter(p, 'thetaMaxDeg', 30);       % joint limit per notch (deg)
addParameter(p, 'kExp', 2);               % joint stiffness k_i = r_i^kExp
addParameter(p, 'cableAngDeg', [30 150 270]);
addParameter(p, 'cableR0', 18.55);        % cable radius law  r = R0 - R1*z
addParameter(p, 'cableR1', 0.0845);
addParameter(p, 'axialMaxMm', 15);         % max axial closure per joint when all 3 cables pull (mm) - ASSUMED, tune
addParameter(p, 'hw', 2.5);               % half-width of blend zone at joints (mm)
addParameter(p, 'framesPerRamp', 45);
addParameter(p, 'fps', 30);
addParameter(p, 'graphics', true);
addParameter(p, 'saveVideo', false);
parse(p, varargin{:});
o = p.Results;

%% ---------------------------------------------------------------- load STL
stlPath = o.stl;
if ~exist(stlPath, 'file')
    here = fileparts(mfilename('fullpath'));
    if exist(fullfile(here, o.stl), 'file')
        stlPath = fullfile(here, o.stl);
    else
        [f, d] = uigetfile({'*.stl;*.STL'}, 'Select the STL file');
        if isequal(f, 0), error('No STL file selected.'); end
        stlPath = fullfile(d, f);
    end
end
[V, F, T] = readSTL(stlPath);             % V: 3n x 3 triangle soup, T: n x 9
zTop = max(V(:,3));

%% ---------------------------- find backbone axis and the notch joints
% Sample points along every triangle edge so long, thin triangles in the
% tube are not missed.
tt = linspace(0, 1, 12);
P = zeros(0, 3);
for e = 1:3
    a = T(:, 3*(e-1)+(1:3));
    b = T(:, 3*mod(e,3)+(1:3));
    for t = tt
        P = [P; a + t*(b - a)]; %#ok<AGROW>
    end
end

axy = [18.77 20.13];                       % starting guess (from earlier STL measurements)
for it = 1:4
    r = hypot(P(:,1)-axy(1), P(:,2)-axy(2));
    m = r < 2.6 & P(:,3) > 25 & P(:,3) < 135;   % points on the thin central tube
    axy = mean(P(m, 1:2), 1);
end
r = hypot(P(:,1)-axy(1), P(:,2)-axy(2));

dz = 0.25; z0 = 15;
bin = floor((P(:,3) - z0)/dz) + 1;
ok = bin >= 1;
nb = max(bin);
R = accumarray(bin(ok), r(ok), [nb 1], @max, NaN);
zc = z0 + ((1:nb)' - 0.5)*dz;
below = R < 2.3;                           % only the tube (r~1.9) is left here
d = diff([0; below; 0]);
st = find(d == 1); en = find(d == -1) - 1;
zj = [];
for k = 1:numel(st)
    zs = zc(st(k)); ze = zc(en(k));
    if zs < 25                             % base block cuts this notch short
        zj(end+1) = ze;                    %#ok<AGROW>
    elseif ze < zTop - 8                   % ignore the rounded tip
        zj(end+1) = (zs + ze)/2;           %#ok<AGROW>
    end
end
zj = zj(:);
nJ = numel(zj);
if nJ ~= 8
    warning('Found %d joints (expected 8). Check the STL / detection settings.', nJ);
end
jp = [repmat(axy, nJ, 1), zj];             % joint points (on axis)
rLever = o.cableR0 - o.cableR1*zj;         % cable lever arm at each joint
kJ = rLever.^o.kExp;
thMax = o.thetaMaxDeg*pi/180;
phi = o.cableAngDeg(:)'*pi/180;
U = [cos(phi); sin(phi)];                  % 2 x 3 unit vectors toward each cable
DFull = sum(rLever)*thMax;                 % pull that closes every joint fully
fprintf('Axis at (%.2f, %.2f) mm, joints z = %s mm\n', axy(1), axy(2), mat2str(zj', 4));
fprintf('Lever arms (mm): %s\n', mat2str(rLever', 3));
fprintf('Full-curl cable pull ~ %.1f mm (all joints at %.0f deg)\n', DFull, o.thetaMaxDeg);

%% -------------------------------------------------- build the trajectory
n = o.framesPerRamp;
ease = @(s) 0.5 - 0.5*cos(pi*s);           % smooth 0..1
up = ease(linspace(0, 1, n));
dn = fliplr(up);
pulls = zeros(0, 3); acts = false(0, 3); labels = {};
for j = 1:3                                % single-cable curl and release
    for f = [up dn]
        dd = zeros(1, 3); dd(j) = f*DFull;
        aa = false(1, 3); aa(j) = true;
        pulls(end+1, :) = dd; acts(end+1, :) = aa; %#ok<AGROW>
        labels{end+1} = sprintf('Pull cable %d (%d deg)', j, o.cableAngDeg(j)); %#ok<AGROW>
    end
end
for amp = [0.5 0.9]                        % steer the bend direction around 360 deg
    psi = linspace(0, 2*pi, 3*n);
    for f = up                             % ramp amplitude in first
        [pulls(end+1, :), acts(end+1, :)] = steerPull(0, amp*f, U, DFull);
        labels{end+1} = sprintf('Curl toward 0 deg, %d%%', round(amp*100)); %#ok<AGROW>
    end
    for ps = psi
        [pulls(end+1, :), acts(end+1, :)] = steerPull(ps, amp, U, DFull);
        labels{end+1} = sprintf('Steer bend direction (%.0f%% curl)', amp*100); %#ok<AGROW>
    end
    for f = dn
        [pulls(end+1, :), acts(end+1, :)] = steerPull(2*pi, amp*f, U, DFull);
        labels{end+1} = sprintf('Release, %d%%', round(amp*100)); %#ok<AGROW>
    end
end
% --- all three cables pulled together: no net bending, the body shortens ---
for f = [up dn]
    pulls(end+1, :) = f*nJ*o.axialMaxMm*[1 1 1];
    acts(end+1, :) = true(1, 3);
    labels{end+1} = 'Pull all 3 cables (axial shortening)'; %#ok<AGROW>
end
nT = size(pulls, 1);

%% ------------------------------------------------ solve every frame (FK)
bAll = zeros(2, nJ, nT);
Hall = zeros(4, 4, nJ+1, nT);
for t = 1:nT
    bAll(:,:,t) = solveJoints(pulls(t,:)', acts(t,:)', rLever, kJ, thMax, U);
    % Equal pull on all 3 cables is the common mode: it cannot bend the
    % backbone, so it closes the notches axially, shared equally by the joints.
    delta = zeros(nJ, 1);
    if all(acts(t,:))
        delta(:) = min(mean(pulls(t,:))/nJ, o.axialMaxMm);
    end
    Hall(:,:,:,t) = chainTransforms(bAll(:,:,t), jp, delta);
end

% Cable polylines (reference positions inside the channels)
zc_cab = linspace(2, zTop - 3, 70)';
cabRef = cell(1, 3);
for j = 1:3
    rr = o.cableR0 - o.cableR1*zc_cab;
    cabRef{j} = [axy(1) + rr*cos(phi(j)), axy(2) + rr*sin(phi(j)), zc_cab];
end
tipRef = [axy(1) axy(2) zTop];

skMesh = prepSkin(V, zj, o.hw);
skCab = cellfun(@(c) prepSkin(c, zj, o.hw), cabRef, 'UniformOutput', false);
skTip = prepSkin(tipRef, zj, o.hw);

tip = zeros(nT, 3);
for t = 1:nT
    tip(t,:) = applySkin(skTip, tipRef, Hall(:,:,:,t));
end

res = struct('joints', zj, 'axis', axy, 'b', bAll, 'pulls', pulls, ...
             'tip', tip, 'acts', acts, 'labels', {labels}, 'DFull', DFull, 'H', Hall, ...
             'V', V, 'F', F, 'skin', skMesh);
if ~o.graphics, return; end

%% ------------------------------------------------------------ animation
fig = figure('Color', 'w', 'Name', 'CR V3 log-spiral simulation', ...
             'Position', [60 60 1150 620]);
ax1 = axes('Parent', fig, 'Position', [0.02 0.04 0.64 0.88]);
hold(ax1, 'on');
hp = patch('Parent', ax1, 'Faces', F, 'Vertices', V, ...
           'FaceColor', [0.20 0.55 0.85], 'EdgeColor', 'none', ...
           'FaceLighting', 'gouraud', 'AmbientStrength', 0.45, ...
           'DiffuseStrength', 0.7, 'SpecularStrength', 0.15);
light('Parent', ax1, 'Position', [-1 -1.5 2], 'Style', 'infinite');
light('Parent', ax1, 'Position', [1.5 1 0.5], 'Style', 'infinite', 'Color', [0.5 0.5 0.5]);
cabCol = [0.85 0.2 0.2; 0.1 0.65 0.25; 0.9 0.6 0.05];
hc = [];
for j = 1:3
    hc(j) = plot3(ax1, cabRef{j}(:,1), cabRef{j}(:,2), cabRef{j}(:,3), '-', ...
                  'Color', cabCol(j,:), 'LineWidth', 1.8);
end
htr = plot3(ax1, tip(1,1), tip(1,2), tip(1,3), 'k-', 'LineWidth', 1.2);
htp = plot3(ax1, tip(1,1), tip(1,2), tip(1,3), 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 7);
axis(ax1, 'equal');
xlim(ax1, axy(1) + [-110 110]); ylim(ax1, axy(2) + [-110 110]); zlim(ax1, [0 170]);
view(ax1, 38, 22); grid(ax1, 'on'); box(ax1, 'on');
xlabel(ax1, 'x (mm)'); ylabel(ax1, 'y (mm)'); zlabel(ax1, 'z (mm)');
ht = title(ax1, '', 'FontWeight', 'bold');
legend(ax1, hc, arrayfun(@(a) sprintf('Cable %d (%d\\circ)', find(o.cableAngDeg==a,1), a), ...
       o.cableAngDeg, 'UniformOutput', false), 'Location', 'northwest');

ax2 = axes('Parent', fig, 'Position', [0.74 0.52 0.24 0.38]);
hb = bar(ax2, 1:nJ, zeros(1, nJ), 0.7, 'FaceColor', [0.2 0.55 0.85]);
hold(ax2, 'on'); plot(ax2, [0.4 nJ+0.6], o.thetaMaxDeg*[1 1], 'r--');
xlim(ax2, [0.4 nJ+0.6]); ylim(ax2, [0 o.thetaMaxDeg*1.25]);
xlabel(ax2, 'joint (1 = base)'); ylabel(ax2, 'bend (deg)');
title(ax2, 'Joint bend vs. limit'); grid(ax2, 'on');

ax3 = axes('Parent', fig, 'Position', [0.74 0.10 0.24 0.28]);
hl = bar(ax3, 1:3, zeros(1, 3), 0.6, 'FaceColor', [0.4 0.4 0.4]);
xlim(ax3, [0.4 3.6]); ylim(ax3, [-DFull*0.55, max(DFull, nJ*o.axialMaxMm)*1.05]);
set(ax3, 'XTick', 1:3);
xlabel(ax3, 'cable'); ylabel(ax3, 'pull (mm), <0 = paid out'); title(ax3, 'Cable pull'); grid(ax3, 'on');

vw = [];
if o.saveVideo && exist('VideoWriter', 'file')
    vw = VideoWriter('CR_V3_LogSpiral_sim.mp4', 'MPEG-4'); vw.FrameRate = o.fps; open(vw);
end

for t = 1:nT
    if ~ishandle(fig), break; end
    H = Hall(:,:,:,t);
    set(hp, 'Vertices', applySkin(skMesh, V, H));
    for j = 1:3
        Q = applySkin(skCab{j}, cabRef{j}, H);
        set(hc(j), 'XData', Q(:,1), 'YData', Q(:,2), 'ZData', Q(:,3));
    end
    set(htr, 'XData', tip(1:t,1), 'YData', tip(1:t,2), 'ZData', tip(1:t,3));
    set(htp, 'XData', tip(t,1), 'YData', tip(t,2), 'ZData', tip(t,3));
    set(hb, 'YData', sqrt(sum(bAll(:,:,t).^2, 1))*180/pi);
    set(hl, 'YData', pulls(t,:));
    set(ht, 'String', labels{t});
    drawnow;
    if ~isempty(vw), writeVideo(vw, getframe(fig)); end
    pause(1/o.fps);
end
if ~isempty(vw), close(vw); end
end

%% =====================================================================
%  Local functions
%% =====================================================================

function [d, act] = steerPull(psi, amp, U, DFull)
% Cable pulls that bend the robot toward direction psi (rad). The two cables
% that bracket psi are held taut (the third goes slack); cable j must shorten
% by amp*DFull*(vhat . u_j) when every joint bends by the same fraction amp.
vh = [cos(psi); sin(psi)];
d = zeros(1, 3); act = false(1, 3);
for a = 1:3
    for b = a+1:3
        c = [U(:,a) U(:,b)] \ vh;
        if all(c >= -1e-9)
            act([a b]) = true;
            d([a b]) = amp*DFull*(vh'*U(:,[a b]));   % may be negative = cable paid out
            return
        end
    end
end
end

function b = solveJoints(d, actMask, r, k, thMax, U)
% Static joint bend vectors b (2 x nJ, rad) for cable pulls d (3 x 1, mm)
% on the taut cables actMask (3 x 1 logical).
%   cable j shortens by  sum_i r_i * (b_i . u_j)   (must equal d_j if pulled)
%   minimise sum k_i |b_i|^2/2  ->  b_i = (r_i/k_i) * U*mu
% Joints at the limit are clamped and the remainder re-solved.
nJ = numel(r);
b = zeros(2, nJ);
act = find(actMask);                       % taut cables are constraints, the rest are slack
if isempty(act), return; end
UA = U(:, act);
sat = false(nJ, 1);
for it = 1:nJ+1
    free = ~sat;
    if ~any(free), break; end
    dres = d(act) - UA' * (b(:, sat) * r(sat));
    S = sum(r(free).^2 ./ k(free));
    mu = pinv(S*(UA'*UA)) * dres;
    v = UA*mu;
    b(:, free) = v * (r(free)./k(free))';
    mag = sqrt(sum(b.^2, 1))';
    ratio = mag/thMax; ratio(sat) = 0;
    [mx, i] = max(ratio);
    if mx <= 1 + 1e-9, break; end
    dirv = b(:, i)/mag(i);
    b(:, i) = thMax*dirv;
    sat(i) = true;
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
    Hi = [Rm, pi_ - Rm*pi_ + [0; 0; -delta(i)]; 0 0 0 1];   % rotate about joint, then close notch along z
    H(:,:,i+1) = H(:,:,i)*Hi;
end
end

function Rm = rotvec(w)
th = norm(w);
if th < 1e-12, Rm = eye(3); return; end
a = w/th;
K = [0 -a(3) a(2); a(3) 0 -a(1); -a(2) a(1) 0];
Rm = eye(3) + sin(th)*K + (1 - cos(th))*(K*K);
end

function sk = prepSkin(P, zj, hw)
% For every point: which two bodies it blends between and the weight.
z = P(:,3);
nJ = numel(zj);
body = sum(bsxfun(@ge, z, zj(:)'), 2);      % 0..nJ
lo = body; hi = body; w = zeros(size(z));
for i = 1:nJ
    m = abs(z - zj(i)) < hw;
    s = (z(m) - zj(i) + hw)/(2*hw);
    lo(m) = i-1; hi(m) = i; w(m) = s.^2.*(3 - 2*s);
end
sk = struct('lo', lo, 'hi', hi, 'w', w, 'nB', nJ+1);
end

function Q = applySkin(sk, P, H)
Q = zeros(size(P));
for b = 0:sk.nB-1
    Rm = H(1:3,1:3,b+1); tv = H(1:3,4,b+1)';
    m = sk.lo == b;
    if any(m)
        Q(m,:) = Q(m,:) + bsxfun(@times, 1 - sk.w(m), bsxfun(@plus, P(m,:)*Rm', tv));
    end
    m = sk.hi == b;
    if any(m)
        Q(m,:) = Q(m,:) + bsxfun(@times, sk.w(m), bsxfun(@plus, P(m,:)*Rm', tv));
    end
end
end

function [V, F, T] = readSTL(fn)
% Binary or ASCII STL -> triangle soup (no toolbox needed).
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
    nT = size(A,1)/3;
    T = [A(1:3:end,:) A(2:3:end,:) A(3:3:end,:)];
end
V = [T(:,1:3); T(:,4:6); T(:,7:9)];
F = [(1:nT)' (nT+1:2*nT)' (2*nT+1:3*nT)'];
end
