function spirobs3_length_width()
% SCRIPT 3 - Unit length (along the axis) and unit width, SpiRobs model (red)
% against our STL (green), unit by unit from base (1) to tip.
%
%   STL length  : distance between consecutive joints (centre of the tube-only
%                 section at each notch). Last unit: joint to the tip end
%                 (the tip is rounded, so it reads slightly short).
%   STL width   : 2 x largest radial extent of the unit in the arm plane
%                 (includes the cable-guide bosses).
%   SpiRobs     : same quantities for the paper model (dtheta = 30 deg, b
%                 fitted to the joint spacing, i.e. lengths match by design;
%                 widths then follow from the spiral and are NOT fitted).
%
% Run:  spirobs3_length_width

    % ------------------------- user settings --------------------------
    STL_FILE   = 'CR_3D_V3_LogSpiral';
    ARM_AZ_DEG = 270;
    DTHETA_DEG = 30;
    B_FIXED    = [];          % [] = fit b to the STL joint spacing
    ALPHA_DEG  = [];          % [] = DTHETA_DEG/2
    X_MODE     = 'index';     % 'index' = unit number,  'z' = axial position of the unit centre (mm)
    SAVE_PNG   = false;
    % -------------------------------------------------------------------
    if isempty(ALPHA_DEG), ALPHA_DEG = DTHETA_DEG/2; end

    [P1, P2, P3] = readSTL(locateSTL(STL_FILE));
    G = analyzeSTL(P1, P2, P3);
    N = numel(G.zj);
    [S, ~] = slicePlane(P1, P2, P3, G.c, ARM_AZ_DEG);
    F = fitSpiral(G.zj, DTHETA_DEG, B_FIXED);
    M = spirobModel(F, N, ALPHA_DEG);

    stlLen = diff([G.zj(:); G.zhi]);                    % unit lengths along the axis
    stlR   = stlUnitWidths(S, G.zj, G.zhi);
    stlW   = 2*stlR;
    zc     = (G.zj(:) + [G.zj(2:end); G.zhi])/2;
    if strcmpi(X_MODE, 'z'), x = zc; xl = 'Axial position of unit centre z (mm)'; else, x = (1:N).'; xl = 'Unit number (1 = base end, N = tip)'; end

    % STL taper from a straight-line fit of radius vs z
    pf = polyfit(zc, stlR, 1);  taperSTL = 2*atand(abs(pf(1)));

    fprintf('\n unit |  z_base  |  STL len | SpiRobs len | STL width | SpiRobs width | width ratio (S/STL)\n');
    for k = 1:N
        fprintf(' %3d  | %8.2f | %8.2f | %11.2f | %9.2f | %13.2f | %6.0f %%\n', ...
            k, G.zj(k), stlLen(k), M.len(k), stlW(k), M.width(k), 100*M.width(k)/stlW(k));
    end
    fprintf('\nb = %.4f, beta = %.4f (unit-to-unit ratio %.4f), fit rms = %.3f mm\n', F.b, F.beta, 1/F.beta, F.rms);
    fprintf('Taper (full angle): STL %.1f deg | SpiRobs Eq.(3) %.1f deg\n', taperSTL, M.taperEq3);
    fprintf('STL length ratio between neighbours: %s\n', sprintf('%.4f ', stlLen(2:end)./stlLen(1:end-1)));
    fprintf('STL width  ratio between neighbours: %s\n\n', sprintf('%.4f ', stlW(2:end)./stlW(1:end-1)));

    cG = [0.10 0.62 0.20];  cR = [0.85 0.10 0.10];
    figure('Color', 'w', 'Position', [60 60 760 900]);
    subplot(3,1,1); hold on; grid on; box on;
    plot(x, stlLen, '-o', 'Color', cG, 'MarkerFaceColor', cG, 'LineWidth', 1.6);
    plot(x, M.len,  '-s', 'Color', cR, 'MarkerFaceColor', cR, 'LineWidth', 1.6);
    ylabel('Unit length along axis (mm)'); legend({'STL (Team 5)', 'SpiRobs'}, 'Location', 'northeast');
    title(sprintf('Unit length and width: SpiRobs vs our STL  (\\Delta\\theta = %g\\circ, b = %.3f)', DTHETA_DEG, F.b));
    subplot(3,1,2); hold on; grid on; box on;
    plot(x, stlW, '-o', 'Color', cG, 'MarkerFaceColor', cG, 'LineWidth', 1.6);
    plot(x, M.width, '-s', 'Color', cR, 'MarkerFaceColor', cR, 'LineWidth', 1.6);
    ylabel('Unit width (mm)'); legend({'STL (Team 5)', 'SpiRobs'}, 'Location', 'northeast');
    subplot(3,1,3); hold on; grid on; box on;
    plot(x, 100*M.len./stlLen, '-s', 'Color', [0.2 0.2 0.8], 'MarkerFaceColor', [0.2 0.2 0.8], 'LineWidth', 1.4);
    plot(x, 100*M.width./stlW, '-d', 'Color', [0.9 0.5 0.1], 'MarkerFaceColor', [0.9 0.5 0.1], 'LineWidth', 1.4);
    plot(xlim, [100 100], 'k:');
    ylabel('SpiRobs / STL (%)'); xlabel(xl); legend({'length', 'width'}, 'Location', 'northwest');
    if SAVE_PNG, print(gcf, '-dpng', '-r200', fullfile(fileparts(mfilename('fullpath')), 'spirobs3_length_width.png')); end
end

%% ======================================================================
%  SHARED HELPERS (identical block in all three scripts, so each file runs
%  on its own). No toolboxes needed; works in MATLAB R2016b+ and Octave.
%% ======================================================================
function fname = locateSTL(fname)
    here = fileparts(mfilename('fullpath'));
    cands = {fname, fullfile(here, fname)};
    for i = 1:numel(cands)
        if exist(cands{i}, 'file') == 2, fname = cands{i}; return; end
    end
    [f, p] = uigetfile({'*.stl;*.STL', 'STL files'}, 'Select CR_3D_Part1_V2.STL');
    if isequal(f, 0), error('No STL file selected.'); end
    fname = fullfile(p, f);
end

function [P1, P2, P3] = readSTL(fname)
% Reads a binary (or ASCII) STL. P1,P2,P3 are n-by-3 triangle corner lists.
    fid = fopen(fname, 'r');
    if fid < 0, error('Cannot open %s', fname); end
    raw = fread(fid, Inf, 'uint8=>uint8'); fclose(fid); raw = raw(:);
    nTri = double(typecast(raw(81:84), 'uint32'));
    if numel(raw) == 84 + 50*nTri                       % binary STL
        rec = reshape(raw(85:84+50*nTri), 50, nTri);
        B = rec(13:48, :); B = B(:);                    % skip normal, drop attr bytes
        V = reshape(double(typecast(B, 'single')), 3, []).';
    else                                                % ASCII STL
        tok = regexp(char(raw.'), 'vertex\s+(\S+)\s+(\S+)\s+(\S+)', 'tokens');
        V = zeros(numel(tok), 3);
        for i = 1:numel(tok), V(i, :) = str2double(tok{i}); end
    end
    P1 = V(1:3:end, :); P2 = V(2:3:end, :); P3 = V(3:3:end, :);
end

function S = crossSegs(P1, P2, P3, d1, d2, d3)
% Where a plane cuts the triangles (d = signed distance of each corner to
% the plane) return the cut segments S = [x1 y1 z1 x2 y2 z2].
    k = find(((d1.*d2 < 0) + (d2.*d3 < 0) + (d3.*d1 < 0)) == 2);
    A1 = P1(k, :); A2 = P2(k, :); A3 = P3(k, :);
    e1 = d1(k); e2 = d2(k); e3 = d3(k);
    m12 = e1.*e2 < 0;  m31 = e3.*e1 < 0;
    Q12 = A1 + (e1./(e1-e2)) .* (A2-A1);
    Q23 = A2 + (e2./(e2-e3)) .* (A3-A2);
    Q31 = A3 + (e3./(e3-e1)) .* (A1-A3);
    first = Q12;  first(~m12, :)  = Q23(~m12, :);
    second = Q31; second(~m31, :) = Q23(~m31, :);
    S = [first second];
end

function G = analyzeSTL(P1, P2, P3)
% Finds the robot axis (x,y centre of the central tube), the joints (where
% the cross-section shrinks to just the central tube) and the tip height.
    zlo = min([P1(:,3); P2(:,3); P3(:,3)]);  zhi = max([P1(:,3); P2(:,3); P3(:,3)]);
    tmin = min([P1(:,3), P2(:,3), P3(:,3)], [], 2);
    tmax = max([P1(:,3), P2(:,3), P3(:,3)], [], 2);
    L = zhi - zlo;
    zs = (zlo + 0.05) : 0.1 : (zhi - 0.05);   n = numel(zs);
    Sl = cell(n, 1);  ext = NaN(n, 1);  ctr = NaN(n, 2);
    for i = 1:n
        z = zs(i) + 1.2e-5;
        idx = find(tmin < z & tmax > z);
        S = crossSegs(P1(idx,:), P2(idx,:), P3(idx,:), P1(idx,3)-z, P2(idx,3)-z, P3(idx,3)-z);
        Sl{i} = S;
        if isempty(S), continue; end
        X = [S(:,1); S(:,4)];  Y = [S(:,2); S(:,5)];
        ext(i) = max(max(X)-min(X), max(Y)-min(Y));
        ctr(i, :) = [(max(X)+min(X))/2, (max(Y)+min(Y))/2];
    end
    win = zs(:) > zlo + 0.17*L & zs(:) < zhi - 0.07*L;
    e2 = ext;  e2(~win | isnan(e2)) = Inf;
    [emin, im] = min(e2);                       % thinnest slice = tube only
    c = ctr(im, :);  Rtube = emin/2;
    Rmax = NaN(n, 1);
    for i = 1:n
        S = Sl{i}; if isempty(S), continue; end
        X = [S(:,1); S(:,4)] - c(1);  Y = [S(:,2); S(:,5)] - c(2);
        Rmax(i) = max(sqrt(X.^2 + Y.^2));
    end
    isJ = Rmax < Rtube + 0.3;   isJ(isnan(isJ)) = false;
    d = diff([0; isJ(:); 0]);   st = find(d == 1);  en = find(d == -1) - 1;
    zj = zeros(numel(st), 1);  zA = zs(st);  zB = zs(en);
    for q = 1:numel(st), zj(q) = (zA(q) + zB(q))/2; end
    keep = zj < zhi - 3;                        % last run is the tip apex, not a joint
    zj = zj(keep);  zA = zA(keep);  zB = zB(keep);
    % The first notch is bounded below by the flat top of the base block, so
    % its tube-only run is cut short: take the joint from the run's upper end.
    if numel(zj) > 2
        runL = zB - zA;  medL = median(runL(2:end));
        if runL(1) < 0.85*medL, zj(1) = zB(1) - medL/2; end
    end
    G.c = c;  G.Rtube = Rtube;  G.zj = zj(:);  G.zlo = zlo;  G.zhi = zhi;
end

function [S, u] = slicePlane(P1, P2, P3, c, azDeg)
% Slice by the vertical plane through the axis c at azimuth azDeg.
% S = [r1 z1 r2 z2] with r = radial distance from the axis along azDeg.
    u = [cosd(azDeg) sind(azDeg)];  nrm = [-u(2) u(1)];
    f = @(P) (P(:,1)-c(1))*nrm(1) + (P(:,2)-c(2))*nrm(2) - 1.3e-5;
    S3 = crossSegs(P1, P2, P3, f(P1), f(P2), f(P3));
    rr = @(X, Y) (X-c(1))*u(1) + (Y-c(2))*u(2);
    S = [rr(S3(:,1), S3(:,2)), S3(:,3), rr(S3(:,4), S3(:,5)), S3(:,6)];
end

function F = fitSpiral(zj, dthetaDeg, bFixed)
% Fits the SpiRobs uncurled-joint model  z_j = zv - S*beta^(-j)  (j = 0 at
% the base) to the measured joint heights. beta = exp(b*dtheta) is paper Eq.(9).
% zv = virtual tip (cone apex), S = distance from it to the base joint.
    dth = dthetaDeg*pi/180;  N = numel(zj);  j = (0:N-1).';  zj = zj(:);
    pitch = diff(zj);
    beta0 = mean(pitch(1:end-1) ./ pitch(2:end));
    lsq = @(lb) deal_cost(lb, j, zj);
    if isempty(bFixed)
        lb = fminbnd(@(x) lsq(x), log(1.0005), log(1.8), optimset('TolX', 1e-12));
    else
        lb = bFixed * dth;
    end
    A = [ones(N,1), -exp(-j*lb)];  sol = A \ zj;
    F.zv = sol(1);  F.S = sol(2);  F.beta = exp(lb);  F.b = lb/dth;  F.dth = dth;
    F.rms = sqrt(mean((A*sol - zj).^2));  F.beta0 = beta0;
end

function cost = deal_cost(lb, j, zj)
    A = [ones(numel(j),1), -exp(-j*lb)];  sol = A \ zj;
    cost = sum((A*sol - zj).^2);
end

function M = spirobModel(F, N, alphaDeg)
% Uncurled SpiRobs model with N units (unit 1 at the base), built from the
% paper's equations: central spiral rho_c, unit width delta(theta)=2*rho_c*tanh(pi*b)
% (Eq. in 'Conceptual design'), constant ratio beta (Eq. 9).
% Each unit is a hexagon: apexes on the axis at its two joints, end faces
% running along the rays (half-length delta/2, tilted alphaDeg from radial).
    beta = F.beta;  b = F.b;  dth = F.dth;  al = alphaDeg*pi/180;
    j = (0:N).';
    M.zJ = F.zv - F.S*beta.^(-j);                      % joint heights, N+1 of them
    qp = sqrt(1 + beta^-2 - 2*cos(dth)/beta);          % chord / rho_c(base-side joint)
    M.len = diff(M.zJ);                                % unit length along the axis (chord)
    rhoB = M.len / qp;                                 % rho_c at each unit's base-side joint
    M.deltaB = 2*rhoB*tanh(pi*b);                      % face width at base-side joint
    M.deltaT = M.deltaB / beta;                        % face width at tip-side joint
    rB = M.deltaB/2*cos(al);  rT = M.deltaT/2*cos(al); % shoulder radii
    M.width = 2*max(rB, rT);                           % widest diameter of the unit
    M.poly = cell(N, 1);
    for k = 1:N
        zb = M.zJ(k); zt = M.zJ(k+1);
        ob = M.deltaB(k)/2*sin(al); ot = M.deltaT(k)/2*sin(al);
        z = [zb, zb+ob, zt-ot, zt, zt-ot, zb+ob, zb];
        r = [0, rB(k), rT(k), 0, -rT(k), -rB(k), 0];
        M.poly{k} = [z(:), r(:)];
    end
    M.rhoJ = [rhoB; rhoB(end)/beta];                   % rho_c at all N+1 joints
    M.qp = qp;  M.alphaDeg = alphaDeg;
    M.taperEq3 = 2*atand(b*(exp(2*pi*b)-1) / (sqrt(b^2+1)*(exp(2*pi*b)+1)));  % paper Eq.(3)
end

function R = stlUnitWidths(S, zJ, zTip)
% Radial half-width of each STL unit (largest r on the arm side in the slice).
    N = numel(zJ);  R = zeros(N, 1);
    zEdges = [zJ(:); zTip];
    zm = (S(:,2) + S(:,4))/2;
    for k = 1:N
        in = zm >= zEdges(k) & zm <= zEdges(k+1);
        R(k) = max([S(in,1); S(in,3)]);
    end
end
