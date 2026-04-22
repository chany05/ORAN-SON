%% MASAC / Baseline interactive visualizer - Display CIO + TXP + Edge UE
clear; clc; close all;

%% Load data
dataDir = '.';
traj   = readtable(fullfile(dataDir, 'ue_trajectory.csv'), 'VariableNamingRule', 'preserve');
kpm    = readtable(fullfile(dataDir, 'cell_metrics.csv'), 'VariableNamingRule', 'preserve');
action = readtable(fullfile(dataDir, 'maddpg_actions.csv'), 'VariableNamingRule', 'preserve');

traj   = keepLastEpisode(traj);
kpm    = keepLastEpisode(kpm);
action = keepLastEpisode(action);

cioVar = pickVar(action, {'selfCio_display_dB', 'selfCioDB'});
if isempty(cioVar)
    error('maddpg_actions.csv: CIO column not found.');
end

times = unique(traj.time_s);
nUEs  = max(traj.ueIndex) + 1;
fprintf('Time: %.1f~%.1fs (%d steps), %d UEs\n', min(times), max(times), numel(times), nUEs);

%% Current MASAC scenario parameters
areaCx    = 500;
areaCy    = 500;
enbRadius = 500 / sqrt(3);
enbPos    = [areaCx + enbRadius*cosd(90),  areaCy + enbRadius*sind(90);
             areaCx + enbRadius*cosd(210), areaCy + enbRadius*sind(210);
             areaCx + enbRadius*cosd(330), areaCy + enbRadius*sind(330)];
nEnb      = 3;
ueRadius  = enbRadius;
mobBounds = [areaCx - ueRadius - 10, areaCx + ueRadius + 10, ...
             areaCy - ueRadius - 10, areaCy + ueRadius + 10];
colors    = [0.2 0.4 0.8; 0.9 0.3 0.2; 0.2 0.7 0.3];
TXP_REF   = 32.0;
RAD_REF   = 0.82 * ueRadius;   % visualization-only reference radius at 32 dBm

%% Precompute UE trail
ueTrail = cell(nUEs, 1);
for ue = 0:nUEs-1
    rows = sortrows(traj(traj.ueIndex == ue, :), 'time_s');
    ueTrail{ue+1} = rows;
end

%% GUI
fig = figure('Name', 'MASAC Visualizer', ...
    'Position', [30 30 1500 850], 'Color', [0.97 0.97 0.97]);

axMap = axes('Parent', fig, 'Position', [0.03 0.10 0.58 0.85]);
axUE  = axes('Parent', fig, 'Position', [0.66 0.72 0.32 0.22]);
axThp = axes('Parent', fig, 'Position', [0.66 0.42 0.32 0.22]);
axAct = axes('Parent', fig, 'Position', [0.66 0.12 0.32 0.22]);

D.traj = traj;
D.kpm = kpm;
D.action = action;
D.times = times;
D.enbPos = enbPos;
D.nEnb = nEnb;
D.mobBounds = mobBounds;
D.areaCx = areaCx;
D.areaCy = areaCy;
D.ueRadius = ueRadius;
D.colors = colors;
D.TXP_REF = TXP_REF;
D.RAD_REF = RAD_REF;
D.cioVar = cioVar;
D.nUEs = nUEs;
D.ueTrail = ueTrail;
D.axMap = axMap;
D.axUE = axUE;
D.axThp = axThp;
D.axAct = axAct;
fig.UserData = D;

nSteps = numel(times);
uicontrol('Style', 'text', 'Units', 'normalized', ...
    'Position', [0.03 0.01 0.06 0.04], 'String', 'Time:', ...
    'FontSize', 10, 'BackgroundColor', [0.97 0.97 0.97]);
slider = uicontrol('Style', 'slider', 'Units', 'normalized', ...
    'Position', [0.10 0.01 0.50 0.04], ...
    'Min', 1, 'Max', nSteps, 'Value', 1, ...
    'SliderStep', [1 / max(nSteps - 1, 1), 5 / max(nSteps - 1, 1)]);
fig.UserData.slider = slider;

timeLabel = uicontrol('Style', 'text', 'Units', 'normalized', ...
    'Position', [0.61 0.01 0.08 0.04], ...
    'String', sprintf('t=%.1fs', times(1)), ...
    'FontSize', 10, 'BackgroundColor', [0.97 0.97 0.97]);
fig.UserData.time_label = timeLabel;

playBtn = uicontrol('Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.70 0.01 0.06 0.04], 'String', 'Play', 'FontSize', 10);
set(playBtn, 'Callback', {@playCallback, fig});
fig.UserData.play_btn = playBtn;
fig.UserData.playing = false;

trailChk = uicontrol('Style', 'checkbox', 'Units', 'normalized', ...
    'Position', [0.77 0.01 0.10 0.04], 'String', 'Trail (5s)', ...
    'Value', 1, 'FontSize', 9, 'BackgroundColor', [0.97 0.97 0.97]);
fig.UserData.trail_chk = trailChk;

addlistener(slider, 'Value', 'PostSet', @(~, ~) redraw(fig));

drawStatic(fig);
drawMiniCharts(fig);
redraw(fig);

%% ========================================================================
function drawStatic(fig)
    D = fig.UserData;
    ax = D.axMap;
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');

    rectangle(ax, 'Position', [D.mobBounds(1), D.mobBounds(3), ...
        D.mobBounds(2) - D.mobBounds(1), D.mobBounds(4) - D.mobBounds(3)], ...
        'EdgeColor', [0.6 0.6 0.6], 'LineStyle', '--', 'LineWidth', 1);

    thCirc = linspace(0, 2 * pi, 200);
    plot(ax, D.areaCx + D.ueRadius * cos(thCirc), ...
             D.areaCy + D.ueRadius * sin(thCirc), ...
        '-', 'Color', [0.3 0.6 0.9], 'LineWidth', 1.2);

    triX = [D.enbPos(:, 1); D.enbPos(1, 1)];
    triY = [D.enbPos(:, 2); D.enbPos(1, 2)];
    plot(ax, triX, triY, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1);

    for i = 1:D.nEnb
        plot(ax, D.enbPos(i, 1), D.enbPos(i, 2), '^', ...
            'Color', D.colors(i, :), 'MarkerSize', 14, ...
            'MarkerFaceColor', D.colors(i, :), 'LineWidth', 1.5);
    end

    h1 = plot(ax, NaN, NaN, '^k', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    h2 = plot(ax, NaN, NaN, 'ok', 'MarkerSize', 6, 'MarkerFaceColor', 'w');
    h3 = plot(ax, NaN, NaN, 'ok', 'MarkerSize', 6, 'MarkerFaceColor', [1 0.5 0.5]);
    h4 = plot(ax, NaN, NaN, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
    h5 = plot(ax, NaN, NaN, '-', 'Color', [0.3 0.6 0.9], 'LineWidth', 1.2);
    legend(ax, [h1 h2 h3 h4 h5], {'eNB', 'UE', 'Edge UE', 'Trail', 'UE Boundary'}, ...
        'Location', 'northwest', 'FontSize', 8);
    xlabel(ax, 'X (m)');
    ylabel(ax, 'Y (m)');
    xlim(ax, [0 1000]);
    ylim(ax, [0 1000]);
end

%% ========================================================================
function drawMiniCharts(fig)
    D = fig.UserData;
    cellIds = unique(D.kpm.cellId);
    win = 3;

    ax = D.axUE;
    hold(ax, 'on');
    grid(ax, 'on');
    for ci = 1:D.nEnb
        sub = sortrows(D.kpm(D.kpm.cellId == cellIds(ci), :), 'time_s');
        sm = movmean(sub.ueCount, win);
        plot(ax, sub.time_s, sm, '-', 'Color', D.colors(ci, :), 'LineWidth', 1.5);
    end
    yline(ax, D.nUEs / D.nEnb, '--k', 'LineWidth', 0.8);
    ylabel(ax, 'UE');
    title(ax, 'UE Count', 'FontSize', 9);
    fig.UserData.axUE_line = xline(ax, 0, '-r', 'LineWidth', 1.5);

    ax = D.axThp;
    hold(ax, 'on');
    grid(ax, 'on');
    for ci = 1:D.nEnb
        sub = sortrows(D.kpm(D.kpm.cellId == cellIds(ci), :), 'time_s');
        raw = sub.cellDlThp_kbps / 1000;
        raw(raw == 0) = NaN;
        sm = movmean(raw, win, 'omitnan');
        plot(ax, sub.time_s, sm, '-', 'Color', D.colors(ci, :), 'LineWidth', 1.5);
    end
    ylabel(ax, 'Mbps');
    title(ax, 'DL Throughput', 'FontSize', 9);
    fig.UserData.axThp_line = xline(ax, 0, '-r', 'LineWidth', 1.5);

    ax = D.axAct;
    hold(ax, 'on');
    grid(ax, 'on');
    for ci = 1:D.nEnb
        sub = sortrows(D.action(D.action.cellId == cellIds(ci), :), 'time_s');
        if isempty(sub)
            continue;
        end
        sm = movmean(sub.(D.cioVar), win, 'omitnan');
        plot(ax, sub.time_s, sm, '-', 'Color', D.colors(ci, :), 'LineWidth', 1.5);
    end
    ylabel(ax, 'CIO (dB)');
    xlabel(ax, 'Time (s)');
    title(ax, 'Displayed Self CIO', 'FontSize', 9);
    fig.UserData.axAct_line = xline(ax, 0, '-r', 'LineWidth', 1.5);
end

%% ========================================================================
function redraw(fig)
    D = fig.UserData;
    ax = D.axMap;

    delete(findall(ax, 'Tag', 'dyn'));
    legend(ax, 'AutoUpdate', 'off');
    delete(findall(fig, 'Type', 'annotation'));
    set(ax, 'NextPlot', 'add');

    tIdx = round(get(D.slider, 'Value'));
    tNow = D.times(tIdx);

    ueNow = D.traj(D.traj.time_s == tNow, :);

    kpmTimes = unique(D.kpm.time_s);
    [~, ki] = min(abs(kpmTimes - tNow));
    kpmNow = D.kpm(D.kpm.time_s == kpmTimes(ki), :);

    actTimes = unique(D.action.time_s);
    actT = actTimes(find(actTimes <= tNow, 1, 'last'));
    if isempty(actT)
        actT = actTimes(1);
    end
    actNow = D.action(D.action.time_s == actT, :);

    theta = linspace(0, 2 * pi, 100);
    showTrail = get(D.trail_chk, 'Value');

    for i = 1:D.nEnb
        cx = D.enbPos(i, 1);
        cy = D.enbPos(i, 2);
        col = D.colors(i, :);

        ar = actNow(actNow.cellId == i, :);
        txp = D.TXP_REF;
        cio = 0;
        if ~isempty(ar)
            txp = ar.txpApplied_dBm(1);
            cio = ar.(D.cioVar)(1);
        end
        radius = D.RAD_REF * 10 ^ ((txp - D.TXP_REF) / 20);

        fill(ax, cx + radius * cos(theta), cy + radius * sin(theta), col, ...
            'FaceAlpha', 0.06, 'EdgeColor', col, 'LineWidth', 1.5, 'Tag', 'dyn');

        kr = kpmNow(kpmNow.cellId == i, :);
        nEdge = 0;
        nUE = 0;
        dlThp = 0;
        cqi = 0;
        if ~isempty(kr)
            nEdge = kr.edgeUeCount(1);
            nUE = kr.ueCount(1);
            dlThp = kr.cellDlThp_kbps(1) / 1000;
            cqi = kr.avgCqi(1);
        end

        text(ax, cx, cy + 50, ...
            sprintf('eNB%d  CIO=%+.1fdB\n%.1fdBm  UE=%d(e=%d)', i, cio, txp, nUE, nEdge), ...
            'HorizontalAlignment', 'center', 'FontSize', 8, ...
            'FontWeight', 'bold', 'Color', col, 'Tag', 'dyn');
    end

    if showTrail
        trailDur = 5.0;
        for ue = 0:D.nUEs-1
            trail = D.ueTrail{ue + 1};
            mask = trail.time_s >= (tNow - trailDur) & trail.time_s <= tNow;
            pts = trail(mask, :);
            if height(pts) > 1
                cid = pts.servingCellId(end);
                if cid >= 1 && cid <= D.nEnb
                    tc = D.colors(cid, :);
                else
                    tc = [0.5 0.5 0.5];
                end
                trailColor = 0.75 * [1 1 1] + 0.25 * tc;
                plot(ax, pts.x, pts.y, '-', 'Color', trailColor, ...
                    'LineWidth', 0.8, 'Tag', 'dyn');
            end
        end
    end

    cellIds = unique(D.kpm.cellId);
    for ci = 1:D.nEnb
        mask = ueNow.servingCellId == cellIds(ci);
        if ~any(mask)
            continue;
        end
        ux = ueNow.x(mask);
        uy = ueNow.y(mask);

        kr = kpmNow(kpmNow.cellId == cellIds(ci), :);
        nEdge = 0;
        if ~isempty(kr)
            nEdge = kr.edgeUeCount(1);
        end

        dist = sqrt((ux - D.enbPos(ci, 1)) .^ 2 + (uy - D.enbPos(ci, 2)) .^ 2);
        if nEdge > 0 && nEdge < numel(ux)
            [~, si] = sort(dist, 'descend');
            eMask = false(size(ux));
            eMask(si(1:min(nEdge, numel(ux)))) = true;
            scatter(ax, ux(~eMask), uy(~eMask), 40, ...
                'MarkerEdgeColor', D.colors(ci, :), 'MarkerFaceColor', 'w', ...
                'LineWidth', 1.2, 'Tag', 'dyn');
            scatter(ax, ux(eMask), uy(eMask), 50, ...
                'MarkerEdgeColor', D.colors(ci, :), 'MarkerFaceColor', [1 0.5 0.5], ...
                'LineWidth', 1.2, 'Tag', 'dyn');
        else
            scatter(ax, ux, uy, 40, ...
                'MarkerEdgeColor', D.colors(ci, :), 'MarkerFaceColor', 'w', ...
                'LineWidth', 1.2, 'Tag', 'dyn');
        end
    end

    for u = 0:max(ueNow.ueIndex)
        ur = ueNow(ueNow.ueIndex == u, :);
        if isempty(ur)
            continue;
        end
        cid = ur.servingCellId(1);
        if cid >= 1 && cid <= D.nEnb
            uc = D.colors(cid, :);
        else
            uc = [0.5 0.5 0.5];
        end
        text(ax, ur.x(1) + 8, ur.y(1) + 8, sprintf('%d', u), ...
            'FontSize', 5, 'Color', uc, 'Tag', 'dyn');
    end

    info = {};
    for i = 1:D.nEnb
        kr = kpmNow(kpmNow.cellId == i, :);
        ar = actNow(actNow.cellId == i, :);
        if ~isempty(kr) && ~isempty(ar)
            info{end + 1} = sprintf( ...
                'eNB%d: UE=%d(edge=%d) DL=%.1fMbps CQI=%.1f CIO=%+.1fdB TXP=%.1f', ...
                i, kr.ueCount(1), kr.edgeUeCount(1), ...
                kr.cellDlThp_kbps(1) / 1000, kr.avgCqi(1), ...
                ar.(D.cioVar)(1), ar.txpApplied_dBm(1));
        elseif ~isempty(kr)
            info{end + 1} = sprintf( ...
                'eNB%d: UE=%d(edge=%d) DL=%.1fMbps CQI=%.1f', ...
                i, kr.ueCount(1), kr.edgeUeCount(1), ...
                kr.cellDlThp_kbps(1) / 1000, kr.avgCqi(1));
        end
    end
    if ~isempty(info)
        annotation(fig, 'textbox', [0.04 0.88 0.50 0.08], ...
            'String', info, 'FontSize', 7, ...
            'EdgeColor', [0.7 0.7 0.7], 'BackgroundColor', [1 1 1], ...
            'FitBoxToText', 'on', 'Interpreter', 'none');
    end

    title(ax, sprintf('t = %.1f s', tNow), 'FontSize', 12, 'FontWeight', 'bold');
    set(D.time_label, 'String', sprintf('t=%.1fs', tNow));

    if isfield(fig.UserData, 'axUE_line')
        set(fig.UserData.axUE_line, 'Value', tNow);
        set(fig.UserData.axThp_line, 'Value', tNow);
        set(fig.UserData.axAct_line, 'Value', tNow);
    end
end

%% ========================================================================
function playCallback(src, ~, fig)
    if fig.UserData.playing
        fig.UserData.playing = false;
        set(src, 'String', 'Play');
        return;
    end
    fig.UserData.playing = true;
    set(src, 'String', 'Stop');

    while fig.UserData.playing
        cur = round(get(fig.UserData.slider, 'Value'));
        if cur >= numel(fig.UserData.times)
            fig.UserData.playing = false;
            set(src, 'String', 'Play');
            return;
        end
        set(fig.UserData.slider, 'Value', cur + 1);
        redraw(fig);
        drawnow;
        pause(0.02);
    end
end

%% ========================================================================
function tbl = keepLastEpisode(tbl)
    ri = find(diff(tbl.time_s) < 0, 1, 'last');
    if ~isempty(ri)
        tbl = tbl(ri + 1:end, :);
    end
end

%% ========================================================================
function name = pickVar(tbl, candidates)
    name = '';
    for i = 1:numel(candidates)
        if any(strcmp(tbl.Properties.VariableNames, candidates{i}))
            name = candidates{i};
            return;
        end
    end
end
