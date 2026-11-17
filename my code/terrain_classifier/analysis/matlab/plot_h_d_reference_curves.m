%% plot_h_d_reference_curves.m
% ============================================================================
% build_h_d_reference_curves.m 의 결과(h_d_population.mat)를 검증하기 위한
% 플롯 스크립트.
%
% Figure 1: 대표 feature 3개(IMU_thigh_GYROMAG, EMG_soleus_RMS,
%   EMG_tibialisanterior_MAV)에 대해 7개 mode의 h_d^m(theta) 곡선을
%   mean±std shaded band로 겹쳐그려서 모드별 곡선 모양이 실제로 다른지 확인.
% Figure 2: 1개 feature(IMU_thigh_GYROMAG, WALK 모드)에 대해 7명 population
%   곡선과 개별 피험자 7명 곡선을 겹쳐그려서 population vs 개인 편차를 확인.
%
% 실행: 이 파일이 있는 폴더(Camargo_matlab)를 현재 폴더로 하고 실행.
%       h_d_population.mat 이 먼저 build_h_d_reference_curves.m 으로 생성돼
%       있어야 한다.
% ============================================================================

clear; clc;
S = load(fullfile(fileparts(mfilename('fullpath')), 'h_d_population.mat'));
h_d_population = S.h_d_population;
h_d_subject = S.h_d_subject;
thetaGrid = S.thetaGrid;

MODE_NAMES = {'STAND','WALK','TURN','STAIRS_UP','STAIRS_DOWN','RAMP_UP','RAMP_DOWN'};
MODE_COLORS = [ ...
    0.50 0.50 0.50;  % STAND
    0.00 0.45 0.74;  % WALK
    0.93 0.69 0.13;  % TURN
    0.85 0.33 0.10;  % STAIRS_UP
    0.49 0.18 0.56;  % STAIRS_DOWN
    0.47 0.67 0.19;  % RAMP_UP
    0.30 0.75 0.93]; % RAMP_DOWN

%% ---- Figure 1: 대표 feature 3개 x 7 mode 겹쳐그리기 ----
repFeatures = {'IMU_thigh_GYROMAG', 'EMG_soleus_RMS', 'EMG_tibialisanterior_MAV'};

figure('Name', 'h_d^m(theta) 모드별 비교', 'Position', [80 80 1500 420]);
for fi = 1:numel(repFeatures)
    fname = repFeatures{fi};
    subplot(1, numel(repFeatures), fi); hold on;
    legendHandles = [];
    legendLabels = {};
    for mi = 1:numel(MODE_NAMES)
        modeName = MODE_NAMES{mi};
        if ~isfield(h_d_population, modeName) || ~isfield(h_d_population.(modeName), fname)
            continue;
        end
        curve = h_d_population.(modeName).(fname);
        mu = curve.mean; sd = curve.std;
        col = MODE_COLORS(mi, :);

        fillX = [thetaGrid, fliplr(thetaGrid)];
        fillY = [mu - sd, fliplr(mu + sd)];
        fill(fillX, fillY, col, 'FaceAlpha', 0.12, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        h = plot(thetaGrid, mu, 'Color', col, 'LineWidth', 1.8);
        legendHandles(end+1) = h; %#ok<SAGROW>
        legendLabels{end+1} = sprintf('%s (n=%d,strd=%d)', modeName, curve.nSubject, curve.nStride); %#ok<SAGROW>
    end
    xlabel('gait phase \theta (0-1)');
    ylabel(strrep(fname, '_', '\_'));
    title(strrep(fname, '_', '\_'));
    legend(legendHandles, legendLabels, 'Location', 'best', 'FontSize', 7);
    grid on; box on;
end
sgtitle('h\_d^m(\theta) — 7개 mode 대표 feature 비교 (population, mean\pm std shaded)');

outPng1 = fullfile(fileparts(mfilename('fullpath')), 'h_d_mode_comparison.png');
saveas(gcf, outPng1);
fprintf('저장: %s\n', outPng1);

%% ---- Figure 2: population vs 개인(subject) 곡선 비교 (1 feature, WALK) ----
compFeature = 'IMU_thigh_GYROMAG';
compMode = 'WALK';

figure('Name', 'population vs 개인 편차', 'Position', [80 80 900 500]); hold on;
subjIds = fieldnames(h_d_subject);
subjColors = lines(numel(subjIds));
legendHandles = [];
legendLabels = {};
for si = 1:numel(subjIds)
    subjId = subjIds{si};
    if ~isfield(h_d_subject.(subjId), compMode) || ~isfield(h_d_subject.(subjId).(compMode), compFeature)
        continue;
    end
    sc = h_d_subject.(subjId).(compMode).(compFeature);
    h = plot(thetaGrid, sc.mean, '--', 'Color', [subjColors(si,:) 0.6], 'LineWidth', 1.0);
    legendHandles(end+1) = h; %#ok<SAGROW>
    legendLabels{end+1} = sprintf('%s (n=%d)', subjId, sc.nStride); %#ok<SAGROW>
end

popCurve = h_d_population.(compMode).(compFeature);
fillX = [thetaGrid, fliplr(thetaGrid)];
fillY = [popCurve.mean - popCurve.std, fliplr(popCurve.mean + popCurve.std)];
fill(fillX, fillY, [0 0 0], 'FaceAlpha', 0.10, 'EdgeColor', 'none', 'HandleVisibility', 'off');
hPop = plot(thetaGrid, popCurve.mean, 'k-', 'LineWidth', 3);
legendHandles(end+1) = hPop;
legendLabels{end+1} = sprintf('POPULATION (n=%d subj, %d strides)', popCurve.nSubject, popCurve.nStride);

xlabel('gait phase \theta (0-1)');
ylabel(strrep(compFeature, '_', '\_'));
title(sprintf('%s / %s: population(굵은 검은선) vs 개별 피험자(점선)', compMode, strrep(compFeature,'_','\_')));
legend(legendHandles, legendLabels, 'Location', 'best', 'FontSize', 7);
grid on; box on;

outPng2 = fullfile(fileparts(mfilename('fullpath')), 'h_d_population_vs_subject.png');
saveas(gcf, outPng2);
fprintf('저장: %s\n', outPng2);
