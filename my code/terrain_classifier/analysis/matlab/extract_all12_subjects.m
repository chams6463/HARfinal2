%% extract_all12_subjects.m
% ============================================================================
% build_h_d_reference_curves.m 을 12명(AB06~AB12 원본 7명 + AB13/14/15/28/30
% 신규 5명)으로 확장 실행한다. 원본 h_d_population.mat(7명)은 그대로 두고,
% 별도 파일 2개를 새로 만든다:
%
%   1) h_d_population_12.mat  — build_h_d_reference_curves.m 과 동일 스키마
%      (41 feature x 7 mode, population + per-subject 평균곡선). 검증2/검증3에서
%      "7명 템플릿 vs 12명 템플릿" 비교, "핵심 결론 재검증"에 쓴다.
%
%   2) stride_bank_12.mat     — 뱅크가 실제 쓰는 4개 feature만, "평균 안 낸
%      개별 stride" 그대로 저장. 검증1(held-out 정확도), 검증5(LOSO 교차검증)에서
%      개별 stride 단위 z-score 분류(펌웨어 _ClassifyStride 재현)를 하려면
%      평균곡선이 아니라 개별 stride가 필요하기 때문에 따로 뺀다.
%      strideBank.(subjId).(modeName).(FEAT) = [nStride x 101]
%      (FEAT 이름은 ref_curve_tables.h 의 RcFeature_t 접미사와 동일하게 맞춤:
%       TRUNK_TILT, THIGH_GYROMAG, EMG_SITE0_RMS, EMG_SITE1_RMS)
%
% 나머지 로직(라벨 파싱, gcRight 위상 기준, EMG envelope, stride 경계 검출)은
% build_h_d_reference_curves.m 과 완전히 동일 — 그 파일의 상세 주석 참고.
%
% 실행: Camargo_matlab 폴더에서 그냥 실행 (또는
%   matlab -batch "run('extract_all12_subjects.m')" )
% ============================================================================

clear; clc;

%% ---- 설정 ----
GAIT_ROOT   = 'D:\gait_data';
SUBJECT_IDS = {'AB06','AB07','AB08','AB09','AB10','AB11','AB12', ...   % 원본 7명 (기존 뱅크 학습)
               'AB13','AB14','AB15','AB28','AB30'};                    % 신규 held-out 5명
ORIGINAL_SUBJECT_IDS = {'AB06','AB07','AB08','AB09','AB10','AB11','AB12'};

IMU_SEGMENTS = {'trunk','thigh','shank','foot'};
EMG_MUSCLES  = {'gastrocmed','tibialisanterior','soleus','vastusmedialis','vastuslateralis', ...
                'rectusfemoris','bicepsfemoris','semitendinosus','gracilis','gluteusmedius', ...
                'rightexternaloblique'};

MODE_NAMES = {'STAND','WALK','TURN','STAIRS_UP','STAIRS_DOWN','RAMP_UP','RAMP_DOWN'};

% 뱅크 4-feature: h_d_population.mat 필드명 -> ref_curve_tables.h RcFeature_t 접미사
BANK_FEAT_MAP = struct( ...
    'IMU_trunk_TILT',        'TRUNK_TILT', ...
    'IMU_thigh_GYROMAG',     'THIGH_GYROMAG', ...
    'EMG_gastrocmed_RMS',    'EMG_SITE0_RMS', ...
    'EMG_vastusmedialis_RMS','EMG_SITE1_RMS');
BANK_SRC_FEATS = fieldnames(BANK_FEAT_MAP);

SOURCE_DEFS = struct('folderName', {}, 'labelToMode', {});
SOURCE_DEFS(1).folderName  = 'levelground';
SOURCE_DEFS(1).labelToMode = containers.Map( ...
    {'idle','walk','turn1','turn2'}, {'STAND','WALK','TURN','TURN'});
SOURCE_DEFS(2).folderName  = 'stair';
SOURCE_DEFS(2).labelToMode = containers.Map( ...
    {'idle','stairascent','stairdescent'}, {'STAND','STAIRS_UP','STAIRS_DOWN'});
SOURCE_DEFS(3).folderName  = 'ramp';
SOURCE_DEFS(3).labelToMode = containers.Map( ...
    {'idle','rampascent','rampdescent'}, {'STAND','RAMP_UP','RAMP_DOWN'});

THETA_GRID = linspace(0, 1, 101);

ENV_WINDOW_S     = 0.15;
HEELSTRIKE_WRAP_DROP = 50;
MIN_IMU_SAMPLES_PER_STRIDE = 10;
MIN_EMG_SAMPLES_PER_STRIDE = 50;

%% ---- 누적 컨테이너 ----
accPop = struct();     % accPop.(mode).(feat41).data/subj   -> h_d_population_12
accSub = struct();     % accSub.(subj).(mode).(feat41).data -> h_d_subject_12
strideBank = struct(); % strideBank.(subj).(mode).(FEAT4)   -> [nStride x 101] raw

strideCountLog = struct();

for si = 1:numel(SUBJECT_IDS)
    subjId = SUBJECT_IDS{si};
    subjRoot = fullfile(GAIT_ROOT, subjId);

    try
        dateFolder = find_date_folder(subjRoot);
    catch dateErr
        fprintf('[SKIP 전체] %s: 날짜 폴더 자동탐색 실패 - %s\n', subjId, dateErr.message);
        continue;
    end
    fprintf('\n########## %s (날짜폴더: %s) ##########\n', subjId, dateFolder);

    for mNameIdx = 1:numel(MODE_NAMES)
        strideCountLog.(subjId).(MODE_NAMES{mNameIdx}) = 0;
    end

    for srcIdx = 1:numel(SOURCE_DEFS)
        srcFolder   = SOURCE_DEFS(srcIdx).folderName;
        labelToMode = SOURCE_DEFS(srcIdx).labelToMode;

        sourceRoot = fullfile(subjRoot, dateFolder, srcFolder);
        condDir = fullfile(sourceRoot, 'conditions');
        emgDir  = fullfile(sourceRoot, 'emg');
        imuDir  = fullfile(sourceRoot, 'imu');
        gcDir   = fullfile(sourceRoot, 'gcRight');

        if ~isfolder(condDir)
            fprintf('  [SKIP] %s/%s: conditions 폴더 없음\n', subjId, srcFolder);
            continue;
        end

        condFiles = dir(fullfile(condDir, '*.mat'));
        fprintf('  -- %-11s : %d개 trial --\n', srcFolder, numel(condFiles));

        for fi = 1:numel(condFiles)
            trialName = condFiles(fi).name;

            condS = load(fullfile(condDir, trialName));
            if ~isfield(condS, 'labels')
                fprintf('    [SKIP] %s: labels 필드 없음\n', trialName);
                continue;
            end
            labelTbl = condS.labels;
            runs = build_label_runs(labelTbl.Header, labelTbl.Label);

            relevantRun = false;
            for r = 1:numel(runs)
                if isKey(labelToMode, runs(r).label)
                    relevantRun = true;
                    break;
                end
            end
            if ~relevantRun
                continue;
            end

            emgPath = fullfile(emgDir, trialName);
            imuPath = fullfile(imuDir, trialName);
            if ~isfile(emgPath) || ~isfile(imuPath)
                fprintf('    [SKIP] %s: emg 또는 imu 파일 없음\n', trialName);
                continue;
            end
            emgS = load(emgPath); emgTbl = emgS.data;
            imuS = load(imuPath); imuTbl = imuS.data;

            gcPath = fullfile(gcDir, trialName);
            useGC = isfile(gcPath);
            if useGC
                gcS = load(gcPath);
                gcHeader = gcS.data.Header;
                gcHeelStrike = gcS.data.HeelStrike;
                strideStarts = compute_stride_boundaries(gcHeader, gcHeelStrike, HEELSTRIKE_WRAP_DROP);
            else
                gcHeader = []; gcHeelStrike = []; strideStarts = [];
                fprintf('    [정보] %s: gcRight 없음 -> 균등분할(fallback) 사용\n', trialName);
            end

            emgEnv = compute_emg_envelope(emgTbl, EMG_MUSCLES, ENV_WINDOW_S);
            fsGc = NaN;
            if useGC && numel(gcHeader) > 1
                fsGc = 1 / mean(diff(gcHeader));
            end

            for r = 1:numel(runs)
                runLabel = runs(r).label;
                if ~isKey(labelToMode, runLabel)
                    continue;
                end
                modeName = labelToMode(runLabel);

                if strcmp(modeName, 'STAND') || ~useGC
                    strideWindows = {[runs(r).t0, runs(r).t1]};
                    strideMethod = 'uniform-run';
                else
                    tol = 0.5 / max(fsGc, 1);
                    strideWindows = {};
                    for k = 1:numel(strideStarts)-1
                        if strideStarts(k) >= runs(r).t0 - tol && strideStarts(k+1) <= runs(r).t1 + tol
                            strideWindows{end+1} = [strideStarts(k), strideStarts(k+1)]; %#ok<SAGROW>
                        end
                    end
                    strideMethod = 'gcRight-heelstrike';
                    if isempty(strideWindows)
                        continue;
                    end
                end

                for w = 1:numel(strideWindows)
                    ts0 = strideWindows{w}(1); ts1 = strideWindows{w}(2);
                    useGcForStride = useGC && ~strcmp(modeName, 'STAND') && strcmp(strideMethod, 'gcRight-heelstrike');
                    [ok, curveStruct] = extract_stride_curves(ts0, ts1, imuTbl, gcHeader, gcHeelStrike, ...
                        useGcForStride, emgTbl.Header, emgEnv, THETA_GRID, IMU_SEGMENTS, EMG_MUSCLES, ...
                        MIN_IMU_SAMPLES_PER_STRIDE, MIN_EMG_SAMPLES_PER_STRIDE);
                    if ~ok
                        continue;
                    end

                    featNames = fieldnames(curveStruct);
                    for fnI = 1:numel(featNames)
                        fname = featNames{fnI};
                        row = curveStruct.(fname);
                        accPop = append_curve(accPop, modeName, fname, row, subjId);
                        if ~isfield(accSub, subjId); accSub.(subjId) = struct(); end
                        accSub.(subjId) = append_curve(accSub.(subjId), modeName, fname, row, subjId);
                    end

                    % ---- 뱅크 4-feature 원시 stride 저장 (평균 내지 않음) ----
                    for bfI = 1:numel(BANK_SRC_FEATS)
                        srcName = BANK_SRC_FEATS{bfI};
                        featTag = BANK_FEAT_MAP.(srcName);
                        if ~isfield(curveStruct, srcName)
                            continue;
                        end
                        strideBank = append_stride_row(strideBank, subjId, modeName, featTag, curveStruct.(srcName));
                    end

                    strideCountLog.(subjId).(modeName) = strideCountLog.(subjId).(modeName) + 1;
                end
            end
        end
    end
end

%% ---- 진단 로그 ----
fprintf('\n================ 피험자 x 모드 stride 수 요약 (12명) ================\n');
fprintf('%-8s', 'subject');
for mIdx = 1:numel(MODE_NAMES); fprintf('%14s', MODE_NAMES{mIdx}); end
fprintf('\n');
missingList = {};
for si = 1:numel(SUBJECT_IDS)
    subjId = SUBJECT_IDS{si};
    if ~isfield(strideCountLog, subjId)
        fprintf('%-8s  (전체 스킵됨)\n', subjId);
        continue;
    end
    fprintf('%-8s', subjId);
    for mIdx = 1:numel(MODE_NAMES)
        modeName = MODE_NAMES{mIdx};
        n = strideCountLog.(subjId).(modeName);
        fprintf('%14d', n);
        if n == 0
            missingList{end+1} = sprintf('%s / %s', subjId, modeName); %#ok<SAGROW>
        end
    end
    fprintf('\n');
end
if ~isempty(missingList)
    fprintf('\n[경고] stride 0개인 조합:\n');
    for i = 1:numel(missingList)
        fprintf('  - %s\n', missingList{i});
    end
end

%% ---- population / subject 평균 곡선 (h_d_population_12.mat, 41 feature) ----
h_d_population = struct();
for mIdx = 1:numel(MODE_NAMES)
    modeName = MODE_NAMES{mIdx};
    if ~isfield(accPop, modeName); continue; end
    featNames = fieldnames(accPop.(modeName));
    for fnI = 1:numel(featNames)
        fname = featNames{fnI};
        d = accPop.(modeName).(fname).data;
        subjList = accPop.(modeName).(fname).subj;
        h_d_population.(modeName).(fname).mean = mean(d, 1, 'omitnan');
        h_d_population.(modeName).(fname).std  = std(d, 0, 1, 'omitnan');
        h_d_population.(modeName).(fname).nStride  = size(d, 1);
        h_d_population.(modeName).(fname).nSubject  = numel(unique(subjList));
    end
end

h_d_subject = struct();
for si = 1:numel(SUBJECT_IDS)
    subjId = SUBJECT_IDS{si};
    if ~isfield(accSub, subjId); continue; end
    for mIdx = 1:numel(MODE_NAMES)
        modeName = MODE_NAMES{mIdx};
        if ~isfield(accSub.(subjId), modeName); continue; end
        featNames = fieldnames(accSub.(subjId).(modeName));
        for fnI = 1:numel(featNames)
            fname = featNames{fnI};
            d = accSub.(subjId).(modeName).(fname).data;
            h_d_subject.(subjId).(modeName).(fname).mean = mean(d, 1, 'omitnan');
            h_d_subject.(subjId).(modeName).(fname).std  = std(d, 0, 1, 'omitnan');
            h_d_subject.(subjId).(modeName).(fname).nStride = size(d, 1);
        end
    end
end

h_d_meta = struct();
h_d_meta.subjects = SUBJECT_IDS;
h_d_meta.originalSubjects = ORIGINAL_SUBJECT_IDS;
h_d_meta.newSubjects = setdiff(SUBJECT_IDS, ORIGINAL_SUBJECT_IDS, 'stable');
h_d_meta.modes = MODE_NAMES;
h_d_meta.imuSegments = IMU_SEGMENTS;
h_d_meta.emgMuscles = EMG_MUSCLES;
h_d_meta.envWindowS = ENV_WINDOW_S;
h_d_meta.thetaMethod = 'gcRight.HeelStrike(0-100%%) wrap-based, STAND/gc-missing은 run 균등분할 fallback';
h_d_meta.generatedAt = datestr(now);
h_d_meta.missingCombos = missingList;

thetaGrid = THETA_GRID; %#ok<NASGU>

outDir = fileparts(mfilename('fullpath'));
save(fullfile(outDir, 'h_d_population_12.mat'), ...
    'h_d_population', 'h_d_subject', 'thetaGrid', 'h_d_meta');
fprintf('\n결과 저장 완료: h_d_population_12.mat\n');

%% ---- stride_bank_12.mat (뱅크 4-feature, 원시 stride) ----
bankMeta = struct();
bankMeta.subjects = SUBJECT_IDS;
bankMeta.originalSubjects = ORIGINAL_SUBJECT_IDS;
bankMeta.newSubjects = setdiff(SUBJECT_IDS, ORIGINAL_SUBJECT_IDS, 'stable');
bankMeta.bankModes = {'WALK','STAIRS_UP','STAIRS_DOWN','RAMP_UP','RAMP_DOWN'};
bankMeta.bankFeats = struct2cell(BANK_FEAT_MAP)';   % {'TRUNK_TILT','THIGH_GYROMAG','EMG_SITE0_RMS','EMG_SITE1_RMS'}
bankMeta.thetaGrid = THETA_GRID;
bankMeta.generatedAt = datestr(now);

save(fullfile(outDir, 'stride_bank_12.mat'), 'strideBank', 'bankMeta');
fprintf('결과 저장 완료: stride_bank_12.mat\n');


%% ============================== 로컬 함수 ==============================
% (find_date_folder / build_label_runs / compute_stride_boundaries /
%  compute_emg_envelope / extract_stride_curves / append_curve 는
%  build_h_d_reference_curves.m 과 완전히 동일 — 그대로 재사용)

function dateFolder = find_date_folder(subjRoot)
    dirEntries = dir(subjRoot);
    isSubdir = [dirEntries.isdir];
    subdirNames = {dirEntries(isSubdir).name};
    subdirNames = subdirNames(~ismember(subdirNames, {'.', '..', 'osimxml'}));
    if numel(subdirNames) ~= 1
        error('find_date_folder:ambiguous', ...
            '%s 밑에서 날짜 폴더를 하나로 특정할 수 없음 (발견: %s)', ...
            subjRoot, strjoin(subdirNames, ', '));
    end
    dateFolder = subdirNames{1};
end

function runs = build_label_runs(labelHeader, labelVals)
    runs = struct('label', {}, 't0', {}, 't1', {});
    i = 1;
    n = numel(labelVals);
    while i <= n
        j = i;
        while j < n && strcmp(labelVals{j+1}, labelVals{i})
            j = j + 1;
        end
        runs(end+1) = struct('label', labelVals{i}, 't0', labelHeader(i), 't1', labelHeader(j)); %#ok<AGROW>
        i = j + 1;
    end
end

function strideStarts = compute_stride_boundaries(gcHeader, gcHeelStrike, wrapDropThresh)
    d = diff(gcHeelStrike);
    wrapIdx = find(d < -wrapDropThresh);
    strideStarts = [gcHeader(1); gcHeader(wrapIdx + 1)];
end

function emgEnv = compute_emg_envelope(emgTbl, emgMuscles, envWindowS)
    header = emgTbl.Header;
    fsEmg = 1 / mean(diff(header));
    winSamples = max(1, round(envWindowS * fsEmg));

    emgEnv = struct('mav', struct(), 'rms', struct(), 'zc', struct());
    for m = 1:numel(emgMuscles)
        musc = emgMuscles{m};
        raw = emgTbl.(musc);
        emgEnv.mav.(musc) = movmean(abs(raw), winSamples);
        emgEnv.rms.(musc) = sqrt(movmean(raw.^2, winSamples));

        sgnRaw = sign(raw);
        sgnRaw(sgnRaw == 0) = 1;
        crossFlag = [0; diff(sgnRaw) ~= 0];
        emgEnv.zc.(musc) = movmean(double(crossFlag), winSamples);
    end
end

function [ok, curveStruct] = extract_stride_curves(ts0, ts1, imuTbl, gcHeader, gcHeelStrike, useGC, ...
    emgHeader, emgEnv, thetaGrid, imuSegments, emgMuscles, minImuSamples, minEmgSamples)
    ok = true;
    curveStruct = struct();

    imuMask = imuTbl.Header >= ts0 & imuTbl.Header < ts1;
    if sum(imuMask) < minImuSamples
        ok = false; return;
    end
    tImu = imuTbl.Header(imuMask);
    if useGC
        thetaImu = interp1(gcHeader, gcHeelStrike, tImu, 'linear', 'extrap') / 100;
    else
        thetaImu = (tImu - ts0) / (ts1 - ts0);
    end
    thetaImu = min(max(thetaImu, 0), 1);
    [thetaImuU, uIdxImu] = unique(thetaImu, 'stable');
    if numel(thetaImuU) < 3
        ok = false; return;
    end

    for s = 1:numel(imuSegments)
        seg = imuSegments{s};
        ax = imuTbl.([seg '_Accel_X'])(imuMask);
        az = imuTbl.([seg '_Accel_Z'])(imuMask);
        tilt = atan2(ax, az) * 180 / pi;

        gx = imuTbl.([seg '_Gyro_X'])(imuMask);
        gy = imuTbl.([seg '_Gyro_Y'])(imuMask);
        gz = imuTbl.([seg '_Gyro_Z'])(imuMask);
        gyroMag = sqrt(gx.^2 + gy.^2 + gz.^2);

        curveStruct.(sprintf('IMU_%s_TILT', seg))    = interp1(thetaImuU, tilt(uIdxImu),    thetaGrid, 'linear', 'extrap');
        curveStruct.(sprintf('IMU_%s_GYROMAG', seg)) = interp1(thetaImuU, gyroMag(uIdxImu), thetaGrid, 'linear', 'extrap');
    end

    emgMask = emgHeader >= ts0 & emgHeader < ts1;
    if sum(emgMask) < minEmgSamples
        ok = false; return;
    end
    tEmg = emgHeader(emgMask);
    if useGC
        thetaEmg = interp1(gcHeader, gcHeelStrike, tEmg, 'linear', 'extrap') / 100;
    else
        thetaEmg = (tEmg - ts0) / (ts1 - ts0);
    end
    thetaEmg = min(max(thetaEmg, 0), 1);
    [thetaEmgU, uIdxEmg] = unique(thetaEmg, 'stable');
    if numel(thetaEmgU) < 3
        ok = false; return;
    end

    for m = 1:numel(emgMuscles)
        musc = emgMuscles{m};
        mavSeg = emgEnv.mav.(musc)(emgMask);
        rmsSeg = emgEnv.rms.(musc)(emgMask);
        zcSeg  = emgEnv.zc.(musc)(emgMask);

        curveStruct.(sprintf('EMG_%s_MAV', musc)) = interp1(thetaEmgU, mavSeg(uIdxEmg), thetaGrid, 'linear', 'extrap');
        curveStruct.(sprintf('EMG_%s_RMS', musc)) = interp1(thetaEmgU, rmsSeg(uIdxEmg), thetaGrid, 'linear', 'extrap');
        curveStruct.(sprintf('EMG_%s_ZC', musc))  = interp1(thetaEmgU, zcSeg(uIdxEmg),  thetaGrid, 'linear', 'extrap');
    end
end

function acc = append_curve(acc, modeName, featName, curveRow, subjId)
    if ~isfield(acc, modeName)
        acc.(modeName) = struct();
    end
    if ~isfield(acc.(modeName), featName)
        acc.(modeName).(featName).data = zeros(0, numel(curveRow));
        acc.(modeName).(featName).subj = {};
    end
    acc.(modeName).(featName).data(end+1, :) = curveRow;
    acc.(modeName).(featName).subj{end+1, 1} = subjId;
end

function bank = append_stride_row(bank, subjId, modeName, featTag, curveRow)
% strideBank.(subjId).(modeName).(featTag) 에 curveRow(1x101)를 vertcat.
    if ~isfield(bank, subjId); bank.(subjId) = struct(); end
    if ~isfield(bank.(subjId), modeName); bank.(subjId).(modeName) = struct(); end
    if ~isfield(bank.(subjId).(modeName), featTag)
        bank.(subjId).(modeName).(featTag) = zeros(0, numel(curveRow));
    end
    bank.(subjId).(modeName).(featTag)(end+1, :) = curveRow;
end
