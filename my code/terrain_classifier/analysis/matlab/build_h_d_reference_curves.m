%% build_h_d_reference_curves.m
% ============================================================================
% Camargo et al. (2021, J. Biomech.) 하지 보행 데이터셋(D:\gait_data\AB06~AB12)
% 에서, "전환 경계 전후 300ms"가 아니라 "정상상태(steady-state) stride 전체"를
% 보행위상 theta(0~1, 101포인트)로 리샘플링한 모드별 평균곡선(reference curve,
% h_d^m(theta))을 만든다.
%
% 목표 mode 7개와 소스 라벨(순수 라벨만 사용 — 전환구간 하이픈 라벨 제외):
%   STAND       <- idle              (levelground/stair/ramp 공통)
%   WALK        <- walk              (levelground 전용, stair/ramp엔 순수 walk 라벨 없음)
%   TURN        <- turn1, turn2      (levelground 전용)
%   STAIRS_UP   <- stairascent       (stair 전용)
%   STAIRS_DOWN <- stairdescent      (stair 전용)
%   RAMP_UP     <- rampascent        (ramp 전용)
%   RAMP_DOWN   <- rampdescent       (ramp 전용)
% (실제 D:\gait_data 라벨을 dir로 직접 확인해서 결정함 — walk-stairascent 같은
%  하이픈 전환 라벨은 정상상태가 아니므로 전부 제외)
%
% Feature 41개 (extract_transition_features.m / turn_detection_emg_AB06.m 와
% 동일 정의를 "구간 평균 1개 값" 대신 "시간에 따른 연속 신호"로 재사용):
%   IMU: trunk/thigh/shank/foot x {tilt=atan2(Accel_X,Accel_Z)[deg] (순간값),
%        gyro_mag=sqrt(Gx^2+Gy^2+Gz^2) (순간값)}                      -> 8개
%   EMG: 11근육 x {MAV, RMS, ZC}                                       -> 33개
%   EMG의 MAV/RMS/ZC는 extract_transition_features.m 에서 고정된 pre/post
%   250ms 윈도우 하나에 대해 한 번만 계산하던 것을, 여기서는 같은 150ms
%   이동윈도우(turn_detection_emg_AB06.m 의 ENV_WINDOW_S 관례)를 전체 시계열에
%   슬라이딩시켜 "순간값 시계열"로 확장한 것이다 (movmean 기반 envelope).
%   즉 각 theta 포인트에서의 h_d^m(theta) 자체가 여러 stride 평균이므로,
%   원래 스크립트에서 pre/post 구간 평균이 하던 평활화 역할을 여기서는
%   "stride 간 평균"이 대신 담당한다.
%
% theta(보행위상) 정의 방법 (택1, trial마다 자동 판단, 아래 useGC 플래그로 기록):
%   방법 A (우선) - gcRight/*.mat 존재 시: gcRight.data.HeelStrike 컬럼이
%     이미 0~100 %GC로 각 heel-strike마다 리셋되는 컬럼임을 실측 확인했음
%     (D:\gait_data\AB06\10_09_18\levelground\gcRight 실데이터 첫 wrap가
%      idx=244 부근에서 100->0으로 리셋되는 것을 직접 확인).
%     gcLeft가 아니라 gcRight를 쓰는 이유: EMG 채널 중 'rightexternaloblique'
%     만 좌우가 명시되어 있고 나머지 근육/IMU 세그먼트(trunk/thigh/shank/foot)
%     는 좌우 구분 없이 세트가 하나뿐이다 -> 이 데이터셋은 오른쪽 다리 한쪽만
%     계측한 것으로 판단, 따라서 오른쪽 heel-strike 기준 gcRight를 위상 기준으로
%     사용한다. HeelStrike 값이 100->0으로 크게 떨어지는(diff < -50) 지점을
%     heel-strike(stride 경계) 이벤트로 보고, 연속된 두 heel-strike 사이를
%     "1 stride"로 정의한다. 그 stride가 라벨 run(t0,t1) 안에 완전히 포함될
%     때만("정상상태 stride만") 채택한다.
%   방법 B (fallback) - gcRight 파일이 없거나 STAND(idle)처럼 애초에 주기적
%     걸음이 아닌 경우: 같은 라벨이 연속되는 run 구간 [t0,t1] 전체를 1개의
%     "가짜(pseudo) stride"로 보고 theta = (t-t0)/(t1-t0) 균등분할로 정의한다.
%     (STAND는 정의상 걸음 주기가 없으므로 gcRight 존재 여부와 무관하게 항상
%      이 방법을 쓴다.)
%
% 실행: 이 파일이 있는 폴더(Camargo_matlab)를 현재 폴더로 하고 그냥 실행.
%       경로는 D:\gait_data\AB06~AB12 밑에서 'osimxml'이 아닌 하위폴더를
%       날짜 폴더로 자동 인식한다(하드코딩 없음).
%
% 출력: Camargo_matlab\h_d_population.mat
%   h_d_population.(modeName).(featureName) = struct('mean',[1x101],'std',[1x101],
%                                                      'nStride',N,'nSubject',M)
%   h_d_subject.(subjectId).(modeName).(featureName) = struct('mean',...,'std',...,'nStride',...)
%   thetaGrid = linspace(0,1,101)
%   h_d_meta  = 생성 설정/서브젝트 목록/생성일시
% ============================================================================

clear; clc;
addpath('C:\Users\user\Desktop\onephai_ifac\Camargo_matlab');

%% ---- 설정 ----
GAIT_ROOT   = 'D:\gait_data';
SUBJECT_IDS = {'AB06','AB07','AB08','AB09','AB10','AB11','AB12'};

IMU_SEGMENTS = {'trunk','thigh','shank','foot'};
EMG_MUSCLES  = {'gastrocmed','tibialisanterior','soleus','vastusmedialis','vastuslateralis', ...
                'rectusfemoris','bicepsfemoris','semitendinosus','gracilis','gluteusmedius', ...
                'rightexternaloblique'};

MODE_NAMES = {'STAND','WALK','TURN','STAIRS_UP','STAIRS_DOWN','RAMP_UP','RAMP_DOWN'};

% source 폴더 -> {conditions Label -> mode 이름} 매핑 (순수 라벨만, 실측 라벨셋 기준)
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

THETA_GRID = linspace(0, 1, 101);   % 101포인트 0~1 ("0:0.01:0.99"는 100점이라 상단 설명(101point)에 맞춰 통일)

ENV_WINDOW_S     = 0.15;  % EMG envelope 이동윈도우(turn_detection_emg_AB06.m과 동일 관례)
HEELSTRIKE_WRAP_DROP = 50;    % gcRight.HeelStrike가 이 값보다 크게 떨어지면 새 stride 시작으로 판정
MIN_IMU_SAMPLES_PER_STRIDE = 10;   % 200Hz 기준 50ms 미만이면 버림
MIN_EMG_SAMPLES_PER_STRIDE = 50;   % 1000Hz 기준 50ms 미만이면 버림

%% ---- 누적 컨테이너 ----
% accPop.(modeName).(featureName).data : [N x 101], .subj : {N x 1}
% accSub.(subjectId).(modeName).(featureName).data : [N x 101]
accPop = struct();
accSub = struct();

% 진단용: subj x mode x source-file 단위로 gc 유무/미해당 trial 등을 기록
strideCountLog = struct();  % strideCountLog.(subjId).(modeName) = 누적 stride 수

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

            % 이 trial에 우리 목표 모드가 하나도 없으면 굳이 emg/imu 로드 안 함
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
                    % STAND는 정의상 항상 균등분할, 나머지는 gc 없을 때만 fallback
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
                        continue; % 이 run 안에는 완결된 정상상태 stride가 없음(너무 짧음 등)
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
                    strideCountLog.(subjId).(modeName) = strideCountLog.(subjId).(modeName) + 1;
                end
            end
        end
    end
end

%% ---- 진단 로그: 피험자 x 모드 stride 수 요약 (누락 조용히 넘어가지 않기) ----
fprintf('\n================ 피험자 x 모드 stride 수 요약 ================\n');
fprintf('%-8s', 'subject');
for mIdx = 1:numel(MODE_NAMES); fprintf('%14s', MODE_NAMES{mIdx}); end
fprintf('\n');
missingList = {};
for si = 1:numel(SUBJECT_IDS)
    subjId = SUBJECT_IDS{si};
    if ~isfield(strideCountLog, subjId)
        fprintf('%-8s  (전체 스킵됨 - 날짜폴더 탐색 실패)\n', subjId);
        for mIdx = 1:numel(MODE_NAMES)
            missingList{end+1} = sprintf('%s / %s (subject 전체 스킵)', subjId, MODE_NAMES{mIdx}); %#ok<SAGROW>
        end
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
if isempty(missingList)
    fprintf('\n모든 피험자 x 모드 조합에 최소 1개 이상의 stride 있음.\n');
else
    fprintf('\n[경고] 아래 피험자/모드 조합은 stride가 0개 (population/개인 곡선에서 제외됨):\n');
    for i = 1:numel(missingList)
        fprintf('  - %s\n', missingList{i});
    end
end

%% ---- population / subject 평균 곡선 요약 ----
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
h_d_meta.modes = MODE_NAMES;
h_d_meta.imuSegments = IMU_SEGMENTS;
h_d_meta.emgMuscles = EMG_MUSCLES;
h_d_meta.envWindowS = ENV_WINDOW_S;
h_d_meta.thetaMethod = 'gcRight.HeelStrike(0-100%%) wrap-based, STAND/gc-missing은 run 균등분할 fallback';
h_d_meta.generatedAt = datestr(now);
h_d_meta.missingCombos = missingList;

thetaGrid = THETA_GRID; %#ok<NASGU>

save(fullfile(fileparts(mfilename('fullpath')), 'h_d_population.mat'), ...
    'h_d_population', 'h_d_subject', 'thetaGrid', 'h_d_meta');
fprintf('\n결과 저장 완료: h_d_population.mat\n');


%% ============================== 로컬 함수 ==============================

function dateFolder = find_date_folder(subjRoot)
% subjRoot(예: D:\gait_data\AB06) 밑에서 'osimxml'이 아닌 유일한 하위폴더를
% 날짜 폴더로 자동 인식한다.
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
% Label 열을 (같은 라벨이 연속되는) run 단위로 분해. [t0,t1]은 그 run의
% 시작/끝 timestamp.
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
% gcRight.HeelStrike(0~100%)가 wrapDropThresh 이상 급락하는 지점을
% heel-strike(새 stride 시작) 이벤트로 보고, 그 시각들의 리스트를 반환한다.
% 연속된 두 원소 사이가 "1 stride"에 해당한다.
    d = diff(gcHeelStrike);
    wrapIdx = find(d < -wrapDropThresh);
    strideStarts = [gcHeader(1); gcHeader(wrapIdx + 1)];
end

function emgEnv = compute_emg_envelope(emgTbl, emgMuscles, envWindowS)
% 근육별 MAV/RMS/ZC를 envWindowS 이동윈도우로 "순간값 시계열"로 계산.
% (extract_transition_features.m의 고정 250ms 단일윈도우 정의를 슬라이딩윈도우로 확장)
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
% [ts0,ts1) 구간(1 stride 또는 1 pseudo-stride)의 41개 feature를 thetaGrid(101점)
% 위로 보간한다. useGC=true면 gcRight.HeelStrike/100을 theta로, false면
% (t-ts0)/(ts1-ts0) 균등분할을 theta로 사용한다.
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
% acc.(modeName).(featName).data 에 curveRow(1x101)를 vertcat, .subj에 subjId 기록.
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
