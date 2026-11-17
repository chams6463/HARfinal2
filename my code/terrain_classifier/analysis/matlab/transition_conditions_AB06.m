%% transition_conditions_AB06.m
% ============================================================================
% AB06의 실제 데이터로 7개 상태(STAND, WALK, TURN, STAIRS_UP, STAIRS_DOWN,
% RAMP_UP, RAMP_DOWN) 사이에서 "실제로 데이터에 존재하는" 전환만 골라
% IMU + EMG + EMG 특징벡터 기반 판정조건(threshold)을 추출한다.
%
% 중요: 42가지(7*6) 조합 중 실제로 프로토콜상 인접하게 관측되는 전환은
% 12가지뿐이다(아래 TRANSITIONS 표). 나머지 30가지는 이 스크립트 실행 후
% 별도로 "경유(compose)" 방식으로 설명한다(코드가 아니라 텍스트로 별도 정리).
%
% 사용 신호:
%   1) IMU  : trunk/thigh/shank/foot의 Accel+Gyro (AB06 imu 테이블 그대로)
%      - tilt(각도 proxy)  = atan2(Accel_X, Accel_Z) [deg]  (중력 기준 시상면 기울기)
%      - gyro_mag          = sqrt(Gx^2+Gy^2+Gz^2)            (움직임 강도)
%   2) EMG  : 11채널 원시 rectified 신호
%   3) EMG 특징벡터 (채널별 3개): MAV, RMS, ZC(zero-crossing rate)
%      -> 총 11*3 = 33차원 + IMU 파생 4세그먼트*2 = 8차원 => 41차원 후보 feature
%
% 각 전환 경계(라벨이 바뀌는 시점) 전후 300ms 윈도우를 비교해서
% Cohen's d가 큰 순으로 상위 판별 feature와 threshold를 자동 산출한다.
% ============================================================================

clear; clc;
ROOT = 'C:\Users\user\Desktop\onephai_ifac\AB06\10_09_18';
PRE_WIN_S  = [-0.30 -0.05];
POST_WIN_S = [ 0.05  0.30];

EMG_MUSCLES = {'gastrocmed','tibialisanterior','soleus','vastusmedialis','vastuslateralis', ...
               'rectusfemoris','bicepsfemoris','semitendinosus','gracilis','gluteusmedius','rightexternaloblique'};
IMU_SEGMENTS = {'trunk','thigh','shank','foot'};

% --- 전환 정의: {모드, 파일패턴, [이전라벨패턴들], [이후라벨패턴들], 전환이름} ---
TRANSITIONS = cell(12,5);
TRANSITIONS(1,:)  = {'levelground', 'levelground_*.mat', {'stand'},          {'stand-walk'},         'STAND_TO_WALK'};
TRANSITIONS(2,:)  = {'levelground', 'levelground_*.mat', {'walk'},           {'turn1','turn2'},      'WALK_TO_TURN'};
TRANSITIONS(3,:)  = {'levelground', 'levelground_*.mat', {'turn1','turn2'},  {'walk'},               'TURN_TO_WALK'};
TRANSITIONS(4,:)  = {'levelground', 'levelground_*.mat', {'walk'},           {'walk-stand'},         'WALK_TO_STAND'};
TRANSITIONS(5,:)  = {'stair',       'stair_*.mat',       {'idle'},           {'walk-stairascent'},   'STAND_TO_STAIRUP'};
TRANSITIONS(6,:)  = {'stair',       'stair_*.mat',       {'stairascent'},    {'stairascent-walk'},   'STAIRUP_TO_STAND'};
TRANSITIONS(7,:)  = {'stair',       'stair_*.mat',       {'idle'},           {'walk-stairdescent'},  'STAND_TO_STAIRDOWN'};
TRANSITIONS(8,:)  = {'stair',       'stair_*.mat',       {'stairdescent'},   {'stairdescent-walk'},  'STAIRDOWN_TO_STAND'};
TRANSITIONS(9,:)  = {'ramp',        'ramp_*.mat',        {'idle'},           {'walk-rampascent'},    'STAND_TO_RAMPUP'};
TRANSITIONS(10,:) = {'ramp',        'ramp_*.mat',        {'rampascent'},     {'rampascent-walk'},    'RAMPUP_TO_STAND'};
TRANSITIONS(11,:) = {'ramp',        'ramp_*.mat',        {'idle'},           {'walk-rampdescent'},   'STAND_TO_RAMPDOWN'};
TRANSITIONS(12,:) = {'ramp',        'ramp_*.mat',        {'rampdescent'},    {'rampdescent-walk'},   'RAMPDOWN_TO_STAND'};

results = struct();

for tIdx = 1:size(TRANSITIONS,1)
    mode = TRANSITIONS{tIdx,1};
    filePattern = TRANSITIONS{tIdx,2};
    fromLabels = TRANSITIONS{tIdx,3};
    toLabels = TRANSITIONS{tIdx,4};
    tname = TRANSITIONS{tIdx,5};

    emgDir = fullfile(ROOT, mode, 'emg');
    imuDir = fullfile(ROOT, mode, 'imu');
    condDir = fullfile(ROOT, mode, 'conditions');
    files = dir(fullfile(condDir, filePattern));

    % feature accumulators: struct field name -> [pre_vals; post_vals] columns = occurrence
    featPre = containers.Map();
    featPost = containers.Map();
    nOcc = 0;

    for f = 1:numel(files)
        fname = files(f).name;
        try
            cond = load(fullfile(condDir, fname));
        catch
            continue;
        end
        if ~isfield(cond, 'labels'); continue; end
        lab = cond.labels; t_lab = lab.Header; L = lab.Label;

        % 연속 구간(run)으로 분해
        runs = struct('label',{},'t0',{},'t1',{});
        i = 1;
        while i <= numel(L)
            j = i;
            while j < numel(L) && strcmp(L{j+1}, L{i}); j = j + 1; end
            runs(end+1) = struct('label', L{i}, 't0', t_lab(i), 't1', t_lab(j)); %#ok<SAGROW>
            i = j + 1;
        end

        % 이 파일에서 원하는 전환(from->to) 경계 찾기
        boundaryTimes = [];
        for r = 1:numel(runs)-1
            if any(strcmp(runs(r).label, fromLabels)) && any(strcmp(runs(r+1).label, toLabels))
                boundaryTimes(end+1) = runs(r+1).t0; %#ok<AGROW>
            end
        end
        if isempty(boundaryTimes); continue; end

        % EMG, IMU 로드 (해당 전환이 있는 파일만)
        try
            emgData = load(fullfile(emgDir, fname)); emgTbl = emgData.data;
            imuData = load(fullfile(imuDir, fname)); imuTbl = imuData.data;
        catch
            continue;
        end
        t_emg = emgTbl.Header; t_imu = imuTbl.Header;

        for b = 1:numel(boundaryTimes)
            tb = boundaryTimes(b);
            preMask_e  = t_emg >= tb+PRE_WIN_S(1)  & t_emg <= tb+PRE_WIN_S(2);
            postMask_e = t_emg >= tb+POST_WIN_S(1) & t_emg <= tb+POST_WIN_S(2);
            preMask_i  = t_imu >= tb+PRE_WIN_S(1)  & t_imu <= tb+PRE_WIN_S(2);
            postMask_i = t_imu >= tb+POST_WIN_S(1) & t_imu <= tb+POST_WIN_S(2);
            if sum(preMask_e) < 5 || sum(postMask_e) < 5 || sum(preMask_i) < 5 || sum(postMask_i) < 5
                continue;
            end
            nOcc = nOcc + 1;

            % --- EMG 특징벡터: MAV, RMS, ZC (채널별) ---
            for m = 1:numel(EMG_MUSCLES)
                musc = EMG_MUSCLES{m};
                raw_pre  = emgTbl.(musc)(preMask_e);
                raw_post = emgTbl.(musc)(postMask_e);

                mav_pre = mean(abs(raw_pre));   mav_post = mean(abs(raw_post));
                rms_pre = sqrt(mean(raw_pre.^2)); rms_post = sqrt(mean(raw_post.^2));
                zc_pre  = sum(diff(sign(raw_pre))  ~= 0) / numel(raw_pre);
                zc_post = sum(diff(sign(raw_post)) ~= 0) / numel(raw_post);

                keys_vals = {sprintf('EMG_%s_MAV', musc), mav_pre, mav_post; ...
                             sprintf('EMG_%s_RMS', musc), rms_pre, rms_post; ...
                             sprintf('EMG_%s_ZC',  musc), zc_pre,  zc_post};
                for kk = 1:size(keys_vals,1)
                    key = keys_vals{kk,1};
                    if ~isKey(featPre, key); featPre(key) = []; featPost(key) = []; end
                    featPre(key)  = [featPre(key),  keys_vals{kk,2}];
                    featPost(key) = [featPost(key), keys_vals{kk,3}];
                end
            end

            % --- IMU 특징: tilt(accel 기반), gyro magnitude (세그먼트별) ---
            for s = 1:numel(IMU_SEGMENTS)
                seg = IMU_SEGMENTS{s};
                ax_pre = imuTbl.([seg '_Accel_X'])(preMask_i); az_pre = imuTbl.([seg '_Accel_Z'])(preMask_i);
                ax_post = imuTbl.([seg '_Accel_X'])(postMask_i); az_post = imuTbl.([seg '_Accel_Z'])(postMask_i);
                tilt_pre = mean(atan2(ax_pre, az_pre)) * 180/pi;
                tilt_post = mean(atan2(ax_post, az_post)) * 180/pi;

                gx_pre = imuTbl.([seg '_Gyro_X'])(preMask_i); gy_pre = imuTbl.([seg '_Gyro_Y'])(preMask_i); gz_pre = imuTbl.([seg '_Gyro_Z'])(preMask_i);
                gx_post = imuTbl.([seg '_Gyro_X'])(postMask_i); gy_post = imuTbl.([seg '_Gyro_Y'])(postMask_i); gz_post = imuTbl.([seg '_Gyro_Z'])(postMask_i);
                gmag_pre = mean(sqrt(gx_pre.^2+gy_pre.^2+gz_pre.^2));
                gmag_post = mean(sqrt(gx_post.^2+gy_post.^2+gz_post.^2));

                keys_vals = {sprintf('IMU_%s_TILT', seg), tilt_pre, tilt_post; ...
                             sprintf('IMU_%s_GYROMAG', seg), gmag_pre, gmag_post};
                for kk = 1:size(keys_vals,1)
                    key = keys_vals{kk,1};
                    if ~isKey(featPre, key); featPre(key) = []; featPost(key) = []; end
                    featPre(key)  = [featPre(key),  keys_vals{kk,2}];
                    featPost(key) = [featPost(key), keys_vals{kk,3}];
                end
            end
        end
    end

    % --- 이 전환에 대해 feature별 Cohen's d 계산 후 랭킹 ---
    keys = featPre.keys;
    dvals = zeros(numel(keys),1);
    meanPre = zeros(numel(keys),1); meanPost = zeros(numel(keys),1);
    for k = 1:numel(keys)
        pv = featPre(keys{k}); qv = featPost(keys{k});
        pooled_std = sqrt((var(pv)+var(qv))/2) + eps;
        dvals(k) = (mean(qv)-mean(pv)) / pooled_std;
        meanPre(k) = mean(pv); meanPost(k) = mean(qv);
    end
    [~, order] = sort(abs(dvals), 'descend');

    fprintf('\n================ %s (n=%d instances) ================\n', tname, nOcc);
    topN = min(5, numel(order));
    for r = 1:topN
        k = order(r);
        thr = (meanPre(k)+meanPost(k))/2;
        direction = 'increase';
        if meanPost(k) < meanPre(k); direction = 'decrease'; end
        fprintf('%2d. %-28s  pre=%9.4f  post=%9.4f  d=%6.2f  (%s, threshold~=%.4f)\n', ...
            r, keys{k}, meanPre(k), meanPost(k), dvals(k), direction, thr);
    end

    results.(tname).keys = keys;
    results.(tname).d = dvals;
    results.(tname).meanPre = meanPre;
    results.(tname).meanPost = meanPost;
    results.(tname).n = nOcc;
end

save('transition_conditions_AB06_result.mat', 'results');
fprintf('\n\n저장 완료: transition_conditions_AB06_result.mat\n');
