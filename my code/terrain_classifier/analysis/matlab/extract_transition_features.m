function results = extract_transition_features(ROOT, verbose)
%% extract_transition_features  12개 실전환에 대해 IMU+EMG 특징 Cohen's d 랭킹 계산
% ROOT: 예) 'D:\gait_data\AB07\10_14_18'  (mode 폴더들의 상위 날짜 폴더)
% verbose: true면 전환별 top5를 콘솔에 출력
if nargin < 2; verbose = true; end

PRE_WIN_S  = [-0.30 -0.05];
POST_WIN_S = [ 0.05  0.30];

EMG_MUSCLES = {'gastrocmed','tibialisanterior','soleus','vastusmedialis','vastuslateralis', ...
               'rectusfemoris','bicepsfemoris','semitendinosus','gracilis','gluteusmedius','rightexternaloblique'};
IMU_SEGMENTS = {'trunk','thigh','shank','foot'};

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
    modeName = TRANSITIONS{tIdx,1};
    filePattern = TRANSITIONS{tIdx,2};
    fromLabels = TRANSITIONS{tIdx,3};
    toLabels = TRANSITIONS{tIdx,4};
    tname = TRANSITIONS{tIdx,5};

    emgDir = fullfile(ROOT, modeName, 'emg');
    imuDir = fullfile(ROOT, modeName, 'imu');
    condDir = fullfile(ROOT, modeName, 'conditions');
    files = dir(fullfile(condDir, filePattern));

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

        runs = struct('label',{},'t0',{},'t1',{});
        i = 1;
        while i <= numel(L)
            j = i;
            while j < numel(L) && strcmp(L{j+1}, L{i}); j = j + 1; end
            runs(end+1) = struct('label', L{i}, 't0', t_lab(i), 't1', t_lab(j)); %#ok<SAGROW>
            i = j + 1;
        end

        boundaryTimes = [];
        for r = 1:numel(runs)-1
            if any(strcmp(runs(r).label, fromLabels)) && any(strcmp(runs(r+1).label, toLabels))
                boundaryTimes(end+1) = runs(r+1).t0; %#ok<AGROW>
            end
        end
        if isempty(boundaryTimes); continue; end

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

    if verbose
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
    end

    results.(tname).keys = keys;
    results.(tname).d = dvals;
    results.(tname).meanPre = meanPre;
    results.(tname).meanPost = meanPost;
    results.(tname).n = nOcc;
    results.(tname).order = order;
end
end
