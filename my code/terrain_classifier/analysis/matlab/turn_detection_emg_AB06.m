%% turn_detection_emg_AB06.m
% ============================================================================
% Turning(방향 전환) 구간을 gluteus medius / gracilis / right external oblique
% 3개 EMG 채널로 검출할 수 있는지, AB06의 실제 levelground 데이터로 검증.
%
% 배경: IMU(shank 각속도) 기반 gait event 검출은 turning에서 정확도가
% 떨어진다는 문헌(F1 100%->89~91%)을 확인했음. Huang et al. (2011)의
% neuromuscular-mechanical fusion 개념을 참고해, 실제로 이 3개 근육의
% EMG 진폭이 straight-walking(직진, Label='walk') 대비 turning
% (Label='turn1'/'turn2') 구간에서 유의하게 달라지는지를 확인한다.
%
% 이 3개 근육을 고른 이유 (해부학적 근거):
%   - gluteus medius : 고관절 외전근, 골반 관상면 안정화. Turn 시 안쪽/
%     바깥쪽 다리의 부하가 비대칭적으로 바뀌며 활성 패턴이 변할 것으로 예상.
%   - gracilis        : 고관절 내전근. 스윙다리를 회전 방향으로 리디렉션
%     하는 데 관여할 것으로 예상.
%   - right external oblique : 몸통 회전(trunk rotation)의 직접적 지표.
%     Turn 시 몸통을 회전 방향으로 돌리는 동작에서 활성이 커질 것으로 예상.
%
% 방법:
%   AB06의 levelground 30개 trial(5cw+5ccw x slow/normal/fast) 전부에 대해:
%   1) EMG를 정류(rectify)하고 150ms 이동평균으로 envelope 추출
%      (Signal Processing Toolbox 없이 movmean만 사용 — Butterworth 대체)
%   2) conditions/labels로 각 trial 내 'walk'(직진) 구간과 'turn1'/'turn2'
%      (회전) 구간을 시간 정렬
%   3) 같은 trial 내 walk 구간 평균 진폭으로 정규화(trial 간 speed/gain
%      차이를 제거하기 위한 within-trial normalization)
%   4) turn/walk 비율을 전체 trial에 대해 집계, 대응표본 t검정
%   5) 3채널 조합 fuzzy score로 간단한 turn/straight 분류기를 만들고
%      leave-one-trial-out 정확도를 평가 (Terrain_Intent_Classifier.c의
%      tanh 가중합 패턴과 동일한 접근)
%
% 실행: MATLAB에서 이 스크립트가 있는 폴더를 현재 폴더로 하고 실행.
%       DATA_ROOT 경로만 실제 AB06 데이터 위치로 맞으면 됨.
% ============================================================================

clear; clc;

%% ---- 설정 ----
DATA_ROOT = 'C:\Users\user\Desktop\onephai_ifac\AB06\10_09_18\levelground';
MUSCLES = {'gluteusmedius','gracilis','rightexternaloblique'};
CONTROL_MUSCLE = 'rectusfemoris'; % 비교용 대조군(회전에 특이적이지 않을 것으로 예상되는 근육)
ENV_WINDOW_S = 0.15;              % envelope 이동평균 윈도우 (150ms)
MIN_SEGMENT_S = 0.5;              % 너무 짧은 라벨 구간은 전이 노이즈로 보고 제외

emgFiles = dir(fullfile(DATA_ROOT,'emg','levelground_*.mat'));
fprintf('총 %d개 levelground trial 발견\n', numel(emgFiles));

allMuscles = [MUSCLES, {CONTROL_MUSCLE}];
nM = numel(allMuscles);

% 결과 누적용
turnRatios  = cell(1,nM);   % 각 trial의 turn/walk 비율
walkRatios  = cell(1,nM);   % (검증용) walk 내부 재분할 시 자기 자신 비율(~1 이어야 함)
segFeatures = [];           % [nSeg x nM] 각 segment의 정규화된 envelope (분류기 학습/평가용)
segLabels   = [];           % 1=turn, 0=walk

for f = 1:numel(emgFiles)
    trialName = emgFiles(f).name;

    %% EMG 로드 + envelope
    emgData = load(fullfile(DATA_ROOT,'emg',trialName));
    emgTbl  = emgData.data;
    t_emg   = emgTbl.Header;
    fs = 1/mean(diff(t_emg));
    winSamples = max(1, round(ENV_WINDOW_S*fs));

    env = table();
    env.Header = t_emg;
    for m = 1:nM
        raw = emgTbl.(allMuscles{m});
        env.(allMuscles{m}) = movmean(abs(raw), winSamples);
    end

    %% Conditions/labels 로드
    condData = load(fullfile(DATA_ROOT,'conditions',trialName));
    lab = condData.labels;
    t_lab = lab.Header;
    L = lab.Label;

    % 라벨을 연속 구간(run)으로 분해
    runs = struct('label',{},'t0',{},'t1',{});
    i = 1;
    while i <= numel(L)
        j = i;
        while j < numel(L) && strcmp(L{j+1}, L{i})
            j = j + 1;
        end
        runs(end+1) = struct('label', L{i}, 't0', t_lab(i), 't1', t_lab(j)); %#ok<SAGROW>
        i = j + 1;
    end

    % walk(직진) 구간들의 baseline envelope 평균(근육별) 계산
    walkVals = nan(0,nM);
    for r = 1:numel(runs)
        if strcmp(runs(r).label,'walk') && (runs(r).t1-runs(r).t0) >= MIN_SEGMENT_S
            mask = t_emg >= runs(r).t0 & t_emg <= runs(r).t1;
            if any(mask)
                seg = zeros(1,nM);
                for m = 1:nM
                    seg(m) = mean(env.(allMuscles{m})(mask));
                end
                walkVals(end+1,:) = seg; %#ok<AGROW>
            end
        end
    end
    if isempty(walkVals)
        fprintf('  [스킵] %s: walk 구간 없음\n', trialName);
        continue;
    end
    baseline = mean(walkVals,1); % 근육별 1x nM

    % turn 구간(turn1/turn2)들의 envelope -> baseline 대비 비율
    turnVals = nan(0,nM);
    for r = 1:numel(runs)
        if (strcmp(runs(r).label,'turn1') || strcmp(runs(r).label,'turn2')) ...
                && (runs(r).t1-runs(r).t0) >= MIN_SEGMENT_S
            mask = t_emg >= runs(r).t0 & t_emg <= runs(r).t1;
            if any(mask)
                seg = zeros(1,nM);
                for m = 1:nM
                    seg(m) = mean(env.(allMuscles{m})(mask));
                end
                turnVals(end+1,:) = seg; %#ok<AGROW>
                segFeatures(end+1,:) = seg ./ baseline; %#ok<AGROW>
                segLabels(end+1,1) = 1; %#ok<AGROW>
            end
        end
    end

    % 대조: walk 구간을 반으로 갈라 "walk vs walk" 비율도 같이 기록(자기 대조군)
    if size(walkVals,1) >= 2
        half = floor(size(walkVals,1)/2);
        walkBaseline2 = mean(walkVals(1:half,:),1);
        for k = half+1:size(walkVals,1)
            segFeatures(end+1,:) = walkVals(k,:) ./ walkBaseline2; %#ok<AGROW>
            segLabels(end+1,1) = 0; %#ok<AGROW>
        end
    end

    if ~isempty(turnVals)
        ratio = mean(turnVals,1) ./ baseline;
        for m = 1:nM
            turnRatios{m}(end+1) = ratio(m); %#ok<AGROW>
        end
    end

    fprintf('  [%2d/%2d] %-32s walk_seg=%d turn_seg=%d\n', ...
        f, numel(emgFiles), trialName, size(walkVals,1), size(turnVals,1));
end

%% ---- 근육별 turn/walk 비율 통계 + 대응표본 t검정 ----
fprintf('\n===== 근육별 Turn/Walk EMG 진폭 비율 (AB06, %d trial) =====\n', numel(emgFiles));
for m = 1:nM
    r = turnRatios{m};
    r = r(~isnan(r));
    p = one_sample_ttest_p(r, 1); % 귀무가설: 비율이 1(=차이 없음). Statistics Toolbox 없이 betainc로 직접 계산.
    fprintf('%-22s  mean ratio=%.3f  std=%.3f  n=%d  (vs 1.0, paired t-test p=%.4f) %s\n', ...
        allMuscles{m}, mean(r), std(r), numel(r), p, ternary(p<0.01,'***','   '));
end

%% ---- 간단한 turn/straight 분류기 (3근육 fuzzy score, Terrain_Intent_Classifier.c 패턴) ----
% feature: [gluteusmedius_ratio, gracilis_ratio, rightexternaloblique_ratio] (대조근육 제외)
featIdx = 1:numel(MUSCLES);
X = log(segFeatures(:,featIdx));   % 로그 비율 사용(1을 중심으로 대칭화)
y = segLabels;

nSeg = size(X,1);
pred = zeros(nSeg,1);
scoreAll = zeros(nSeg,1);

% Leave-one-out: 매 segment마다 나머지로 가중치(각 채널 표준화 스케일) 추정 후 판정
for i = 1:nSeg
    trainIdx = setdiff(1:nSeg, i);
    mu = mean(X(trainIdx,:),1);
    sd = std(X(trainIdx,:),0,1) + eps;
    z = (X(i,:) - 0) ./ sd;      % 0(=비율 1, 변화없음) 기준 표준화
    score = tanh(mean(z,2));     % 3채널 평균 후 압축 (Final_FSR_Fuzzy_Logic.c와 동일한 tanh 패턴)
    scoreAll(i) = score;
    pred(i) = score > 0.15;      % Terrain_Intent_Classifier.c와 동일한 완충지대 개념의 threshold
end

acc = mean(pred == y);
tp = sum(pred==1 & y==1); fn = sum(pred==0 & y==1);
tn = sum(pred==0 & y==0); fp = sum(pred==1 & y==0);
recall = tp/(tp+fn); precision = tp/max(tp+fp,1);
f1 = 2*precision*recall/max(precision+recall,eps);

fprintf('\n===== 3채널(gluteusmedius+gracilis+rightexternaloblique) Turn 분류기 =====\n');
fprintf('Segment 수: turn=%d, walk=%d\n', sum(y==1), sum(y==0));
fprintf('Leave-one-out Accuracy=%.1f%%  Precision=%.2f  Recall=%.2f  F1=%.2f\n', ...
    acc*100, precision, recall, f1);

%% ---- 저장 ----
save(fullfile(DATA_ROOT,'..','..','..','turn_emg_result_AB06.mat'), ...
    'turnRatios','allMuscles','segFeatures','segLabels','acc','precision','recall','f1');
fprintf('\n결과 저장: turn_emg_result_AB06.mat\n');

function out = ternary(cond,a,b)
    if cond, out = a; else, out = b; end
end

function p = one_sample_ttest_p(x, mu0)
    % Statistics Toolbox 없이 단일표본 양측 t검정 p-value를 betainc로 직접 계산.
    n = numel(x);
    df = n - 1;
    t = (mean(x) - mu0) / (std(x) / sqrt(n));
    p = betainc(df / (df + t^2), df/2, 0.5);
end
