%% diag_stand_curves2.m
% ============================================================================
% diag_stand_curves.m 의 진단 3/4 를 "라벨 run 단위"로 정확히 다시 수행한다.
% (1차 진단은 idle 마스크를 min(t)~max(t) 로 잡는 바람에 trial 전체를 덮어서
%  무효였다 — idle 은 trial 시작과 끝에 모두 등장하므로.)
%
% 확인하려는 것:
%   Q1. idle(=STAND) run 구간의 실제 각속도 크기는 정말 0 근처인가?
%   Q2. idle run 길이 분포가 제각각이라 pseudo-stride 위상정렬이 무의미한가?
%   Q3. build_h_d_reference_curves.m 의 STAND 곡선이 왜 STAIRS_UP 과 겹치는가?
%
% 여러 trial/여러 소스(levelground/stair/ramp)를 훑어서 통계를 낸다.
% ============================================================================

clear; clc;

GAIT_ROOT = 'D:\gait_data';
SUBJ      = 'AB06';
DATEF     = '10_09_18';
SOURCES   = {'levelground','stair','ramp'};

% 라벨 -> 모드 (build_h_d_reference_curves.m 과 동일)
MAPS = containers.Map();
MAPS('levelground') = {'idle','STAND'; 'walk','WALK'; 'turn1','TURN'; 'turn2','TURN'};
MAPS('stair')       = {'idle','STAND'; 'stairascent','STAIRS_UP'; 'stairdescent','STAIRS_DOWN'};
MAPS('ramp')        = {'idle','STAND'; 'rampascent','RAMP_UP'; 'rampdescent','RAMP_DOWN'};

% 모드별 누적: 각속도 크기 샘플, run 길이
agg = struct();
MODES = {'STAND','WALK','TURN','STAIRS_UP','STAIRS_DOWN','RAMP_UP','RAMP_DOWN'};
for i = 1:numel(MODES)
    agg.(MODES{i}).gm   = [];   % thigh gyro magnitude 샘플 (서브샘플링)
    agg.(MODES{i}).durs = [];   % run 길이 [s]
    agg.(MODES{i}).nRun = 0;
end

for si = 1:numel(SOURCES)
    src = SOURCES{si};
    imuDir  = fullfile(GAIT_ROOT, SUBJ, DATEF, src, 'imu');
    condDir = fullfile(GAIT_ROOT, SUBJ, DATEF, src, 'conditions');
    if ~isfolder(imuDir); continue; end
    files = dir(fullfile(imuDir, '*.mat'));
    mapArr = MAPS(src);

    fprintf('--- %s : %d trial ---\n', src, numel(files));

    for fi = 1:numel(files)
        imuFile  = fullfile(imuDir,  files(fi).name);
        condFile = fullfile(condDir, files(fi).name);
        if ~isfile(condFile); continue; end

        Si = load(imuFile);   imuTbl  = Si.data;
        Sc = load(condFile);  condTbl = Sc.labels;

        tImu = imuTbl.Header;
        gm   = sqrt(imuTbl.thigh_Gyro_X.^2 + imuTbl.thigh_Gyro_Y.^2 + imuTbl.thigh_Gyro_Z.^2);

        labs  = string(condTbl.Label);
        tCond = condTbl.Header;

        % --- 연속 run 분해 (문자열 비교로, categorical diff 쓰지 않음) ---
        n = numel(labs);
        if n < 2; continue; end
        boundaries = [1; find(labs(1:end-1) ~= labs(2:end)) + 1; n + 1];

        for k = 1:numel(boundaries) - 1
            i0 = boundaries(k);
            i1 = boundaries(k+1) - 1;
            lb = char(labs(i0));

            % 라벨 -> 모드
            mode = '';
            for r = 1:size(mapArr, 1)
                if strcmp(lb, mapArr{r,1}); mode = mapArr{r,2}; break; end
            end
            if isempty(mode); continue; end   % 하이픈 전환 라벨 등은 제외

            t0 = tCond(i0);
            t1 = tCond(i1);
            dur = t1 - t0;
            if dur <= 0; continue; end

            m = (tImu >= t0) & (tImu <= t1);
            if nnz(m) < 10; continue; end

            v = gm(m);
            % 메모리 절약: run 당 최대 200 샘플만
            if numel(v) > 200
                v = v(round(linspace(1, numel(v), 200)));
            end
            agg.(mode).gm   = [agg.(mode).gm; v(:)];
            agg.(mode).durs = [agg.(mode).durs; dur];
            agg.(mode).nRun = agg.(mode).nRun + 1;
        end
    end
end

fprintf('\n\n==============================================================\n');
fprintf(' Q1. 모드별 thigh 각속도 크기 |w| [rad/s] (run 단위 정확 마스킹)\n');
fprintf('==============================================================\n');
fprintf('%-14s %7s %9s %9s %9s %9s %9s\n', 'MODE', 'nRun', 'mean', 'median', 'p05', 'p95', 'max');
for i = 1:numel(MODES)
    m = MODES{i};
    v = agg.(m).gm;
    if isempty(v)
        fprintf('%-14s %7d  (데이터 없음)\n', m, agg.(m).nRun); continue;
    end
    fprintf('%-14s %7d %9.3f %9.3f %9.3f %9.3f %9.3f\n', m, agg.(m).nRun, ...
        mean(v), median(v), prctile(v,5), prctile(v,95), max(v));
end
fprintf(['\n[기대] STAND 는 median 이 0.05 rad/s(=3 deg/s) 미만이어야 정상.\n' ...
         '       STAND median 이 0.5 rad/s 를 넘으면 idle 라벨이 오염된 것.\n']);

fprintf('\n\n==============================================================\n');
fprintf(' Q2. 모드별 run 길이 분포 [s] (pseudo-stride 위상정렬 타당성)\n');
fprintf('==============================================================\n');
fprintf('%-14s %7s %9s %9s %9s %9s\n', 'MODE', 'nRun', 'mean', 'min', 'max', 'max/min');
for i = 1:numel(MODES)
    m = MODES{i};
    d = agg.(m).durs;
    if isempty(d); continue; end
    fprintf('%-14s %7d %9.2f %9.2f %9.2f %9.1f\n', m, numel(d), ...
        mean(d), min(d), max(d), max(d)/max(min(d), 1e-6));
end
fprintf(['\n[해석] STAND 의 max/min 이 크면(>3), run 전체를 theta 0~1 로 균등분할한\n' ...
         '       STAND 기준곡선은 서로 다른 시간축을 억지로 겹친 것이라 위상 의미가\n' ...
         '       없다 -> h_d^STAND(theta) 는 사실상 상수 + 큰 분산.\n']);

fprintf('\n진단 완료.\n');
