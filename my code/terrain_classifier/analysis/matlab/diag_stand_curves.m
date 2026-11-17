%% diag_stand_curves.m
% ============================================================================
% 진단 목적:
%   h_d_population.mat 에서 STAND 모드의 IMU_thigh_GYROMAG 평균이 1.34,
%   STAIRS_UP 이 1.99 로 거의 겹친다(Cohen's d ~0.6). 물리적으로 "정지"는
%   각속도가 0 근처여야 하므로 이 값은 의심스럽다.
%
%   가설 A: Camargo 자이로 단위가 rad/s 라서 숫자가 작다 (그러면 d는
%           스케일 불변이므로 겹침 문제는 설명 못 함 -> 기각 예상)
%   가설 B: 'idle' 라벨 구간에 실제로 움직임(자리 이동/방향 전환/체중 이동)이
%           섞여 있어서 STAND 기준곡선이 오염되었다
%   가설 C: STAND 는 gcRight 없이 run 전체를 1 pseudo-stride 로 균등분할하는
%           fallback 을 쓰므로, run 길이가 제각각이라 위상 정렬이 무의미하다
%
% 출력: 콘솔 텍스트 (diag_stand_curves_out.txt 로 리다이렉트해서 확인)
% ============================================================================

clear; clc;

GAIT_ROOT = 'D:\gait_data';
SUBJ      = 'AB06';
DATEF     = '10_09_18';

fprintf('==============================================================\n');
fprintf(' 진단 1: 원본 IMU 자이로 단위/범위 확인 (%s, stair trial)\n', SUBJ);
fprintf('==============================================================\n');

imuFile  = fullfile(GAIT_ROOT, SUBJ, DATEF, 'stair', 'imu',  'stair_1_l_01_01.mat');
condFile = fullfile(GAIT_ROOT, SUBJ, DATEF, 'stair', 'conditions', 'stair_1_l_01_01.mat');

S = load(imuFile);
fn = fieldnames(S);
fprintf('IMU mat 변수: %s\n', strjoin(fn', ', '));
imuTbl = S.(fn{1});
fprintf('테이블 클래스: %s\n', class(imuTbl));
vn = imuTbl.Properties.VariableNames;
fprintf('컬럼 수 = %d\n', numel(vn));
fprintf('컬럼: %s\n', strjoin(vn, ', '));

segs = {'trunk','thigh','shank','foot'};
fprintf('\n--- 세그먼트별 Gyro/Accel 범위 (전체 trial) ---\n');
fprintf('%-8s %-10s %10s %10s %10s %10s\n', 'seg', 'signal', 'min', 'max', 'mean', 'std');
for i = 1:numel(segs)
    seg = segs{i};
    for ax = {'X','Y','Z'}
        cn = sprintf('%s_Gyro_%s', seg, ax{1});
        if ismember(cn, vn)
            v = imuTbl.(cn);
            fprintf('%-8s %-10s %10.3f %10.3f %10.3f %10.3f\n', seg, ['Gyro_' ax{1}], ...
                min(v), max(v), mean(v), std(v));
        end
    end
    cn = sprintf('%s_Accel_Z', seg);
    if ismember(cn, vn)
        v = imuTbl.(cn);
        fprintf('%-8s %-10s %10.3f %10.3f %10.3f %10.3f\n', seg, 'Accel_Z', ...
            min(v), max(v), mean(v), std(v));
    end
end
fprintf(['\n[해석] Accel_Z 의 크기가 ~9.8 이면 m/s^2, ~1.0 이면 g 단위.\n' ...
         '       Gyro 최대값이 수백이면 deg/s, 한 자리~5 정도면 rad/s.\n']);

%% ------------------------------------------------------------------
fprintf('\n\n==============================================================\n');
fprintf(' 진단 2: conditions 라벨 run 구조 (idle 이 정말 "정지"인가)\n');
fprintf('==============================================================\n');

C = load(condFile);
cfn = fieldnames(C);
fprintf('conditions mat 변수: %s\n', strjoin(cfn', ', '));
for i = 1:numel(cfn)
    fprintf('  %s : class=%s\n', cfn{i}, class(C.(cfn{i})));
end

% Camargo conditions 는 보통 .Label (string/categorical) + .Header (시간)
condTbl = [];
for i = 1:numel(cfn)
    if istable(C.(cfn{i}))
        condTbl = C.(cfn{i});
        fprintf('  -> 테이블 사용: %s, 컬럼: %s\n', cfn{i}, ...
            strjoin(condTbl.Properties.VariableNames, ', '));
        break;
    end
end

if isempty(condTbl)
    fprintf('  [경고] 테이블 형태 conditions 를 못 찾음. 구조체 내부 탐색:\n');
    for i = 1:numel(cfn)
        v = C.(cfn{i});
        if isstruct(v)
            sfn = fieldnames(v);
            fprintf('    %s 의 필드: %s\n', cfn{i}, strjoin(sfn', ', '));
            for j = 1:numel(sfn)
                fprintf('      .%s class=%s size=%s\n', sfn{j}, class(v.(sfn{j})), ...
                    mat2str(size(v.(sfn{j}))));
            end
        end
    end
end

%% ------------------------------------------------------------------
fprintf('\n\n==============================================================\n');
fprintf(' 진단 3: idle vs stairascent 구간의 실제 thigh gyro 크기 비교\n');
fprintf('==============================================================\n');

if ~isempty(condTbl)
    vnc = condTbl.Properties.VariableNames;
    labCol = '';
    for c = {'Label','label','Condition'}
        if ismember(c{1}, vnc); labCol = c{1}; break; end
    end
    timeColCond = '';
    for c = {'Header','header','Time','time'}
        if ismember(c{1}, vnc); timeColCond = c{1}; break; end
    end
    fprintf('라벨 컬럼=%s, 시간 컬럼=%s\n', labCol, timeColCond);

    if ~isempty(labCol)
        labs = string(condTbl.(labCol));
        uq = unique(labs);
        fprintf('고유 라벨(%d): %s\n', numel(uq), strjoin(cellstr(uq)', ', '));
        for i = 1:numel(uq)
            fprintf('   %-24s 샘플수=%d\n', uq(i), sum(labs == uq(i)));
        end

        % IMU 시간축
        timeColImu = '';
        for c = {'Header','header','Time','time'}
            if ismember(c{1}, vn); timeColImu = c{1}; break; end
        end
        fprintf('IMU 시간 컬럼=%s\n', timeColImu);

        if ~isempty(timeColImu) && ~isempty(timeColCond)
            tImu  = imuTbl.(timeColImu);
            tCond = condTbl.(timeColCond);
            gmag  = sqrt(imuTbl.thigh_Gyro_X.^2 + imuTbl.thigh_Gyro_Y.^2 + imuTbl.thigh_Gyro_Z.^2);

            fprintf('\n%-24s %8s %10s %10s %10s %10s\n', 'LABEL', 'nImu', 'mean|w|', 'median', 'p95', 'max');
            for i = 1:numel(uq)
                lm = (labs == uq(i));
                if ~any(lm); continue; end
                % 라벨 구간에 해당하는 IMU 샘플 마스크
                tsel = tCond(lm);
                imuMask = tImu >= min(tsel) & tImu <= max(tsel);
                % 정확히는 run 단위로 해야 하지만, 여기선 라벨 전체 시간범위 근사
                gm = gmag(imuMask);
                if isempty(gm); continue; end
                fprintf('%-24s %8d %10.3f %10.3f %10.3f %10.3f\n', uq(i), numel(gm), ...
                    mean(gm), median(gm), prctile(gm, 95), max(gm));
            end
        end
    end
end

%% ------------------------------------------------------------------
fprintf('\n\n==============================================================\n');
fprintf(' 진단 4: idle run 들의 길이 분포 (pseudo-stride 위상 정렬 타당성)\n');
fprintf('==============================================================\n');

if ~isempty(condTbl) && ~isempty(labCol) && ~isempty(timeColCond)
    labs  = string(condTbl.(labCol));
    tCond = condTbl.(timeColCond);
    % 연속 run 분해
    changeIdx = [1; find(diff(categorical(labs)) ~= 0) + 1; numel(labs) + 1];
    fprintf('%-24s %8s %10s\n', 'LABEL', 'run#', 'dur[s]');
    runCount = containers.Map('KeyType','char','ValueType','any');
    for k = 1:numel(changeIdx) - 1
        i0 = changeIdx(k); i1 = changeIdx(k+1) - 1;
        lb = char(labs(i0));
        dur = tCond(i1) - tCond(i0);
        if ~isKey(runCount, lb); runCount(lb) = []; end
        runCount(lb) = [runCount(lb), dur];
    end
    ks = keys(runCount);
    fprintf('\n%-24s %6s %10s %10s %10s\n', 'LABEL', 'nRun', 'meanDur', 'minDur', 'maxDur');
    for k = 1:numel(ks)
        v = runCount(ks{k});
        fprintf('%-24s %6d %10.2f %10.2f %10.2f\n', ks{k}, numel(v), mean(v), min(v), max(v));
    end
    fprintf(['\n[해석] idle run 길이 편차가 크면(min vs max 수 배 이상), 이를 하나의\n' ...
             '       pseudo-stride 로 0~1 균등분할한 STAND 기준곡선은 위상 의미가\n' ...
             '       없고 사실상 "구간 평균 + 큰 분산"이 된다.\n']);
end

fprintf('\n진단 완료.\n');
