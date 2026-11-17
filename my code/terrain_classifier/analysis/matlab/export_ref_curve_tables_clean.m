%% export_ref_curve_tables.m
% ============================================================================
% h_d_population.mat 의 위상 기준곡선(reference curve) r^m(theta) 를 STM32
% 펌웨어용 C 헤더(static const float lookup table)로 export 한다.
%
% 출력: D:\ifac_wearable_contest\XM_Apps\User_Algorithm\ref_curve_tables.h
%
% ----------------------------------------------------------------------------
% 이름에 대해 — 왜 "h_d"가 아니라 "reference curve(RC)"인가
% ----------------------------------------------------------------------------
% 이전 버전은 h_d^m(theta) 표기를 썼는데, 이는 HZD(Hybrid Zero Dynamics,
% Grizzle/Westervelt/Ames 계열) 이족보행 제어이론에서 virtual constraint 의
% "desired output"을 가리키는 특정 용어다(y = h(x) - h_d(theta(x)) -> 0 으로
% zero dynamics manifold 를 만드는 제어기 대상). 우리가 만든 것은 그런 제어
% 대상이 아니라, 모드별 평균±표준편차 위상곡선에 대한 통계적 템플릿 매칭
% (z-score 최근접 분류)이다. virtual constraint 도, 이를 추종하는 토크 제어
% 루프도 없다(XM_SetControlMode(XM_CTRL_MONITOR) — 애초에 토크를 안 낸다).
% 따라서 이 세션부터 "h_d" 대신 "reference curve(줄여서 Rc/RC)"로 표기를
% 정정한다. 원본 h_d_population.mat 파일명/내부 필드명은 그대로 둔다 — 이걸
% 만든 build_h_d_reference_curves.m 등 여러 기존 스크립트가 이 스키마를
% 공유하므로, 여기서 바꾸면 그 스크립트들까지 전부 고쳐야 하는 불필요한
% 파급이 생긴다. 이름 정정은 "우리가 만든 산출물"(이 생성기 + 그 출력 헤더 +
% 이를 소비하는 분류기)에 한정한다.
%
% ----------------------------------------------------------------------------
% 왜 41개 feature 를 전부 넣지 않는가 — 선별 근거
% ----------------------------------------------------------------------------
% analyze_locomotion_bank.py / diag_stand_curves2.m 결과에 따른 선별:
%
% [모드] 7개 중 5개만 뱅크에 넣는다.
%   * STAND 제외: Camargo 'idle' 라벨은 정지가 아니다. run 단위 실측 결과
%       STAND median |w| = 0.923 rad/s (53 deg/s) 로 STAIRS_DOWN(0.872)보다
%       크고, run 길이가 0.09~9.51 s (100:1) 라 균등분할 위상정렬이 무의미하다.
%       => r^STAND 는 기준곡선으로 쓸 수 없음. 펌웨어에서 forwardVelocity +
%          각속도 에너지 게이트로 판정한다(단, 아래 IDLE 위상무관 템플릿으로
%          "투표 거부권" 용도로는 재활용한다).
%   * TURN  제외: TURN vs WALK 는 41 feature 전부를 뒤져도 d_subj = 0.80 이
%       최대이고, EMG 를 3자리까지 늘려도 개선되지 않는다. 회전은 yaw 현상인데
%       기준곡선의 IMU feature 는 sagittal tilt 와 gyro '크기'뿐이라 방향
%       정보가 없기 때문. => 펌웨어에서 hip IMU yaw-rate 적분으로 판정한다.
%
% [feature] 41개 중 4개만.
%   * IMU_trunk_TILT     채택 — 판별력의 50%가 DC(자세 기울기)에서 온다.
%                        H10 pelvicAngle 로 그대로 전이된다.
%   * IMU_thigh_GYROMAG  채택 — theta 0.10(입각초기)/0.60~0.70(swing개시)에
%                        물리적으로 타당한 두 피크를 갖는 깨끗한 파형.
%   * IMU_thigh_TILT     제외 — 모드별 평균이 전부 -85~-87 deg 로 동일(DC 기여
%                        -2%)하고, 판별력이 heel-strike/toe-off 충격 스파이크
%                        에서만 나온다. H10 의 칼만 필터링된 leftThighAngle 에는
%                        그 스파이크가 없어 템플릿이 맞지 않는다.
%   * IMU_trunk_GYROMAG  제외 — d_subj 0.92 로 약하고, 유일한 강점이던 TURN 이
%                        뱅크에서 빠졌다.
%   * IMU_shank_*/foot_* 제외 — 무릎/발목 엔코더 없음. 단, 대체 손실은 15쌍
%                        평균 1.7% 에 불과함을 확인했다(12쌍은 손실 0%).
%   * EMG                greedy 선택 결과 gastrocmed(총 gain 5.44) +
%                        vastusmedialis(+0.72). RMS 만 사용(MAV 와 거의 중복).
%
% 메모리: 5 mode x 4 feature x 101 point x (mean+std) x 4 B = 16,160 B ~ 15.8 KB
%         (examples\19_Memory_Aware_Design 의 예산 관례에 맞춰 flash 상수로만
%          잡히고 RAM 은 소비하지 않는다 — static const 이므로 .rodata)
%
% ----------------------------------------------------------------------------
% 단위 주의 (펌웨어에서 반드시 보정해야 함)
% ----------------------------------------------------------------------------
%   Camargo IMU accel = g,  gyro = rad/s   (원본 실측 확인)
%   따라서 GYROMAG 은 rad/s 단위이고, H10 의 deg/s 와 다르다.
%   TILT 는 atan2(AccX,AccZ) [deg] 이라 H10 pelvicAngle 과 offset/sign 이 다르다.
%   EMG 는 Camargo mV 스케일이라 우리 ADC 전압과 완전히 다르다.
%   => 펌웨어는 BTN1 캘리브레이션(평지보행)에서 feature 별 affine 계수
%      (sign, scale, offset)와 위상 offset 을 자동 추정해 이 표에 정렬시킨다.
%      이 스크립트는 원본 단위 그대로 내보낸다(보정은 런타임 담당).
% ============================================================================

clear; clc;

MAT_PATH = 'D:\ifac_wearable_contest\Camargo_matlab\h_d_population_clean.mat';
OUT_PATH = 'D:\ifac_wearable_contest\XM_Apps\User_Algorithm\ref_curve_tables.h';

% 뱅크에 넣을 모드 (STAND/TURN 제외 — 위 주석 참고)
BANK_MODES = {'WALK','STAIRS_UP','STAIRS_DOWN','RAMP_UP','RAMP_DOWN'};

% 선정 feature: {h_d_population.mat 필드명, C 심볼 접미사, 펌웨어 소스 설명}
% (h_d_population.mat 내부 필드명은 원본 스키마 그대로 — 위 설명 참고)
FEATURES = {
    'IMU_trunk_TILT',        'TRUNK_TILT',    'XM.status.h10.pelvicAngle [deg]'
    'IMU_thigh_GYROMAG',     'THIGH_GYROMAG', 'hip IMU |gyro| = sqrt(GyrX^2+GyrY^2+GyrZ^2) [deg/s]'
    'EMG_gastrocmed_RMS',    'EMG_SITE0_RMS', 'EMG site0 (gastrocnemius medialis) RMS [V]'
    'EMG_vastusmedialis_RMS','EMG_SITE1_RMS', 'EMG site1 (vastus medialis) RMS [V]'
};

fprintf('h_d_population.mat 로드: %s\n', MAT_PATH);
S = load(MAT_PATH);
pop       = S.h_d_population;
thetaGrid = S.thetaGrid;
nTheta    = numel(thetaGrid);

nMode = numel(BANK_MODES);
nFeat = size(FEATURES, 1);

assert(nTheta == 101, 'thetaGrid 가 101 포인트가 아님: %d', nTheta);

% ---- 존재 확인 및 수집 ----
meanTab = zeros(nMode, nFeat, nTheta);
stdTab  = zeros(nMode, nFeat, nTheta);

fprintf('\n%-14s', 'MODE');
for f = 1:nFeat, fprintf('%22s', FEATURES{f,2}); end
fprintf('\n');

for m = 1:nMode
    md = BANK_MODES{m};
    assert(isfield(pop, md), 'h_d_population 에 모드 %s 없음', md);
    fprintf('%-14s', md);
    for f = 1:nFeat
        fn = FEATURES{f,1};
        assert(isfield(pop.(md), fn), 'h_d_population.%s 에 feature %s 없음', md, fn);
        c = pop.(md).(fn);
        mu = c.mean(:).';
        sd = c.std(:).';
        assert(numel(mu) == nTheta && numel(sd) == nTheta, ...
            '%s/%s 길이 불일치', md, fn);
        assert(all(isfinite(mu)) && all(isfinite(sd)), '%s/%s 에 NaN/Inf', md, fn);

        % std 가 0 이면 런타임 0 나눗셈 -> 하한 클램프 (해당 feature 스케일의 1%)
        floorVal = 0.01 * max(abs(mu));
        if floorVal <= 0, floorVal = 1e-6; end
        sd = max(sd, floorVal);

        meanTab(m,f,:) = mu;
        stdTab(m,f,:)  = sd;
        fprintf('%22s', sprintf('n=%d', c.nStride));
    end
    fprintf('\n');
end

% ---- 헤더 파일 작성 ----
fprintf('\n헤더 생성: %s\n', OUT_PATH);
fid = fopen(OUT_PATH, 'w');
assert(fid > 0, '파일 열기 실패: %s', OUT_PATH);

W = @(varargin) fprintf(fid, varargin{:});

W('/**\n');
W(' ******************************************************************************\n');
W(' * @file    ref_curve_tables.h\n');
W(' * @brief   위상 기준곡선(reference curve) r^m(theta) lookup table — 자동 생성 파일\n');
W(' *\n');
W(' * @warning 이 파일은 export_ref_curve_tables.m 이 생성합니다. 직접 수정하지 마세요.\n');
W(' *          재생성: Camargo_matlab 폴더에서 MATLAB 로 export_ref_curve_tables.m 실행\n');
W(' *\n');
W(' * @note    "Rc"(Reference Curve) 명명 — 이전 버전은 HZD(Hybrid Zero Dynamics)\n');
W(' *          제어이론의 virtual constraint desired-output 표기인 "h_d"를 빌려\n');
W(' *          썼으나, 이 테이블은 그런 제어 대상이 아니라 순수 통계적 템플릿\n');
W(' *          매칭(z-score 최근접 분류)에 쓰인다. virtual constraint 도, 이를\n');
W(' *          추종하는 토크 제어 루프도 없다 — 그래서 이름을 정정했다.\n');
W(' *\n');
W(' * 원본: Camargo et al. (2021) 하지 보행 데이터셋 22명 중 AB11 제외 21명\n');
W(' *       (AB11 은 gastrocmed EMG 전극 접촉 불량 의심으로 제외 — held-out 재평가\n');
W(' *       결과 52~63%%로 나머지 전원(87~98%%)보다 압도적으로 낮았음. 상세 근거는\n');
W(' *       AB11_AB28_outlier_analysis.md, 검증 스크립트는 validate_1~5.py 참고)\n');
W(' *       -> h_d_population_clean.mat -> 본 파일\n');
W(' *       (원본 .mat 파일명/내부 필드명은 기존 스키마를 그대로 따른다)\n');
W(' * 생성일시: %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
W(' *\n');
W(' * ---------------------------------------------------------------------------\n');
W(' * 모드 선정: 7개 중 5개 (STAND/TURN 제외)\n');
W(' *   STAND 제외 — Camargo ''idle'' 라벨은 정지가 아님. run 단위 실측 결과\n');
W(' *     median |w| = 0.923 rad/s (53 deg/s) 로 STAIRS_DOWN(0.872)보다 크고,\n');
W(' *     run 길이가 0.09~9.51 s (100:1) 라 균등분할 위상정렬이 무의미.\n');
W(' *     -> 펌웨어에서 forwardVelocity + 각속도 에너지 게이트로 판정.\n');
W(' *   TURN 제외 — TURN vs WALK 는 41 feature 전부에서 d_subj <= 0.80 이고\n');
W(' *     EMG 3자리로도 개선 안 됨(회전은 yaw 현상인데 기준곡선에 yaw feature 없음).\n');
W(' *     -> 펌웨어에서 hip IMU yaw-rate 적분으로 판정.\n');
W(' *\n');
W(' * feature 선정: 41개 중 4개\n');
W(' *   IMU_trunk_TILT      판별력의 50%%가 DC(자세 기울기) -> pelvicAngle 로 전이\n');
W(' *   IMU_thigh_GYROMAG   theta 0.10/0.60~0.70 두 피크의 깨끗한 물리 파형\n');
W(' *   EMG gastrocmed RMS  greedy 1순위 (총 gain 5.44)\n');
W(' *   EMG vastusmedialis  greedy 2순위 (+0.72)\n');
W(' *   (IMU_thigh_TILT 제외 — 모드별 평균이 모두 -85~-87 deg 로 동일하고\n');
W(' *    판별력이 heel-strike/toe-off 충격 스파이크에서만 나옴. H10 의 칼만\n');
W(' *    필터링된 leftThighAngle 에는 그 스파이크가 없어 템플릿 불일치.)\n');
W(' *   (shank/foot 제외 — 무릎/발목 엔코더 없음. 대체 손실은 15쌍 평균 1.7%%)\n');
W(' *\n');
W(' * @warning 단위가 H10 과 다릅니다 — 런타임 affine 보정 필수\n');
W(' *   Camargo gyro = rad/s (H10 은 deg/s), accel = g\n');
W(' *   TILT = atan2(AccX,AccZ) [deg] — pelvicAngle 과 offset/sign 다름\n');
W(' *   EMG = Camargo mV 스케일 — 우리 ADC 전압과 무관\n');
W(' *   -> BTN1 평지보행 캘리브레이션에서 sign/scale/offset/위상offset 자동 추정\n');
W(' *\n');
W(' * 메모리: %d mode x %d feature x %d point x (mean+std) x 4 B = %d B (%.1f KB, .rodata)\n', ...
    nMode, nFeat, nTheta, 2*nMode*nFeat*nTheta*4, 2*nMode*nFeat*nTheta*4/1024);
W(' ******************************************************************************\n');
W(' */\n\n');
W('#ifndef REF_CURVE_TABLES_H\n#define REF_CURVE_TABLES_H\n\n');

W('#define RC_THETA_POINTS   %d\n', nTheta);
W('#define RC_MODE_COUNT     %d\n', nMode);
W('#define RC_FEATURE_COUNT  %d\n\n', nFeat);

W('/** @brief 뱅크 모드 인덱스 (기준곡선 표의 1번째 차원 순서) */\n');
W('typedef enum {\n');
for m = 1:nMode
    W('    RC_MODE_%-14s = %d,\n', BANK_MODES{m}, m-1);
end
W('} RcMode_t;\n\n');

W('/** @brief feature 인덱스 (기준곡선 표의 2번째 차원 순서) */\n');
W('typedef enum {\n');
for f = 1:nFeat
    W('    RC_FEAT_%-14s = %d,  /**< %s */\n', FEATURES{f,2}, f-1, FEATURES{f,3});
end
W('} RcFeature_t;\n\n');

W('/** @brief 디버그/로그용 모드 이름 */\n');
W('static const char *const kRcModeName[RC_MODE_COUNT] = {\n');
for m = 1:nMode
    W('    "%s",\n', BANK_MODES{m});
end
W('};\n\n');

% ---- mean 표 ----
W('/**\n');
W(' * @brief r^m(theta) 평균곡선 [mode][feature][theta]\n');
W(' * @details theta = linspace(0,1,%d), 원점은 Camargo gcRight heel-strike.\n', nTheta);
W(' *          온라인 MHE 기준 위상과는 원점이 다르므로 런타임에서 위상 offset\n');
W(' *          을 상관계수 최대화로 추정해 정렬한다.\n');
W(' */\n');
W('static const float kRcMean[RC_MODE_COUNT][RC_FEATURE_COUNT][RC_THETA_POINTS] = {\n');
for m = 1:nMode
    W('    /* ---- %s ---- */\n    {\n', BANK_MODES{m});
    for f = 1:nFeat
        W('        /* %s */\n        {\n', FEATURES{f,2});
        v = squeeze(meanTab(m,f,:));
        writeFloatRows(fid, v, 6, '            ');
        W('        },\n');
    end
    W('    },\n');
end
W('};\n\n');

% ---- std 표 ----
W('/**\n');
W(' * @brief r^m(theta) 표준편차곡선 [mode][feature][theta]\n');
W(' * @details z^m(theta) = (측정값 - kRcMean) / kRcStd 의 분모.\n');
W(' *          0 나눗셈 방지를 위해 |mean| 최대값의 1%%로 하한 클램프되어 있다.\n');
W(' *          21명(AB11 제외) pooled 이므로 피험자간 분산을 포함한다(보수적).\n');
W(' */\n');
W('static const float kRcStd[RC_MODE_COUNT][RC_FEATURE_COUNT][RC_THETA_POINTS] = {\n');
for m = 1:nMode
    W('    /* ---- %s ---- */\n    {\n', BANK_MODES{m});
    for f = 1:nFeat
        W('        /* %s */\n        {\n', FEATURES{f,2});
        v = squeeze(stdTab(m,f,:));
        writeFloatRows(fid, v, 6, '            ');
        W('        },\n');
    end
    W('    },\n');
end
W('};\n\n');

% ---- IDLE(전이/재배치) 위상무관 템플릿 ----
% analyze_idle_as_class.py 결과: STAND(idle) 곡선의 flatness(위상간분산/전체분산)는
% 1.7~7.1% 로 사실상 0 이다(보행모드는 11~59%). 즉 idle 의 위상축에는 애초에
% 정보가 없으므로, 101점 곡선 대신 스칼라 (mean,std) 한 쌍으로 표현해도
% 손실이 없다. 총분산의 법칙으로 주변분포를 구한다:
%     Var = E_theta[ std(theta)^2 ]  +  Var_theta[ mean(theta) ]
%            (위상내 분산)              (위상간=파형 분산)
idleMean = zeros(1, nFeat);
idleStd  = zeros(1, nFeat);
fprintf('\nIDLE(위상무관) 템플릿:\n');
fprintf('%-24s %14s %14s %10s\n', 'FEATURE', 'mean', 'std', 'flatness');
for f = 1:nFeat
    c  = pop.STAND.(FEATURES{f,1});
    mu = c.mean(:).';
    sd = c.std(:).';
    m       = mean(mu);
    within  = mean(sd .^ 2);
    between = mean((mu - m) .^ 2);
    idleMean(f) = m;
    idleStd(f)  = sqrt(within + between);
    fprintf('%-24s %14.6f %14.6f %9.1f%%\n', FEATURES{f,2}, idleMean(f), idleStd(f), ...
        100 * between / max(within + between, eps));
end

W('/**\n');
W(' * @brief IDLE(전이/재배치) 위상무관 템플릿 — 투표 거부권용\n');
W(' * @details Camargo ''idle'' 라벨은 "정지"가 아니라 보행로 끝에서 방향을 바꾸고\n');
W(' *          자리를 잡는 "거쳐가는 상태"다. 이 상태의 위상축 정보량(flatness =\n');
W(' *          위상간분산/전체분산)은 1.7~7.1%% 로 사실상 0 이다(보행모드는 11~59%%).\n');
W(' *          따라서 101점 곡선 대신 스칼라 (mean,std) 한 쌍이면 손실 없이 표현된다.\n');
W(' *\n');
W(' *          펌웨어는 이 평탄한 템플릿 대비 z_IDLE 을 계산한다. 보행 중이면\n');
W(' *          x(theta) 가 크게 출렁이므로 평탄 템플릿에서 멀어져 z_IDLE 이 커지고,\n');
W(' *          전이 구간이면 x(theta) 가 평탄해 z_IDLE 이 작아진다. 즉 "파형이\n');
W(' *          있는가" 자체가 판별 신호로 쓰인다.\n');
W(' *\n');
W(' * @warning 판별력이 약하다(주변분포 기준 보행모드 대비 d_subj = 0.68~1.14).\n');
W(' *          모드 argmin 경쟁자로 쓰면 정상 보행 중 오검출이 잦다. 반드시\n');
W(' *          "지형 투표 억제" 용도로만 쓸 것.\n');
W(' * 총분산의 법칙: Var = E_theta[std^2] + Var_theta[mean]\n');
W(' */\n');
W('static const float kRcIdleMean[RC_FEATURE_COUNT] = {\n');
for f = 1:nFeat
    W('    %14.6ff,   /* %s */\n', idleMean(f), FEATURES{f,2});
end
W('};\n\n');
W('static const float kRcIdleStd[RC_FEATURE_COUNT] = {\n');
for f = 1:nFeat
    W('    %14.6ff,   /* %s */\n', idleStd(f), FEATURES{f,2});
end
W('};\n\n');

W('#endif /* REF_CURVE_TABLES_H */\n');
fclose(fid);

d = dir(OUT_PATH);
fprintf('완료: %s (%.1f KB)\n', OUT_PATH, d.bytes/1024);
fprintf('테이블 메모리(런타임 .rodata): %d B (%.1f KB)\n', ...
    2*nMode*nFeat*nTheta*4, 2*nMode*nFeat*nTheta*4/1024);


%% ---- 헬퍼: float 배열을 여러 줄로 예쁘게 출력 ----
function writeFloatRows(fid, v, perRow, indent)
    n = numel(v);
    for i = 1:n
        if mod(i-1, perRow) == 0
            fprintf(fid, '%s', indent);
        end
        fprintf(fid, '%14.6ff', v(i));
        if i < n
            fprintf(fid, ',');
        end
        if mod(i, perRow) == 0 || i == n
            fprintf(fid, '\n');
        else
            fprintf(fid, ' ');
        end
    end
end
