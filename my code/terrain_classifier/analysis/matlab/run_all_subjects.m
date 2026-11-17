%% run_all_subjects.m  AB06/AB07/AB08 3명에 대해 12개 전환 특징 추출 + 피험자간 일관성 비교
clear; clc;
addpath('C:\Users\user\Desktop\onephai_ifac\Camargo_matlab');

subjects = struct();
subjects.AB06.root = 'C:\Users\user\Desktop\onephai_ifac\AB06\10_09_18';
subjects.AB07.root = 'D:\gait_data\AB07\10_14_18';
subjects.AB08.root = 'D:\gait_data\AB08\10_21_2018';

names = fieldnames(subjects);
allResults = struct();
for i = 1:numel(names)
    sname = names{i};
    fprintf('\n\n########## Subject %s ##########\n', sname);
    allResults.(sname) = extract_transition_features(subjects.(sname).root, true);
end

save('all_subjects_transition_result.mat', 'allResults');

%% ---- 피험자간 top-1 feature 일관성 비교 ----
tnames = fieldnames(allResults.(names{1}));
fprintf('\n\n================ 피험자간 Top-1 판별 feature 일관성 ================\n');
for t = 1:numel(tnames)
    tname = tnames{t};
    fprintf('\n--- %s ---\n', tname);
    top1feats = cell(numel(names),1);
    for i = 1:numel(names)
        sname = names{i};
        r = allResults.(sname).(tname);
        k1 = r.order(1);
        top1feats{i} = r.keys{k1};
        fprintf('  %s: %-28s d=%6.2f  (pre=%.4f post=%.4f, n=%d)\n', ...
            sname, r.keys{k1}, r.d(k1), r.meanPre(k1), r.meanPost(k1), r.n);
    end
    if numel(unique(top1feats)) == 1
        fprintf('  => 3명 모두 동일 feature가 1위 (subject-independent 가능성 높음)\n');
    else
        fprintf('  => 피험자마다 1위 feature가 다름 (subject-specific, 개인 캘리브레이션 필요)\n');
    end
end
