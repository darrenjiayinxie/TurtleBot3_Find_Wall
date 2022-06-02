function l=prob_to_log_odds(p)
% 将概率值p转化为对应的log odds l

% p 可以是标量或者矩阵

% 计算 l.
l = log(p ./ (1 - p));

end