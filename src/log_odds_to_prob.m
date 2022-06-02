function p = log_odds_to_prob(l)
% 将log odds l 转化为对应的概率值p

% l 可以是标量或者矩阵

% 计算 p.
p = 1 - 1 ./ (1 + exp(l));

end