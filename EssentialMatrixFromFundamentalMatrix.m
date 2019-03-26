function [E] = EssentialMatrixFromFundamentalMatrix(F, K)
% K: 3 by 3 camera intrinsic parameter
% F: 3 by 3 fundamental matrix
% E: 3 by 3 essential matrix

E = K' * F * K;
[U, S, V] = svd(E);
% essential matrix clean up
S = diag(S);
S(1:2) = (S(1)+S(2))/2;
S(3) = 0;
S = diag(S);
E = U * S * V';

end