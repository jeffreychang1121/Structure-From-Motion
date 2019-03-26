function [Cset, Rset] = ExtractCameraPose(E)
% E: 3 by 3 essential matrix
% Cset: camera center
% Rset: rotation matrix

Cset = cell(4,1);
Rset = cell(4,1);

Rz = [0 -1 0; 1 0 0; 0 0 1];
[U, ~, V] = svd(E);

T = U(:,3);
Rset{1} = U * Rz * V';
Cset{1} = -Rset{1}' * T;
if det(Rset{1}) < 0
    Rset{1} = -Rset{1};
    Cset{1} = -Cset{1};
end

Rset{2} = U * Rz' * V';
Cset{2} = -Rset{2}' * T;
if det(Rset{2}) < 0
    Rset{2} = -Rset{2};
    Cset{2} = -Cset{2};
end

T = -U(:,3);
U(:,3) = T; 
Rset{3} = U * Rz * V';
Cset{3} = -Rset{3}' * T;
if det(Rset{3}) < 0
    Rset{3} = -Rset{3};
    Cset{3} = -Cset{3};
end

Rset{4} = U * Rz' * V';
Cset{4} = -Rset{4}' * T;
if det(Rset{4}) < 0
    Rset{4} = -Rset{4};
    Cset{4} = -Cset{4};
end

end