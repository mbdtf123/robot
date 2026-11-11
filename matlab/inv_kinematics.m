function J = inv_kinematics(x_left, y_left, x_right, y_right, p)
%INV_KINEMATICS  腿部运动学逆解（lib/Km/Kms.c: inverse_kinematics 的逐行移植）
%
%   J = inv_kinematics(xL, yL, xR, yR, p)    左右独立（对称时传相同值）
%
%   输入 x,y 单位为 mm（hip 到轮轴的平面坐标，x 向前、y 向上）
%   输出 J 为结构体，角度单位 rad，字段与 Kms.c 的 JointAngles 一致
%
%   移植说明
%     * y 先限幅到 [70,180]（Kms.c:52-53）
%     * 求 sqrt 前判非负；Kms.c 中若判据失败则 *不写* 输出结构体，
%       而 main.cpp:266 `JointAngles joint_angles;` 未初始化
%       -> 实物会在该分支上用到栈上的随机值。这里改为返回 NaN + ok=false。
%     * 选择分支（Kms.c:86-94）原样保留，不做"修正"，以便复现真实行为。
%
%   注意: C 版本 alpha 用 (L1,L2)，beta 用 (L4,L5,L3)，两套连杆各自独立，
%         不是一个严格闭合的两连杆机构，这里不做几何一致性假设。

narginchk(5, 5);

[J.alpha_left,  J.beta_left,  J.ok_left ] = solve_one(x_left,  y_left,  p);
[J.alpha_right, J.beta_right, J.ok_right] = solve_one(x_right, y_right, p);
J.ok = J.ok_left && J.ok_right;

% 角度归一化到 [0,2pi]，与 Kms.c:79-83 一致
end

% ------------------------------------------------------------------------
function [alpha, beta, ok] = solve_one(x, y, p)
L1 = p.leg.L1;  L2 = p.leg.L2;  L3 = p.leg.L3;
L4 = p.leg.L4;  L5 = p.leg.L5;

% ---- 1. Y 轴限幅（Kms.c:52-53）----
y = min(max(y, 70), 180);

% ---- 2. alpha 分支（大腿 L1 + 小腿 L2）----
a = 2*x*L1;
b = 2*y*L1;
c = x*x + y*y + L1*L1 - L2*L2;

% ---- 3. beta 分支（L4 + L3，含偏置 L5）----
d = 2*L4*(x - L5);
e = 2*L4*y;
f = (x - L5)^2 + L4*L4 + y*y - L3*L3;

sa = a*a + b*b - c*c;
sb = d*d + e*e - f*f;

ok = (sa >= 0) && (sb >= 0);
if ~ok
    alpha = NaN;  beta = NaN;
    return
end

sa = sqrt(sa);   sb = sqrt(sb);

a1 = 2*atan2(b + sa, a + c);
a2 = 2*atan2(b - sa, a + c);
b1 = 2*atan2(e + sb, d + f);
b2 = 2*atan2(e - sb, d + f);

% ---- 4. 归一化到 [0, 2pi]（Kms.c:79-83）----
a1 = norm2pi(a1);  a2 = norm2pi(a2);
b1 = norm2pi(b1);  b2 = norm2pi(b2);

% ---- 5. 分支选择（Kms.c:86-94 原样保留）----
if a1 >= pi/2
    alpha = a1;
else
    alpha = a2;
end

if b1 >= 0 && b1 <= pi/2
    beta = b1;
else
    beta = b2;
end
end

% ------------------------------------------------------------------------
function v = norm2pi(v)
v = mod(v, 2*pi);
end
