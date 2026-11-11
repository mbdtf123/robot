function dx = wip_dynamics(t, x, tau_tot, p, dx_cm, F_ext)
%WIP_DYNAMICS  轮式倒立摆（WIP）刚体动力学，平面、两轮等效
%
%   dx = wip_dynamics(t, x, tau_tot, p, dx_cm, F_ext)
%
%   状态 x = [位置(m); 速度(m/s); 俯仰角(rad); 俯仰角速度(rad/s)]
%            俯仰角符号与源码一致：**正 = 后倒**（main.cpp:391 注释）
%   tau_tot : 两轮总驱动力矩 (N*m)，正 = 驱动车轮向前（+x）
%   dx_cm   : 腿部姿态引起的质心水平偏移 (m)，正 = 质心前移
%   F_ext   : 作用在车身质心上的水平外力 (N)，正 = 向前推
%
%   建模要点
%     * 车身按"质点 + 绕质心惯量"处理，质心距轮轴 l
%     * 车轮计入平动质量与绕轴惯量 (J_w/R^2 折算)
%     * 忽略滚动阻力与打滑；小角度以外保留完整 sin/cos
%
%   推导（拉格朗日，q = [x, theta]）
%     M_tot = m_b + 2 m_w + 2 J_w/R^2
%     J_tot = m_b l^2 + J_b
%     [ M_tot            -m_b l cos(th) ] [xdd  ]   [ tau/R - m_b l w^2 sin(th) + F    ]
%     [ -m_b l cos(th)    J_tot         ] [thdd ] = [ tau + m_b g (l sinθ - dx) - F l cosθ ]
%
%   外力项来自 Q_j = F·∂r/∂q_j，作用点取车身质心 r = [x - l sinθ, l cosθ]：
%     Q_x = F,  Q_θ = -F·l·cosθ
%   即"向前推车身"会产生使俯仰角减小的力矩（向前倾）。
%
%   符号自检：tau=0, xdd=0 时 J_tot*thdd = m_b*g*l*sin(th)，
%   后倒(th>0)时 thdd>0 -> 越倒越快，符合倒立摆不稳定性。

if nargin < 6, F_ext = 0; end

v   = x(2);
th  = x(3);
w   = x(4);

m_b = p.mech.m_body;
l   = p.mech.l;
R   = p.mech.R;

M_tot = m_b + 2*p.mech.m_wheel + 2*p.mech.J_wheel/R^2;
J_tot = m_b*l^2 + p.mech.J_body;

A = [ M_tot,            -m_b*l*cos(th);
     -m_b*l*cos(th),     J_tot        ];

b = [ tau_tot/R - m_b*l*w^2*sin(th) + F_ext;
      tau_tot   + m_b*p.mech.g*(l*sin(th) - dx_cm) - F_ext*l*cos(th) ];

acc = A \ b;

dx = [ v;  acc(1);  w;  acc(2) ];
end
