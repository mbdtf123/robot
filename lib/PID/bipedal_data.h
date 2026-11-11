#ifndef _BIPEDAL_DATA_h
#define _BIPEDAL_DATA_h

// ========== 电机目标值 ==========
typedef struct {
    float motorLeftFront;
    float motorLeftRear;
    float motorRightFront;
    float motorRightRear;
    float wheelmotorLeft;
    float wheelmotorRight;
} motorstarget;

// ========== 电机参数（角度、零点） ==========
typedef struct {
    float motorLeftFrontangle;
    float motorLeftRearangle;
    float motorRightFrontangle;
    float motorRightRearangle;
    
    float motorleftsita0_origin;
    float motorleftsita1_origin;
    float motorrightsita0_origin;
    float motorrightsita1_origin;

    float motorleftsita1;
    float motorleftsita2;
    float motorrightsita1;
    float motorrightsita2;
} motorsparam;

// ========== 运动学控制参数 ==========
typedef struct {
    int l1;     // 大腿长度
    int l2;     // 小腿长度
    int robotl; // 机器人宽度
    
    float sita1, sita2;     // 关节角度
    float x_t, z_t;         // 当前坐标
    float X, Z;             // 目标坐标
    
    float KP_X, KP_Z;       // 控制增益
    float t1_max, t2_max;   // 力矩限制
} motionControlParams;

#endif