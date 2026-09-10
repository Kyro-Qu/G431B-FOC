/**
 * @file test_foc_math.c
 * @brief FOC 核心算法与数学变换 PC 本地单元测试 (Clarke/Park/SVPWM/PID/Traj)
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

#include "../MDK-ARM/Code/foc/Core/foc_types.h"
#include "../MDK-ARM/Code/foc/Core/foc_utils.h"
#include "../MDK-ARM/Code/foc/Core/foc_transform.h"
#include "../MDK-ARM/Code/foc/Core/foc_svm.h"
#include "../MDK-ARM/Code/foc/Core/foc_pid.h"
#include "../MDK-ARM/Code/foc/Core/foc_traj.h"

#define EPSILON 1e-4f

static int s_tests_passed = 0;
static int s_tests_failed = 0;

#define EXPECT_NEAR(actual, expected, tol, name) do { \
    float diff = fabsf((actual) - (expected)); \
    if (diff > (tol)) { \
        printf("  [FAIL] %s: actual=%.6f, expected=%.6f, diff=%.6f (tol=%.6f)\n", \
               (name), (double)(actual), (double)(expected), (double)diff, (double)(tol)); \
        s_tests_failed++; \
    } else { \
        s_tests_passed++; \
    } \
} while (0)

#define EXPECT_TRUE(cond, name) do { \
    if (!(cond)) { \
        printf("  [FAIL] %s\n", (name)); \
        s_tests_failed++; \
    } else { \
        s_tests_passed++; \
    } \
} while (0)

/* 1. Clarke 与 反 Clarke 变换测试 */
static void test_clarke_transforms(void)
{
    printf("--> 运行 Clarke / Inv-Clarke 正反变换测试...\n");

    /* 测试对称三相平衡量 */
    abc_t i_abc = { 1.0f, -0.5f, -0.5f };
    ab_t i_ab;
    foc_clarke(&i_abc, &i_ab);
    EXPECT_NEAR(i_ab.alpha, 1.0f, EPSILON, "Clarke alpha (balanced)");
    EXPECT_NEAR(i_ab.beta, 0.0f, EPSILON, "Clarke beta (balanced)");

    abc_t i_rec;
    foc_inv_clarke(&i_ab, &i_rec);
    EXPECT_NEAR(i_rec.a, i_abc.a, EPSILON, "Inv-Clarke phase A");
    EXPECT_NEAR(i_rec.b, i_abc.b, EPSILON, "Inv-Clarke phase B");
    EXPECT_NEAR(i_rec.c, i_abc.c, EPSILON, "Inv-Clarke phase C");

    /* 测试非零 Beta 分量 */
    float sqrt3_div_2 = 0.8660254f;
    abc_t i_abc2 = { 0.0f, sqrt3_div_2, -sqrt3_div_2 };
    foc_clarke(&i_abc2, &i_ab);
    EXPECT_NEAR(i_ab.alpha, 0.0f, EPSILON, "Clarke alpha (pure beta)");
    EXPECT_NEAR(i_ab.beta, 1.0f, EPSILON, "Clarke beta (pure beta)");

    foc_inv_clarke(&i_ab, &i_rec);
    EXPECT_NEAR(i_rec.a, i_abc2.a, EPSILON, "Inv-Clarke 2 phase A");
    EXPECT_NEAR(i_rec.b, i_abc2.b, EPSILON, "Inv-Clarke 2 phase B");
    EXPECT_NEAR(i_rec.c, i_abc2.c, EPSILON, "Inv-Clarke 2 phase C");
}

/* 2. Park 与 反 Park 变换测试 */
static void test_park_transforms(void)
{
    printf("--> 运行 Park / Inv-Park 旋转坐标变换测试...\n");

    ab_t ab = { 1.0f, 0.0f };
    dq_t dq;
    float theta = 0.5235987f; /* 30度 = pi/6 */
    float sin_th = sinf(theta);
    float cos_th = cosf(theta);

    foc_park(&ab, sin_th, cos_th, &dq);
    EXPECT_NEAR(dq.d, cos_th, EPSILON, "Park d-axis (theta=30deg)");
    EXPECT_NEAR(dq.q, -sin_th, EPSILON, "Park q-axis (theta=30deg)");

    ab_t ab_rec;
    foc_inv_park(&dq, sin_th, cos_th, &ab_rec);
    EXPECT_NEAR(ab_rec.alpha, ab.alpha, EPSILON, "Inv-Park alpha");
    EXPECT_NEAR(ab_rec.beta, ab.beta, EPSILON, "Inv-Park beta");
}

/* 3. SVPWM 空间矢量调制测试 */
static void test_svpwm(void)
{
    printf("--> 运行 SVPWM 零序中点注入与占空比测试...\n");

    float u_dc = 14.4f;
    foc_svm_t svm;
    ab_t v_zero = { 0.0f, 0.0f };
    foc_svm_calc(&v_zero, u_dc, &svm);

    /* 零电压输入应为 50% 占空比 */
    EXPECT_NEAR(svm.duty_a, 0.5f, EPSILON, "SVPWM Zero Vector Duty A");
    EXPECT_NEAR(svm.duty_b, 0.5f, EPSILON, "SVPWM Zero Vector Duty B");
    EXPECT_NEAR(svm.duty_c, 0.5f, EPSILON, "SVPWM Zero Vector Duty C");

    /* 测试扇区 1 矢量: Alpha > 0, Beta = 0 */
    ab_t v_s1 = { 4.0f, 0.0f };
    foc_svm_calc(&v_s1, u_dc, &svm);
    EXPECT_TRUE(svm.duty_a >= 0.0f && svm.duty_a <= 1.0f, "SVPWM Duty A bounds");
    EXPECT_TRUE(svm.duty_b >= 0.0f && svm.duty_b <= 1.0f, "SVPWM Duty B bounds");
    EXPECT_TRUE(svm.duty_c >= 0.0f && svm.duty_c <= 1.0f, "SVPWM Duty C bounds");
    EXPECT_TRUE(svm.duty_a > svm.duty_b && svm.duty_a > svm.duty_c, "SVPWM Phase A highest in Sector 1/6");

    /* 测试过调制硬限幅保护 */
    ab_t v_over = { 20.0f, 20.0f };
    foc_svm_calc(&v_over, u_dc, &svm);
    EXPECT_TRUE(svm.duty_a >= 0.0f && svm.duty_a <= 1.0f, "Overmodulation Duty A in [0,1]");
    EXPECT_TRUE(svm.duty_b >= 0.0f && svm.duty_b <= 1.0f, "Overmodulation Duty B in [0,1]");
    EXPECT_TRUE(svm.duty_c >= 0.0f && svm.duty_c <= 1.0f, "Overmodulation Duty C in [0,1]");
}

/* 4. PID 控制器与抗积分饱和测试 */
static void test_pid_controller(void)
{
    printf("--> 运行 PID 控制器与 Tustin 离散化抗饱和测试...\n");

    foc_pid_t pid;
    foc_pid_init(&pid, 2.0f, 100.0f, 0.0f, 10.0f, 0.0f);

    float dt = 0.001f; /* 1ms */
    /* 阶跃误差 2.0 */
    float out1 = foc_pid_update(&pid, 2.0f, dt);
    /* P项: 2.0 * 2.0 = 4.0; I项(Tustin): 100 * 0.001 * 0.5 * (2.0 + 0) = 0.1; 总和 4.1 */
    EXPECT_NEAR(out1, 4.1f, 1e-3f, "PID Step Step 1 Output");

    /* 持续大误差，验证抗饱和 Clamping 限幅 */
    for (int i = 0; i < 200; i++) {
        foc_pid_update(&pid, 10.0f, dt);
    }
    float saturated_out = foc_pid_update(&pid, 10.0f, dt);
    EXPECT_NEAR(saturated_out, 10.0f, EPSILON, "PID Saturated Clamping Limit");
    EXPECT_TRUE(pid.integral <= 10.0f, "PID Integral Clamped");

    /* 反向阶跃，积分能够立即退饱和退出，不出现 windup 延迟 */
    float out_recovery = foc_pid_update(&pid, -5.0f, dt);
    EXPECT_TRUE(out_recovery < 10.0f, "PID Instant Anti-windup Recovery");
}

/* 5. 梯形轨迹规划测试 (Trap Trajectory) */
static void test_trajectory_planner(void)
{
    printf("--> 运行梯形速度轨迹规划器 (Trap Trajectory) 测试...\n");

    foc_traj_t traj;
    float xi = 0.0f;
    float xf = 25.0f; /* 移动 25 rad: 加速5 rad, 减速5 rad, 巡航15 rad (3秒) */
    float vi = 0.0f;
    float v_max = 5.0f;
    float a_max = 2.5f;

    foc_traj_plan(&traj, xf, xi, vi, v_max, a_max, a_max);

    /* 验证轨迹总时间合理 */
    float total_t = traj.t_total;
    EXPECT_TRUE(total_t > 0.0f, "Traj Total Time > 0");
    EXPECT_TRUE(traj.t_vel > 1.0f, "Traj Cruise Time > 1s");

    /* 验证起点状态 */
    float p_eval = 0.0f, v_eval = 0.0f;
    foc_traj_eval(&traj, 0.0f, &p_eval, &v_eval);
    EXPECT_NEAR(p_eval, 0.0f, EPSILON, "Traj Start Position");
    EXPECT_NEAR(v_eval, 0.0f, EPSILON, "Traj Start Velocity");

    /* 步进推进到巡航段中间时刻 */
    float dt_step = 0.01f;
    float target_time = traj.t_acc + (traj.t_vel * 0.5f);
    while (traj.t < target_time && traj.active) {
        foc_traj_eval(&traj, dt_step, &p_eval, &v_eval);
    }
    EXPECT_NEAR(v_eval, v_max, 1e-1f, "Traj Cruise Velocity");

    /* 推进直到轨迹结束 */
    while (traj.active) {
        foc_traj_eval(&traj, dt_step, &p_eval, &v_eval);
    }
    EXPECT_NEAR(p_eval, xf, 1e-3f, "Traj Final Position Exact Match");
    EXPECT_NEAR(v_eval, 0.0f, EPSILON, "Traj Final Velocity Zero");
}

int main(void)
{
    printf("========================================================\n");
    printf("       FOC Core 数学算法 PC 单元测试套件 (Unit Tests)\n");
    printf("========================================================\n");

    test_clarke_transforms();
    test_park_transforms();
    test_svpwm();
    test_pid_controller();
    test_trajectory_planner();

    printf("========================================================\n");
    printf("测试统计: 通过 %d 项, 失败 %d 项\n", s_tests_passed, s_tests_failed);
    if (s_tests_failed == 0) {
        printf(">>>>> 所有的 PC 单元测试用例全部圆满通过 (100%% PASS)！\n");
        return 0;
    } else {
        printf(">>>>> 存在单元测试失败项，请检查！\n");
        return 1;
    }
}
