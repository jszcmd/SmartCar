#include "encoder.h"

float encoderL_oringinal = 0.0f; // 编码器原始值
float encoderR_oringinal = 0.0f;
uint16_t encoderL_100 = 0, encoderR_100 = 0; // 编码器值乘以100后的整数部分
uint16_t encoderL_last = 0;                  // 上次编码器值
uint16_t encoderR_last = 0;
uint16_t encoderL = 0; // 低通滤波后的编码器最终值
uint16_t encoderR = 0;
float encoder_L_final = 0.0f; // 最终编码器值
float encoder_R_final = 0.0f;

// 文件作用域的编码器实例指针，供初始化和循环读取使用
static ls_encoder_pwm *enc1 = nullptr;
static ls_encoder_pwm *enc4 = nullptr;

void encoder_Init(void)
{
    // 在 init 中分配实例，保证生命周期在整个程序运行期间有效
    if (!enc4)
        enc4 = new ls_encoder_pwm(ENC_PWM0_PIN64, PIN_72);
    // ls_encoder_pwm *enc2 = new ls_encoder_pwm(ENC_PWM1_PIN65, PIN_73);
    // ls_encoder_pwm *enc3 = new ls_encoder_pwm(ENC_PWM2_PIN66, PIN_74);
    if (!enc1)
        enc1 = new ls_encoder_pwm(ENC_PWM3_PIN67, PIN_75);
    // 编码器选用的 1 和 4 号
    printf("Encoder initialized\n");
}

void encoder_update(void) // 编码器循环获取数据，放中断里执行
{
    if (ls_system_running.load()) // 如果系统正在运行，继续获取编码器数据
    {
        if (!enc1 || !enc4)
            return; // 若未初始化则直接返回

        encoderL_oringinal = enc1->encoder_get_count();
        encoderR_oringinal = enc4->encoder_get_count(); // 获取编码器原始值

        if (encoderL_oringinal < 0.0f)
            encoderL_oringinal = -encoderL_oringinal;
        if (encoderR_oringinal < 0.0f)
            encoderR_oringinal = -encoderR_oringinal;
        encoderL_100 = (uint16_t)(encoderL_oringinal * 100);
        encoderR_100 = (uint16_t)(encoderR_oringinal * 100);
        // 乘以100，转换为整数运算

        encoderL = encoderL_100 / 10 + encoderL_last / 10 * 9;
        encoderR = encoderR_100 / 10 + encoderR_last / 10 * 9;
        encoderL_last = encoderL;
        encoderR_last = encoderR;
        // 低通滤波，当前值占10%，上次值占90%

        encoder_L_final = encoderL / 100.0f; // 转换回浮点数
        encoder_R_final = encoderR / 100.0f;
    }
} // 编码器数据更新函数
