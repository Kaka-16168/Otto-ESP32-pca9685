#ifndef __OTTO_MOVEMENTS_H__
#define __OTTO_MOVEMENTS_H__

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//-- Constants
#define FORWARD 1
#define BACKWARD -1
#define LEFT 1
#define RIGHT -1
#define BOTH 0
#define SMALL 5
#define MEDIUM 15
#define BIG 30

// -- Servo delta limit default. degree / sec
#define SERVO_LIMIT_DEFAULT 240

// -- Servo indexes for easy access
#define LEFT_LEG 0
#define RIGHT_LEG 1
#define LEFT_FOOT 2
#define RIGHT_FOOT 3
#define LEFT_HAND 4
#define RIGHT_HAND 5
#define SERVO_COUNT 6

class Otto {
public:
    Otto();
    ~Otto();

    //-- Otto initialization
    void Init(int left_leg, int right_leg, int left_foot, int right_foot, int left_hand = -1,
              int right_hand = -1);
    //-- Attach & detach functions
    void AttachServos();
    void DetachServos();

    //-- Oscillator Trims
    void SetTrims(int left_leg, int right_leg, int left_foot, int right_foot, int left_hand = 0,
                  int right_hand = 0);

    //-- Predetermined Motion Functions
    void MoveServos(int time, int servo_target[]);
    void MoveSingle(int position, int servo_number);
    void OscillateServos(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period,
                         double phase_diff[SERVO_COUNT], float cycle);
    void Execute2(int amplitude[SERVO_COUNT], int center_angle[SERVO_COUNT], int period,
                  double phase_diff[SERVO_COUNT], float steps);

    //-- HOME = Otto at rest position
    void Home(bool hands_down = true);
    bool GetRestState();
    void SetRestState(bool state);

    //-- Predetermined Motion Functions
    void Jump(float steps = 1, int period = 2000);

    void Walk(float steps = 4, int period = 1000, int dir = FORWARD, int amount = 0);
    void Turn(float steps = 4, int period = 2000, int dir = LEFT, int amount = 0);
    void Bend(int steps = 1, int period = 1400, int dir = LEFT);
    void ShakeLeg(int steps = 1, int period = 2000, int dir = RIGHT);
    void Sit();  // 坐下

    void UpDown(float steps = 1, int period = 1000, int height = 20);
    void Swing(float steps = 1, int period = 1000, int height = 20);
    void TiptoeSwing(float steps = 1, int period = 900, int height = 20);
    void Jitter(float steps = 1, int period = 500, int height = 20);
    void AscendingTurn(float steps = 1, int period = 900, int height = 20);

    void Moonwalker(float steps = 1, int period = 900, int height = 20, int dir = LEFT);
    void Crusaito(float steps = 1, int period = 900, int height = 20, int dir = FORWARD);
    void Flapping(float steps = 1, int period = 1000, int height = 20, int dir = FORWARD);
    void WhirlwindLeg(float steps = 1, int period = 300, int amplitude = 30);

    // -- 手部动作
    void HandsUp(int period = 1000, int dir = 0);      // 双手举起
    void HandsDown(int period = 1000, int dir = 0);    // 双手放下
    void HandWave(int dir = LEFT);  // 挥手
    void Windmill(float steps = 10, int period = 500, int amplitude = 90);  // 大风车
    void Takeoff(float steps = 5, int period = 300, int amplitude = 40);   // 起飞
    void Fitness(float steps = 5, int period = 1000, int amplitude = 25);  // 健身
    void Greeting(int dir = LEFT, float steps = 5);  // 打招呼
    void Shy(int dir = LEFT, float steps = 5);  // 害羞
    void RadioCalisthenics();  // 广播体操
    void MagicCircle();  // 爱的魔力转圈圈
    void Showcase();  // 展示动作（串联多个动作）

    // -- Servo limiter
    void EnableServoLimit(int speed_limit_degree_per_sec = SERVO_LIMIT_DEFAULT);
    void DisableServoLimit();

    // 供内部调用的动作执行函数
    void Execute(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period,
                 double phase_diff[SERVO_COUNT], float steps);

private:
    // 删除了原来的 Oscillator servo_[SERVO_COUNT]; 替换为记录当前角度
    int current_positions_[SERVO_COUNT];
    
    int servo_pins_[SERVO_COUNT]; // 注意：在 PCA9685 模式下，这里存的是通道号(0-15)
    int servo_trim_[SERVO_COUNT];

    bool is_otto_resting_;
    bool has_hands_;  // 是否有手部舵机
};

#endif  // __OTTO_MOVEMENTS_H__
