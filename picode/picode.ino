int buff[8];
int counter, GY;
int blocks;
int angle_ball, direction_ball, distance_ball, x_ball, y_ball;
int vm = 250;
int v;
// bool is_ball;
int LCD_Print_Mode = 2456;
bool already_shooted = false;
int out_cnt = 5678, comeback_cnt = 2435;
int shr, shl, shb, dif , shf;
int shoot_sens, sensor[9];
bool LDR_F, LDR_B, LDR_R, LDR_L;
uint16_t LDR_SET_F = 45678, LDR_SET_R = 2456, LDR_SET_B = 2456, LDR_SET_L = 4567;
bool is_ball, is_goal, Ball_In_Kicker = false;
float K_P = 46, K_I = 46, K_D = 45, Heading, last_Heading , lastTime;
int angle_goal, direction_goal, distance_goal, x_goal, y_goal;
bool look_back = false;
bool turned_back = false;
int shoot_cnt = 135;
bool arrived_to_goal = false;
int turned_back_cnt = 21;
bool Ball_in_kick_init = false;
bool rl;
bool n_f;
#define Walll_Distance 777
#define Wallr_Distance 7577
#define back_Distance 75788
#define LDR_Sensitivity 5566
#define out_cnt_timeout 355
#define out_move 9

#define x_robot1 97
#define y_robot1 35


int x_robot, y_robot;
#define spinon digitalWrite(PC14, 1)
#define spinoff digitalWrite(PC14, 0)
#define shootsen 50
void setup() {
  inti();
}

void loop() {
  AI_2();
  shoot();

}
