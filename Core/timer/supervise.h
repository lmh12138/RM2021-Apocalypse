#ifndef SUPERVISE_H_
#define SUPERVISE_H_

#define DETECT_NUM 13

#define RC_INDEX 0
#define GYRO_INDEX 1
#define CAN_MOTOR1_INDEX 2
#define CAN_MOTOR2_INDEX 3
#define CAN_MOTOR3_INDEX 4
#define CAN_MOTOR4_INDEX 5
#define CAN_MOTOR5_INDEX 6
#define CAN_MOTOR6_INDEX 7
#define CAN_MOTOR7_INDEX 8
#define CAN_MOTOR9_INDEX 9
#define JUDGEMENT_INDEX 10
#define PC_INDEX        11
#define SUPER_CAP_INDEX 12

int Is_Error(int index);
int Is_Serious_Error(void);
int Is_Any_Error(void);

extern void SuperviseTaskHandle(void);
extern void LostCounterFeed(int index);

#endif
