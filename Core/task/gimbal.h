#ifndef GIMBAL_H
#define GIMBAL_H

#define YAW 0
#define PITCH 1

#define NOW  0
#define LAST 1

void GimbalParamCalculate(void);
extern void GimbalParamChange(void);

extern void GimbalCtrl_Remote(void);
extern void GimbalCtrl_MouseKey(void);

extern void GIMBAL_InitArgument(void);


#endif
