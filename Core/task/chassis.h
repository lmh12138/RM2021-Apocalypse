#ifndef CHASSIS_H
#define CHASSIS_H

void ChassisParamCalculate(void);
void ChassisAccelerationLimit(void);
extern void ChassisParamChange(void);

void Set_Chassis_param(float fb,float lr,float turn,float rot);
extern void ChassisCtrl_Remote(void);
extern void ChassisCtrl_MouseKey(void);
void chassis_distance_calc_task(void);


#endif
