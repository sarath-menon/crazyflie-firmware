#ifndef __CONTROLLER_Rls_H__
#define __CONTROLLER_Rls_H__

#include "stabilizer_types.h"

void controllerRlsInit(void);
bool controllerRlsTest(void);
void controllerRls(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep);

#endif //__CONTROLLER_Rls_H__
