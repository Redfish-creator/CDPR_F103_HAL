#ifndef TASK_H
#define TASK_H

#include <stdint.h>

/* 应用层任务: 在 main 里 USER CODE BEGIN 2 调一次 Task_Init(),
 * 然后 while(1) 里反复调 Task_Loop() 即可。 */
void Task_Init(void);
void Task_Loop(void);

#endif /* TASK_H */
