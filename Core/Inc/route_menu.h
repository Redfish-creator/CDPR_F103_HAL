#ifndef __ROUTE_MENU_H
#define __ROUTE_MENU_H

#include "main.h"
#include <stdint.h>

#define ROUTE_POINT_NUM 5

typedef struct
{
    uint8_t order[ROUTE_POINT_NUM];
    uint8_t count;
    uint8_t ready;
} RouteOrder_t;

void RouteMenu_Init(void);
void RouteMenu_Task_10ms(void);

uint8_t RouteMenu_IsReady(void);
void RouteMenu_GetOrder(RouteOrder_t *dst);
void RouteMenu_Reset(void);

#endif
