/* ============================================================
 * motor_test.c - guarded, UART1-driven motor commissioning
 *
 * No motor moves at boot. Every movement is a 30 degree relative
 * position command at 10 RPM and must be requested from UART1.
 * ============================================================ */
#include "motor_test.h"
#include "motor.h"
#include "usart.h"
#include "vision.h"
#include <stdint.h>
#include <stdio.h>

#define MOTOR_TEST_COUNT              4U
#define MOTOR_TEST_ANGLE_0P1DEG     300U  /* 30.0 degrees */
#define MOTOR_TEST_SPEED_0P1RPM      100U  /* 10.0 RPM */
#define MOTOR_TEST_SETTLE_MS         900U
#define MOTOR_TEST_TOL_0P1DEG         30L  /* +/-3.0 degrees */

static uint8_t s_selected_motor = 1U;
static uint8_t s_sync_offset = 0U;
static uint8_t s_motor_offset[MOTOR_TEST_COUNT];
static uint8_t s_motion_lockout = 0U;

static void MotorTest_PrintMenu(void);
static void MotorTest_StopAll(const char *reason);

static uint8_t MotorTest_ReadCommand(uint8_t *command)
{
    return (HAL_UART_Receive(&huart1, command, 1U, 0U) == HAL_OK) ? 1U : 0U;
}

static uint32_t MotorTest_AbsError(int32_t actual, int32_t expected)
{
    int64_t error = (int64_t)actual - (int64_t)expected;
    return (uint32_t)((error < 0) ? -error : error);
}

static uint8_t MotorTest_AnyIndividualOffset(void)
{
    for (uint8_t i = 0U; i < MOTOR_TEST_COUNT; i++) {
        if (s_motor_offset[i] != 0U) return 1U;
    }
    return 0U;
}

static uint8_t MotorTest_EnableAll(void)
{
    uint8_t ok = 1U;

    printf("[ENABLE] enabling and holding all four motors\r\n");
    for (uint8_t id = 1U; id <= MOTOR_TEST_COUNT; id++) {
        mb_result_t result = motor_enable(id, 1U, MB_SYNC_NOW);
        printf("  M%u enable: %s\r\n", (unsigned int)id, motor_result_str(result));
        if (result != MB_OK) ok = 0U;
        HAL_Delay(20U);
    }

    if (ok == 0U) {
        MotorTest_StopAll("enable result uncertain");
    }
    return ok;
}

static uint8_t MotorTest_ReadAllPositions(int32_t position[MOTOR_TEST_COUNT])
{
    uint8_t ok = 1U;

    for (uint8_t id = 1U; id <= MOTOR_TEST_COUNT; id++) {
        mb_result_t result = motor_read_position(id, &position[id - 1U]);
        if (result == MB_OK) {
            printf("  M%u position=%ld (0.1deg)\r\n",
                   (unsigned int)id, (long)position[id - 1U]);
        } else {
            position[id - 1U] = 0;
            printf("  M%u position: %s\r\n",
                   (unsigned int)id, motor_result_str(result));
            ok = 0U;
        }
        HAL_Delay(20U);
    }
    return ok;
}

static void MotorTest_CheckCommunication(void)
{
    uint8_t ok = 1U;

    printf("\r\n[COMM CHECK] read position and status; no movement\r\n");
    for (uint8_t id = 1U; id <= MOTOR_TEST_COUNT; id++) {
        int32_t position = 0;
        uint8_t status = 0U;
        mb_result_t pos_result = motor_read_position(id, &position);
        HAL_Delay(20U);
        mb_result_t status_result = motor_read_status(id, &status);

        printf("  M%u pos=%s", (unsigned int)id, motor_result_str(pos_result));
        if (pos_result == MB_OK) printf("(%ld)", (long)position);
        printf(" status=%s", motor_result_str(status_result));
        if (status_result == MB_OK) {
            printf("(0x%02X EN=%u REACHED=%u STALL=%u)",
                   (unsigned int)status,
                   (unsigned int)((status & ST_ENABLED) != 0U),
                   (unsigned int)((status & ST_REACHED) != 0U),
                   (unsigned int)((status & ST_STALL) != 0U));
        }
        printf("\r\n");

        if (pos_result != MB_OK || status_result != MB_OK ||
            ((status_result == MB_OK) && ((status & ST_STALL) != 0U))) {
            ok = 0U;
        }
        HAL_Delay(20U);
    }

    printf(ok ? "[COMM PASS] all motor IDs 1..4 responded\r\n"
              : "[COMM FAIL] fix the reported motor/RS485 problem before moving\r\n");
}

static void MotorTest_StopAll(const char *reason)
{
    printf("\r\n[STOP] %s\r\n", reason);
    for (uint8_t id = 1U; id <= MOTOR_TEST_COUNT; id++) {
        mb_result_t result = motor_stop(id, MB_SYNC_NOW);
        printf("  M%u stop: %s\r\n", (unsigned int)id, motor_result_str(result));
        HAL_Delay(20U);
    }
    s_motion_lockout = 1U;
    printf("[LOCKOUT] position may be uncertain; make the mechanism safe, then power-cycle MCU and motor drivers\r\n");
}

static uint8_t MotorTest_DelayWithEmergencyStop(uint32_t delay_ms)
{
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < delay_ms) {
        uint8_t command;
        vision_task();
        if (MotorTest_ReadCommand(&command) != 0U &&
            (command == 'x' || command == 'X')) {
            MotorTest_StopAll("emergency command received");
            return 0U;
        }
        HAL_Delay(5U);
    }
    return 1U;
}

static uint8_t MotorTest_VerifyAll(const int32_t before[MOTOR_TEST_COUNT],
                                   uint8_t cable_action)
{
    int32_t after[MOTOR_TEST_COUNT];
    uint8_t ok = 1U;

    printf("[VERIFY] checking each encoder against its mounted winding direction\r\n");
    for (uint8_t id = 1U; id <= MOTOR_TEST_COUNT; id++) {
        uint8_t direction = motor_cable_direction(id, cable_action);
        int32_t expected = (direction == MB_DIR_CW) ?
                           (int32_t)MOTOR_TEST_ANGLE_0P1DEG :
                           -(int32_t)MOTOR_TEST_ANGLE_0P1DEG;
        mb_result_t result = motor_read_position(id, &after[id - 1U]);
        if (result != MB_OK) {
            printf("  M%u read: %s\r\n", (unsigned int)id, motor_result_str(result));
            ok = 0U;
        } else {
            int32_t delta = after[id - 1U] - before[id - 1U];
            uint8_t pass = (MotorTest_AbsError(delta, expected) <=
                            (uint32_t)MOTOR_TEST_TOL_0P1DEG) ? 1U : 0U;
            printf("  M%u dir=%s before=%ld after=%ld delta=%ld expected=%ld %s\r\n",
                   (unsigned int)id,
                   (direction == MB_DIR_CW) ? "CW" : "CCW",
                   (long)before[id - 1U],
                   (long)after[id - 1U], (long)delta, (long)expected,
                   pass ? "PASS" : "FAIL");
            if (pass == 0U) ok = 0U;
        }
        HAL_Delay(20U);
    }

    if (ok == 0U) {
        MotorTest_StopAll("synchronous position verification failed");
        return 0U;
    }
    printf("[SYNC PASS] all four motors accepted the same displacement\r\n");
    printf("            visually confirm that their start time was simultaneous\r\n");
    return 1U;
}

static uint8_t MotorTest_RunSynchronous(uint8_t cable_action)
{
    int32_t before[MOTOR_TEST_COUNT];

    if (s_motion_lockout != 0U) {
        printf("[REFUSED] emergency/uncertain-position lockout is active\r\n");
        return 0U;
    }
    if (MotorTest_EnableAll() == 0U) return 0U;

    printf("[BASELINE] reading all four motor positions\r\n");
    if (MotorTest_ReadAllPositions(before) == 0U) {
        MotorTest_StopAll("baseline read failed");
        return 0U;
    }

    printf("[SYNC BUFFER] cable=%s angle=30.0deg speed=10.0RPM\r\n",
           (cable_action == MOTOR_CABLE_TAKEUP) ? "TAKE-UP" : "PAY-OUT");
    for (uint8_t id = 1U; id <= MOTOR_TEST_COUNT; id++) {
        uint8_t direction = motor_cable_direction(id, cable_action);
        mb_result_t result = motor_move_pos(id, direction,
                                            MOTOR_TEST_SPEED_0P1RPM,
                                            MOTOR_TEST_ANGLE_0P1DEG,
                                            MB_MODE_REL_CUR,
                                            MB_SYNC_BUFFER);
        printf("  M%u dir=%s buffer: %s\r\n", (unsigned int)id,
               (direction == MB_DIR_CW) ? "CW" : "CCW",
               motor_result_str(result));
        if (result != MB_OK) {
            MotorTest_StopAll("buffered command result uncertain; trigger cancelled");
            return 0U;
        }
        HAL_Delay(20U);
    }

    printf("[SYNC TRIGGER] all four motors start now; press X for emergency stop\r\n");
    if (motor_sync_trigger() != MB_OK) {
        MotorTest_StopAll("broadcast trigger failed");
        return 0U;
    }
    if (MotorTest_DelayWithEmergencyStop(MOTOR_TEST_SETTLE_MS) == 0U) return 0U;

    return MotorTest_VerifyAll(before, cable_action);
}

static void MotorTest_SynchronousCommand(uint8_t take_up)
{
    if (MotorTest_AnyIndividualOffset() != 0U) {
        printf("[REFUSED] return the selected motor with '-' before the sync test\r\n");
        return;
    }

    if (take_up != 0U) {
        if (s_sync_offset != 0U) {
            printf("[REFUSED] synchronized take-up already done; press 'r' to return first\r\n");
            return;
        }
        printf("\r\n=== STAGE 1: FOUR-MOTOR SYNCHRONOUS TAKE-UP ===\r\n");
        printf("[MAP] M1/M3=CCW, M2/M4=CW\r\n");
        if (MotorTest_RunSynchronous(MOTOR_CABLE_TAKEUP) != 0U) {
            s_sync_offset = 1U;
            printf("[NEXT] press 'r' for the equal synchronized pay-out return\r\n");
        }
    } else {
        if (s_sync_offset == 0U) {
            printf("[REFUSED] no synchronized take-up offset to return\r\n");
            return;
        }
        printf("\r\n=== STAGE 1 RETURN: FOUR-MOTOR SYNCHRONOUS PAY-OUT ===\r\n");
        printf("[MAP] M1/M3=CW, M2/M4=CCW\r\n");
        if (MotorTest_RunSynchronous(MOTOR_CABLE_PAYOUT) != 0U) {
            s_sync_offset = 0U;
            printf("[NEXT] select motor 1..4, then use '+' and '-' to check winding direction\r\n");
        }
    }
}

static uint8_t MotorTest_VerifySingle(uint8_t id, int32_t before,
                                      uint8_t direction)
{
    int32_t after = 0;
    int32_t expected = (direction == MB_DIR_CW) ?
                       (int32_t)MOTOR_TEST_ANGLE_0P1DEG :
                       -(int32_t)MOTOR_TEST_ANGLE_0P1DEG;
    mb_result_t result = motor_read_position(id, &after);

    if (result != MB_OK) {
        printf("  M%u read after move: %s\r\n", (unsigned int)id,
               motor_result_str(result));
        MotorTest_StopAll("single-motor position read failed");
        return 0U;
    }

    int32_t delta = after - before;
    uint8_t pass = (MotorTest_AbsError(delta, expected) <=
                    (uint32_t)MOTOR_TEST_TOL_0P1DEG) ? 1U : 0U;
    printf("  M%u before=%ld after=%ld delta=%ld expected=%ld %s\r\n",
           (unsigned int)id, (long)before, (long)after, (long)delta,
           (long)expected, pass ? "PASS" : "FAIL");

    if (pass == 0U) {
        MotorTest_StopAll("single-motor position verification failed");
        return 0U;
    }
    return 1U;
}

static void MotorTest_MoveSelected(uint8_t take_up)
{
    uint8_t index = (uint8_t)(s_selected_motor - 1U);
    uint8_t cable_action = (take_up != 0U) ? MOTOR_CABLE_TAKEUP : MOTOR_CABLE_PAYOUT;
    uint8_t direction = motor_cable_direction(s_selected_motor, cable_action);
    int32_t before = 0;

    if (s_motion_lockout != 0U) {
        printf("[REFUSED] emergency/uncertain-position lockout is active\r\n");
        return;
    }
    if (s_sync_offset != 0U) {
        printf("[REFUSED] press 'r' to return the four-motor sync step first\r\n");
        return;
    }
    if (take_up != 0U && s_motor_offset[index] != 0U) {
        printf("[REFUSED] M%u already has a take-up test offset; press '-' to return\r\n",
               (unsigned int)s_selected_motor);
        return;
    }
    if (take_up == 0U && s_motor_offset[index] == 0U) {
        printf("[REFUSED] M%u has no take-up test offset; '+' must be tested first\r\n",
               (unsigned int)s_selected_motor);
        return;
    }
    if (take_up != 0U && MotorTest_AnyIndividualOffset() != 0U) {
        printf("[REFUSED] return the previously tested motor with '-' first\r\n");
        return;
    }
    if (MotorTest_EnableAll() == 0U) return;

    mb_result_t read_result = motor_read_position(s_selected_motor, &before);
    if (read_result != MB_OK) {
        printf("[M%u] baseline read: %s\r\n", (unsigned int)s_selected_motor,
               motor_result_str(read_result));
        MotorTest_StopAll("single-motor baseline read failed");
        return;
    }

    printf("\r\n=== STAGE 2: M%u %s 30.0deg (drive %s) ===\r\n",
           (unsigned int)s_selected_motor,
           (take_up != 0U) ? "TAKE-UP" : "PAY-OUT",
           (direction == MB_DIR_CW) ? "CW" : "CCW");
    printf("[OBSERVE] cable should %s\r\n",
           (take_up != 0U) ? "TAKE UP (shorten)" : "PAY OUT (lengthen)");

    mb_result_t move_result = motor_move_pos(s_selected_motor, direction,
                                              MOTOR_TEST_SPEED_0P1RPM,
                                              MOTOR_TEST_ANGLE_0P1DEG,
                                              MB_MODE_REL_CUR,
                                              MB_SYNC_NOW);
    printf("  M%u move command: %s\r\n", (unsigned int)s_selected_motor,
           motor_result_str(move_result));
    if (move_result != MB_OK) {
        MotorTest_StopAll("single-motor command result uncertain");
        return;
    }
    if (MotorTest_DelayWithEmergencyStop(MOTOR_TEST_SETTLE_MS) == 0U) return;
    if (MotorTest_VerifySingle(s_selected_motor, before, direction) == 0U) return;

    s_motor_offset[index] = (take_up != 0U) ? 1U : 0U;
    if (take_up != 0U) {
        printf("[OBSERVE] if the cable shortened, M%u direction mapping is correct\r\n",
               (unsigned int)s_selected_motor);
        printf("[RETURN] press '-' before selecting another motor\r\n");
    } else {
        printf("[RETURN PASS] M%u returned by the equal reverse step\r\n",
               (unsigned int)s_selected_motor);
    }
}

static void MotorTest_SelectMotor(uint8_t id)
{
    if (MotorTest_AnyIndividualOffset() != 0U &&
        s_motor_offset[id - 1U] == 0U) {
        printf("[REFUSED] return M%u with '-' before selecting M%u\r\n",
               (unsigned int)s_selected_motor, (unsigned int)id);
        return;
    }
    s_selected_motor = id;
    printf("[SELECT] motor M%u; '+'=take-up test, '-'=pay-out return\r\n",
           (unsigned int)s_selected_motor);
}

static void MotorTest_PrintMenu(void)
{
    printf("\r\n=== GUARDED MOTOR TEST MODE ===\r\n");
    printf("No automatic movement. Test step: 30.0deg at 10.0RPM (~2.75mm cable).\r\n");
    printf("UART3 RS485 motors: M1..M4 = addresses 1..4.\r\n");
    printf("Commands (send one ASCII character):\r\n");
    printf("  c       communication/status check (no movement)\r\n");
    printf("  s       stage 1: synchronized TAKE-UP on all four motors\r\n");
    printf("  r       synchronized PAY-OUT return after 's'\r\n");
    printf("  1..4    select one motor for stage 2 direction test\r\n");
    printf("  +       selected motor TAKE-UP step (direction is mapped per motor)\r\n");
    printf("  -       selected motor PAY-OUT return\r\n");
    printf("  x       emergency stop all and lock further movement\r\n");
    printf("  h       print this menu\r\n");
    printf("Recommended order: c -> s -> r -> 1,+,- -> 2,+,- -> 3,+,- -> 4,+,-\r\n");
    printf("Keep an emergency power cut within reach before sending a move command.\r\n\r\n");
}

void MotorTest_Init(void)
{
    s_selected_motor = 1U;
    s_sync_offset = 0U;
    s_motion_lockout = 0U;
    for (uint8_t i = 0U; i < MOTOR_TEST_COUNT; i++) s_motor_offset[i] = 0U;
    MotorTest_PrintMenu();
}

void MotorTest_Loop(void)
{
    uint8_t command;

    if (MotorTest_ReadCommand(&command) == 0U) {
        HAL_Delay(5U);
        return;
    }

    switch (command) {
    case 'c': case 'C':
        MotorTest_CheckCommunication();
        break;
    case 's': case 'S':
        MotorTest_SynchronousCommand(1U);
        break;
    case 'r': case 'R':
        MotorTest_SynchronousCommand(0U);
        break;
    case '1': case '2': case '3': case '4':
        MotorTest_SelectMotor((uint8_t)(command - '0'));
        break;
    case '+':
        MotorTest_MoveSelected(1U);
        break;
    case '-':
        MotorTest_MoveSelected(0U);
        break;
    case 'x': case 'X':
        MotorTest_StopAll("emergency command received");
        break;
    case 'h': case 'H': case '?':
        MotorTest_PrintMenu();
        break;
    case '\r': case '\n': case ' ': case '\t':
        break;
    default:
        printf("[UNKNOWN COMMAND] 0x%02X; press 'h' for help\r\n",
               (unsigned int)command);
        break;
    }
}
