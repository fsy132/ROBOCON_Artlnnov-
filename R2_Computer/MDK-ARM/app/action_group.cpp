#include "action_group.h"

#include "app_config.h"
#include "app_shared.h"
#include "distance_control.h"
#include "main.h"
#include "protocol.h"
#include "usart.h"

#define BLOCK_GPIO_SET(pins)     (GPIOI->BSRR = (uint32_t)(pins))
#define BLOCK_GPIO_RESET(pins)   (GPIOI->BSRR = ((uint32_t)(pins) << 16U))

static uint8_t action_step_elapsed(uint32_t ms)
{
    return (HAL_GetTick() - action_step_start_tick >= ms);
}

static void action_next_step(uint8_t step)
{
    action_step = step;
    action_step_start_tick = HAL_GetTick();
}

static void action_done_after(uint32_t ms)
{
    if (action_step_elapsed(ms))
    {
        action_status = ACTION_STATUS_DONE;
    }
}

static void lift3_set_height_cmd(uint8_t cmd)
{
    lift3_cmd = cmd;
}

static void block_gpio_reset(void)
{
    BLOCK_GPIO_RESET(GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
}

static uint8_t action_id_is_valid(uint8_t id)
{
    return ((id >= ACTION_ID_CLIMB_UP_LOW && id <= ACTION_ID_CLIMB_UP_HIGH_GRAB) ||
            id == ACTION_ID_CONFRONT_BLOCK_PLACE);
}

static uint8_t action_is_block(uint8_t id)
{
    return (id >= ACTION_ID_BLOCK_LOW_TO_HIGH_PICK && id <= ACTION_ID_BLOCK_TAKE_OUT_STORAGE);
}

static uint8_t action_is_climb(uint8_t id)
{
    return (id <= ACTION_ID_CLIMB_DOWN_HIGH ||
            id == ACTION_ID_CLIMB_UP_LOW_FINISH ||
            id == ACTION_ID_CLIMB_UP_LOW_GRAB ||
            id == ACTION_ID_CLIMB_UP_HIGH_GRAB);
}

void action_group_start(uint8_t id)
{
    if (!action_id_is_valid(id))
    {
        action_status = ACTION_STATUS_FAIL;
        chassis_drive_pid_restore_requested();
        return;
    }

    if (action_is_climb(id) && (!lift1_zero_inited || !lift2_zero_inited))
    {
        action_status = ACTION_STATUS_FAIL;
        action_id = ACTION_ID_NONE;
        action_step = ACTION_STEP_FAIL;
        chassis_drive_pid_restore_requested();
        return;
    }

    action_id = id;
    action_status = ACTION_STATUS_RUNNING;
    action_next_step(ACTION_STEP_CHASSIS_1);

    chassis_distance_enable = 0;
    side_distance_enable = 0;
    side_wheel_enable = 0;
    chassis_distance_failed = 0;
    side_distance_failed = 0;

    chassis_drive_pid_use_climb(action_is_climb(id));

    if (action_is_block(id))
    {
        action_next_step(ACTION_STEP_BLOCK_LIFT_1);
    }
}

void action_group_stop(void)
{
    action_id = ACTION_ID_NONE;
    action_step = ACTION_STEP_IDLE;
    action_status = ACTION_STATUS_IDLE;

    chassis_drive_pid_restore_requested();

    chassis_distance_enable = 0;
    side_distance_enable = 0;
    side_wheel_enable = 0;
    chassis_distance_failed = 0;
    side_distance_failed = 0;

    chassis_cmd_vx_mm_s = 0.0f;
    chassis_cmd_vy_mm_s = 0.0f;
    chassis_cmd_w_mdeg_s = 0.0f;

    lift1_cmd = 0;
    lift2_cmd = 0;
    lift3_cmd = 0;
    can2_2006_target_angle_12 = 0.0f;
    block_gpio_reset();
}

static void action_fail(void)
{
    action_status = ACTION_STATUS_FAIL;
    chassis_drive_pid_restore_requested();
    chassis_distance_enable = 0;
    side_distance_enable = 0;
    side_wheel_enable = 0;
}

static void climb_action_task(void)
{
    switch (action_id)
    {
        case ACTION_ID_CLIMB_UP_LOW:
        case ACTION_ID_CLIMB_UP_HIGH:
        case ACTION_ID_CLIMB_UP_LOW_GRAB:
        case ACTION_ID_CLIMB_UP_HIGH_GRAB:
        {
            uint8_t lift_cmd = (action_id & 1U) ? 1U : 2U;
            uint8_t do_grab = (action_id >= ACTION_ID_CLIMB_UP_LOW_GRAB);

            switch (action_step)
            {
                case ACTION_STEP_CHASSIS_1:
                    chassis_distance_start(MTF01_MIDDLE_INDEX, CLIMB_UP_FRONT_CHASSIS_MM);
                    action_next_step(ACTION_STEP_LIFT_1);
                    break;

                case ACTION_STEP_LIFT_1:
                    if (!chassis_distance_enable)
                    {
                        lift1_cmd = lift_cmd;
                        lift2_cmd = lift_cmd;
                        if (do_grab)
                        {
                            lift3_set_height_cmd(LIFT3_CMD_10CM);
                        }
                        action_next_step(ACTION_STEP_SIDE);
                    }
                    break;

                case ACTION_STEP_SIDE:
                    if (action_step_elapsed(ACTION_LIFT_WAIT_MS))
                    {
                        if (do_grab)
                        {
                            BLOCK_GPIO_SET(GPIO_PIN_6 | GPIO_PIN_7);
                            chassis_distance_start(MTF01_MIDDLE_INDEX, CLIMB_UP_FRONT_COMBINED_MM);
                            side_distance_start(MTF01_MIDDLE_INDEX, CLIMB_UP_FRONT_COMBINED_MM);
                            action_next_step(ACTION_STEP_LIFT_2);
                        }
                        else
                        {
                            chassis_distance_start(MTF01_MIDDLE_INDEX, CLIMB_UP_FRONT_COMBINED_MM);
                            side_distance_start(MTF01_MIDDLE_INDEX, CLIMB_UP_FRONT_COMBINED_MM);
                            action_next_step(ACTION_STEP_LIFT_2);
                        }
                    }
                    break;

                case ACTION_STEP_LIFT_2:
                    if (!chassis_distance_enable && !side_distance_enable)
                    {
                        lift1_cmd = 0;
                        action_next_step(do_grab ? ACTION_STEP_BLOCK_WAIT_1 : ACTION_STEP_DONE);
                    }
                    break;

                case ACTION_STEP_BLOCK_WAIT_1:
                    if (action_step_elapsed(ACTION_GPIO_WAIT_MS))
                    {
                        lift3_set_height_cmd(LIFT3_CMD_40CM);
                        action_next_step(ACTION_STEP_DONE);
                    }
                    break;

                case ACTION_STEP_DONE:
                    action_done_after(ACTION_LIFT_WAIT_MS);
                    break;

                default:
                    action_fail();
                    break;
            }
            break;
        }

        case ACTION_ID_CLIMB_UP_LOW_FINISH:
        {
            switch (action_step)
            {
                case ACTION_STEP_CHASSIS_1:
                    chassis_distance_start(MTF01_FRONT_INDEX, CLIMB_CHASSIS_UP_MM);
                    side_distance_start(MTF01_FRONT_INDEX, CLIMB_SIDE_UP_MM);
                    action_next_step(ACTION_STEP_LIFT_3);
                    break;

                case ACTION_STEP_LIFT_3:
                    if (!chassis_distance_enable && !side_distance_enable)
                    {
                        lift2_cmd = 0;
                        action_next_step(ACTION_STEP_CHASSIS_2);
                    }
                    break;

                case ACTION_STEP_CHASSIS_2:
                    if (action_step_elapsed(ACTION_LIFT_WAIT_MS))
                    {
                        chassis_distance_start(MTF01_FRONT_INDEX, CLIMB_UP_FINISH_EXTRA_CHASSIS_MM);
                        action_next_step(ACTION_STEP_DONE);
                    }
                    break;

                case ACTION_STEP_DONE:
                    if (!chassis_distance_enable)
                    {
                        action_status = ACTION_STATUS_DONE;
                    }
                    break;

                default:
                    action_fail();
                    break;
            }
            break;
        }

        case ACTION_ID_CLIMB_DOWN_LOW:
        case ACTION_ID_CLIMB_DOWN_HIGH:
        {
            uint8_t lift_cmd = (action_id == ACTION_ID_CLIMB_DOWN_LOW) ? 1 : 2;

            switch (action_step)
            {
                case ACTION_STEP_CHASSIS_1:
                    chassis_distance_start(MTF01_MIDDLE_INDEX, CLIMB_DOWN_CHASSIS_MM);
                    action_next_step(ACTION_STEP_LIFT_1);
                    break;

                case ACTION_STEP_LIFT_1:
                    if (!chassis_distance_enable)
                    {
                        lift2_cmd = lift_cmd;
                        action_next_step(ACTION_STEP_CHASSIS_2);
                    }
                    break;

                case ACTION_STEP_CHASSIS_2:
                    if (action_step_elapsed(ACTION_LIFT_WAIT_MS))
                    {
                        chassis_distance_start(MTF01_FRONT_INDEX, CLIMB_DOWN_CHASSIS_MM);
                        action_next_step(ACTION_STEP_LIFT_2);
                    }
                    break;

                case ACTION_STEP_LIFT_2:
                    if (!chassis_distance_enable)
                    {
                        lift1_cmd = lift_cmd;
                        action_next_step(ACTION_STEP_SIDE);
                    }
                    break;

                case ACTION_STEP_SIDE:
                    if (action_step_elapsed(ACTION_LIFT_WAIT_MS))
                    {
                        side_distance_start(MTF01_MIDDLE_INDEX, CLIMB_DOWN_SIDE_MM);
                        action_next_step(ACTION_STEP_LIFT_3);
                    }
                    break;

                case ACTION_STEP_LIFT_3:
                    if (!side_distance_enable)
                    {
                        lift1_cmd = 0;
                        lift2_cmd = 0;
                        action_next_step(ACTION_STEP_DONE);
                    }
                    break;

                case ACTION_STEP_DONE:
                    action_done_after(ACTION_LIFT_WAIT_MS);
                    break;

                default:
                    action_fail();
                    break;
            }
            break;
        }

        default:
            action_fail();
            break;
    }
}

static void block_pick_task(uint8_t lift_cmd)
{
    switch (action_step)
    {
        case ACTION_STEP_BLOCK_LIFT_1:
            lift3_set_height_cmd(lift_cmd);
            action_next_step(ACTION_STEP_BLOCK_GPIO_1);
            break;

        case ACTION_STEP_BLOCK_GPIO_1:
            if (action_step_elapsed(ACTION_LIFT_WAIT_MS))
            {
                BLOCK_GPIO_SET(GPIO_PIN_6);
                action_next_step(ACTION_STEP_BLOCK_WAIT_1);
            }
            break;

        case ACTION_STEP_BLOCK_WAIT_1:
            if (action_step_elapsed(ACTION_GPIO_WAIT_MS))
            {
                BLOCK_GPIO_SET(GPIO_PIN_7);
                action_next_step(ACTION_STEP_DONE);
            }
            break;

        case ACTION_STEP_DONE:
            action_status = ACTION_STATUS_DONE;
            break;

        default:
            action_fail();
            break;
    }
}

static void block_suck_task(void)
{
    switch (action_step)
    {
        case ACTION_STEP_BLOCK_LIFT_1:
            lift3_set_height_cmd(LIFT3_CMD_10CM);
            action_next_step(ACTION_STEP_BLOCK_GPIO_1);
            break;

        case ACTION_STEP_BLOCK_GPIO_1:
            if (action_step_elapsed(ACTION_LIFT_WAIT_MS))
            {
                BLOCK_GPIO_SET(GPIO_PIN_5);
                action_next_step(ACTION_STEP_DONE);
            }
            break;

        case ACTION_STEP_DONE:
            action_status = ACTION_STATUS_DONE;
            break;

        default:
            action_fail();
            break;
    }
}

static void block_store_task(void)
{
    switch (action_step)
    {
        case ACTION_STEP_BLOCK_LIFT_1:
            lift3_set_height_cmd(LIFT3_CMD_40CM);
            action_next_step(ACTION_STEP_BLOCK_MOTOR_1);
            break;

        case ACTION_STEP_BLOCK_MOTOR_1:
            if (action_step_elapsed(ACTION_LIFT_WAIT_MS))
            {
                can2_2006_target_angle_12 = BLOCK_MOTOR12_CCW_270_ANGLE;
                action_next_step(ACTION_STEP_BLOCK_GPIO_1);
            }
            break;

        case ACTION_STEP_BLOCK_GPIO_1:
            if (action_step_elapsed(ACTION_MOTOR_WAIT_MS))
            {
                BLOCK_GPIO_RESET(GPIO_PIN_7);
                action_next_step(ACTION_STEP_BLOCK_WAIT_1);
            }
            break;

        case ACTION_STEP_BLOCK_WAIT_1:
            if (action_step_elapsed(ACTION_GPIO_WAIT_MS))
            {
                BLOCK_GPIO_RESET(GPIO_PIN_6);
                action_next_step(ACTION_STEP_BLOCK_LIFT_2);
            }
            break;

        case ACTION_STEP_BLOCK_LIFT_2:
            lift3_set_height_cmd(LIFT3_CMD_20CM);
            action_next_step(ACTION_STEP_BLOCK_GPIO_2);
            break;

        case ACTION_STEP_BLOCK_GPIO_2:
            if (action_step_elapsed(ACTION_LIFT_WAIT_MS))
            {
                BLOCK_GPIO_RESET(GPIO_PIN_5);
                action_next_step(ACTION_STEP_BLOCK_MOTOR_2);
            }
            break;

        case ACTION_STEP_BLOCK_MOTOR_2:
            can2_2006_target_angle_12 = 0.0f;
            action_next_step(ACTION_STEP_DONE);
            break;

        case ACTION_STEP_DONE:
            action_done_after(ACTION_MOTOR_WAIT_MS);
            break;

        default:
            action_fail();
            break;
    }
}

static void block_place_task(void)
{
    switch (action_step)
    {
        case ACTION_STEP_BLOCK_LIFT_1:
            lift1_cmd = 3;
            lift2_cmd = 3;
            lift3_set_height_cmd(LIFT3_CMD_60CM);
            action_next_step(ACTION_STEP_BLOCK_MOTOR_1);
            break;

        case ACTION_STEP_BLOCK_MOTOR_1:
            if (action_step_elapsed(ACTION_LIFT_WAIT_MS))
            {
                can2_2006_target_angle_12 = BLOCK_MOTOR12_CCW_90_ANGLE;
                BLOCK_GPIO_SET(GPIO_PIN_6);
                action_next_step(ACTION_STEP_BLOCK_WAIT_1);
            }
            break;

        case ACTION_STEP_BLOCK_WAIT_1:
            if (action_step_elapsed(2000U))
            {
                BLOCK_GPIO_SET(GPIO_PIN_5);
            }
            if (action_step_elapsed(5000U))
            {
                BLOCK_GPIO_RESET(GPIO_PIN_7);
                action_next_step(ACTION_STEP_DONE);
            }
            break;

        case ACTION_STEP_DONE:
            action_done_after(ACTION_MOTOR_WAIT_MS);
            break;

        default:
            action_fail();
            break;
    }
}

static void confront_block_place_task(void)
{
    switch (action_step)
    {
        case ACTION_STEP_BLOCK_LIFT_1:
            lift1_cmd = 3;
            lift2_cmd = 3;
            lift3_set_height_cmd(LIFT3_CMD_20CM);
            action_next_step(ACTION_STEP_BLOCK_MOTOR_1);
            break;

        case ACTION_STEP_BLOCK_MOTOR_1:
            if (action_step_elapsed(ACTION_LIFT_WAIT_MS))
            {
                can2_2006_target_angle_12 = BLOCK_MOTOR12_CCW_90_ANGLE;
                BLOCK_GPIO_SET(GPIO_PIN_5);
                action_next_step(ACTION_STEP_BLOCK_WAIT_1);
            }
            break;

        case ACTION_STEP_BLOCK_WAIT_1:
            if (action_step_elapsed(3000U))
            {
                BLOCK_GPIO_RESET(GPIO_PIN_7);
                action_next_step(ACTION_STEP_DONE);
            }
            break;

        case ACTION_STEP_DONE:
            action_done_after(ACTION_MOTOR_WAIT_MS);
            break;

        default:
            action_fail();
            break;
    }
}

static void block_take_out_storage_task(void)
{
    switch (action_step)
    {
        case ACTION_STEP_BLOCK_LIFT_1:
            can2_2006_target_angle_12 = BLOCK_MOTOR12_CCW_270_ANGLE;
            action_next_step(ACTION_STEP_BLOCK_LIFT_2);
            break;

        case ACTION_STEP_BLOCK_LIFT_2:
            if (action_step_elapsed(ACTION_MOTOR_WAIT_MS))
            {
                lift3_set_height_cmd(LIFT3_CMD_10CM);
                action_next_step(ACTION_STEP_BLOCK_GPIO_1);
            }
            break;

        case ACTION_STEP_BLOCK_GPIO_1:
            if (action_step_elapsed(ACTION_LIFT_WAIT_MS))
            {
                BLOCK_GPIO_SET(GPIO_PIN_5);
                action_next_step(ACTION_STEP_BLOCK_MOTOR_2);
            }
            break;

        case ACTION_STEP_BLOCK_MOTOR_2:
            can2_2006_target_angle_12 = BLOCK_MOTOR12_CCW_180_ANGLE;
            action_next_step(ACTION_STEP_DONE);
            break;

        case ACTION_STEP_DONE:
            action_done_after(ACTION_MOTOR_WAIT_MS);
            break;

        default:
            action_fail();
            break;
    }
}

static void block_action_task(void)
{
    switch (action_id)
    {
        case ACTION_ID_BLOCK_LOW_TO_HIGH_PICK:
            block_pick_task(LIFT3_CMD_40CM);
            break;

        case ACTION_ID_BLOCK_HIGH_TO_LOW_PICK:
            block_pick_task(LIFT3_CMD_10CM);
            break;

        case ACTION_ID_BLOCK_SUCK:
            block_suck_task();
            break;

        case ACTION_ID_BLOCK_STORE:
            block_store_task();
            break;

        case ACTION_ID_BLOCK_PLACE:
            block_place_task();
            break;

        case ACTION_ID_BLOCK_TAKE_OUT_STORAGE:
            block_take_out_storage_task();
            break;

        case ACTION_ID_CONFRONT_BLOCK_PLACE:
            confront_block_place_task();
            break;

        default:
            action_fail();
            break;
    }
}

void action_group_task(void)
{
    if (action_status != ACTION_STATUS_RUNNING)
    {
        return;
    }

    if (action_id == ACTION_ID_GRIPPER_PICK)
    {
        return;
    }

    if (action_step_elapsed(ACTION_STEP_TIMEOUT_MS))
    {
        action_fail();
        return;
    }

    if (chassis_distance_failed || side_distance_failed)
    {
        action_fail();
        return;
    }

    if (action_is_block(action_id))
    {
        block_action_task();
    }
    else
    {
        climb_action_task();
    }

    if (action_status != ACTION_STATUS_RUNNING)
    {
        chassis_drive_pid_restore_requested();
    }
}

void action_feedback_task(void)
{
    uint32_t now = HAL_GetTick();
    Packet_ActionGroupFeedback pkt;
    static uint8_t last_status = 0xFF;

    if (now - action_fb_last_tick < ACTION_FB_PERIOD_MS)
    {
        return;
    }

    action_fb_last_tick = now;

    if ((action_status != ACTION_STATUS_RUNNING) && (action_status == last_status))
    {
        return;
    }

    last_status = action_status;

    pkt.status = action_status;
    send_ActionGroupFeedback(&pkt);
}
