// #include "Copter.h"

// /*
//  * Init and run calls for stabilize flight mode
//  */

// // stabilize_run - runs the main stabilize controller
// // should be called at 100hz or more
// void ModeStabilize::run()
// {
//     // apply simple mode transform to pilot inputs
//     update_simple_mode();

//     // convert pilot input to lean angles
//     float target_roll, target_pitch;
//     get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

//     // get pilot's desired yaw rate
//     float target_yaw_rate = get_pilot_desired_yaw_rate();

//     if (!motors->armed()) {
//         // Motors should be Stopped
//         motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
//     } else if (copter.ap.throttle_zero
//                || (copter.air_mode == AirMode::AIRMODE_ENABLED && motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN)) {
//         // throttle_zero is never true in air mode, but the motors should be allowed to go through ground idle
//         // in order to facilitate the spoolup block

//         // Attempting to Land
//         motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
//     } else {
//         motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
//     }




    
//     float pilot_desired_throttle = get_pilot_desired_throttle();

//     switch (motors->get_spool_state()) {
//     case AP_Motors::SpoolState::SHUT_DOWN:
//         // Motors Stopped
//         attitude_control->reset_yaw_target_and_rate();
//         attitude_control->reset_rate_controller_I_terms();
//         pilot_desired_throttle = 0.0f;
//         break;

//     case AP_Motors::SpoolState::GROUND_IDLE:
//         // Landed
//         attitude_control->reset_yaw_target_and_rate();
//         attitude_control->reset_rate_controller_I_terms_smoothly();
//         pilot_desired_throttle = 0.0f;
//         break;

//     case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
//         // clear landing flag above zero throttle
//         if (!motors->limit.throttle_lower) {
//             set_land_complete(false);
//         }
//         break;

//     case AP_Motors::SpoolState::SPOOLING_UP:
//     case AP_Motors::SpoolState::SPOOLING_DOWN:
//         // do nothing
//         break;
//     }

//     // call attitude controller
//     attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

//     // output pilot's throttle
//     attitude_control->set_throttle_out(pilot_desired_throttle, true, g.throttle_filt);
// }




/*
2022/2/24
@CHENxiaomingming
e-mail:2210138207@qq.com
newfly
*/
#include "Copter.h"


/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeStabilize::run()
{


    update_simple_mode();

    // 自动滚转控制逻辑
    static float auto_roll_angle = 5.0f;  // 初始滚转角5度
    static uint32_t last_switch_time_ms = 0;
    
    // 初始化时间（只在第一次运行时执行）
    if (last_switch_time_ms == 0) {
        last_switch_time_ms = millis();
    }
    
    // 检查是否达到0.5秒切换时间
    uint32_t current_time_ms = millis();
    if (current_time_ms - last_switch_time_ms >= 500) {  // 500毫秒 = 0.5秒
        // 切换滚转角：5度 <-> -5度
        auto_roll_angle = -auto_roll_angle;
        last_switch_time_ms = current_time_ms;
    }



    float target_roll = auto_roll_angle * 100.0f;  // 度 -> 厘度

    //dummy_roll只接收，不参与最终控制
    float dummy_roll, target_pitch;
    get_pilot_desired_lean_angles(dummy_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate();
    

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero
               || (copter.air_mode == AirMode::AIRMODE_ENABLED && motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN)) {
        // throttle_zero is never true in air mode, but the motors should be allowed to go through ground idle
        // in order to facilitate the spoolup block

        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }




    
    float pilot_desired_throttle = get_pilot_desired_throttle();

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    pilot_desired_throttle = 0.6f;
    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_desired_throttle, true, g.throttle_filt);
}
