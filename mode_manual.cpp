#include "mode.h"
#include "Plane.h"


#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>
#include <GCS_MAVLink/GCS.h>    //地面站

#include <AP_HAL/AP_HAL.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <time.h>  // 用于时间格式化
#include <AP_HAL/AP_HAL.h>




// // GPS数据打印函数实现，每2秒向地面站发送一次消息
// static void send_gps_to_gcs()
// {
//     // （每2秒发送一次）
//     static uint32_t last_send_ms = 0;
//     uint32_t now_ms = AP_HAL::millis();
    
//     if (now_ms - last_send_ms < 2000) {  // 每2秒发送一次
//         return;
//     }
    
//     last_send_ms = now_ms;
    
//     AP_GPS &gps = AP::gps();
    
//     if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
//         const Location &loc = gps.location();
//         float alt_m = loc.alt * 0.01f;
//         // 获取地速，单位：米/秒
//         float ground_speed = gps.ground_speed();  

 
//         // 向地面站发送消息
//         gcs().send_text(MAV_SEVERITY_INFO, 
//                        "GPS: Lat=%.6f Lon=%.6f Alt=%.1fm Sats=%d Speed=%.1fm/s",
//                        loc.lat * 1.0e-7f,
//                        loc.lng * 1.0e-7f,
//                        alt_m,
//                        gps.num_sats(),
//                        ground_speed);
//     }
// }




// // GPS数据打印函数实现，每2秒向地面站发送一次消息
// static void send_gps_to_gcs()
// {
//     // （每2秒发送一次）
//     static uint32_t last_send_ms = 0;
//     uint32_t now_ms = AP_HAL::millis();
    
//     if (now_ms - last_send_ms < 2000) {  // 每2秒发送一次
//         return;
//     }
    
//     last_send_ms = now_ms;
    
//     AP_GPS &gps = AP::gps();
    
//     if (gps.status() < AP_GPS::GPS_OK_FIX_3D) {
//         gcs().send_text(MAV_SEVERITY_WARNING, "GPS: Waiting for 3D fix");
//         return;
//     }
    
//     // 1. 获取当前飞机状态
//     const Location &loc = gps.location();
//     float alt_m = loc.alt * 0.01f;
//     float ground_speed = gps.ground_speed();  // 地速（标量）
//     float ground_course = gps.ground_course(); // 地面航向
    
//     // 2. 获取飞机姿态（用于坐标转换）
//     AP_AHRS &ahrs = AP::ahrs();
    
//     // 检查AHRS是否已初始化
//     if (!ahrs.initialised()) {
//         gcs().send_text(MAV_SEVERITY_WARNING, "PNG: AHRS not initialised");
//         return;
//     }
    
//     // 检查AHRS是否健康（可选，但推荐）
//     if (!ahrs.healthy()) {
//         gcs().send_text(MAV_SEVERITY_WARNING, "PNG: AHRS not healthy");
//         return;
//     }
    
//     float roll_rad, pitch_rad, yaw_rad;
    
//     // 方法1：直接调用to_euler()，它没有返回值
//     ahrs.get_rotation_body_to_ned().to_euler(&roll_rad, &pitch_rad, &yaw_rad);
    
    
//     // 3. 定义固定目标点（写死的坐标）
//     Location target_loc;
//     target_loc.lat = -353632377;  // -35.3632377度
//     target_loc.lng = 1491724789;  // 149.1724789度
//     target_loc.alt = 50000;       // 500米（50,000厘米）
    
//     // 4. 使用Location类的方法计算相对距离和方位角（更精确）
//     float distance_cm = loc.get_distance(target_loc);      // 水平距离（厘米）
//     float distance_m = distance_cm * 0.01f;                // 水平距离（米）
//     int32_t bearing_cd = loc.get_bearing_to(target_loc);   // 方位角（厘度）
//     float bearing_deg = bearing_cd * 0.01f;                // 方位角（度）
    
//     // 5. 计算高度差
//     float alt_diff_m = (target_loc.alt - loc.alt) * 0.01f; // 高度差（米）
    
//     // 6. 计算相对位置矢量（NED坐标系）
//     // 注意：NED坐标系中，北向为正，东向为正，地向（向下）为正
//     Vector3f rel_pos_ned;
//     rel_pos_ned.x = distance_m * cosf(radians(bearing_deg));  // 北向
//     rel_pos_ned.y = distance_m * sinf(radians(bearing_deg));  // 东向
//     rel_pos_ned.z = -alt_diff_m;  // 地向（向下为正），注意负号
    
//     // 7. 计算视线矢量（从飞机指向目标）
//     Vector3f los_ned = rel_pos_ned;  // 这里rel_pos_ned已经是从飞机指向目标
//     float los_distance = los_ned.length();  // 视线距离
    
//     // 避免除零
//     if (los_distance < 0.1f) {
//         los_distance = 0.1f;
//     }
    
//     // 8. 计算飞机速度矢量（NED系）
//     float ground_course_rad = radians(ground_course);
//     Vector3f vel_ned;
//     vel_ned.x = ground_speed * cosf(ground_course_rad);  // 北向速度
//     vel_ned.y = ground_speed * sinf(ground_course_rad);  // 东向速度
//     vel_ned.z = 0;  // 假设水平飞行，没有垂直速度
    
//     // 9. 目标速度设为0
//     Vector3f target_vel_ned(0, 0, 0);
    
//     // 10. 使用矢量法计算视线角速度（导航系下）
//     // 公式：ω_los = (r × (V_t - V_m)) / |r|^2
//     Vector3f relative_vel = target_vel_ned - vel_ned;  // 相对速度
//     Vector3f los_rate_ned = los_ned % relative_vel;   // 叉乘
//     los_rate_ned /= (los_distance * los_distance);    // 除以距离平方
    
//     // 11. 构建从导航系(NED)到机体系(FRD)前右下的旋转矩阵
//     Matrix3f R_ned_to_body;
    
//     float cr = cosf(roll_rad);
//     float sr = sinf(roll_rad);
//     float cp = cosf(pitch_rad);
//     float sp = sinf(pitch_rad);
//     float cy = cosf(yaw_rad);
//     float sy = sinf(yaw_rad);
    
//     // NED到机体系（FRD）的旋转矩阵
//     R_ned_to_body.a.x = cp * cy;
//     R_ned_to_body.a.y = cp * sy;
//     R_ned_to_body.a.z = -sp;
    
//     R_ned_to_body.b.x = sr * sp * cy - cr * sy;
//     R_ned_to_body.b.y = sr * sp * sy + cr * cy;
//     R_ned_to_body.b.z = sr * cp;
    
//     R_ned_to_body.c.x = cr * sp * cy + sr * sy;
//     R_ned_to_body.c.y = cr * sp * sy - sr * cy;
//     R_ned_to_body.c.z = cr * cp;
    
//     // 12. 将视线角速度从导航系转换到机体系
//     Vector3f los_rate_body = R_ned_to_body * los_rate_ned;
    
//     // 13. 计算视线角速度的大小
//     float los_rate_mag = los_rate_body.length();  // 角速度大小（rad/s）
//     float los_rate_deg_per_sec = degrees(los_rate_mag);  // 转换为度/秒
    
//     // 14. 纯比例导引：指令 = 比例系数 × 视线角速度
//     static const float PNG_GAIN = 3.0f;  // 比例导引系数，通常取3-5
//     Vector3f cmd_body = los_rate_body * PNG_GAIN;
    
//     // 15. 发送消息到地面站
//     // 基础GPS信息
//     gcs().send_text(MAV_SEVERITY_INFO, 
//                    "GPS: Lat=%.6f Lon=%.6f Alt=%.1fm Spd=%.1fm/s Hdg=%.1f",
//                    loc.lat * 1.0e-7f,
//                    loc.lng * 1.0e-7f,
//                    alt_m,
//                    ground_speed,
//                    ground_course);
    
//     // 目标点信息
//     gcs().send_text(MAV_SEVERITY_INFO,
//                    "Target: Lat=%.6f Lon=%.6f Alt=%.1fm",
//                    target_loc.lat * 1.0e-7f,
//                    target_loc.lng * 1.0e-7f,
//                    target_loc.alt * 0.01f);
    
//     // 相对位置信息
//     gcs().send_text(MAV_SEVERITY_INFO,
//                    "RelPos: N=%.1f E=%.1f D=%.1fm Dist=%.1fm Bearing=%.1f°",
//                    rel_pos_ned.x,
//                    rel_pos_ned.y,
//                    rel_pos_ned.z,
//                    los_distance,
//                    bearing_deg);
    
//     // 视线角速度信息（机体系）
//     gcs().send_text(MAV_SEVERITY_INFO,
//                    "LOS Rate Body: X=%.4f Y=%.4f Z=%.4f rad/s | %.2f deg/s",
//                    los_rate_body.x,
//                    los_rate_body.y,
//                    los_rate_body.z,
//                    los_rate_deg_per_sec);
    
//     // 比例导引指令
//     gcs().send_text(MAV_SEVERITY_INFO,
//                    "PNG Cmd(N=%.1f): X=%.4f Y=%.4f Z=%.4f",
//                    PNG_GAIN,
//                    cmd_body.x,
//                    cmd_body.y,
//                    cmd_body.z);
    
//     // 计算并显示视线角
//     float los_azimuth = atan2f(los_ned.y, los_ned.x);  // 方位角（弧度）
//     float los_elevation = asinf(los_ned.z / los_distance);  // 俯仰角（弧度）
    
//     gcs().send_text(MAV_SEVERITY_INFO,
//                    "LOS Angles: Az=%.1f° El=%.1f°",
//                    degrees(los_azimuth),
//                    degrees(los_elevation));
// }


// // GPS数据打印函数实现，每2秒向地面站发送一次消息
// static void send_gps_to_gcs()
// {
//     // （每2秒发送一次）
//     static uint32_t last_send_ms = 0;
//     uint32_t now_ms = AP_HAL::millis();
    
//     if (now_ms - last_send_ms < 2000) {  // 每2秒发送一次
//         return;
//     }
    
//     last_send_ms = now_ms;
    
//     AP_GPS &gps = AP::gps();
    
//     if (gps.status() < AP_GPS::GPS_OK_FIX_3D) {
//         gcs().send_text(MAV_SEVERITY_WARNING, "GPS: Waiting for 3D fix");
//         return;
//     }
    
//     // 1. 获取当前飞机状态
//     const Location &loc = gps.location();
//     float alt_m = loc.alt * 0.01f;
//     float ground_speed = gps.ground_speed();  // 地速（标量）
//     float ground_course = gps.ground_course(); // 地面航向
    
//     // 2. 获取飞机姿态（用于坐标转换）
//     AP_AHRS &ahrs = AP::ahrs();
    
//     // 检查AHRS是否已初始化
//     if (!ahrs.initialised()) {
//         gcs().send_text(MAV_SEVERITY_WARNING, "PNG: AHRS not initialised");
//         return;
//     }
    
//     // 检查AHRS是否健康（可选，但推荐）
//     if (!ahrs.healthy()) {
//         gcs().send_text(MAV_SEVERITY_WARNING, "PNG: AHRS not healthy");
//         return;
//     }
    
//     float roll_rad, pitch_rad, yaw_rad;
    
//     // 方法1：直接调用to_euler()，它没有返回值
//     ahrs.get_rotation_body_to_ned().to_euler(&roll_rad, &pitch_rad, &yaw_rad);
    
    
//     // 3. 定义固定目标点（写死的坐标）
//     Location target_loc;
//     target_loc.lat = -353632377;  // -35.3632377度
//     target_loc.lng = 1491724789;  // 149.1724789度
//     target_loc.alt = 50000;       // 500米（50,000厘米）
    
//     // 4. 使用Location类的方法计算相对距离和方位角（更精确）
//     float distance_cm = loc.get_distance(target_loc);      // 水平距离（厘米）
//     float distance_m = distance_cm * 0.01f;                // 水平距离（米）
//     int32_t bearing_cd = loc.get_bearing_to(target_loc);   // 方位角（厘度）
//     float bearing_deg = bearing_cd * 0.01f;                // 方位角（度）
    
//     // 5. 计算高度差
//     float alt_diff_m = (target_loc.alt - loc.alt) * 0.01f; // 高度差（米）
    
//     // 6. 计算相对位置矢量（NED坐标系）
//     // 注意：NED坐标系中，北向为正，东向为正，地向（向下）为正
//     Vector3f rel_pos_ned;
//     rel_pos_ned.x = distance_m * cosf(radians(bearing_deg));  // 北向
//     rel_pos_ned.y = distance_m * sinf(radians(bearing_deg));  // 东向
//     rel_pos_ned.z = -alt_diff_m;  // 地向（向下为正），注意负号
    
//     // 7. 计算视线矢量（从飞机指向目标）
//     Vector3f los_ned = rel_pos_ned;  // 这里rel_pos_ned已经是从飞机指向目标
//     float los_distance = los_ned.length();  // 视线距离
    
//     // 避免除零
//     if (los_distance < 0.1f) {
//         los_distance = 0.1f;
//     }
    
//     // 8. 计算飞机速度矢量（NED系）
//     float ground_course_rad = radians(ground_course);
//     Vector3f vel_ned;
//     vel_ned.x = ground_speed * cosf(ground_course_rad);  // 北向速度
//     vel_ned.y = ground_speed * sinf(ground_course_rad);  // 东向速度
//     vel_ned.z = 0;  // 假设水平飞行，没有垂直速度
    
//     // 9. 目标速度设为0
//     Vector3f target_vel_ned(0, 0, 0);
    
//     // 10. 使用矢量法计算视线角速度（导航系下）
//     // 公式：ω_los = (r × (V_t - V_m)) / |r|^2
//     Vector3f relative_vel = target_vel_ned - vel_ned;  // 相对速度
//     Vector3f los_rate_ned = los_ned % relative_vel;   // 叉乘
//     los_rate_ned /= (los_distance * los_distance);    // 除以距离平方
    
//     // 11. 构建从导航系(NED)到机体系(FRD)前右下的旋转矩阵
//     Matrix3f R_ned_to_body;
    
//     float cr = cosf(roll_rad);
//     float sr = sinf(roll_rad);
//     float cp = cosf(pitch_rad);
//     float sp = sinf(pitch_rad);
//     float cy = cosf(yaw_rad);
//     float sy = sinf(yaw_rad);
    
//     // NED到机体系（FRD）的旋转矩阵
//     R_ned_to_body.a.x = cp * cy;
//     R_ned_to_body.a.y = cp * sy;
//     R_ned_to_body.a.z = -sp;
    
//     R_ned_to_body.b.x = sr * sp * cy - cr * sy;
//     R_ned_to_body.b.y = sr * sp * sy + cr * cy;
//     R_ned_to_body.b.z = sr * cp;
    
//     R_ned_to_body.c.x = cr * sp * cy + sr * sy;
//     R_ned_to_body.c.y = cr * sp * sy - sr * cy;
//     R_ned_to_body.c.z = cr * cp;
    
//     // 12. 将视线角速度从导航系转换到机体系
//     Vector3f los_rate_body = R_ned_to_body * los_rate_ned;
    
//     // 13. 计算视线角速度的大小
//     float los_rate_mag = los_rate_body.length();  // 角速度大小（rad/s）
//     float los_rate_deg_per_sec = degrees(los_rate_mag);  // 转换为度/秒
    
//     // 14. 纯比例导引：指令 = 比例系数 × 视线角速度
//     static const float PNG_GAIN = 3.0f;  // 比例导引系数，通常取3-5
//     Vector3f cmd_body = los_rate_body * PNG_GAIN;
    
//     // 15. 发送消息到地面站
//     // 基础GPS信息
//     gcs().send_text(MAV_SEVERITY_INFO, 
//                    "GPS: Lat=%.6f Lon=%.6f Alt=%.1fm Spd=%.1fm/s Hdg=%.1f",
//                    loc.lat * 1.0e-7f,
//                    loc.lng * 1.0e-7f,
//                    alt_m,
//                    ground_speed,
//                    ground_course);
    
//     // 目标点信息
//     gcs().send_text(MAV_SEVERITY_INFO,
//                    "Target: Lat=%.6f Lon=%.6f Alt=%.1fm",
//                    target_loc.lat * 1.0e-7f,
//                    target_loc.lng * 1.0e-7f,
//                    target_loc.alt * 0.01f);
    
//     // 相对位置信息
//     gcs().send_text(MAV_SEVERITY_INFO,
//                    "RelPos: N=%.1f E=%.1f D=%.1fm Dist=%.1fm Bearing=%.1f°",
//                    rel_pos_ned.x,
//                    rel_pos_ned.y,
//                    rel_pos_ned.z,
//                    los_distance,
//                    bearing_deg);
    
//     // 视线角速度信息（机体系）
//     gcs().send_text(MAV_SEVERITY_INFO,
//                    "LOS Rate Body: X=%.4f Y=%.4f Z=%.4f rad/s | %.2f deg/s",
//                    los_rate_body.x,
//                    los_rate_body.y,
//                    los_rate_body.z,
//                    los_rate_deg_per_sec);
    

// }




// 自定义限幅函数
static inline float my_constrain_float(float value, float min_val, float max_val)
{
    // 确保 min_val <= max_val
    if (min_val > max_val) {
        // 交换最小值与最大值
        float temp = min_val;
        min_val = max_val;
        max_val = temp;
    }
    
    // 执行限幅
    if (value < min_val) {
        return min_val;
    } else if (value > max_val) {
        return max_val;
    } else {
        return value;
    }
}


// GPS数据打印函数实现，每1秒向地面站发送一次消息
static void send_gps_to_gcs(float& acc_z, float& acc_y)
{
    // // （每2秒发送一次）
    // static uint32_t last_send_ms = 0;
    // uint32_t now_ms = AP_HAL::millis();
    
    // if (now_ms - last_send_ms < 100) {  // 每1秒发送一次
    //     return;
    // }
    
    // last_send_ms = now_ms;
    
    AP_GPS &gps = AP::gps();
    
    if (gps.status() < AP_GPS::GPS_OK_FIX_3D) {
        gcs().send_text(MAV_SEVERITY_WARNING, "GPS: Waiting for 3D fix");
        return;
    }
    
    // 1. 获取当前飞机状态（WGS-84坐标系）
    const Location &loc = gps.location();
    // float alt_m = loc.alt * 0.01f;
    const Vector3f &ned_velocity = gps.velocity();


    
    // 2. 获取飞机姿态（用于坐标转换）
    AP_AHRS &ahrs = AP::ahrs();
    
    // 检查AHRS是否已初始化
    if (!ahrs.initialised()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "PNG: AHRS not initialised");
        return;
    }
    
    // 检查AHRS是否健康
    if (!ahrs.healthy()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "PNG: AHRS not healthy");
        return;
    }
    
    // float roll_rad, pitch_rad, yaw_rad;
    
    // // 获取姿态欧拉角（滚转、俯仰、偏航）
    // ahrs.get_rotation_body_to_ned().to_euler(&roll_rad, &pitch_rad, &yaw_rad);


    float roll_rad = ahrs.get_roll(); 
    float pitch_rad = ahrs.get_pitch();
    float yaw_rad = ahrs.get_yaw();



    
    // 3. 定义固定目标点（WGS-84坐标系）
    Location target_loc;
    target_loc.lat = -353632377;  // -35.3632377度
    target_loc.lng = 1491724789;  // 149.1724789度
    // target_loc.alt = 58400;       // 500米（50,000厘米）
    target_loc.alt = 58400;       // 500米（50,000厘米）

    
    // 4. WGS-84到NED坐标系转换
    // 方法1：使用get_distance_NE()方法（最精确）
    Vector2f offset_ne = loc.get_distance_NE(target_loc);
    float offset_north_m = offset_ne.x;  // 北向距离（米）
    float offset_east_m = offset_ne.y;   // 东向距离（米）
    
    // 5. 计算高度差
    float alt_diff_m = (target_loc.alt - loc.alt) * 0.01f; // 高度差（米）
    
    // 6. 构建相对位置矢量（局部NED坐标系）
    // NED坐标系：X=北向，Y=东向，Z=地向（向下为正）
    Vector3f rel_pos_ned;
    rel_pos_ned.x = offset_north_m;    // 北向距离（飞机指向目标的北向分量）
    rel_pos_ned.y = offset_east_m;     // 东向距离（飞机指向目标的东向分量）
    rel_pos_ned.z = -alt_diff_m;       // 地向距离（向下为正，注意负号）
    
    // 7. 计算视线矢量（从飞机指向目标）
    Vector3f los_ned = rel_pos_ned;
    float los_distance = los_ned.length();  // 视线距离（三维欧几里得距离）
    
    // 避免除零
    if (los_distance < 0.1f) {
        los_distance = 0.1f;
    }
    
    
    // 9. 计算飞机速度矢量（NED系）
    Vector3f vel_ned;

    vel_ned.x = ned_velocity.x;  // 北向速度
    vel_ned.y = ned_velocity.y;  // 东向速度
    vel_ned.z = ned_velocity.z;  // 垂直速度
    
    // 10. 目标速度设为0（静止目标）
    Vector3f target_vel_ned(0, 0, 0);
    
    // 11. 使用矢量法计算视线角速度（导航系下）
    // 公式：ω_los = (r × (V_t - V_m)) / |r|^2
    Vector3f relative_vel = target_vel_ned - vel_ned;  // 相对速度
    Vector3f los_rate_ned = los_ned % relative_vel;   // 叉乘
    los_rate_ned /= (los_distance * los_distance);    // 除以距离平方
    
    // 12. 构建从导航系(NED)到机体系(FRD)的旋转矩阵
    Matrix3f R_ned_to_body;
    
    float cr = cosf(roll_rad);
    float sr = sinf(roll_rad);
    float cp = cosf(pitch_rad);
    float sp = sinf(pitch_rad);
    float cy = cosf(yaw_rad);
    float sy = sinf(yaw_rad);
    
    // NED到机体系（FRD）的旋转矩阵
    R_ned_to_body.a.x = cp * cy;
    R_ned_to_body.a.y = cp * sy;
    R_ned_to_body.a.z = -sp;
    
    R_ned_to_body.b.x = -sr * sp * cy - cr * sy;
    R_ned_to_body.b.y = sr * sp * sy + cr * cy;
    R_ned_to_body.b.z = sr * cp;
    
    R_ned_to_body.c.x = cr * sp * cy + sr * sy;
    R_ned_to_body.c.y = cr * sp * sy - sr * cy;
    R_ned_to_body.c.z = cr * cp;
    
    // 13. 将视线角速度从导航系转换到机体系
    Vector3f los_rate_body = R_ned_to_body * los_rate_ned;
    
    //重力加速度做补偿
    Vector3f g_ned;
    g_ned.x = 0;
    g_ned.y = 0;
    g_ned.z = -9.80665;

    Vector3f g_body = R_ned_to_body * g_ned;



    // 14. 纯比例导引：指令 = 比例系数 × 视线角速度
    static const float PNG_GAIN = 4.0f;  // 比例导引系数，通常取3-5
    float speed = ned_velocity.length();
    const float MIN_SPEED = 0.5f;               // 最低速度阈值，避免数值不稳（m/s）
    if (speed < MIN_SPEED) speed = MIN_SPEED;


    acc_y = PNG_GAIN * speed *  los_rate_body.z +g_body.y;
 
    acc_z = -PNG_GAIN * speed * los_rate_body.y + g_body.z;

    //20260311
    uint64_t now_us = AP_HAL::micros64();
    // 调用日志写入函数
    AP::logger().Write_BTT(now_us, acc_z, acc_y, los_distance);

    
    // // 16. 发送消息到地面站
    // // 基础GPS信息
    // gcs().send_text(MAV_SEVERITY_INFO, 
    //                "GPS: Lat=%.6f Lon=%.6f Alt=%.1fm",
    //                loc.lat * 1.0e-7f,
    //                loc.lng * 1.0e-7f,
    //                loc.alt * 0.01);



  
    // gcs().send_text(MAV_SEVERITY_INFO, 
    //                "nor_east: offset_north_m=%.6fm  offset_east_m=%.6fm",
    //                offset_north_m,
    //                offset_east_m);


    // gcs().send_text(MAV_SEVERITY_INFO, 
    //                "los_rate_ned: los_rate_ned.z=%.6fm  los_rate_ned.y=%.6fm",
    //                los_rate_ned.z,
    //                los_rate_ned.y);

    // gcs().send_text(MAV_SEVERITY_INFO, 
    //                "北向距离: rel_pos_ned.x=%.6fm  rel_pos_ned.y=%.6fm  rel_pos_ned.z=%.6fm  roll_rad=%.6fm  pitch_rad=%.6fm  yaw_rad=%.6fm",
    //                rel_pos_ned.x,
    //                rel_pos_ned.y,
    //                rel_pos_ned.z,
    //                roll_rad,
    //                pitch_rad,
    //                yaw_rad);

    // gcs().send_text(MAV_SEVERITY_INFO, 
    //                "g_body: los_rate_body.y=%.6fm  los_rate_body.z=%.6fm  alt_diff_m=%.2fm",
    //                los_rate_body.y,
    //                los_rate_body.z,
    //                alt_diff_m);
    
    // // 加速度
    // gcs().send_text(MAV_SEVERITY_INFO,
    //                "加速度: acc_z=%.3f acc_y=%.3f speed=%.1fm/s vel_ned.y=%.3f vel_ned.z=%.3f  vel_ned.x=%.3f",
    //                acc_z,
    //                acc_y,
    //                speed,
    //                vel_ned.y,
    //                vel_ned.z,
    //                vel_ned.x);

    // // 视线距离
    // gcs().send_text(MAV_SEVERITY_INFO,
    //                "视线距离: los_distance=%.3f",
    //                los_distance);

}


//phi滚转角 q俯仰角速度，p滚转角速度
static void acc_control(float  acc_z, float  acc_y , float& u_pitch, float& u_roll)
{

    // //静态变量：正弦波开始时间
    // static uint32_t sine_start_time = 0;
    
    // // 初始化正弦波计时器
    // if (sine_start_time == 0) {
    //     sine_start_time = AP_HAL::millis();
    // }


    // 2. 获取飞机姿态（用于坐标转换）
    AP_AHRS &ahrs = AP::ahrs();
    float roll_rad = ahrs.get_roll(); 

    // float roll_rad, pitch_rad, yaw_rad;
    // 获取姿态欧拉角（滚转、俯仰、偏航）
    // ahrs.get_rotation_body_to_ned().to_euler(&roll_rad, &pitch_rad, &yaw_rad);


    // 机体坐标系，单位：rad/s
    Vector3f gyro_body = ahrs.get_gyro(); 
    // 转换为度/秒
    const float RAD_T = 57.2957795131f;  // 180 / π
    const float q = gyro_body.y ;
    const float p = gyro_body.x * RAD_T;


    Vector3f accel_body = ahrs.get_accel();
    const float accel_z = accel_body.z; // 法向加速度 (包含重力-9.8)

    //计算期望滚转  这里单位是弧度
    float angle_rc = atan2f(acc_y,-acc_z);

    //angle_rc 限幅处理±30度
    const float MAX_ROLL_CMD = 30.0f/RAD_T; 
    if (angle_rc > MAX_ROLL_CMD) {
        angle_rc = MAX_ROLL_CMD;
    }
    else if (angle_rc < -MAX_ROLL_CMD) 
    {
        angle_rc = -MAX_ROLL_CMD;
    }

    //计算总法向加速度
    float an = acc_z / cosf(angle_rc);

    // // 2. 计算正弦波
    // const float SINE_AMP = 2.0f;    // 2度幅值
    // const float F_HZ = 0.7f;        // 0.7Hz频率
    
    // uint32_t now = AP_HAL::millis();
    // float time_sec = (now - sine_start_time) * 0.001f;
    // float sine_wave = SINE_AMP * sinf(2.0f * M_PI * F_HZ * time_sec);
    // // 3. 只使用正弦波
    // an = -9.8f +sine_wave;  // 直接赋值




    //计算俯仰和滚转控制量
    // 加速度控制律
    // u_pitch：（法向加速度）
    // u_roll ：（侧向加速度）
    // static const float Kp_n = 0.7;
    // static const float Kw_q = 0.2;
    static const float Kp_n = 0.7;
    static const float Kw_q = 0.2;
    static const float Kp_y = 1* RAD_T;
    static const float Kw_p = 0.5;


    u_pitch = Kp_n * (an-accel_z) - Kw_q * q+8.0f;             // q 只作为阻尼
    u_roll  = Kp_y * (angle_rc - roll_rad) - Kw_p * p;         // p 只作为阻尼


    // 1. 完全忽略导航计算，只生成正弦波
    // u_pitch = 8.0f;
    // u_roll = 0.0f;  // 基础值设为0
    
    // 2. 计算正弦波
    // const float SINE_AMP1 = 5.0f;    // 5度幅值
    // const float F_HZ1 = 1.5f;        // 1.5Hz频率
    
    // // uint32_t now = AP_HAL::millis();
    // // float time_sec = (now - sine_start_time) * 0.001f;
    // float sine_wave1 = SINE_AMP1 * sinf(2.0f * M_PI * F_HZ1 * time_sec);
    // // 3. 只使用正弦波
    // u_roll = sine_wave1;  // 直接赋值

    u_pitch = -1.05f * u_pitch;
    u_roll  =  4.5f * u_roll;


    
    // 调试输出
    // static uint32_t last_debug_ms = 0;
    // if (now - last_debug_ms > 200) {
    //     last_debug_ms = now;
    //     gcs().send_text(MAV_SEVERITY_INFO,
    //                    "deep: Roll=%.2f°",
    //                    u_roll);
    // }
    
    // gcs().send_text(MAV_SEVERITY_INFO,
    //     "u_roll=%.6f",
    //     u_roll);

    // u_roll =0;
    //输出总法向加速度
    // gcs().send_text(MAV_SEVERITY_INFO,
    //     "总法向加速度: an=%.1f",
    //     an);

    //Z轴线加速度
    // gcs().send_text(MAV_SEVERITY_INFO,
    //     "Z轴线加速度: accel_z=%.1f",
    //     accel_z);


    //angle_rc
    // gcs().send_text(MAV_SEVERITY_INFO,
    //     "angle_rc=%.6f roll_rad=%.6f",
    //     angle_rc*RAD_T,
    //     roll_rad*RAD_T);

    //角速度q p
    // gcs().send_text(MAV_SEVERITY_INFO,
    //     "角速度: q=%.6f p=%.1f",
    //     q,
    //     p);


    //输出舵面混控
    // gcs().send_text(MAV_SEVERITY_INFO,
    //     "舵面混控: u_pitch=%.1f   u_roll=%.1f",
    //     u_pitch,
    //     u_roll);
}




static void pwm_control(float u_pitch, float u_roll)
{


    // // （每0.25秒发送一次）
    // static uint32_t last_send_ms = 0;
    // uint32_t now_ms = AP_HAL::millis();
    
    // if (now_ms - last_send_ms < 200) {  // 每1秒发送一次
    //     return;
    // }
    
    // last_send_ms = now_ms;


    //1控制量限幅
    const float MAX_PITCH_DEG = 30.0f*1.05;  // ±30度俯仰限制
    const float MAX_ROLL_DEG = 30.0f*4.5;   // ±15度滚转限制
    

    u_pitch = my_constrain_float(u_pitch, -MAX_PITCH_DEG, MAX_PITCH_DEG);
    u_roll  = my_constrain_float(u_roll,  -MAX_ROLL_DEG, MAX_ROLL_DEG);

    //2根据实测数据校准的映射系数
    const float PITCH_TO_PWM = 10.0f;      // 俯仰：度 → PWM
    // const float ROLL_TO_PWM = 4.1667f;    // 滚转：度 → PWM
    // const float ROLL_TO_PWM = 16.5f;    // 滚转：度 → PWM

    // 3升降副翼混控
    // 注意极性：根据您的测试数据
    // - 正俯仰（抬头）：PWM > 1500
    // - 正滚转（右滚）：左舵机PWM减小，右舵机PWM增加
    float left_pwm_offset  = (u_pitch+u_roll) * PITCH_TO_PWM;
    float right_pwm_offset = (u_pitch-u_roll) * PITCH_TO_PWM;

    // 4映射到PWM范围
    const uint16_t PWM_CENTER = 1500;
    const uint16_t PWM_MIN = 1100;
    const uint16_t PWM_MAX = 1900;

    // 计算浮点PWM值
    float pwm_left_float  = PWM_CENTER + left_pwm_offset;
    float pwm_right_float = PWM_CENTER + right_pwm_offset;

    // 5最终限幅（安全保护）
    pwm_left_float = my_constrain_float(pwm_left_float, 
                                       static_cast<float>(PWM_MIN), 
                                       static_cast<float>(PWM_MAX));
    pwm_right_float = my_constrain_float(pwm_right_float, 
                                        static_cast<float>(PWM_MIN), 
                                        static_cast<float>(PWM_MAX));

    // 6转换为整数PWM值
    uint16_t PWM_LEFT  = static_cast<uint16_t>(pwm_left_float);
    uint16_t PWM_RIGHT = static_cast<uint16_t>(pwm_right_float);

    
    // 设置舵机输出（使用带超时的版本）
    const uint8_t AIL_LEFT_CH   = 1;  // 左副翼通道
    const uint8_t AIL_RIGHT_CH  = 0;  // 右副翼通道

    
    const uint16_t TIMEOUT_MS = 500;  // 超时时间（ms）


    
    // 设置舵机PWM输出
    // PWM_LEFT = 1550;
    // PWM_RIGHT = 1450;
    SRV_Channels::set_output_pwm_chan_timeout(AIL_LEFT_CH,   PWM_LEFT,  TIMEOUT_MS);
    SRV_Channels::set_output_pwm_chan_timeout(AIL_RIGHT_CH,  PWM_RIGHT, TIMEOUT_MS);



    // //输出舵面混控
    // gcs().send_text(MAV_SEVERITY_INFO,
    //     "PWM_LEFT=%u   PWM_RIGHT=%u",
    //     PWM_LEFT,
    //     PWM_RIGHT);

    // gcs().send_text(MAV_SEVERITY_INFO,
    //     "u_pitch=%.3f   u_roll=%.3f",
    //     u_pitch,
    //     u_roll);

    // gcs().send_text(MAV_SEVERITY_INFO,
    //     "PWM: pwm_left_float=%.3f, pwm_right_float=%.3f",  // 清晰标注单位，使用 %d
    //     pwm_left_float,
    //     pwm_right_float);

    
}

// 静态变量
// static   uint32_t roll_sine_start_time = 0;



void ModeManual::update()
{


    // 使用静态变量，避免每次清零
    static float u_pitch = 0.0f;
    static float u_roll = 0.0f;

    // 1. 获取加速度指令
    float acc_z = 0.0f;
    float acc_y = 0.0f;
    send_gps_to_gcs(acc_z, acc_y);  // 这个函数会更新acc_z和acc_y
    
    // 2. 计算姿态控制量
    acc_control(acc_z, acc_y, u_pitch, u_roll);

    // 3. 转换为舵机PWM输出
    pwm_control(u_pitch, u_roll);

    const float throttle = 20.0;
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);

}


void ModeManual::run()
{

    reset_controllers();
}

// true if throttle min/max limits should be applied
bool ModeManual::use_throttle_limits() const
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.available() && quadplane.option_is_set(QuadPlane::OPTION::IDLE_GOV_MANUAL)) {
        return true;
    }
#endif
    return false;
}







// void ModeManual::update()
// {
//     SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.roll_in_expo(false));
//     SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.pitch_in_expo(false));

//     output_rudder_and_steering(plane.rudder_in_expo(false));


//     // SRV_Channels::set_output_pwm_chan(9, 1700);

//     const float throttle = plane.get_throttle_input(true);

//     const float throttle = 50.0;
//     SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);

//     plane.nav_roll_cd = ahrs.roll_sensor;
//     plane.nav_pitch_cd = ahrs.pitch_sensor;

// }
