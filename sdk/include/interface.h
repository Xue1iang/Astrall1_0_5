#ifndef __ASTRALL_SDK_INTERFACE_H__
#define __ASTRALL_SDK_INTERFACE_H__

#include <stdint.h>
#include <string>
#include <cstring>
#include <iostream>
#include <functional>

#define ASTRALL_RES_FAILED          0x8004   // 执行失败/申请失败/订阅失败
#define ASTRALL_RES_TIMEOUT         0x8005   // 超时
#define ASTRALL_RES_RUNNING         0x8006   // 执行中
#define ASTRALL_RES_SUCCESSED       0x8008   // 执行成功
#define ASTRALL_RES_INVALID_PARAM   0x8010   // 无效参数
#define ASTRALL_RES_RC_NORELEASE    0x8020   // 遥控器抢夺未释放控制权
#define ASTRALL_RES_BEEN_OBTAINED   0x8021   // 已经有设备获得控制权未释放
#define ASTRALL_RES_WITHOUT_AUTH    0x8022   // 没有控制权

enum AstrallSystemCode : uint8_t
{
    ASTRALL_SYSTEM_CODE_INIT     = 0,   // 初始化 & 系统自检
    ASTRALL_SYSTEM_CODE_STANDBY  = 1,   // 待机 -> 阻尼模式
    ASTRALL_SYSTEM_CODE_RUNNING  = 2,   // 正常运行
    ASTRALL_SYSTEM_CODE_WARNING  = 3,   // 警告 -> 需要尽快处理
    ASTRALL_SYSTEM_CODE_ERROR    = 4,   // 出错 -> 紧急停机
    ASTRALL_SYSTEM_CODE_SHUTDOWN = 5    // 关机
};

enum AstrallErrorCode : uint32_t
{
    ASTRALL_ERR_NONE                = 0,
    ASTRALL_ERR_MOTOR_VERSION       = (1U << 0),    // 电机驱动版本错误
    ASTRALL_ERR_MOTOR_COMM_LOSE     = (1U << 1),    // 电机通讯丢失
    ASTRALL_ERR_MOTOR_STATUS        = (1U << 2),    // 电机异常，请检查电机状态
    ASTRALL_ERR_MOTOR_OVER_TEMP     = (1U << 3),    // 电机过温
    ASTRALL_ERR_STO_TRIGGERED       = (1U << 4),    // STO 触发
    ASTRALL_ERR_IMU_ERROR           = (1U << 5),    // IMU 错误
    ASTRALL_ERR_LOW_BATTERY_LEVEL   = (1U << 6),    // 电池电量过低
    ASTRALL_ERR_OVER_LOAD           = (1U << 7),    // 电机过载
    ASTRALL_ERR_MBUS_DRIVER_FAIL    = (1u << 8),    // 驱动版本错误
    ASTRALL_ERR_LOSE_CAMERA         = (1u << 9),    // 未发现相机设备
};

enum AstrallWarnCode : uint32_t
{
    ASTRALL_WARN_NONE               = 0,
    ASTRALL_WARN_MOTOR_OVER_TEMP    = (1U << 0),    // 电机即将过温
    ASTRALL_WARN_JOYSTICK_LOCK      = (1U << 1),    // 遥控器已锁定，开关拨至三档可解锁
    ASTRALL_INFO_JOYSTICK_INTERVENTION = (1U << 2), // 遥控器已获取控制权
    ASTRALL_WARN_MOTOR_COMM_UNSTABLE = (1U << 3),   // 电机通讯不稳定
    ASTRALL_WARN_LOW_BATTERY_LEVEL  = (1U << 4),    // 电池电量不足警告
    ASTRALL_WARN_JOYSTICK_LOSE      = (1U << 5),    // 遥控器离线
    ASTRALL_WARN_MOTOR_OVER_LOAD    = (1U << 6),    // 电机即将过载
    ASTRALL_WARNING_CONNECT_SDK_LOSE = (1u << 7),   // 已经连接过的SDK离线
};

enum AstrallSportModeCmd : uint16_t
{
    ASTRALL_SPORT_MODE_CMD_DAMPING       = 0xA101,       // 阻尼(软急停)
    ASTRALL_SPORT_MODE_CMD_FIXEDSTAND    = 0xA102,       // 锁定站立
    ASTRALL_SPORT_MODE_CMD_FIXEDDOWN     = 0xA103,       // 锁定趴下
    ASTRALL_SPORT_MODE_CMD_MOVE          = 0xA104,       // 行走(RL步态全地形覆盖)
    ASTRALL_SPORT_MODE_CMD_AUTOCHARGE    = 0xA105,       // 自主充电 >> 需要先走到充电桩附近
    ASTRALL_SPORT_MODE_CMD_EXITCHARGE    = 0xA106,       // 退出充电桩
    ASTRALL_SPORT_MODE_CMD_GET_RIGHT     = 0xA1FF        // 摔倒自主恢复站立
};

enum AstrallSportStatus : uint16_t
{
    ASTRALL_SPORT_STATUS_UNKNOWN        = 0x0000,       // 未知状态
    ASTRALL_SPORT_STATUS_INIT           = 0xB100,       // 初始化中
    ASTRALL_SPORT_STATUS_DAMPING        = 0xB101,       // 阻尼状态
    ASTRALL_SPORT_STATUS_FIXEDSTAND     = 0xB102,       // 锁定站立状态
    ASTRALL_SPORT_STATUS_FIXEDDOWN      = 0xB103,       // 锁定趴下状态
    ASTRALL_SPORT_STATUS_MOVE           = 0xB104,       // 行走(RL步态全地形覆盖)中
    ASTRALL_SPORT_STATUS_UPSTAIRS       = 0xB105,       // 爬楼梯中
    ASTRALL_SPORT_STATUS_CAR            = 0xB106,       // 负重小车模式
    ASTRALL_SPORT_STATUS_AUTOCHAEGE     = 0xB107,       // 准备执行自动充电
    ASTRALL_SPORT_STATUS_SEARCHCHARGING = 0xB108,       // 寻找充电桩中
    ASTRALL_SPORT_STATUS_PLUGGINGCHARGING = 0xB109,     // 已找到，正在上桩
    ASTRALL_SPORT_STATUS_ATCHARGING     = 0xB10A,       // 在充电桩上
    ASTRALL_SPORT_STATUS_EXITCHARGING   = 0xB10B,       // 退出充电桩中
    ASTRALL_SPORT_STATUS_CHARGING       = 0xB10C,       // 充电中
    ASTRALL_SPORT_STATUS_FIXEDSTANDING  = 0xB10D,       // 执行站立中
    ASTRALL_SPORT_STATUS_FIXEDDOWNING   = 0xB10E,       // 执行趴下中
    ASTRALL_SPORT_STATUS_GET_RIGHT      = 0xB1FF        // 摔倒自主恢复站立
};

enum AstrallAuth : uint16_t
{
    ASTRALL_AUTH_SDK                = 1U,           // SDK控制
    ASTRALL_AUTH_JOYSTICK           = 2U            // 遥控器控制
};

enum AstrallLightCmd : uint16_t
{
    ASTRALL_CMD_LIGHT_OPEN          = 1U,           // 开灯
    ASTRALL_CMD_LIGHT_CLOSE         = 2U            // 关灯
};

enum AstrallSubscribeTopicId : uint16_t
{
    ASTRALL_SUB_TOPIC_ID_IMU        = 0x0001U,    // 订阅IMU数据
    ASTRALL_SUB_TOPIC_ID_SPORT      = 0x0002U,    // 订阅运动数据
    ASTRALL_SUB_TOPIC_ID_CAMERA_RGB = 0x0003U,    // 摄像头 RGB 数据
    ASTRALL_SUB_TOPIC_ID_JOYSTICK   = 0x0010U,    // 摇杆数据
};

enum AstrallSubscribeFreq : uint16_t
{
    ASTRALL_SUB_FREQ_CLOSE          = 0U,
    ASTRALL_SUB_FREQ_1HZ,
    ASTRALL_SUB_FREQ_25HZ,
    ASTRALL_SUB_FREQ_50HZ,
    ASTRALL_SUB_FREQ_125HZ,
    ASTRALL_SUB_FREQ_250HZ
};

struct AstrallConfig 
{
    // 心跳回调函数
    // data: NULL
    std::function<void(void *data, uint16_t len)> heartbeatCb;
    // SDK 状态回调函数
    // data: 见结构体 AstrallSdkStatus
    std::function<void(void *data, uint16_t len)> sdkStatusCb;
};

struct AstrallSdkStatus {
    uint8_t link : 1;          // 是否已连接
    uint8_t ctrlAuthority : 1; // 是否拥有控制权
    uint8_t reserve : 6;       // 保留位

    AstrallSdkStatus()
    {
        link = 0;
        ctrlAuthority = 0;
        reserve  = 0;
    }
};

struct AstrallDeviceInfo
{
    std::string version;          // 版本号
    std::string serialNumber;     // SN
    uint32_t model;               // 型号
    
    AstrallDeviceInfo()
    {
        version = "";
        serialNumber = "";
	model = 0;
    }
};

struct AstrallSystemStatus
{
    AstrallSystemCode sysStatus;       // 系统状态字
    AstrallErrorCode errorCode;        // 错误码
    AstrallWarnCode warnCode;          // 警告码

    AstrallSystemStatus()
    {
        sysStatus = AstrallSystemCode::ASTRALL_SYSTEM_CODE_INIT;
        errorCode = AstrallErrorCode::ASTRALL_ERR_NONE;
        warnCode = AstrallWarnCode::ASTRALL_WARN_NONE;
    }
};

struct AstrallPowerStatus
{
    float soc;                  // 剩余电量: %
    float temp;                 // 电池温度
    float voltage;              // 电池电压
    uint16_t cycleCount;        // 循环充电次数
    uint16_t charged;           // 充电状态
    
    AstrallPowerStatus()
    {
        soc = 0.0f;
        temp = 0.0f;
        voltage = 0.0f;
        cycleCount = 0;
        charged = 0;
    }
};

struct AstrallImuData
{
    long long timestamp;        // IMU时间戳
    float accelerometer[3];     // 加速度：m/(s*s)
    float gyroscope[3];         // 角速度：rad/s
    float quaternion[4];        // 四元数
    float pitch;                // 俯仰角：rad
    float roll;                 // 滚转角：rad
    float yaw;                  // 航向角：rad
    float odomX;                // 里程计
    float odomY;

    AstrallImuData()
    {
        timestamp = 0;
        memset(accelerometer, 0, sizeof(accelerometer));
        memset(gyroscope, 0, sizeof(gyroscope));
        memset(quaternion, 0, sizeof(quaternion));
        pitch = 0.0f;
        yaw = 0.0f;
        roll = 0.0f;
        odomX = 0.0f;
        odomY = 0.0f;
    }
};

struct AstrallSportData
{
    long long timestamp;
    float wheelSpeed[4];

    AstrallSportData()
    {
        timestamp = 0;
        memset(wheelSpeed, 0, sizeof(wheelSpeed));
    }
};

struct AstrallJoystickData
{
    long long timestamp;
    int16_t rocker[6];
    uint16_t l1 : 1;
    uint16_t l2 : 1;
    uint16_t l3 : 1;
    uint16_t r1 : 1;
    uint16_t r2 : 1;
    uint16_t r3 : 1;
    uint16_t sl : 1;
    uint16_t sr : 1;
    uint16_t h : 1;
    uint16_t sw : 2;
    uint16_t reserve : 5;
};

uint16_t AstrallSdkInit(AstrallConfig& cfg, uint32_t timeout = 60000);
void AstrallSdkDeinit();
uint16_t AstrallHeartbeat(uint32_t timeout = 20);
uint16_t AstrallGetSystemStatus(AstrallSystemStatus& status);
uint16_t AstrallGetPowerStatus(AstrallPowerStatus& power);
uint16_t AstrallGetSportStatus();
uint16_t AstrallGetDeviceInfo(AstrallDeviceInfo& info, uint32_t timeout = 20);
uint16_t AstrallGetImuData(AstrallImuData& data);
uint16_t AstrallRegisterLoadTask(std::function<void(void *data, uint16_t len)> cb);
uint16_t AstrallSubscriptionData(AstrallSubscribeTopicId topicId,
                                 AstrallSubscribeFreq freq,
                                 std::function<void(void *data, uint16_t len)> cb, 
                                 uint32_t timeout = 20);
inline uint16_t AstrallSubscriptionCameraRgb(AstrallSubscribeFreq freq,
                                             std::function<void(void *data, uint16_t len)> cb,
                                             uint32_t timeout = 20)
{
    if (freq != ASTRALL_SUB_FREQ_CLOSE)
    {
        uint16_t res = AstrallRegisterLoadTask(cb);
        if (res != ASTRALL_RES_SUCCESSED)
        {
            return res;
        }
    }

    return AstrallSubscriptionData(ASTRALL_SUB_TOPIC_ID_CAMERA_RGB, freq, cb, timeout);
}
uint16_t AstrallSportModeControl(AstrallSportModeCmd mode, uint32_t timeout = 20);
uint16_t AstrallAuthControl(AstrallAuth auth, uint32_t timeout = 20);
uint16_t AstrallMove(float vx, float vy, float vyaw, uint32_t timeout = 20);
uint16_t AstrallLightControl(AstrallLightCmd cmd, uint32_t timeout = 20);
uint16_t AstrallSendMessage(char *data, uint16_t len);

#endif /* __ASTRALL_SDK_INTERFACE_H__ */

