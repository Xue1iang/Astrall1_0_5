#include <stdio.h>
#include "src/keyboard.h"
#include "lib/include/interface.h"
#include <time.h>
#include <string.h>


float vx = 0.0f, vy = 0.0f, vyaw = 0.0f;

void keyboardHandle(char c)
{
    uint16_t res = 0;
    float tmp_vx = vx, tmp_vy = vy, tmp_vyaw = vyaw;
    printf("keyboard: %c\r\n", c);
    switch (c)
    {
    case '1':
	res = AstrallSportModeControl(ASTRALL_SPORT_MODE_CMD_DAMPING);
	printf("Switch sport mode(DAMPING) res:0x%04X\r\n", res);
	break;
    case '2':
        res = AstrallSportModeControl(ASTRALL_SPORT_MODE_CMD_FIXEDSTAND);
	printf("Switch sport mode(FIXEDSTAND) res:0x%04X\r\n", res);
	break;
    case '3':
        res = AstrallSportModeControl(ASTRALL_SPORT_MODE_CMD_FIXEDDOWN);
	printf("Switch sport mode(FIXEDDOWN) res:0x%04X\r\n", res);
	break;
    case '4':
        res = AstrallSportModeControl(ASTRALL_SPORT_MODE_CMD_MOVE);
	printf("Switch sport mode(MOVE) res:0x%04X\r\n", res);
	break;
    case '5':
        res = AstrallSportModeControl(ASTRALL_SPORT_MODE_CMD_AUTOCHARGE);
        printf("Switch sport mode(AUTOCHARGE) res:0x%04X\r\n", res);
        break;
    case '6':
        res = AstrallSportModeControl(ASTRALL_SPORT_MODE_CMD_EXITCHARGE);
        printf("Switch sport mode(EXITCHARGE) res:0x%04X\r\n", res);
        break;
    case '9':
        res = AstrallAuthControl(ASTRALL_AUTH_SDK);
	printf("Switch auth to SDK,res:0x%04X\r\n", res);
	break;
    case '0':
        vx = 0;
	vy = 0;
	vyaw = 0;
	break;
    case 'w':
	if (AstrallGetSportStatus() != ASTRALL_SPORT_STATUS_MOVE)
	    return;
	tmp_vx += 0.10f;
	if (tmp_vx >= 1.0f)
	    tmp_vx = 1.0f;
        vx = tmp_vx;
	break;
    case 's':
	if (AstrallGetSportStatus() != ASTRALL_SPORT_STATUS_MOVE)
            return;
	tmp_vx -= 0.10f;
	if (tmp_vx <= -1.0f)
	    tmp_vx = -1.0f;
        vx = tmp_vx;
	break;
    case 'a':
	if (AstrallGetSportStatus() != ASTRALL_SPORT_STATUS_MOVE)
            return;
	tmp_vy += 0.10f;
	if (tmp_vy >= 1.0f)
	    tmp_vy = 1.0f;
        vy = tmp_vy;
	break;
    case 'd':
	if (AstrallGetSportStatus() != ASTRALL_SPORT_STATUS_MOVE)
            return;
	tmp_vy -= 0.10f;
	if (tmp_vy <= -1.0f)
	    tmp_vy = -1.0f;
        vy = tmp_vy;
	break;
    case 'q':
	if (AstrallGetSportStatus() != ASTRALL_SPORT_STATUS_MOVE)
            return;
	tmp_vyaw += 0.10f;
        if (tmp_vyaw >= 1.0f)
	    tmp_vyaw = 1.0f;
	vyaw = tmp_vyaw;
	break;
    case 'e':
	if (AstrallGetSportStatus() != ASTRALL_SPORT_STATUS_MOVE)
            return;
	tmp_vyaw -= 0.10f;
	if (tmp_vyaw <= -1.0f)
	    tmp_vyaw = -1.0f;
	vyaw = tmp_vyaw;
	break;
    case 'o':
        res = AstrallLightControl(ASTRALL_CMD_LIGHT_CLOSE);
	printf("Close light:0x%04X\r\n", res);
	break;
    case 'n':
	res = AstrallLightControl(ASTRALL_CMD_LIGHT_OPEN);
        printf("Open light:0x%04X\r\n", res);
	break;
    default:
	return;
    }
}

void heartbeatHandle(void *data, uint16_t len)
{
    printf("Heartbeat\r\n");
}

void sdkStatusHandle(void *data, uint16_t len)
{
    struct AstrallSdkStatus *sdk = (struct AstrallSdkStatus *)data;
    printf("SDK status,link:%d ctrl:%d\r\n", sdk->link, sdk->ctrlAuthority);
}

void imuDataHandle(void *data, uint16_t len)
{
    if (len == sizeof(struct AstrallImuData))
    {
        struct AstrallImuData *imu = (struct AstrallImuData *)data;
        printf("IMU,time:%llu pitch:%.1f roll:%.1f yaw:%.1f\r\n", imu->timestamp, imu->pitch, imu->roll, imu->yaw);
    }
}

void sportDataHandle(void *data, uint16_t len)
{
    if (len == sizeof(struct AstrallSportData))
    {
        struct AstrallSportData *sport = (struct AstrallSportData *)data;
	printf("Sport,time:%llu fl:%.1f fr:%.1f bl:%.1f br:%.1f\r\n", sport->timestamp, sport->wheelSpeed[0], sport->wheelSpeed[1], sport->wheelSpeed[2], sport->wheelSpeed[3]);
    }
}

void joystickDataHandle(void *data, uint16_t len)
{
    if (len == sizeof(struct AstrallJoystickData))
    {
        struct AstrallJoystickData *js = (struct AstrallJoystickData *)data;
	printf("Joystick,time:%llu rc0:%d rc1:%d rc2:%d rc3:%d w0:%d w1:%d l1:%u l2:%u l3:%u r1:%u r2:%u r3:%u sl:%u sr:%u h:%u sw:%u\r\n", js->timestamp, js->rocker[0], js->rocker[1], js->rocker[2], js->rocker[3], js->rocker[4], js->rocker[5], js->l1, js->l2, js->l3, js->r1, js->r2, js->r3, js->sl, js->sr, js->h, js->sw);
    }
}

void cameraRgbDataHandle(void *data, uint16_t len)
{
    static uint32_t frameCnt = 0;
    const uint8_t *rgb = (const uint8_t *)data;

    frameCnt++;
    if (rgb != NULL && len >= 3)
    {
        printf("Camera RGB,frame:%u len:%u first_pixel:%u,%u,%u\r\n", frameCnt, len, rgb[0], rgb[1], rgb[2]);
    }
    else
    {
        printf("Camera RGB,frame:%u len:%u\r\n", frameCnt, len);
    }
}

int main(int argc, char **argv)
{
    int counter = 0, errCnt = 0;
    uint16_t res = 0;
    bool subscribeCamera = false;
    AstrallSystemStatus status;
    AstrallPowerStatus pwrSta;
    AstrallImuData imuData;
    AstrallConfig sdkCfg;
    AstrallDeviceInfo devInfo;
    KeyBoard kb;
    kb.registerEventCb(keyboardHandle);

    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--camera") == 0 || strcmp(argv[i], "-c") == 0)
        {
            subscribeCamera = true;
        }
    }
    
    sdkCfg.heartbeatCb = heartbeatHandle;
    sdkCfg.sdkStatusCb = sdkStatusHandle;

    if(AstrallSdkInit(sdkCfg) != ASTRALL_RES_SUCCESSED)
    {
        printf("SDK initialization failed\n");
        return -1;
    }

    res = AstrallSubscriptionData(ASTRALL_SUB_TOPIC_ID_IMU, ASTRALL_SUB_FREQ_250HZ, imuDataHandle);
    printf("Subscription sensor data:0x%04X\r\n", res);
    res = AstrallSubscriptionData(ASTRALL_SUB_TOPIC_ID_SPORT, ASTRALL_SUB_FREQ_250HZ, sportDataHandle);
    printf("Subscription sport data:0x%04X\r\n", res);
    // res = AstrallSubscriptionData(ASTRALL_SUB_TOPIC_ID_JOYSTICK, ASTRALL_SUB_FREQ_50HZ, joystickDataHandle);
    // printf("Subscription joystick data:0x%04X\r\n", res);
    if (subscribeCamera)
    {
        res = AstrallSubscriptionCameraRgb(ASTRALL_SUB_FREQ_25HZ, cameraRgbDataHandle);
        printf("Subscription camera RGB data:0x%04X\r\n", res);
    }
    while (1)
    {
	counter++;
	if (counter % 5 == 0)
	{
	    res = AstrallHeartbeat();
	    printf("Heartbeat,res:0x%04X\r\n", res);
	    devInfo = AstrallDeviceInfo();
	    res = AstrallGetDeviceInfo(devInfo);
	    printf("Device info,res: 0x%04X ver:%s sn:%s\r\n", res, devInfo.version.c_str(), devInfo.serialNumber.c_str());
	    status = AstrallSystemStatus();
	    res = AstrallGetSystemStatus(status);
	    printf("System status,res:0x%04X sta:%u err:0x%04X warn:0x%04X\r\n", res, (uint16_t)status.sysStatus, status.errorCode, status.warnCode);
            res = AstrallGetSportStatus();
	    printf("Sport status,res:0x%04X\r\n", res);
	    pwrSta = AstrallPowerStatus();
	    res = AstrallGetPowerStatus(pwrSta);
	    printf("Bat status,res:0x%04X soc:%.1f temp:%.1f vol:%.1f chg:%u cc:%u\r\n", res, pwrSta.soc, pwrSta.temp, pwrSta.voltage, pwrSta.charged, pwrSta.cycleCount);   
	}
	if (counter % 15 == 0)
	{
	    char msg[32] = {0};
	    uint16_t msgLen = sprintf(msg, "Counter: %u", counter);
	    res = AstrallSendMessage(msg, msgLen);
	    printf("Send Message,res:0x%04X\r\n", res);
	}
	if (AstrallGetSportStatus() == ASTRALL_SPORT_STATUS_MOVE)
	{
	    res = AstrallMove(vx, vy, vyaw);
	    printf("Move,res:0x%04X x:%.1f y:%.1f yaw:%.1f\r\n", res, vx, vy, vyaw); 
	}
        usleep(20000);
    }

    AstrallSdkDeinit();
    return 0;
}
