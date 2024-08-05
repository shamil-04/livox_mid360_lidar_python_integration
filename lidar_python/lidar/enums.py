# lidar/enums.py
from enum import Enum

class LivoxLidarDeviceType(Enum):
    kLivoxLidarTypeHub = 0
    kLivoxLidarTypeMid40 = 1
    kLivoxLidarTypeTele = 2
    kLivoxLidarTypeHorizon = 3
    kLivoxLidarTypeMid70 = 6
    kLivoxLidarTypeAvia = 7
    kLivoxLidarTypeMid360 = 9
    kLivoxLidarTypeIndustrialHAP = 10
    kLivoxLidarTypeHAP = 15
    kLivoxLidarTypePA = 16

class ParamKeyName(Enum):
    kKeyPclDataType = 0x0000
    kKeyPatternMode = 0x0001
    kKeyDualEmitEn = 0x0002
    kKeyPointSendEn = 0x0003
    kKeyLidarIpCfg = 0x0004
    kKeyStateInfoHostIpCfg = 0x0005
    kKeyLidarPointDataHostIpCfg = 0x0006
    kKeyLidarImuHostIpCfg = 0x0007
    kKeyCtlHostIpCfg = 0x0008
    kKeyLogHostIpCfg = 0x0009
    kKeyVehicleSpeed = 0x0010
    kKeyEnvironmentTemp = 0x0011
    kKeyInstallAttitude = 0x0012
    kKeyBlindSpotSet = 0x0013
    kKeyFrameRate = 0x0014
    kKeyFovCfg0 = 0x0015
    kKeyFovCfg1 = 0x0016
    kKeyFovCfgEn = 0x0017
    kKeyDetectMode = 0x0018
    kKeyFuncIoCfg = 0x0019
    kKeyWorkModeAfterBoot = 0x0020
    kKeyWorkMode = 0x001A
    kKeyGlassHeat = 0x001B
    kKeyImuDataEn = 0x001C
    kKeyFusaEn = 0x001D
    kKeyForceHeatEn = 0x001E
    kKeyLogParamSet = 0x7FFF
    kKeySn = 0x8000
    kKeyProductInfo = 0x8001
    kKeyVersionApp = 0x8002
    kKeyVersionLoader = 0x8003
    kKeyVersionHardware = 0x8004
    kKeyMac = 0x8005
    kKeyCurWorkState = 0x8006
    kKeyCoreTemp = 0x8007
    kKeyPowerUpCnt = 0x8008
    kKeyLocalTimeNow = 0x8009
    kKeyLastSyncTime = 0x800A
    kKeyTimeOffset = 0x800B
    kKeyTimeSyncType = 0x800C
    kKeyStatusCode = 0x800D
    kKeyLidarDiagStatus = 0x800E
    kKeyLidarFlashStatus = 0x800F
    kKeyFwType = 0x8010
    kKeyHmsCode = 0x8011
    kKeyCurGlassHeatState = 0x8012
    kKeyRoiMode = 0xFFFE
    kKeyLidarDiagInfoQuery = 0xFFFF
