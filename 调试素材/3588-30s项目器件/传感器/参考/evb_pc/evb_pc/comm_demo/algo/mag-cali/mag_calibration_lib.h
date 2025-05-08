#ifndef MAG_CALI_LIB_H
#define MAG_CALI_LIB_H

#define ANDROID_P

struct magCaliDataOutPut {
    int64_t timeStamp;
    float x, y, z;
    float x_bias, y_bias, z_bias;
    int8_t status;
};
struct magCaliDataInPut {
    int64_t timeStamp;
    float x, y, z;
    int32_t status;
};
struct magChipInfo {
    int32_t layout;
    int32_t deviceid;
#ifdef ANDROID_P    
  	int8_t hwGyro;
#endif
};
struct mag_lib_interface_t {
    const char *module;
    int (*initLib)(struct magChipInfo *info);
#ifdef ANDROID_P
  	int (*enableLib)(int en);
#endif
    int (*caliApiGetOffset)(float offset[3]);
    int (*caliApiSetOffset)(float offset[3]);
    int (*caliApiSetGyroData)(struct magCaliDataInPut *inputData);
    int (*caliApiSetAccData)(struct magCaliDataInPut *inputData);
    int (*caliApiSetMagData)(struct magCaliDataInPut *inputData);
    int (*doCaliApi)(struct magCaliDataInPut *inputData,
        struct magCaliDataOutPut *outputData);
    int (*getGravity)(struct magCaliDataOutPut *outputData);
    int (*getRotationVector)(struct magCaliDataOutPut *outputData);
    int (*getOrientation)(struct magCaliDataOutPut *outputData);
    int (*getLinearaccel)(struct magCaliDataOutPut *outputData);
    int (*getGameRotationVector)(struct magCaliDataOutPut *outputData);
    int (*getGeoMagnetic)(struct magCaliDataOutPut *outputData);
    int (*getVirtualGyro)(struct magCaliDataOutPut *outputData);
};

#endif
