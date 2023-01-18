LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS) 

LOCAL_MODULE := ureg_test

LOCAL_SHARED_LIBRARIES := libc

LOCAL_SRC_FILES += \
		ureg_test.c	

include $(BUILD_EXECUTABLE)		
