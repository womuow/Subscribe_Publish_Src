#ifndef _MET_SOC_CV_FAULT_H_
#define _MET_SOC_CV_FAULT_H_
#include <stdint.h>

#define MET_SOC_CV_FAULT_VERSION        (1)
#define MET_SOC_CV_FAULT_TOPIC          "CV/fault"
#define MET_SOC_CV_FAULT_FLAG_NUM       (10)

#define CV_FAULT_READ_INTRINSIC         (0)
#define CV_FAULT_GET_AE_INFO            (1)
#define CV_FAULT_GET_VIN                (2)
#define CV_FAULT_GET_IVPS               (3)
#define CV_FAULT_FORWARD                (4)
#define CV_FAULT_NPU_SELF_TEST          (5)
#define CV_FAULT_MAX                    (MET_SOC_CV_FAULT_FLAG_NUM * 8 - 1)


typedef struct met_soc_cv_fault {
    uint8_t flag[MET_SOC_CV_FAULT_FLAG_NUM] = {0};
} MET_SOC_CV_FAULT;

#endif
