/*  */#ifndef MET_SOC_TOP_STATUS_H_
#define MET_SOC_TOP_STATUS_H_


/*
 * topic: Monitor/top_status
 */

#include <stdint.h>

#define MET_SOC_TOP_STATUS_VERSION 3

#define MET_SOC_TOP_PROCESS_MAX_NUM 64
#define MET_SOC_TOP_MAX_CPU_NUM 8
#define MET_SOC_TOP_PROCESS_NAME_LENGTH 16

typedef struct met_soc_top_process_info
{
	char name[MET_SOC_TOP_PROCESS_NAME_LENGTH]; /*process name string*/
	float cpu;                                  /*process cpu total loading: 0.0 ~ 1.0*/
	float ddr;                                  /*process ddr memRss: KB*/
}met_soc_top_process_info;

typedef struct MET_SOC_TOPStatus
{
	float total_cpu_loading;                                             /*system cpu total loading: 0.0 ~ 1.0*/
	
	float npu_loading;                                                   /*system npu total loading: 0.0 ~ 1.0*/

	int cpu_num;                                                         /*system cpu number: 1 ~ 64*/
	float cpu_loading[MET_SOC_TOP_MAX_CPU_NUM];                          /*system per-cpu loading: 0.0 ~ 1.0*/
	
	float ddr_bw;                                                        /*system ddr bandwidth percentage: 0.0 ~ 1.0*/

	int process_num;                                                     /*process number*/
	met_soc_top_process_info process_info[MET_SOC_TOP_PROCESS_MAX_NUM];  /*process information*/

}MET_SOC_TOPStatus;

#endif
