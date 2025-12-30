#ifndef MET_SOC_VEHINFO_H_
#define MET_SOC_VEHINFO_H_

#include <stdint.h>


/*****************************************************************************************************************************
**                                                  Revision Control History                                                                   **
*****************************************************************************************************************************/
/*  <VERSION>       <DATE>        <AUTHOR>                 <REVISION LOG>                                          */
/*  version:  1          2024-03-21     SH                              Initial version                                                  */

/****************************************************************************************************************************/

#define MET_VEHINFO_HEADFILE_VERSION (1u)

typedef struct MET_Validity_T
{
  uint8_t vehicleLatAccelValidity;
  uint8_t vehicleLongAccelValidity;
  uint8_t abs_active;
  uint8_t vehicleVerticalAccelValidity;
  uint8_t steeringWheelAngleValidity;
  uint8_t accelPedPosPctValidity;
  uint8_t RR_WheelRotatedDirection;  
  uint8_t RR_WheelRotatedDirectionValid;
} MET_Validity_T;
/* Process by Structure Alignment */


typedef struct  MET_mwrb_T
{
  uint8_t mainBeamIndication;
  uint8_t wiperFrontCmd;
  uint8_t reverseGear;
  uint8_t brakePedalPressed;
} MET_mwrb_T;

typedef struct MET_VEHINFO_TO_VSE
{
  MET_Validity_T  met_Validity;
  MET_mwrb_T      met_mwrb;
  float vehicleLatAccel;
  float vehicleLongAccel;
  uint32_t YawRateTimeStamp_ms;
  float vehicleVelocity;
  float vehicleVerticalAccel;
  float vehicleYawRate;
  float steeringWheelAngle;
  float accelPedPosPct;
  float LF_WheelSpd;  /* range -20 ~ 100 , unit: m/s*/
  float RF_WheelSpd;  /* range -20 ~ 100 , unit: m/s*/
  float LR_WheelSpd;  /* range -20 ~ 100 , unit: m/s*/
  float RR_WheelSpd;  /* range -20 ~ 100 , unit: m/s*/
  uint32_t mcu_pts_value_high;
  uint32_t mcu_pts_value_low;
  uint32_t vehIndex;
  uint32_t odometer;
  uint16_t countryIdentification;
  uint8_t LF_WheelRotatedDirection;
  uint8_t LF_WheelRotatedDirectionValid;
  uint8_t RF_WheelRotatedDirection;
  uint8_t RF_WheelRotatedDirectionValid;
  uint16_t Calendar_Year;
  uint16_t crc_16;
  uint8_t  met_turnIndicator;
  uint8_t  met_wiperSpeedInfo;
  uint8_t  LR_WheelRotatedDirection;
  uint8_t  LR_WheelRotatedDirectionValid;
  uint8_t  apiVersionMajor;
  uint8_t  apiVersionMinor;
  uint8_t  currentGear;
  uint8_t  fcwWarningSensitivityLevel;
  uint8_t  vehicleVelocityValidity;
  uint8_t  WheelSlipEvent;
  uint8_t  vehicleYawRateValidity;
  uint8_t  Calendar_Month;
  uint8_t  Calendar_Day;
  uint8_t  Hours;
  uint8_t  Minutes;
  uint8_t  Seconds;
  uint8_t  kl15_state;
  uint8_t  brakePedalPosPctValidity;
  uint8_t  LF_WheelSpdValidity;    /* 0: invalid , 1:valid*/
  uint8_t  RF_WheelSpdValidity;    /* 0: invalid , 1:valid*/
  uint8_t  LR_WheelSpdValidity;    /* 0: invalid , 1:valid*/
  uint8_t  RR_WheelSpdValidity;    /* 0: invalid , 1:valid*/
} MET_VEHINFO_TO_VSE;



//gearInfo
typedef enum METGearInfo
{
	MET_GEAR_UNKNOWN = -3,
	MET_GEAR_STOP,
	MET_GEAR_BACKUP,
	MET_GEAR_NEUTRAL,
	MET_GEAR_ADVANCE,
	MET_GEAR_LEVEL1,
	MET_GEAR_LEVEL2,
	MET_GEAR_LEVEL3,
	MET_GEAR_LEVEL4,
	MET_GEAR_LEVEL5,
	MET_GEAR_LEVEL6,
	MET_GEAR_LEVEL7,
	MET_GEAR_LEVEL8,
	MET_GEAR_LEVEL9,
	MET_GEAR_LEVEL10
}METGearInfo;

typedef struct METVehicleGPSInfo
{
	int32_t	  longitude;
	int32_t   latitude;
	float	  height;
}METVehicleGPSInfo;


typedef struct METVehicleInfo
{
	float			vehicle_speed;					/*speed: 0~500km/h, accuracy: 0.1km/h*/
	float			steering_wheel_angle;			/*steering wheel angle: -1000~1000 degree, accuracy: 0.1 degree*/

	float			accelerator_pedal_pos;			/*accelerator pedal position: 0~100%, accuracy: 0.1%*/
	short			accelerator_pedal_speed;		/*val:-5120~5080, unit:%/s*/
	float			brake_pedal_pos;				/*brake pedal position: 0~100%, accuracy: 0.1%*/

	float			vehicle_yaw_angle;				/*vehicle yaw angle: 0~360 degree, accuracy: 0.1 degree*/
	float			vehicle_yaw_rate;				/*vehicle yaw rate: -255~255 degree/s, accuracy: 0.1 degree/s*/

	float			vehicle_pitch_angle;			/*vehicle pitch angle: -45~45 degree, accuracy: 0.01 degree*/
	float			vehicle_pitch_rate;				 /*vehicle pitch rate: -3000~3000 degree/s, accuracy: 0.1 degree/s*/

	unsigned char	Lft_turnlight_active;			    /*left turn light active tag, 1-yes, 0-no*/
	unsigned char	Rght_turnlight_active;			/*right turn light active tag, 1-yes, 0-no*/

	unsigned char	windWiperStatus;				/*val:0-10, speed level*/
   
	short			        light_sensor_val;				/*light sensor value*/
	unsigned char	rain_sensor_val;				/*rain sensor value*/

	unsigned char	lowBeamStatus;					/*0-closed, 1-open*/
	unsigned char	highBeamStatus;					/*0-closed, 1-open*/

	unsigned char	envirLightStatus;				/*0-unknown, 1- night, 2-day*/
	
	short			        brakePedalGrd;					/*val:-5120~5080, unit:%/s*/
	unsigned char	brakePedalFlag;					/*val 0:no brake 1:brake */
	
	float			temperatureValue;				/*val:-40~87.5, unit: degree Celsius, accuracy: 0.5*/
	short			steeringWheelSpeed;				/*val:-2048~2057, unit:degree/s, accuracy: 1*/

	METGearInfo		gearInfo;						/*-3:unknown, -2:stop, -1:backup,0:neutral,1:advance,2:level1,
													3:level2, 4:level3, 5:level4, 6:level5, 7:level6, 8:level7, 9:level8,
													10:level9, 11:level10*/
	unsigned char	estimated_gear;
	int				    mileage;						/*0~1000000, unit:km*/
	
	float			vehicle_roll_angle;				/*vehicle roll angle: -45~45 degree, accuracy: 0.01 degree*/
	float			vehicel_roll_rate;				/*vehicle roll rate: -3000~3000 degree/s, accuracy: 0.1 degree/s*/

	METVehicleGPSInfo	gpsInfo;
	/************************************************************************/
	/* ACC AEB LKA Related                                                  */
	/************************************************************************/
	/// AEB //////////////////////////////////////////////////////////////////
	unsigned char    aeb_brake_failed;
	short			        engine_min_torque_value;
	short		            engine_max_torque_value;
	short		            engine_cur_torque_value;
	/// ACC //////////////////////////////////////////////////////////////////
	unsigned char	acc_brake_failed;
	unsigned char	cruise_control_enable;
	unsigned char	cruise_gap_switch_activation;
	unsigned char	cruise_secondary_switch_status;
	unsigned char	cruise_speed_limit_switch_status;
	/// LKA //////////////////////////////////////////////////////////////////
	unsigned char	lka_overlay_torque_status;
	float			        lka_steering_total_torque;
	unsigned char	lka_handoff_steering_wheel_mode;
	unsigned char	lka_handoff_steering_wheel_stat;
	float			lka_steering_delta_torque;
	float			lka_driver_applied_torque;
	/************************************************************************/
	/* Special Car:Vehicle Inertia Dynamic Parameters                       */
	/************************************************************************/
	float			lateral_acceleration_primary;
	float			lateral_acceleration_secondary;
	float			longitudinal_acceleration_primary;
	float			filter_longitudinal_acceleration_primary;
	float			longitudinal_acceleration_secondary;
	float			filter_yawrate_primary;
	float			yawrate_primary;
	float			yawrate_secondary;
	float			wheel_brake_pressure_estimated;
	float			braking_actual_deceleration;
	float			braking_target_deceleration;
	unsigned char	gateway_button;
	unsigned char	wheel_status_feback_zcsd;
}METVehicleInfo;



typedef struct MET_SOC_VehInfo
{
	uint32_t                 frameID;

	uint64_t                 timestamp;
 
	uint8_t                  Speed_Mode;  /* 1:can 2:opticalflow 3:gps 4:gps&opticalflow 5:smm */
  
	METVehicleInfo           VehInfo;

	MET_VEHINFO_TO_VSE       ToVseVehInfo;

}MET_SOC_VehInfo;

#endif
