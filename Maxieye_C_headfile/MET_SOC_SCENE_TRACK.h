/*
 *  version_idx:1
 *	topic: Trk/scene_track
 */

#ifndef MET_IPC_SCENE_TRACK_BUF_H_
#define MET_IPC_SCENE_TRACK_BUF_H_

#include <stdint.h>

/*
buf name scene_track
 */

 #define MET_SOC_SCENE_TRACK_VERSION 1

typedef struct MET_SOC_SceneTrackResult {
	unsigned int frameId;
	uint64_t	    picTimestamp;	  		// camera exposure complete time
	uint64_t	    resultTimestamp;  		// ST finished time
	int tunnel;                 //0:out, 1:in, 2:middle
	int dynt;                   //0:day, 1:night, 2:dark
	/*-------------------------------------------------------*\
	 | bit7 + bit6 | bit5 + bit4 | bit3 + bit2 | bit1 + bit0 |
	 | dayRain     | dayFog      | nightRain   | backLight   |
	 | 00--normal  | 00--normal  | 00--normal  | 00--normal  |
	 | 01--low     | 01--low     | 01--low     | 01--low     |
	 | 10--middle  | 10--middle  | 10--middle  | 10--middle  |
	 | 11--high    | 11--high    | 11--high    | 11--high    |
	 *-------------------------------------------------------*/
	unsigned char extremeScene;
	unsigned char reserved_u8;
} MET_SOC_SceneTrackResult;

#endif /* MET_IPC_Scene_Track_BUF */

/**********************************************************************************************************************
*  REVISION HISTORY
*  -------------------------------------------------------------------------------------------------------------------
*  Version   Date        Author  Require Id     Description
*  -------------------------------------------------------------------------------------------------------------------
*  01.00.00  2025-02-21  
**********************************************************************************************************************/
