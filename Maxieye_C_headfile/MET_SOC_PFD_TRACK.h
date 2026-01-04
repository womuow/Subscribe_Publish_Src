/*
* version : 1
* topic : Trk/pfd_track
*/

#ifndef MET_SOC_PFD_TRACK_BUF_H_
#define MET_SOC_PFD_TRACK_BUF_H_

#include <stdint.h>

/*********************************************************************************************************************
*  LOCAL CONSTANT MACROS
********************************************************************************************************************/
#define MET_SOC_PFD_TRACK_VERSION		1
#define MET_PFDTRK_OUTPUT_MAXNUM		128

/*********************************************************************************************************************
*  DATA STRUCTURE DEFINITIONS
********************************************************************************************************************/
typedef struct
{
	unsigned char    is_match;           // is pfd obj match with veh or vru
	unsigned char    is_plate_accurate;  // is car plate pos accurate
	unsigned char    is_face_accurate;   // is face pos accurate 
	int    IDobj_match;        // match obj id
	int    statusobj_match;    // match obj status
	float  xmin;               // pfd trkobj 2d info
	float  ymin;               // pfd trkobj 2d info
	float  xmax;               // pfd trkobj 2d info
	float  ymax;               // pfd trkobj 2d info
	float  xmin_raw;           // pfd trkobj 2d info in raw pic
	float  ymin_raw;               // pfd trkobj 2d info in raw pic
	float  xmax_raw;               // pfd trkobj 2d info in raw pic
	float  ymax_raw;               // pfd trkobj 2d info in raw pic

}MET_PFD_output_Element;

typedef struct MET_SOC_PFD_TrkRst
{
	unsigned int frameId;
	uint64_t	picTimestamp;	//camera exposure complete time
	uint64_t	resultTimestamp;//pfd finished time
	uint32_t	pfdTrkNum;
	MET_PFD_output_Element PfdTrkRstObj[MET_PFDTRK_OUTPUT_MAXNUM];
}MET_SOC_PFD_TrkRst;

#endif /* MET_SOC_PFD_TRACK_BUF_H_ */

/**********************************************************************************************************************
*  REVISION HISTORY
*  -------------------------------------------------------------------------------------------------------------------
*  Version   Date        Author  Require Id     Description
*  -------------------------------------------------------------------------------------------------------------------
*  01.00.00  2025-02-21  LJK     Update PFD result elements
*
*  -------------------------------------------------------------------------------------------------------------------
*
**********************************************************************************************************************/
