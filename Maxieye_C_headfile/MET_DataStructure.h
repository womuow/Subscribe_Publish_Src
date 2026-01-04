/************************************************************************/
/*
Date:			2015/12/21
Author:			Chunlan Ye
Compile Mode:	C
Function:		basic data structure define
*/
/************************************************************************/

#ifndef MET_DATASTRUCTURE_H
#define MET_DATASTRUCTURE_H

#define MET_MAX_ROI_NUM 10

typedef struct METSize
{
	int16_t	w;
	int16_t	h;
}METSize;

typedef struct METPoint		/*unit: pixel*/
{
	int16_t	x;				/*horizontal coordinate*/
	int16_t	y;				/*vertical coordinate*/
}METPoint;

typedef struct METPoint3D		/*unit: km,m,cm etc */
{
	int16_t	x;
	int16_t	y;
	int16_t   z;
}METPoint3D;

typedef struct METPoint2D32f
{
	float x;
	float y;
}METPoint2D32f;

typedef struct METPoint3D32f
{
	float x;
	float y;
	float z;
}METPoint3D32f;

typedef struct METRect		/*unit: pixel*/
{
	int16_t	x;				/*horizontal coordinate*/
	int16_t	y;				/*vertical coordinate*/
	int16_t	width;			/*width*/
	int16_t	height;			/*height*/
}METRect;

typedef struct METRect32f		/*unit: pixel*/
{
	float	x;				/*horizontal coordinate*/
	float	y;				/*vertical coordinate*/
	float	width;			/*width*/
	float	height;			/*height*/
}METRect32f;

typedef struct METLowPassFilter //IIR
{
	char		initFlag;
	int32_t			dim;
	float		*NL;
	float		*DL;
	int32_t			cnt;
	float		*src;
	float		*des;
	float		inputData;
}METLowPassFilter;


typedef struct {
    /* camera install */
    float position_x;
    float position_y;
    float position_z;
    float yaw;
    float roll;
    float pitch;

    /* pose */
    float cam2Body[4][4];
    float body2Cam[4][4];
} CamExtrinsic;

typedef struct {
    /* Common */
    float pixelPitch_X;
    float pixelPitch_Y;
    float focalLength;
    float focalLength_X;
    float focalLength_Y;
    float undistort_focalLength;
	float undistort_focalLength_X;
	float undistort_focalLength_Y;

    /* distort image info */
    int16_t distort_imgWidth;
    int16_t distort_imgHeight;
    float distort_principalPoint_X;
    float distort_principalPoint_Y;
    float distort_camFov_H;
    float distort_camFov_V;

    /* undistort image info */
    int16_t undistort_imgWidth;
    int16_t undistort_imgHeight;
    float undistort_principalPoint_X;
    float undistort_principalPoint_Y;
    float undistort_camFov_H;
    float undistort_camFov_V;
} CamIntrinsic;

typedef struct {
    float Cint[16];
    float CintInv[16];
    float Rrot[16];
    float RrotInv[16];
    float Ttrs[16];
    float TtrsInv[16];
    float K[16];
    float Kinv[16];
    float H[9];
    float Hinv[9];
} MET_CamParams;

typedef struct METCamConfig
{
	int32_t			imgWidth;		//source image size: SD(1024, 512), HD(1280, 1080)
	int32_t			imgHeight;
	float		flength;
	float		pixel_pitch_x;
	float		pixel_pitch_y;
	float		princ_x;
	float		princ_y;
	float		k0;
	float		k1;
	float		k2;
	float		tangential_distortion_1;
	float		tangential_distortion_2;
	float		cam_pos_x;		/*unit: m*/
	float		cam_pos_y;
	float		cam_pos_z;
	float		roll;			/*unit: radian, save as degree*/
	float		yaw;
	float		pitch;

	int32_t			imgHD_offsetY;	/*unit: pixel*/

	float		hFov;	/*unit: degree*/
	float		vFov;

	float		cam_cx_compensate;
	float		cam_cy_compensate;

	unsigned char cam_real2ref_valid;	/*default: 0*/
	float		cam_real2ref_coef1;
	float		cam_real2ref_coef2;
	float		cam_real2ref_coef3;
	float		cam_real2ref_coef4;
	float		cam_real2ref_coef5;
	float		cam_real2ref_coef6;

	unsigned char cam_ref2real_valid;	/*default: 0*/
	float		cam_ref2real_coef1;
	float		cam_ref2real_coef2;
	float		cam_ref2real_coef3;
	float		cam_ref2real_coef4;
	float		cam_ref2real_coef5;
	float		cam_ref2real_coef6;
}METCamConfig;


typedef struct {
    float wxmin;
    float wxmax;
    float wymin;
    float wymax;
    float wzmin;
    float wzmax;
    float resolutionX;
    float resolutionY;
    float resolutionZ;
} BEVGridParams;



#endif
