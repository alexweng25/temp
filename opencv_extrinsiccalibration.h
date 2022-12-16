#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/affine.hpp>

#define APP_MAX_FILE_PATH           (512u)
#define MAX_INPUT_CAMERAS           4U
#define SRV_CALMAT_SIZE             (12U * 4U)
#define R_SHIFT 30
#define t_SHIFT 10
#define LDC_MAX_NUM_CAMERAS 6
#define LUT_TABLE_LEN 1024 
#define LDC_U2D_TABLE_MAX_LENGTH LUT_TABLE_LEN
#define LDC_D2U_TABLE_MAX_LENGTH LUT_TABLE_LEN
using namespace cv;
using namespace std;

typedef struct  {
   int32_t  numCameras;   /* 4 Bytes for number of Cameras */
   int32_t  calMatSize[MAX_INPUT_CAMERAS]; /* 4 Btyes /calmat size          */
   int32_t  calMatBuf [48 *MAX_INPUT_CAMERAS]; //Cross check 
}  svCalmat_t;

typedef struct
{
    int32_t ldcLUT_numCams;
    /**< Num of cameras */
    int32_t ldcLUT_distortionCenters[2*LDC_MAX_NUM_CAMERAS];
    /**< Num of Lens Distortion centres */
    float ldcLUT_focalLength;
    /**< Lens focal length */

    int32_t U2D_type;
    /**< Lens Undistort to Distort type (must match macro LDC_LIB_U2D_LUT_TYPE)*/
    int32_t ldcLUT_U2D_length;
    /**< Lens Undistort to Distort length */
    float ldcLUT_U2D_step;
    /**< Lens Undistort to Distort step */
    float ldcLUT_U2D_table[LDC_U2D_TABLE_MAX_LENGTH];
    /**< Lens Undistort to Distort table */

    int32_t D2U_type;
    /**< Lens Distort to Undistort type (must match macro LDC_LIB_D2U_LUT_TYPE)*/
    int32_t ldcLUT_D2U_length;
    /**< Lens Distort to Undistort length */
    float ldcLUT_D2U_step;
    /**< Lens Distort to Undistort step */
    float ldcLUT_D2U_table[LDC_D2U_TABLE_MAX_LENGTH];
    /**< Lens Distort to Undistort table */
}ldc_lensParameters;

typedef struct {
    Mat image;
    bool drawflag;
    float scale;
    Point tempPoint;
    vector<cv::Point2f> corner;
}MouseParameters;

Mat GetRotationMatrix(Vec3f theta, string rotationorder, bool isDegrees);
Mat eulerAnglesToRotationMatrix(Vec3f theta, bool isDegrees);
Vec3f rotationMatrixToEulerAngles(Mat &R);
void EstimateCameraParameters(string path,string mainview, string sideview,
                              cv::Vec3f &position, cv::Vec3f &theta, int mapindex);

void WriteData(const char* filename, vector<std::string> &Data);
void Construct_calmat(string path, vector<Affine3d> all_Rt);
void Write_calmat_file(const char*fileName, vector<Mat> calmat_mtx);
void StoreExtrinsic(string filename, Mat rotation_mtx, Mat translation);
void DistortedToUndistorted(Vec4d k, TermCriteria criteria, vector<double> &lut);
void UndistortedToDistorted(Vec4d k, vector<double> &lut);
void Locator(int event, int x, int y, int flags, void* userdata);
void Read_lut_file(ldc_lensParameters*, const char* );



