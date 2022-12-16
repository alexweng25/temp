#include "opencv_extrinsiccalibration.h"
#include "TI_ExtrinsicCalibration.h"
#include <map>
#include <iomanip>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
// #include <eigen3/Eigen/Dense>
// using namespace Eigen; 
// #include <opencv2/core/eigen.hpp>

#define TI
#define MANUALPOINT 1
#define showundistorted_image 0
#define showcorner_image 1
#define DEBUG 1
#define bStoredparameters 0
#define b20cmChessboard 1
//#define World_Y_shifted 200*11+200*4+35*2  // 11格, 4格邊界, 2段預留邊緣 unit:mm
#define World_Y_shifted 4748 // unit:mm
#define World_X_shifted 600 // unit:mm
#define MAXPATHLENGTH  512
#define FRONT_X_GAP 4348
#define SIDE_X_GAP 3200

const bool bDoCalibration = true;
/*--------- Setting of chessboard---------*/
#if b20cmChessboard
#define SQUARE_SIZE 200
int board_width = 4, board_height = 3;
cv::Size board_size = cv::Size(board_width, board_height);
#else
#define SQUARE_SIZE 40
int board_width = 8, board_height = 5;
cv::Size board_size = cv::Size(board_width, board_height);
#endif

// float square_size = 80;
// int board_width = 6, board_height = 4;
// cv::Size board_size = cv::Size(board_width, board_height);

/*--------- Setting of chessboard---------*/

const int iCarviews_count = MAX_INPUT_CAMERAS;
string sCarViews[iCarviews_count] = {"front", "right", "rear", "left"};
void CameraIntrinsicLUT(LensDistortionCorrection *ldc);
void TI_ExtrinsicProcess(svPoseEstimation_t *sv, tivxPoseEstimationParams *prms, LensDistortionCorrection *ldc);
void TransformPointsFormat(vector<Point2D_f> object_points, vector<cv::Point2f> img_Points, Point2D_f *chart_points, Point2D_f *corner_points);
void TransformExtrinsic(tivxPoseEstimationParams *prms, int camera_index, Matx33d &rotation_mtx, Matx31d &translation_vector);
void OpenCV_ExtrinsicProcess2();

int main(int argc, char *argv[])
{
  LensDistortionCorrection ldc[iCarviews_count];
  for(int i = 0 ; i < iCarviews_count; i++)
  {
    ldc[i].lenParas = new ldc_lensParameters;
  }
  tivxPoseEstimationParams *prms = new tivxPoseEstimationParams;
  svPoseEstimation_t *sv = new svPoseEstimation_t;

  CameraIntrinsicLUT(ldc);  
  if(!InitialParameters(ldc, prms))
  {
    cout << "Error: Initial TI pose parameters" << endl;
    return 0;
  }
  /* Create the relationship between real world and chessboard */

  /* TI algorithm */
  TI_ExtrinsicProcess(sv, prms, ldc);

  /* OpenCV algorithm */
  //OpenCV_ExtrinsicProcess2();

  return 0;
}

void EstimateCameraParameters(string path,string mainview, string sideview,
                              cv::Vec3f &cam_position, cv::Vec3f &theta, int mapindex)
{
  /* Load extrinsic parametes from file*/
  map<string, vector<Mat>> carViewsRvector;
  map<string, vector<Mat>> carViewsTvector;
  string pattern_orientation = "horizontal";
  // string pattern_orientation = "vertical";

  for (size_t i = 0; i < iCarviews_count; i++)
  {
    cv::FileStorage Extrinsic_file(path + sCarViews[i] + ".xml",
                                   cv::FileStorage::READ);
    if (Extrinsic_file.isOpened())
    {
      vector<Mat> vR_mat, vT_mat;
      Extrinsic_file["Rotation"] >> vR_mat;
      Extrinsic_file["Translation"] >> vT_mat;
      carViewsRvector.insert(
          std::pair<string, vector<Mat>>(sCarViews[i], vR_mat));
      carViewsTvector.insert(
          std::pair<string, vector<Mat>>(sCarViews[i], vT_mat));
    }
    Extrinsic_file.release();
  }

  Vec3f angle = Vec3f(0.0, 0.0, 0.0);
  Mat rotation_mtx = GetRotationMatrix(angle, "zyx", true);
  if(mainview == "front" && sideview == "")
  {
    // mainview = "rear";

    /* Find camera position in the world corrdinate */
    Mat mainviewR_vector, mainviewT_vector;
    Mat mainR_matrix;
    mainviewR_vector = carViewsRvector[mainview][mapindex];
    mainviewT_vector = carViewsTvector[mainview][mapindex];

    cv::Rodrigues(mainviewR_vector, mainR_matrix);
    Mat position = -mainR_matrix.t() * mainviewT_vector; 
    cam_position = position;

    // mainR_matrix =  rotation_mtx * mainR_matrix;
    mainR_matrix = mainR_matrix.t();
    //cout << "R:" << endl << mainR_matrix << endl;
    Vec3f eulerangles = rotationMatrixToEulerAngles(mainR_matrix);

    if(pattern_orientation == "horizontal")
      theta[0] = eulerangles[0] - 180;
    else
      theta[0] =  eulerangles[0];
    theta[1] = -eulerangles[1];
    theta[2] = -eulerangles[2];
    
    // Mat _R = eulerAnglesToRotationMatrix(theta, true);
    // StoreExtrinsic("./parameters/warperExtrinsic_"+ mainview +".xml", _R);
    // cout << "EularToR:" << endl << _R << endl;
    // Mat _t = -_R * cam_position;
    // Rt = Affine3d(_R, _t);

    #if DEBUG
    cout << "Front x y z: " << endl << cam_position << endl;
    cout << "Front euler angles: " << eulerangles << endl;
    cout << "theta:" << theta << endl;
    #endif
  }  

  /* Find new rotation and translation matrix for mainview in the world
   * corrdinate*/
  Mat mainfrontview_R_vector, main2sideview_R_vector;
  Mat sidefrontview_R_vector, side2frontview_R_vector,
      side2rearview_R_vector;
  Mat rearfrontview_R_vector, rear2sideview_R_vector;
  Mat main2side_R_matrix, side2front_R_matrix, sidefront_R_matrix;
  Mat side2rear_R_matrix, rear2side_R_matrix, rearfront_R_matrix;

  if (sideview == "left")
  {
    /* left view */
    Mat R_matrix, T_vector;
    Mat main2side_T_vector, side2front_T_vector;

    main2sideview_R_vector = carViewsRvector[mainview][1];
    side2frontview_R_vector = carViewsRvector["left"][2];
    // sidefrontview_R_vector = carViewsRvector["left"][0];
    main2side_T_vector = carViewsTvector[mainview][1];
    side2front_T_vector = carViewsTvector["left"][2];
    // Mat front_T_vector = carViewsTvector["left"][0];

    cv::Rodrigues(main2sideview_R_vector, main2side_R_matrix);
    cv::Rodrigues(side2frontview_R_vector, side2front_R_matrix);
    // cv::Rodrigues(sidefrontview_R_vector, sidefront_R_matrix);
    Mat front_postion = -main2side_R_matrix.t() * main2side_T_vector;
    Mat side_postion = -side2front_R_matrix.t() * side2front_T_vector;
    // cout << "front: " << front_postion << endl;
    // cout << "left : " << side_postion << endl;

    // Mat realfront_postion = -sidefront_R_matrix.t() * front_T_vector;
    // cout << "left front: " << realfront_postion << endl;
    Mat position = front_postion - side_postion;
    cam_position = position;

    // main2side_R_matrix = rotation_mtx * main2side_R_matrix;
    // side2front_R_matrix = rotation_mtx * side2front_R_matrix;
    // sidefront_R_matrix = rotation_mtx * sidefront_R_matrix;
    // Vec3f fronteulerangles = rotationMatrixToEulerAngles(sidefront_R_matrix);
    // cout << "left front angle: " << fronteulerangles << endl;

    R_matrix = side2front_R_matrix.t() * main2side_R_matrix;
    side2front_R_matrix = side2front_R_matrix.t();
    Vec3f eulerangles = rotationMatrixToEulerAngles(side2front_R_matrix);
    Vec3f leftview_eulerangles = rotationMatrixToEulerAngles(R_matrix);

    cam_position[2] = float(side_postion.at<double>(2,0));

    if(pattern_orientation == "horizontal")
      theta[0] = eulerangles[0] - 180;
    else
      theta[0] =  eulerangles[0];
    theta[1] = -eulerangles[1];
    theta[2] = -leftview_eulerangles[2];
    // float _tmp = eulerangles[0];
    // eulerangles[0] = -eulerangles[1];
    // eulerangles[1] = _tmp;

    // Mat _R = eulerAnglesToRotationMatrix(theta, true);
    // StoreExtrinsic("./parameters/warperExtrinsic_"+ sideview +".xml", _R);
    // Mat _t = -_R * cam_position;
    // Rt = Affine3d(_R, _t);

    #if DEBUG
    cout << "Left x y z: " << endl << cam_position << endl;
    cout << "Left front euler angles: " << eulerangles << endl;
    //cout << "Left yaw(z) angles: " << leftview_eulerangles[2] << endl;
    cout << "Left theta: " << theta << endl;
    #endif
  }
  else if (sideview == "right")
  {
    /* right view */
    Mat R_matrix, T_vector;
    Mat main2side_T_vector, side2front_T_vector;

    main2sideview_R_vector = carViewsRvector[mainview][2];
    side2frontview_R_vector = carViewsRvector["right"][1];
    sidefrontview_R_vector = carViewsRvector["right"][0];
    main2side_T_vector = carViewsTvector[mainview][2];
    side2front_T_vector = carViewsTvector["right"][1];
    Mat front_T_vector = carViewsTvector["right"][0];

    cv::Rodrigues(main2sideview_R_vector, main2side_R_matrix);
    cv::Rodrigues(side2frontview_R_vector, side2front_R_matrix);
    // cv::Rodrigues(sidefrontview_R_vector, sidefront_R_matrix);
 
    Mat front_postion = -main2side_R_matrix.t() * main2side_T_vector;
    Mat side_postion = -side2front_R_matrix.t() * side2front_T_vector;
    // cout << "front: " << front_postion << endl;
    // cout << "right : " << side_postion << endl;
    // Mat realfront_postion = -sidefront_R_matrix.t() * front_T_vector;
    // cout << "right front: " << realfront_postion << endl;
    //Rt = Affine3d(side2front_R_matrix, side2front_T_vector);
    Mat position = front_postion - side_postion;
    cam_position = position;
    
    // main2side_R_matrix =  rotation_mtx * main2side_R_matrix;
    // side2front_R_matrix =  rotation_mtx * side2front_R_matrix;   

    // sidefront_R_matrix = rotation_mtx * sidefront_R_matrix;  
    // Vec3f fronteulerangles = rotationMatrixToEulerAngles(sidefront_R_matrix);
    // cout << "right front angle: " << fronteulerangles << endl;

    R_matrix = side2front_R_matrix.t() * main2side_R_matrix;
    side2front_R_matrix = side2front_R_matrix.t();
    Vec3f eulerangles = rotationMatrixToEulerAngles(side2front_R_matrix);
    Vec3f rightview_eulerangles = rotationMatrixToEulerAngles(R_matrix);
    
    cam_position[2] = float(side_postion.at<double>(2,0));

    if(pattern_orientation == "horizontal")
      theta[0] = eulerangles[0] - 180;
    else
      theta[0] = eulerangles[0];  
    theta[1] = -eulerangles[1];
    theta[2] = -rightview_eulerangles[2];
    // float _tmp = eulerangles[0];
    // eulerangles[0] = eulerangles[1];
    // eulerangles[1] = -_tmp;
    // theta = eulerangles;   

    // Mat _R = eulerAnglesToRotationMatrix(theta, true);
    // StoreExtrinsic("./parameters/warperExtrinsic_"+ sideview +".xml", _R);
    // Mat _t = -_R * cam_position;
    // Rt = Affine3d(_R, _t);

    #if DEBUG
    cout << "Right x y z: " << endl << cam_position << endl;
    cout << "Right front euler angles: " << eulerangles << endl;
    // cout << "Right yaw(z) angles: " << rightview_eulerangles[2] << endl;
    cout << "Right theta: " << theta << endl;
    #endif
  }
  else if (sideview == "rear")
  {
    /* rear view */
    Mat R_matrix, T_vector;
    Mat main2side_T_vector, side2front_T_vector, rear2side_T_vector, side2rear_T_vector;

    main2sideview_R_vector = carViewsRvector[mainview][1];
    side2frontview_R_vector = carViewsRvector["left"][2];
    side2rearview_R_vector = carViewsRvector["left"][1];
    rear2sideview_R_vector = carViewsRvector["rear"][2];
    
    main2side_T_vector = carViewsTvector[mainview][1];
    side2front_T_vector = carViewsTvector["left"][2];
    side2rear_T_vector = carViewsTvector["left"][1];
    rear2side_T_vector = carViewsTvector["rear"][2];
    
    // rearfrontview_R_vector = carViewsRvector["rear"][2];
    // Mat rear_T_vector = carViewsTvector["rear"][2];
    
    main2sideview_R_vector = carViewsRvector[mainview][2];
    side2frontview_R_vector = carViewsRvector["right"][1];
    side2rearview_R_vector = carViewsRvector["right"][2];
    rear2sideview_R_vector = carViewsRvector["rear"][1];
  
    main2side_T_vector = carViewsTvector[mainview][2];
    side2front_T_vector = carViewsTvector["right"][1];
    side2rear_T_vector = carViewsTvector["right"][2];
    rear2side_T_vector = carViewsTvector["rear"][1];

    cv::Rodrigues(main2sideview_R_vector, main2side_R_matrix);
    cv::Rodrigues(side2frontview_R_vector, side2front_R_matrix);
    cv::Rodrigues(side2rearview_R_vector, side2rear_R_matrix);
    cv::Rodrigues(rear2sideview_R_vector, rear2side_R_matrix);
    // cv::Rodrigues(rearfrontview_R_vector, rearfront_R_matrix);

    Mat front2side_postion = -main2side_R_matrix.t() * main2side_T_vector;
    Mat side2front_postion = -side2front_R_matrix.t() * side2front_T_vector;
    Mat rear2side_postion = -rear2side_R_matrix.t() * rear2side_T_vector;
    Mat side2rear_postion = -side2rear_R_matrix.t() * side2rear_T_vector;
    // Mat rear_postion = -rearfront_R_matrix.t() * rear_T_vector;
    // cout << "rear position: " << rear_postion << endl;

    // cout << front2side_postion << endl;
    // cout << side2front_postion << endl;
    // cout << rear2side_postion << endl;
    // cout << side2rear_postion << endl;

    Mat position1 = cv::abs(rear2side_postion - side2rear_postion) - cv::abs(front2side_postion - side2front_postion);
    Mat position2 = (rear2side_postion - side2rear_postion) + (front2side_postion - side2front_postion);
    cam_position[0] = float(position1.at<double>(0,0));
    cam_position[1] = float(position2.at<double>(1,0));

    main2side_R_matrix = rotation_mtx * main2side_R_matrix;
    side2front_R_matrix = rotation_mtx * side2front_R_matrix;
    side2rear_R_matrix = rotation_mtx * side2rear_R_matrix;
    rear2side_R_matrix = rotation_mtx * rear2side_R_matrix;
    // rearfront_R_matrix = rotation_mtx * rearfront_R_matrix;

    R_matrix =  rear2side_R_matrix.t() * side2rear_R_matrix *
                side2front_R_matrix.t() * main2side_R_matrix; 
    
    rear2side_R_matrix = rear2side_R_matrix.t();
    Vec3f eulerangles = rotationMatrixToEulerAngles(rear2side_R_matrix);
    Vec3f rearview_eulerangles = rotationMatrixToEulerAngles(R_matrix);

    cam_position[2] = float(rear2side_postion.at<double>(2,0));
    if(pattern_orientation == "horizontal")
      theta[0] = eulerangles[0] - 180;
    else
      theta[0] = eulerangles[0];
    theta[1] = -eulerangles[1];
    theta[2] = -rearview_eulerangles[2];

    // Mat _R = eulerAnglesToRotationMatrix(theta, true);
    // StoreExtrinsic("./parameters/warperExtrinsic_"+ sideview +".xml", _R);

    // Mat _t = -_R * cam_position;
    // Rt = Affine3d(_R, _t);

    #if DEBUG
    cout << "Rear x y z: " << endl << cam_position << endl;
    cout << "Rear front euler angles: " << eulerangles << endl;
    // cout << "Rear yaw(z) angles: " << rearview_eulerangles[2] << endl;
    cout << "Rear theta: " << theta << endl;
    #endif
  }
}

Mat GetRotationMatrix(Vec3f theta, string rotationorder, bool isDegrees)
{
  if (isDegrees)
    theta = theta * CV_PI / 180.0;
    float theta90 = 90 * CV_PI / 180.0;
    
  Mat R_x1 = (Mat_<double>(3, 3) << 1, 0, 0, 0, cos(theta90), -sin(theta90), 0,
             sin(theta90), cos(theta90));

  /* Calculate rotation about x axis */
  Mat R_x = (Mat_<double>(3, 3) << 1, 0, 0, 0, cos(theta[0]), -sin(theta[0]), 0,
             sin(theta[0]), cos(theta[0]));

  /* Calculate rotation about y axis */
  Mat R_y = (Mat_<double>(3, 3) << cos(theta[1]), 0, sin(theta[1]), 0, 1, 0,
             -sin(theta[1]), 0, cos(theta[1]));

  /* Calculate rotation about z axis */
  Mat R_z = (Mat_<double>(3, 3) << cos(theta[2]), -sin(theta[2]), 0,
             sin(theta[2]), cos(theta[2]), 0, 0, 0, 1);

  /* Assemble to rotation matrix 内旋.右乘順序等於旋轉順序 */
  Mat R;
  if (rotationorder == "xyz")
    R = R_x * R_y * R_z;
  else if (rotationorder == "xzyx")
    R = R_x1 * R_z * R_y * R_x;
  else if (rotationorder == "zyx")
    R = R_z * R_y * R_x;

  return R;
}

/* Calculates rotation matrix given euler angles. */
Mat eulerAnglesToRotationMatrix(Vec3f theta, bool isDegrees)
{
  if (isDegrees)
    theta = theta * CV_PI / 180;

  /* Calculate rotation about x axis */
  Mat R_x = (Mat_<double>(3, 3) << 1, 0, 0, 0, cos(theta[0]), -sin(theta[0]), 0,
             sin(theta[0]), cos(theta[0]));

  /* Calculate rotation about y axis */
  Mat R_y = (Mat_<double>(3, 3) << cos(theta[1]), 0, sin(theta[1]), 0, 1, 0,
             -sin(theta[1]), 0, cos(theta[1]));

  /* Calculate rotation about z axis */
  Mat R_z = (Mat_<double>(3, 3) << cos(theta[2]), -sin(theta[2]), 0,
             sin(theta[2]), cos(theta[2]), 0, 0, 0, 1);

  /* R_z * R_y * R_x 旋轉順序(z->y->x),内旋.右乘*/ 
  Mat R = R_z * R_y * R_x;

  return R;
}

/* Checks if a matrix is a valid rotation matrix. */
bool isRotationMatrix(Mat &R)
{
  Mat Rt;
  transpose(R, Rt);
  Mat shouldBeIdentity = Rt * R;
  Mat I = Mat::eye(3, 3, shouldBeIdentity.type());

  return norm(I, shouldBeIdentity) < 1e-6;
}

/* Calculates rotation matrix to euler angles. */
Vec3f rotationMatrixToEulerAngles(Mat &R)
{
  //assert(isRotationMatrix(R));

  float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +
                  R.at<double>(1, 0) * R.at<double>(1, 0));

  bool singular = sy < 1e-6;

  float x, y, z;
  if (!singular)
  {
    x = atan2(R.at<double>(2, 1), R.at<double>(2, 2)) * 180 / CV_PI;
    y = atan2(-R.at<double>(2, 0), sy) * 180 / CV_PI;
    z = atan2(R.at<double>(1, 0), R.at<double>(0, 0)) * 180 / CV_PI;
  }
  else
  {
    x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1)) * 180 / CV_PI;
    y = atan2(-R.at<double>(2, 0), sy) * 180 / CV_PI;
    z = 0;
  }

  /* eular angle, rotate order:z->y->x*/ 
  return Vec3f(x, y, z);
}

void WriteData(const char* filename, vector<std::string> &Data)
{
  ofstream outputfile;
  outputfile.open(filename, std::ofstream::out);
  vector<string>::iterator iter_Data;

  for (iter_Data = Data.begin(); iter_Data != Data.end(); iter_Data++)
      outputfile << *iter_Data << endl;  
}

void Construct_calmat(string path, vector<Affine3d> all_Rt)
{
  vector<Mat> calmat_mtx;
  calmat_mtx.reserve(MAX_INPUT_CAMERAS);
  int idx = 0;
  vector<Affine3d>::iterator iter_Data;

  for (iter_Data = all_Rt.begin(); iter_Data != all_Rt.end(); iter_Data++)
  {
    Matx33d _R = iter_Data->rotation();
    Vec3d _t = iter_Data->translation();
    Mat _mtx(1, 12, CV_32SC1);
    double R_mfactor = pow(2, R_SHIFT);
    double T_mfactor = pow(2, t_SHIFT);
    int32_t* _mtx_prt = _mtx.ptr<int32_t>(0);
    
    *(_mtx_prt + 0) = (int32_t)(_R.val[0] * R_mfactor);
    *(_mtx_prt + 1) = (int32_t)(_R.val[1] * R_mfactor);
    *(_mtx_prt + 2) = (int32_t)(_R.val[2] * R_mfactor);
    *(_mtx_prt + 3) = (int32_t)(_R.val[3] * R_mfactor);
    *(_mtx_prt + 4) = (int32_t)(_R.val[4] * R_mfactor);
    *(_mtx_prt + 5) = (int32_t)(_R.val[5] * R_mfactor);
    *(_mtx_prt + 6) = (int32_t)(_R.val[6] * R_mfactor);
    *(_mtx_prt + 7) = (int32_t)(_R.val[7] * R_mfactor);
    *(_mtx_prt + 8) = (int32_t)(_R.val[8] * R_mfactor);
    *(_mtx_prt + 9) = (int32_t)(_t[0] * T_mfactor);
    *(_mtx_prt + 10) = (int32_t)(_t[1] * T_mfactor);
    *(_mtx_prt + 11) = (int32_t)(_t[2] * T_mfactor);

    calmat_mtx.push_back(_mtx);
  }

  Write_calmat_file((path + "CALMAT.BIN").c_str(), calmat_mtx);
}

void Write_calmat_file(const char*fileName, vector<Mat> calmat_mtx)
{
    int cnt, idx, jdx;
    FILE* fp = 0;
    size_t sz;
    svCalmat_t calmat;

    memset(&calmat, 0, sizeof(svCalmat_t));

    calmat.numCameras = MAX_INPUT_CAMERAS;

    for (idx = 0; idx < MAX_INPUT_CAMERAS; idx++) 
    {
        calmat.calMatSize[idx] = SRV_CALMAT_SIZE;
        int32_t* row_ptr = calmat_mtx[idx].ptr<int32_t>(0);
        for (jdx = 0; jdx < 12; jdx++ ) 
        {
          calmat.calMatBuf[12*idx + jdx] = *(row_ptr + jdx);
        }
    }

    if (!fileName)
    {
        printf("calmat file name not specified\n");
        return;
    }

    fp = fopen(fileName, "wb");
    if (!fp)
    {
        printf("Error: could not open file path\n");
        return;
    }

    fwrite(&calmat.numCameras,sizeof(uint8_t),4,fp);

    for (cnt = 0; cnt < calmat.numCameras; cnt++) 
    {
        fwrite(&calmat.calMatSize[cnt],sizeof(uint8_t), 4, fp);
    }

    /* Set Pointer ahead by 128 bytes to skip over metadata */
    fseek(fp, 128, SEEK_SET);

    /* Read calmat per camera */
    for (cnt = 0; cnt < calmat.numCameras;cnt++) 
    {
        fwrite((int8_t *)calmat.calMatBuf + 48*cnt, sizeof(uint8_t), calmat.calMatSize[cnt], fp);
    }

    fclose(fp);
}
   
// front: 
// [ -2.32322414  -0.43025298 129.63685461]
// left: 
// [  91.66620346   30.68842222 -178.5993359 ]
// rear: 
// [-177.96639573   -3.41875425 -120.54070113]
// right: 
// [-88.58017852 -33.18759674 179.55757632]

void StoreExtrinsic(string filename, Mat rotation_mtx, Mat translation)
{
  cv::FileStorage Extrinsic_file(filename, cv::FileStorage::WRITE);
  Extrinsic_file << "Rotation_mtx" << rotation_mtx;
  Extrinsic_file << "Translation_vec" << translation;

  Extrinsic_file.release();        
}

void CameraIntrinsicLUT(LensDistortionCorrection *ldc)
{
  const int table_size = 2057;
  vector<double> lut(table_size, 0.0);
  TermCriteria criteria = TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 1e-6);   

  Mat cameraMatrix, newcameraMatrix, distCoeffs;
  for (size_t viewindex = 0; viewindex < iCarviews_count; viewindex++)
  {
    cv::FileStorage Intrinsic_file(
      "./tempData/test/Intrinsic_" + sCarViews[viewindex] + ".xml",
      cv::FileStorage::READ);

    Intrinsic_file["Intrinsic"] >> cameraMatrix;
    Intrinsic_file["distCoeffs"] >> distCoeffs;
    Intrinsic_file.release();
    Vec4d d = (Vec4d)*distCoeffs.ptr<Vec4d>();
    cv::Vec2d f, c;

    f = Vec2d(cameraMatrix.at<double>(0, 0), cameraMatrix.at<double>(1, 1));
    lut[0] = (f[0] + f[1])*0.5;
    lut[1] = 0;
    lut[2] = 2;
    lut[3] = 2;
    ldc[viewindex].distFocalLength = (dtype)lut[0];
    ldc[viewindex].distFocalLengthInv = (dtype)(1.0/lut[0]);
    UndistortedToDistorted(d, lut);

    lut[1030] = 0;
    DistortedToUndistorted(d, criteria, lut);
    
    /* For testing */
    #if DEBUG_TEST
    fstream file;
    file.open("./tempData/test/" + sCarViews[viewindex]+ "_LUT.txt", ios::out);
    for (int i = 0; i < lut.size(); i++)
    {
      file << std::fixed << std::setprecision(7) << lut[i] << endl;
    }
    file.close();
    #endif 

    /* Writing len's LUT into the file */
    FILE *fp;
    string filename = "./tempData/test/" + sCarViews[viewindex]+ "_LUT.BIN";
    fp = fopen(filename.c_str(), "wb");
    if (!fp)
    {
        printf("Error: could not open file path\n");
        return;
    }

    ldc_lensParameters *ldcParams = ldc[viewindex].lenParas;
    ldcParams->ldcLUT_numCams = 4;
    ldcParams->ldcLUT_focalLength = lut[0];
    for(int j = 0; j < LDC_MAX_NUM_CAMERAS*2; j+=2)
    {
      ldcParams->ldcLUT_distortionCenters[j] = cameraMatrix.at<double>(0, 2);
      ldcParams->ldcLUT_distortionCenters[j+1] = cameraMatrix.at<double>(1, 2);
    }
    ldc[viewindex].distCenterX = cameraMatrix.at<double>(0, 2);
    ldc[viewindex].distCenterY =  cameraMatrix.at<double>(1, 2);

    ldcParams->U2D_type = lut[3];
    ldcParams->ldcLUT_U2D_step = lut[4];
    ldcParams->ldcLUT_U2D_length = lut[5];
    ldc[viewindex].lut_u2d_stepInv = (dtype)(1.0/lut[4]);
    for(int i = 0; i < 1024; i++)
    {
      ldcParams->ldcLUT_U2D_table[i] = lut[i + 6];
    }

    ldcParams->D2U_type = lut[1030];
    ldcParams->ldcLUT_D2U_step = lut[1031];
    ldcParams->ldcLUT_D2U_length = lut[1032];
    ldc[viewindex].lut_d2u_stepInv = (dtype)(1.0/lut[1031]);
    for(int i = 0; i < 1024; i++)
    {
      ldcParams->ldcLUT_D2U_table[i] = lut[i + 1033];
    }

    fwrite((uint8_t *)ldcParams, sizeof(uint8_t), sizeof(ldc_lensParameters), fp);
    fclose(fp);
  }  
}

void UndistortedToDistorted(Vec4d k, vector<double> &lut)
{
  double theta_Max = 90.0*CV_PI/180;
  double theta_step = theta_Max/(LUT_TABLE_LEN - 1);

  lut[4] = theta_step;
  lut[5] = LUT_TABLE_LEN;
  lut[6] = 0;
  for(int i = 1; i < LUT_TABLE_LEN; i++)
  {
    double theta = theta_step * i;
    double theta2 = theta*theta, theta3 = theta2*theta, theta4 = theta2*theta2, theta5 = theta4*theta, theta6 = theta3*theta3, theta7 = theta6*theta, theta8 = theta4*theta4, theta9 = theta8*theta;

    double theta_d = theta + k[0]*theta3 + k[1]*theta5 + k[2]*theta7 + k[3]*theta9;
    lut[i+6] = lut[0] * theta_d; // rd = focal length* theta
  }
}

void DistortedToUndistorted(Vec4d k, TermCriteria criteria, vector<double> &lut)
{
  double theta_d_Max = 89.0*CV_PI/180;

  double theta2 = theta_d_Max*theta_d_Max, theta3 = theta2*theta_d_Max, theta4 = theta2*theta2, theta5 = theta4*theta_d_Max,
      theta6 = theta3*theta3, theta7 = theta6*theta_d_Max, theta8 = theta4*theta4, theta9 = theta8*theta_d_Max;

  double theta_d = theta_d_Max + k[0]*theta3 + k[1]*theta5 + k[2]*theta7 + k[3]*theta9;
  double r_d = lut[0] * theta_d;
  double r_d_step = pow(r_d, 2) / (LUT_TABLE_LEN - 1);
  const bool isEps = (criteria.type & TermCriteria::EPS) != 0;
  int maxCount = std::numeric_limits<int>::max();
  if (criteria.type & TermCriteria::MAX_ITER) {
      maxCount = criteria.maxCount;
  }
  
  lut[1031] = r_d_step;
  lut[1032] = LUT_TABLE_LEN;
  lut[1033] = 1;
  for(int i = 1; i < LUT_TABLE_LEN; i++)
  {
    bool converged = false;

    theta_d = sqrt(r_d_step * i) / lut[0];
    double theta = theta_d;

    double scale = 0.0;
    if (!isEps || fabs(theta_d) > criteria.epsilon)
    {
        // compensate distortion iteratively
        for (int j = 0; j < maxCount; j++)
        {
            double theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta4*theta2, theta8 = theta6*theta2;
            double k0_theta2 = k[0] * theta2, k1_theta4 = k[1] * theta4, k2_theta6 = k[2] * theta6, k3_theta8 = k[3] * theta8;
            /* new_theta = theta - theta_fix, theta_fix = f0(theta) / f0'(theta) */
            double theta_fix = (theta * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) - theta_d) /
                                (1 + 3*k0_theta2 + 5*k1_theta4 + 7*k2_theta6 + 9*k3_theta8);
            theta = theta - theta_fix;

            if (isEps && (fabs(theta_fix) < criteria.epsilon))
            {
                converged = true;
                break;
            }
        }
        double r_u = lut[0] * std::tan(theta);
        r_d = sqrt(r_d_step * i);
        scale = r_u / r_d;
        lut[i+1033] = scale;
    }
    else
    {
      converged = true;
    }
  }
}

void Locator(int event, int x, int y, int flags, void* userdata)
{ 
  //function to track mouse movement and click//
  MouseParameters* mousePara = (MouseParameters*) userdata;
  if (event == EVENT_LBUTTONDOWN)
  { //when left button clicked//
    //cout << "Left click has been made, Position:(" << x << "," << y << ")" << endl;
    mousePara->drawflag = true;
    mousePara->tempPoint = Point(x,y);
  }
  else if (event == EVENT_LBUTTONUP)
  {
    mousePara->drawflag = false;
    Point2f pt = Point2f(x,y)/mousePara->scale ;
    mousePara->corner.push_back(pt);
  }
  else if (event == EVENT_RBUTTONDOWN)
  { //when right button clicked//
    //cout << "Rightclick has been made, Position:(" << x << "," << y << ")" << endl;
  } 
  else if (event == EVENT_MBUTTONDOWN)
  { //when middle button clicked//
    //cout << "Middleclick has been made, Position:(" << x << "," << y << ")" << endl;
  } 
  else if (event == EVENT_MOUSEMOVE)
  { //when mouse pointer moves//
    mousePara->tempPoint = Point(x,y);
  }
}

void Read_lut_file(ldc_lensParameters *ldcParams, const char*fileName)
{
    char file[MAXPATHLENGTH];
    uint32_t  read_size;
    FILE* f = NULL;

    if (!fileName)
    {
        printf("lut file name not specified\n");
        return;
    }

    f = fopen(fileName, "rb");
    if (!f)
    {
        printf("Can't open LUT file: %s\n", fileName);
        return;
    }

    read_size = fread((uint8_t *)ldcParams,sizeof(uint8_t),sizeof(ldc_lensParameters),f);
    if (read_size != sizeof(ldc_lensParameters))
    {
        printf("Incorrect Bytes read from  LUT file: %s\n", fileName);
        fclose(f);
        return;
    }

    fclose(f);
}

void OpenCV__ExtrinsicProcess1()
{
  //ldc_lensParameters  lens_params;
  //read_lut_file(&lens_params, "./tempData/test/front_LUT.BIN");
  #if showundistorted_image
  namedWindow("Image Window", WINDOW_FREERATIO); // 命名一個影像視窗
  #endif

  #if showcorner_image
  namedWindow("Corner Image", WINDOW_FREERATIO); // 命名一個影像視窗
  #endif
  if( bDoCalibration)
  {
    for (size_t viewindex = 0; viewindex < iCarviews_count; viewindex++)
    {
        /* Load camera intrinsic parameter */
        Mat cameraMatrix, newcameraMatrix, distCoeffs;
        cv::FileStorage Intrinsic_file(
            "./tempData/test/Intrinsic_" + sCarViews[viewindex] + ".xml",
            cv::FileStorage::READ);

        // cv::FileStorage Intrinsic_file(
        //     "./Intrinsic para/simulator/fisheye/Intrinsic_" + sCarViews[viewindex] + ".xml",
        //     cv::FileStorage::READ);

        Intrinsic_file["Intrinsic"] >> cameraMatrix;
        Intrinsic_file["distCoeffs"] >> distCoeffs;
        Intrinsic_file.release();

        /* Path of the folder containing checkerboard images */
        vector<cv::String> images_path;
        #if b20cmChessboard
        string path = "./images/extrinsic/Highest_20cm/" + sCarViews[viewindex] + "/*.bmp";
        path = "./images/extrinsic/TI_20cm/" + sCarViews[viewindex] + "/*.bmp";
        // string path = "./extrinsic/4_singleCameraRotation/*.bmp";
        // path = "./images/*.jpg";
        #else
        string path = "./images/extrinsic/4cm/" + sCarViews[viewindex] + "/*.bmp";
        #endif
        
        cv::glob(path, images_path);
        vector<Mat> vRotation, vTranslation;
        cv::TermCriteria subpixCriteria = cv::TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1);        
        
        for (size_t img_index = 0; img_index < images_path.size(); img_index++)
        {
            Mat original_img, undistorted_img, gray_img;
            Mat mapx, mapy;
            MouseParameters mousePara;
            
            original_img = imread(images_path[img_index]);
            
            #if b20cmChessboard
            /* For 20cm chessboard */
            cv::cvtColor(original_img, gray_img, COLOR_BGR2GRAY);
            #else
            /* For 4cm chessboard */
            cv::cvtColor(original_img, gray_img, COLOR_RGB2GRAY);
            #endif

            /* undistored fish-eye images */
            cv::fisheye::estimateNewCameraMatrixForUndistortRectify( cameraMatrix, distCoeffs, original_img.size(), Matx33d::eye(), newcameraMatrix, 1.0);

            #if showundistorted_image
            cv::fisheye::initUndistortRectifyMap( cameraMatrix, distCoeffs, Matx33d::eye(), newcameraMatrix, original_img.size(), CV_16SC2, mapx, mapy);
            cv::remap(original_img, undistorted_img, mapx, mapy, INTER_LINEAR, BORDER_CONSTANT);

            imshow("Image Window", undistorted_img);
            imwrite("./undistorted/" + sCarViews[viewindex] + to_string(img_index)+".jpg", undistorted_img);
            waitKey(0);
            #endif

            /* Find corner points on chessboard */
            vector<cv::Point2f> corners;
            vector<cv::Point2f> undistored_corners;
            bool success = cv::findChessboardCorners(gray_img, board_size, corners,
                                                 CALIB_CB_ADAPTIVE_THRESH |
                                                     CALIB_CB_NORMALIZE_IMAGE);
            if(!success)
            {
              Mat resize_img;
              float scale = 0.75;
              mousePara.scale = scale;
              CV:resize(original_img, resize_img, Size(original_img.cols*scale, original_img.rows*scale));
              resize_img.copyTo(mousePara.image);

              namedWindow("Track"); 
              setMouseCallback("Track", Locator, &mousePara); 
              while(1)
              {
                if (mousePara.drawflag)
                {
                  Point pt = mousePara.tempPoint;
                  cv::line(mousePara.image, Point(pt.x-20, pt.y), Point(pt.x+20, pt.y),Scalar(0,0,255));
                  cv::line(mousePara.image, Point(pt.x, pt.y-20), Point(pt.x, pt.y+20),Scalar(0,0,255));
                }           
                if(mousePara.corner.size() > 0)
                {
                  for (size_t i = 0; i < mousePara.corner.size(); i++)
                  {
                    Point pt = mousePara.corner[i]*scale;
                    cv::line(mousePara.image, Point(pt.x-10, pt.y-10), Point(pt.x+10, pt.y+10),Scalar(0,0,255));
                    cv::line(mousePara.image, Point(pt.x+10, pt.y-10), Point(pt.x-10, pt.y+10),Scalar(0,0,255));
                  }
                }  
                imshow("Track", mousePara.image);               
                resize_img.copyTo(mousePara.image);

                if (waitKey(1) == 'q' && mousePara.corner.size() >= 4)
                {
                  corners = mousePara.corner;
                  success = true;
                  
                  break;
                }               
                else if (waitKey(1) == 'd')
                {
                  if (mousePara.corner.size() > 0)
                    mousePara.corner.pop_back();
                }
              }
            }

            /* Creating vector to store 3D real world points for each chessboard image */
            vector<cv::Point3f> object_points;
            vector<cv::Point2f> img_Points;
            if (success)
            {
                cv::cornerSubPix(gray_img, corners, cv::Size(3, 3), cv::Size(-1, -1), subpixCriteria);

                cv::fisheye::undistortPoints(corners, undistored_corners, cameraMatrix, distCoeffs, Matx33d::eye(), cameraMatrix);

                #if showcorner_image
                #if showundistorted_image
                cv::drawChessboardCorners(undistorted_img, board_size, undistored_corners, success);
                imshow("Corner Image", undistorted_img);
                waitKey(0);
                #else
                cv::drawChessboardCorners(original_img, board_size, corners, success);
                #endif
                #endif

                /* Initial real world points  and pick corner points.*/
                for (int i = 0; i < board_height; i++)
                {
                    for (int j = 0; j < board_width; j++)
                    {
                      /* Get around corner points */
                      if(j == 0 || j % (board_width - 1) == 0)
                      {
                        cv::Point3f point = cv::Point3f(float(j * SQUARE_SIZE), float(i * SQUARE_SIZE), 0);
                        object_points.push_back(point);
                        
                        img_Points.push_back(undistored_corners[i * board_width + j]);
                      }                        
                    }
                }
            }
            else cout << "fail image: " << images_path[img_index] << endl;

            /* Find homogeneous transform using solvePnP */
            Matx31d rotation_vec, translation_vec;
            vector<Matx31d> rotation_vectors, translation_vectors;
            vector<cv::Point2f> out_fisheyeImagePoints;
            if (success)
            {
                Mat fake_distCoeffs = (Mat_<float>(4,1) << 0, 0, 0, 0);
                Mat reprojection_error;
                // cv::solvePnP(object_points, img_Points, cameraMatrix, fake_distCoeffs, rotation_vec, translation_vec, false, SOLVEPNP_ITERATIVE);
                cv::solvePnPGeneric(object_points, img_Points, cameraMatrix, fake_distCoeffs, rotation_vectors, translation_vectors, false, SOLVEPNP_ITERATIVE, rotation_vec, translation_vec,  reprojection_error);

                cv::solvePnPRefineLM(object_points, img_Points, cameraMatrix, fake_distCoeffs, rotation_vectors[0], translation_vectors[0]);

                cout << images_path[img_index] << "reprojection error: " << reprojection_error << endl;

                vRotation.push_back(Mat(rotation_vectors[0]));
                vTranslation.push_back(Mat(translation_vectors[0]));
                
                #if showcorner_image
                cv::fisheye::projectPoints(object_points, out_fisheyeImagePoints, rotation_vectors[0], translation_vectors[0], cameraMatrix, distCoeffs);
                
                for (size_t point = 0; point < out_fisheyeImagePoints.size(); point++)
                {
                  cv::circle(original_img, cv::Point(int(out_fisheyeImagePoints[point].x),int(out_fisheyeImagePoints[point].y)), 4, (0,255,0), -1);
                }
                imshow("Corner Image", original_img);
                waitKey(0);   
                #endif             
            }            
        }

        /* Store rotation vector and translation vector */
        char cnumber[100];
        cv::FileStorage Extrinsic_file(
            "./parameters/Extrinsic_" + sCarViews[viewindex] + ".xml",
            cv::FileStorage::WRITE);
        Extrinsic_file << "Rotation" << vRotation;
        Extrinsic_file << "Translation" << vTranslation;

        Extrinsic_file.release();        
    }
  }
    
  /* 計算相機間的世界座標關係,以其一為主建立相對關係求出各相機在世界的位置 */
  vector<cv::Vec3f> camera_position;
  vector<cv::Vec3f> theta;
  camera_position.resize(iCarviews_count);
  theta.resize(iCarviews_count);
  vector<Affine3d> all_Rt;
  all_Rt.resize(MAX_INPUT_CAMERAS);
  Vec3f temp_theta;
  /* Get camera position and pose in real world coordinate */
  /* 世界原點在前相機正下方地面 */
  string path = "./parameters/Extrinsic_";
  EstimateCameraParameters(path, "front", "", camera_position[0], theta[0], 0);

  EstimateCameraParameters(path, "front", "", camera_position[0], temp_theta, 1);
  /* verify different rotation angle for single chessboard */
  // estimateCameraParameters("front", "", camera_position[0], theta[0], 1);
  // estimateCameraParameters("front", "", camera_position[0], theta[0], 2);
  // estimateCameraParameters("front", "", camera_position[0], theta[0], 3);

  EstimateCameraParameters(path, "front", "left", camera_position[1], theta[1], 0);
  EstimateCameraParameters(path, "front", "right", camera_position[2], theta[2], 0);
  EstimateCameraParameters(path, "front", "rear", camera_position[3], theta[3], 0);

  /* TI coordinate transform */
  /* TI 世界原點預設為最左下棋盤格角點 */
  #ifdef TI
  Vec3f cam_World[4];
  // front
  cam_World[0][0] = camera_position[0][0] - 200;
  cam_World[0][1] = World_Y_shifted - camera_position[0][1];
  cam_World[0][2] = -camera_position[0][2];

  // left
  cam_World[1][0] = cam_World[0][0] - camera_position[1][0];
  cam_World[1][1] = cam_World[0][1] + camera_position[1][1];
  cam_World[1][2] = -camera_position[1][2];

  // right
  cam_World[2][0] = cam_World[0][0] - camera_position[2][0];
  cam_World[2][1] = cam_World[0][1] + camera_position[2][1];
  cam_World[2][2] = -camera_position[2][2];

  // rear
  cam_World[3][0] = cam_World[0][0] - camera_position[3][0];
  cam_World[3][1] = cam_World[0][1] + camera_position[3][1];
  cam_World[3][2] = -camera_position[3][2];

  vector<Mat> _viewR;
  for (size_t i = 0; i < iCarviews_count; i++)
  {
    /* _viewR -> WC */
    _viewR.push_back(eulerAnglesToRotationMatrix(theta[i], true));
    Mat T = Mat_<Vec3f>(cam_World[i]);
    
    StoreExtrinsic("./parameters/TI_Extrinsic_"+ sCarViews[i] +".xml", _viewR[i], T);
    // cout << sCarViews[i] << " R_mtx: " << endl << _viewR[i] << endl;
    cout << sCarViews[i] << " position in the world: " << cam_World[i] << endl;
    cout << sCarViews[i] << " camera pose[pitch roll yaw]: "<< endl << theta[i] << endl;

    // _viewR[i].t(): CW
    Mat _R;
    _R = _viewR[i].t();
    // cout << sCarViews[i] << "(R_mtx)_T: " << endl << _viewR[i] << endl;
    Mat _t = Mat::zeros(1,3,CV_64F);
    _t.at<double>(0,0) = -(_R.at<double>(0,0) * cam_World[i][0] + _R.at<double>(0,1) * cam_World[i][1] + _R.at<double>(0,2) * cam_World[i][2]);
    _t.at<double>(0,1) = -(_R.at<double>(1,0) * cam_World[i][0] + _R.at<double>(1,1) * cam_World[i][1] + _R.at<double>(1,2) * cam_World[i][2]);
    _t.at<double>(0,2) = -(_R.at<double>(2,0) * cam_World[i][0] + _R.at<double>(2,1) * cam_World[i][1] + _R.at<double>(2,2) * cam_World[i][2]);
    cout << "cam_t" << _t << endl;
    
    // _viewR[i].t(): WC
    if(sCarViews[i] == "front")
      all_Rt[0] = Affine3d(_viewR[i], _t);
    else if(sCarViews[i] == "right")  
      all_Rt[1] = Affine3d(_viewR[i], _t);
    else if(sCarViews[i] == "rear")     
      all_Rt[2] = Affine3d(_viewR[i], _t);
    else if(sCarViews[i] == "left")    
      all_Rt[3] = Affine3d(_viewR[i], _t);
  }
  path = "./parameters/OpenCV_";
  Construct_calmat(path, all_Rt);
  #else
  #if bStoredparameters
  for (size_t viewindex = 0; viewindex < iCarviews_count; viewindex++)
  {
    vector<std::string> Data;
    string filename = "./parameters/Extrinsic_" + sCarViews[viewindex] + ".txt";
    Data.push_back("# "+ sCarViews[viewindex]);
    Data.push_back("# position X");
    if (sCarViews[viewindex]  == "front")
      Data.push_back("0.0");
    else
      Data.push_back(to_string(camera_position[viewindex][1]));
    Data.push_back("# position Y");
    if (sCarViews[viewindex]  == "front")
      Data.push_back("0.0");
    else
      Data.push_back(to_string(camera_position[viewindex][0]));
    Data.push_back("# position Z");
    Data.push_back(to_string(-camera_position[viewindex][2]));
    Data.push_back("# rotation x (roll)");
    Data.push_back(to_string(theta[viewindex][1]));
    Data.push_back("# rotation y (pitch)");
    Data.push_back(to_string(theta[viewindex][0]));
    Data.push_back("# rotation z (yaw)");
    Data.push_back(to_string(theta[viewindex][2]));

    WriteData(filename.c_str(), Data);
  }
  #endif 
  #endif
}  

void OpenCV_ExtrinsicProcess2()
{
  vector<vector<Point3f>> object_points;
  object_points.resize(iCarviews_count);
  for (size_t i = 0; i < object_points.size(); i++)
    object_points[i].resize(NUM_CHART_CORNERS);
  initChartPoints(object_points);

  for (size_t viewindex = 0; viewindex < iCarviews_count; viewindex++)
  {
    /* Load camera intrinsic parameter */
    Mat cameraMatrix, newcameraMatrix, distCoeffs;
    cv::FileStorage Intrinsic_file(
        "./tempData/test/Intrinsic_" + sCarViews[viewindex] + ".xml",
        cv::FileStorage::READ);

    // cv::FileStorage Intrinsic_file(
    //     "./Intrinsic para/simulator/fisheye/Intrinsic_" + sCarViews[viewindex] + ".xml",
    //     cv::FileStorage::READ);

    Intrinsic_file["Intrinsic"] >> cameraMatrix;
    Intrinsic_file["distCoeffs"] >> distCoeffs;
    Intrinsic_file.release();

    /* Path of the folder containing checkerboard images */
    vector<cv::String> images_path;
    #if b20cmChessboard
    string path = "./images/extrinsic/Highest_20cm/" + sCarViews[viewindex] + "/*.bmp";
    path = "./images/extrinsic/TI_20cm/" + sCarViews[viewindex] + "/*.bmp";
    // string path = "./extrinsic/4_singleCameraRotation/*.bmp";
    // path = "./images/*.jpg";
    #else
    string path = "./images/extrinsic/4cm/" + sCarViews[viewindex] + "/*.bmp";
    #endif
    
    cv::glob(path, images_path);
    vector<Mat> vRotation, vTranslation;
    cv::TermCriteria subpixCriteria = cv::TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1);        
    vector<cv::Point2f> img_Points;
    bool success = false;
    for (size_t img_index = 1; img_index < images_path.size(); img_index++)
    {
      Mat original_img, undistorted_img, gray_img;
      Mat mapx, mapy;
      MouseParameters mousePara;
      mousePara.drawflag = false;
      mousePara.corner.clear();

      original_img = imread(images_path[img_index]);
      cv::cvtColor(original_img, gray_img, COLOR_BGR2GRAY);
     
      vector<cv::Point2f> corners;
      vector<cv::Point2f> undistored_corners;
      success = false;
      if(!success)
      {
        Mat resize_img;
        float scale = 0.75;
        mousePara.scale = scale;
        CV:resize(original_img, resize_img, Size(original_img.cols*scale, original_img.rows*scale));
        resize_img.copyTo(mousePara.image);

        namedWindow("Track"); 
        setMouseCallback("Track", Locator, &mousePara); 
        while(1)
        {
          if (mousePara.drawflag)
          {
            Point pt = mousePara.tempPoint;
            cv::line(mousePara.image, Point(pt.x-20, pt.y), Point(pt.x+20, pt.y),Scalar(0,0,255));
            cv::line(mousePara.image, Point(pt.x, pt.y-20), Point(pt.x, pt.y+20),Scalar(0,0,255));
          }           
          if(mousePara.corner.size() > 0)
          {
            for (size_t i = 0; i < mousePara.corner.size(); i++)
            {
              Point pt = mousePara.corner[i]*scale;
              cv::line(mousePara.image, Point(pt.x-10, pt.y-10), Point(pt.x+10, pt.y+10),Scalar(0,0,255));
              cv::line(mousePara.image, Point(pt.x+10, pt.y-10), Point(pt.x-10, pt.y+10),Scalar(0,0,255));
            }
          }  
          imshow("Track", mousePara.image);               
          resize_img.copyTo(mousePara.image);

          if (waitKey(1) == 'q' && mousePara.corner.size() >= 4)
          {
            corners = mousePara.corner;
            success = true;
            break;
          }               
          else if (waitKey(1) == 'd')
          {
            if (mousePara.corner.size() > 0)
              mousePara.corner.pop_back();
          }
        }
      }
    
    /* Creating vector to store 3D real world points for each chessboard image */
      if (success)
      {
          cv::cornerSubPix(gray_img, corners, cv::Size(3, 3), cv::Size(-1, -1), subpixCriteria); 
          cv::fisheye::undistortPoints(corners, undistored_corners, cameraMatrix, distCoeffs, Matx33d::eye(), cameraMatrix);           
      }
      else 
        cout << "fail image: " << images_path[img_index] << endl;

      for (size_t i = 0; i < undistored_corners.size(); i++)
      {
        img_Points.push_back(undistored_corners[i]);       
      }
    }

    /* Find homogeneous transform using solvePnP */
    Matx31d rotation_vec, translation_vec;
    vector<Matx31d> rotation_vectors, translation_vectors;
    vector<cv::Point2f> out_fisheyeImagePoints;
    if(success)
    {
      Mat fake_distCoeffs = (Mat_<float>(4,1) << 0, 0, 0, 0);
      Mat reprojection_error;
      // cv::solvePnP(object_points, img_Points, cameraMatrix, fake_distCoeffs, rotation_vec, translation_vec, false, SOLVEPNP_ITERATIVE);
      cv::solvePnPGeneric(object_points[viewindex], img_Points, cameraMatrix, fake_distCoeffs, rotation_vectors, translation_vectors, false, SOLVEPNP_ITERATIVE, rotation_vec, translation_vec,  reprojection_error);

      cv::solvePnPRefineLM(object_points[viewindex], img_Points, cameraMatrix, fake_distCoeffs, rotation_vectors[0], translation_vectors[0]);

      cout << sCarViews[viewindex] << "reprojection error: " << reprojection_error << endl;

      vRotation.push_back(Mat(rotation_vectors[0]));
      vTranslation.push_back(Mat(translation_vectors[0]));
      
      // #if showcorner_image
      // cv::fisheye::projectPoints(object_points, out_fisheyeImagePoints, rotation_vectors[0], translation_vectors[0], cameraMatrix, distCoeffs);
      // Mat original_img = imread(images_path[1]);
      // for (size_t point = 0; point < out_fisheyeImagePoints.size(); point++)
      // {
      //   cv::circle(original_img, cv::Point(int(out_fisheyeImagePoints[point].x),int(out_fisheyeImagePoints[point].y)), 4, (0,255,0), -1);
      // }
      // imshow("Corner Image", original_img);
      // waitKey(0);   
      // #endif                    
    }

    /* Store rotation vector and translation vector */
    char cnumber[100];
    cv::FileStorage Extrinsic_file(
        "./parameters/OpenCV2_Extrinsic_" + sCarViews[viewindex] + ".xml",
        cv::FileStorage::WRITE);
    Extrinsic_file << "Rotation" << vRotation;
    Extrinsic_file << "Translation" << vTranslation;

    Extrinsic_file.release(); 
  }
  /* 計算相機間的世界座標關係,以其一為主建立相對關係求出各相機在世界的位置 */
  vector<cv::Vec3f> camera_position;
  vector<cv::Vec3f> theta;
  Vec3f temp_theta;
  camera_position.resize(iCarviews_count);
  theta.resize(iCarviews_count);

  vector<Affine3d> all_Rt;
  all_Rt.resize(MAX_INPUT_CAMERAS);
  string path = "./parameters/OpenCV2_Extrinsic_";
  /* Get camera position and pose in real world coordinate */
  //EstimateCameraParameters(path, "front", "", camera_position[0], theta[0], 0);

  //EstimateCameraParameters(path, "front", "", camera_position[0], temp_theta, 1);
  /* verify different rotation angle for single chessboard */
  // estimateCameraParameters("front", "", camera_position[0], theta[0], 1);
  // estimateCameraParameters("front", "", camera_position[0], theta[0], 2);
  // estimateCameraParameters("front", "", camera_position[0], theta[0], 3);

  //EstimateCameraParameters(path, "front", "left", camera_position[1], theta[1], 0);
  //EstimateCameraParameters(path, "front", "right", camera_position[2], theta[2], 0);
  //EstimateCameraParameters(path, "front", "rear", camera_position[3], theta[3], 0);
  for (size_t i = 0; i < iCarviews_count; i++)
  {
    vector<Mat> vR_mat;
    vector<Mat> vTranslation;
    Mat _viewR;
    cv::FileStorage Extrinsic_file(path + sCarViews[i] + ".xml",
                                   cv::FileStorage::READ);
    if (Extrinsic_file.isOpened())
    {
      Extrinsic_file["Rotation"] >> vR_mat;
      Extrinsic_file["Translation"] >> vTranslation;
    }      

    Extrinsic_file.release();

    cv::Rodrigues(vR_mat[0], _viewR);
    Mat Rt = _viewR.t();
    Vec3f euler = rotationMatrixToEulerAngles(Rt);
    cout << "Euler" << euler << endl;

    Mat _t = vTranslation[0];
    Mat camera_in_world = -Rt * _t;
    cout << "Camera position" << camera_in_world << endl;

    _viewR = _viewR.t();
    if(sCarViews[i] == "front")
      all_Rt[0] = Affine3d(_viewR, _t);
    else if(sCarViews[i] == "right")  
      all_Rt[1] = Affine3d(_viewR, _t);
    else if(sCarViews[i] == "rear")     
      all_Rt[2] = Affine3d(_viewR, _t);
    else if(sCarViews[i] == "left")    
      all_Rt[3] = Affine3d(_viewR, _t);
  }

  path = "./parameters/OpenCV_";
  Construct_calmat(path, all_Rt);
}

void TI_ExtrinsicProcess(svPoseEstimation_t *sv, tivxPoseEstimationParams *prms, LensDistortionCorrection *ldc)
{

  vector<vector<Point2D_f>> object_points;
  object_points.resize(iCarviews_count);
  for (size_t i = 0; i < object_points.size(); i++)
    object_points[i].resize(NUM_CHART_CORNERS);
  initChartPoints(object_points);

  for (size_t viewindex = 0; viewindex < iCarviews_count; viewindex++)
  {
    /* Load camera intrinsic parameter */
    Mat cameraMatrix, newcameraMatrix, distCoeffs;
    cv::FileStorage Intrinsic_file(
        "./tempData/test/Intrinsic_" + sCarViews[viewindex] + ".xml",
        cv::FileStorage::READ);
        Intrinsic_file["Intrinsic"] >> cameraMatrix;
        Intrinsic_file["distCoeffs"] >> distCoeffs;
        Intrinsic_file.release();

    /* Path of the folder containing checkerboard images */
    vector<cv::String> images_path;
    string path;
    #if b20cmChessboard
    string path1 = "./images/extrinsic/Highest_20cm/" + sCarViews[viewindex] + "/*.bmp";
    string path2 = "./images/extrinsic/TI_20cm/" + sCarViews[viewindex] + "/*.bmp";

    path = path2;
    #else
    string path = "./images/extrinsic/4cm/" + sCarViews[viewindex] + "/*.bmp";
    #endif
    cv::glob(path, images_path);

    vector<Mat> vRotation, vTranslation;
    cv::TermCriteria subpixCriteria = cv::TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1);
    vector<cv::Point2f> img_Points;
    for (size_t img_index = 1; img_index < images_path.size(); img_index++)
    {
      Mat original_img, gray_img;
      MouseParameters mousePara;
      mousePara.drawflag = false;
      mousePara.corner.clear();

      original_img = imread(images_path[img_index]);
      #if b20cmChessboard
      /* For 20cm chessboard */
      cv::cvtColor(original_img, gray_img, COLOR_BGR2GRAY);
      #else
      /* For 4cm chessboard */
      cv::cvtColor(original_img, gray_img, COLOR_RGB2GRAY);
      #endif

      /* Find corner points on single chessboard */
      vector<cv::Point2f> corners;
      // bool success = cv::findChessboardCorners(gray_img, board_size, corners, CALIB_CB_ADAPTIVE_THRESH|CALIB_CB_NORMALIZE_IMAGE);
      bool success = false;
      if(!success)
      {
        Mat resize_img;
        float scale = 0.75;
        mousePara.scale = scale;
        CV:resize(original_img, resize_img, Size(original_img.cols*scale, original_img.rows*scale));
        resize_img.copyTo(mousePara.image);

        namedWindow("Track"); 
        setMouseCallback("Track", Locator, &mousePara); 
        while(1)
        {
          if (mousePara.drawflag)
          {
            Point pt = mousePara.tempPoint;
            /* Draw cross line */
            cv::line(mousePara.image, Point(pt.x-20, pt.y), Point(pt.x+20, pt.y),Scalar(0,0,255));
            cv::line(mousePara.image, Point(pt.x, pt.y-20), Point(pt.x, pt.y+20),Scalar(0,0,255));
          }
          if(mousePara.corner.size() > 0)
          {
            for (size_t i = 0; i < mousePara.corner.size(); i++)
            {
              /* code */
              Point pt = mousePara.corner[i]*scale;
              cv::line(mousePara.image, Point(pt.x-10, pt.y-10), Point(pt.x+10, pt.y+10),Scalar(0,0,255));
              cv::line(mousePara.image, Point(pt.x+10, pt.y-10), Point(pt.x-10, pt.y+10),Scalar(0,0,255));
            }
          }              
          imshow("Track", mousePara.image);
          resize_img.copyTo(mousePara.image);
          int signal = waitKey(1);
          if (signal == 'q' && mousePara.corner.size() >= 4)
          {
            /* Quit process */
            corners = mousePara.corner;
            success = true;
            break;
          }               
          else if (signal == 'd')
          {
            /* Delete last corner */
            if (mousePara.corner.size() > 0)
              mousePara.corner.pop_back();
          }
        }
      }
     
      if (success)
      {
        cv::cornerSubPix(gray_img, corners, cv::Size(3, 3), cv::Size(-1, -1), subpixCriteria);  

        // for (int i = 0; i < board_height; i++)
        // {
        //   for (int j = 0; j < board_width; j++)
        //   {
        //     /* Get around corner points of a chessboard && (i == 0 || i %(board_height - 1)==0)*/
        //     if((j == 0 || j % (board_width - 1) == 0 || j % (board_width - 2) == 0 ) )
        //     {
        //       cv::Point3f point;

        //       point = cv::Point3f(float(j * SQUARE_SIZE), -(float(i * SQUARE_SIZE)-4748), 0.0);
        //       object_points.push_back(point);
        //       img_Points.push_back(corners[i * board_width + j]);
        //     }                        
        //   }
        // }
      }
      else 
        cout << "fail image: " << images_path[img_index] << endl; 
      for (size_t i = 0; i < corners.size(); i++)
      {
        img_Points.push_back(corners[i]);       
      }
    }

    // object_points.erase(object_points.begin()+4);
    // img_Points.erase(img_Points.begin()+4);
    Point2D_f *chart_points = (Point2D_f *)malloc(sizeof(Point2D_f)*object_points[viewindex].size());
    Point2D_f *corner_points = (Point2D_f *)malloc(sizeof(Point2D_f)*img_Points.size());
    Point2D_f *normcorner_points = (Point2D_f *)malloc(sizeof(Point2D_f)*img_Points.size());

    TransformPointsFormat(object_points[viewindex], img_Points, chart_points, corner_points);

    svExtrinsicPoseEstimation(sv, prms, ldc[viewindex], viewindex, chart_points, corner_points, normcorner_points);

    Matx33d rotation_mtx;
    Matx31d translation_vector;
    TransformExtrinsic(prms, viewindex, rotation_mtx, translation_vector);
    vRotation.push_back(Mat(rotation_mtx));
    vTranslation.push_back(Mat(translation_vector));
    
    free(chart_points);
    free(corner_points);
    free(normcorner_points);

    /* Store rotation vector and translation vector */
    char cnumber[100];
    cv::FileStorage Extrinsic_file(
        "./parameters/TI_Extrinsic_" + sCarViews[viewindex] + ".xml",
        cv::FileStorage::WRITE);
    Extrinsic_file << "Rotation" << vRotation;
    Extrinsic_file << "Translation" << vTranslation;

    Extrinsic_file.release();        
  }

  /* 計算相機間的世界座標關係,以其一為主建立相對關係求出各相機在世界的位置 */
  vector<cv::Vec3f> camera_position;
  vector<cv::Vec3f> theta;
  Vec3f temp_theta;
  camera_position.resize(iCarviews_count);
  theta.resize(iCarviews_count);

  vector<Affine3d> all_Rt;
  all_Rt.resize(MAX_INPUT_CAMERAS);
  string path = "./parameters/TI_Extrinsic_";
  /* Get camera position and pose in real world coordinate */
  //EstimateCameraParameters(path, "front", "", camera_position[0], theta[0], 0);

  //EstimateCameraParameters(path, "front", "", camera_position[0], temp_theta, 1);
  /* verify different rotation angle for single chessboard */
  // estimateCameraParameters("front", "", camera_position[0], theta[0], 1);
  // estimateCameraParameters("front", "", camera_position[0], theta[0], 2);
  // estimateCameraParameters("front", "", camera_position[0], theta[0], 3);

  //EstimateCameraParameters(path, "front", "left", camera_position[1], theta[1], 0);
  //EstimateCameraParameters(path, "front", "right", camera_position[2], theta[2], 0);
  //EstimateCameraParameters(path, "front", "rear", camera_position[3], theta[3], 0);
  Mat _viewR;
  for (size_t i = 0; i < iCarviews_count; i++)
  {
    vector<Mat> vR_mat;
    cv::FileStorage Extrinsic_file(path + sCarViews[i] + ".xml",
                                   cv::FileStorage::READ);
    if (Extrinsic_file.isOpened())
    {
      Extrinsic_file["Rotation"] >> vR_mat;
    }
    Extrinsic_file.release();
  
    _viewR = vR_mat[0];
    Mat _t = Mat::zeros(1,3,CV_64F);
    _t.at<double>(0,0) = prms->buf_t_gc_ptr[i].x;
    _t.at<double>(0,1) = prms->buf_t_gc_ptr[i].y;
    _t.at<double>(0,2) = prms->buf_t_gc_ptr[i].z;

    if(sCarViews[i] == "front")
      all_Rt[0] = Affine3d(_viewR, _t);
    else if(sCarViews[i] == "right")  
      all_Rt[1] = Affine3d(_viewR, _t);
    else if(sCarViews[i] == "rear")     
      all_Rt[2] = Affine3d(_viewR, _t);
    else if(sCarViews[i] == "left")    
      all_Rt[3] = Affine3d(_viewR, _t);
  }
  path = "./parameters/TI_";
  Construct_calmat(path, all_Rt);
}

void TransformPointsFormat(vector<Point2D_f> object_points, vector<cv::Point2f> img_Points, Point2D_f *chart_points, Point2D_f *corner_points)
{
  for (size_t i = 0; i < object_points.size(); i++)
  {
    chart_points[i].x = object_points[i].x;
    chart_points[i].y = object_points[i].y;
  }
  
  for (size_t i = 0; i < img_Points.size(); i++)
  {
    corner_points[i].x = img_Points[i].x;
    corner_points[i].y = img_Points[i].y;
  }
}

void TransformExtrinsic(tivxPoseEstimationParams *prms, int camera_index, Matx33d &rotation_mtx, Matx31d &translation_vector)
{
  int vIdx = camera_index;
  // rotation_mtx.val[0] = prms->buf_R_gc_ptr[vIdx].xx;  
  // rotation_mtx.val[1] = prms->buf_R_gc_ptr[vIdx].xy;
  // rotation_mtx.val[2] = prms->buf_R_gc_ptr[vIdx].xz;
  // rotation_mtx.val[3] = prms->buf_R_gc_ptr[vIdx].yx;  
  // rotation_mtx.val[4] = prms->buf_R_gc_ptr[vIdx].yy;  
  // rotation_mtx.val[5] = prms->buf_R_gc_ptr[vIdx].yz;
  // rotation_mtx.val[6] = prms->buf_R_gc_ptr[vIdx].zx;  
  // rotation_mtx.val[7] = prms->buf_R_gc_ptr[vIdx].zy;  
  // rotation_mtx.val[8] = prms->buf_R_gc_ptr[vIdx].zz;

  // translation_vector.val[0] = prms->buf_t_gc_ptr[vIdx].x;  
  // translation_vector.val[1] = prms->buf_t_gc_ptr[vIdx].y;  
  // translation_vector.val[2] = prms->buf_t_gc_ptr[vIdx].z;

  rotation_mtx.val[0] = prms->buf_R_cg_ptr[vIdx].xx;  
  rotation_mtx.val[1] = prms->buf_R_cg_ptr[vIdx].xy;
  rotation_mtx.val[2] = prms->buf_R_cg_ptr[vIdx].xz;
  rotation_mtx.val[3] = prms->buf_R_cg_ptr[vIdx].yx;  
  rotation_mtx.val[4] = prms->buf_R_cg_ptr[vIdx].yy;  
  rotation_mtx.val[5] = prms->buf_R_cg_ptr[vIdx].yz;
  rotation_mtx.val[6] = prms->buf_R_cg_ptr[vIdx].zx;  
  rotation_mtx.val[7] = prms->buf_R_cg_ptr[vIdx].zy;  
  rotation_mtx.val[8] = prms->buf_R_cg_ptr[vIdx].zz;
  
  cout<< rotation_mtx << endl;
  Mat R = Mat(rotation_mtx);
  Vec3f eulerangles = rotationMatrixToEulerAngles(R);
  cout<< eulerangles << endl;

  translation_vector.val[0] = prms->buf_t_cg_ptr[vIdx].x;  
  translation_vector.val[1] = prms->buf_t_cg_ptr[vIdx].y;  
  translation_vector.val[2] = prms->buf_t_cg_ptr[vIdx].z;

  cout<< translation_vector << endl;
}