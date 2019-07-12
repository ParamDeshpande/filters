#include <System.h>
//#include <Osmap.h>

#include <iostream>
#include <chrono>
#include <deque>
#include <vector>
#include <mutex>
#include <cmath>
#include <cstdlib>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/ext.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaarithm.hpp>
//#include "ajabase/common/types.h"

#include "VizPlayout.h"


#define matval(a,i,j) a.at<float>(i,j)
#define toRAD(theta) (theta * CV_PI/180.0)
#define toDEG(theta) (theta * 180.0/CV_PI)
#define minimum(x, y) ( x < y ? x : y )
#define remap_angle(x) (x < 0 ? x + 360 : x)
#define ang_diff(a, b) ( minimum ( fabs(remap_angle(toDEG(b)) - remap_angle(toDEG(a))), 360.0f - fabs(remap_angle(toDEG(b)) - remap_angle(toDEG(a)) ) ) )

//#define sigma 0.1
//#define ewma(sum,x) (sigma*x + (1-sigma)*sum)

using namespace ORB_SLAM2;


bool fromCV2GLM(const cv::Mat& , glm::mat4* );
bool fromGLM2CV(const glm::mat4& , cv::Mat* );


class Plane
{
public:
	Plane(const std::vector<MapPoint *> &vMPs, const cv::Mat &Tcw);
	Plane(const float &nx, const float &ny, const float &nz, const float &ox, const float &oy, const float &oz);

	void Recompute();

	//normal
	cv::Mat n;
	//origin
	cv::Mat o;
	//arbitrary orientation along normal
	float rang;
	//transformation from world to the plane
	cv::Mat Tpw;
	//pangolin::OpenGlMatrix glTpw;
	//MapPoints that define the plane
	std::vector<MapPoint *> mvMPs;
	//camera pose when the plane was first observed (to compute normal direction)
	cv::Mat mTcw, XC;

	const float eps = 1e-4;

	cv::Mat ExpSO3(const float &x, const float &y, const float &z);

	cv::Mat ExpSO3(const cv::Mat &v);
};

enum FilterType {
	FILTER_ANGLE,
	FILTER_TIMESERIES
};

class Filter {

public:
	Filter(FilterType _type, int _order);
	~Filter();

	//void changeOrder(int _order);

	float input(float val, float auxval = 0.0);
	float output;

protected:
	std::mutex mMutex;
	FilterType type;
	int order;
	std::deque<float> inputs;
	std::deque<float> inputs_aux;
	std::vector<float> coeff;
};

class MADFilter {
public:
	MADFilter(FilterType _type, int _order);
	~MADFilter();

	float input(float val, float auxval = 0.0);
	bool outlier;

protected:
	std::mutex mMutex;
	FilterType type;
	int order;
	float MAD;
	std::deque<float> inputs;
	std::deque<float> inputs_aux;
	std::deque<float> sorted;
};


struct cam_pos {
	Filter *posx;
	Filter *posy;
	Filter *posz;
	Filter *rotx;
	Filter *roty;
	Filter *rotz;
};

struct cam_outlier_detect {
	MADFilter *posx;
	MADFilter *posy;
	MADFilter *posz;
	MADFilter *rotx;
	MADFilter *roty;
	MADFilter *rotz;
};

class SlamUtils
{
public:

	SlamUtils();
	~SlamUtils();

	void Initialize(VizPlayout *_playout);
	void TrackFrame(void* ptrHostVideoBuffer);
	void UpdatePose(VizPlayout *mPlayout);

	void ActivateLocalizationMode();
	void DeactivateLocalizationMode();
	void Reset();
	void SaveMap(string filename);
	void LoadMap(string filename);
	void getState(bool &trackingmode, int &trackstate, int &mappoints, int &numKFs, int &matches, int &VOmatches);

	void sendObjPosition(string slug, float x, float y, float z);
	Plane *DetectPlane(const cv::Mat Tcw, const std::vector<MapPoint *> &vMPs, const int iterations);

	//Simulation Controls
	void setSimulate(bool val);
	bool getSimulate();
	
	float diff_ang(float a, float b);

	
	glm::mat4 getViewMatrix();
	cv::Mat getCurrentImage();

	void triggerPlane();

	float xang, yang, zang, xrad, yrad, zrad;
	float tx, ty, tz;
	cv::Mat T, rx, ry, rz;
	bool anim_tx, anim_ty, anim_tz, anim_ax, anim_ay, anim_az;
	bool bSimulate;
	std::mutex camlock;


	bool mRGB;
	cv::Mat mK;
	cv::Mat mDistCoef;

	// frame rate
	float mFPS, mT;
	float mFx, mFy, mCx, mCy;

	//SLAM Variables
	ORB_SLAM2::System *mSLAM;
	std::mutex slamlock;
	//ORB_SLAM2::Osmap *osmap;
	int mTrackingState;
	cv::Mat mTcw;
	glm::mat4	glm_Tcw;
	cv::Mat Rwc, twc;
	cv::Mat pcorrect;
	vector<ORB_SLAM2::MapPoint *> vMPs; // = mpSLAM->GetTrackedMapPoints();
	vector<cv::KeyPoint> vKeys;         // = mpSLAM->GetTrackedKeyPointsUn();
	bool mLocalizationMode;


	//motion model and outlier checking
	cv::Mat prevmTcw;
	bool bInterpolate;			//flip-flop flag for position interpolation;
	float vx, vy, vz, wx, wy, wz;
	float prevtx, prevty, prevtz;
	float prevxang, prevyang, prevzang;
	float curtx, curty, curtz;
	float curxang, curyang, curzang;
	float rawtx, rawty, rawtz;
	float rawxang, rawyang, rawzang;
	struct cam_pos filter;
	//struct cam_outlier_detect outliers;
	float ang_thresh, pos_thresh;


	cv::Mat currentImage;
	std::mutex imlock;

	Plane *mPlane;
	bool mbPlane;
	cv::Mat o, ocw;


	VizPlayout *playout;
};

