#include "MapRender.h"

#include "slam_utils.h"


#include <mutex>



using namespace ORB_SLAM2;

bool fromCV2GLM(const cv::Mat& cvmat, glm::mat4* glmmat) {
	if (cvmat.cols != 4 || cvmat.rows != 4 || cvmat.type() != CV_32FC1) {
		std::cerr << "Mat_CV2GLM Matrix conversion error!" << std::endl;
		return false;
	}
	memcpy(glm::value_ptr(*glmmat), cvmat.data, 16 * sizeof(float));
	*glmmat = glm::transpose(*glmmat);
	return true;
}

bool fromGLM2CV(const glm::mat4& glmmat, cv::Mat* cvmat) {
	if (cvmat->cols != 4 || cvmat->rows != 4) {
		(*cvmat) = cv::Mat(4, 4, CV_32F);
	}
	memcpy(cvmat->data, glm::value_ptr(glmmat), 16 * sizeof(float));
	*cvmat = cvmat->t();
	return true;
}


SlamUtils::SlamUtils()
{
	mLocalizationMode = false;
	mTrackingState = -1;

	bSimulate = false;
	anim_tx = false;
	anim_ty = false;
	anim_tz = false;
	anim_ax = false;
	anim_ay = false;
	anim_az = false;
	tx = ty = tz = 0.0;
	xang = yang = zang = 0.0;
	xrad = yrad = zrad = 0.0;

	bInterpolate = false;
	prevtx = prevty = prevtz = 0;
	prevxang = prevyang = prevzang = 0;
	rawtx = rawty = rawtz = 0;
	rawxang = rawyang = rawzang = 0;


	int order = 1;
	filter.posx = new Filter(FilterType::FILTER_TIMESERIES, order);
	filter.posy = new Filter(FilterType::FILTER_TIMESERIES, order);
	filter.posz = new Filter(FilterType::FILTER_TIMESERIES, order);

	filter.rotx = new Filter(FilterType::FILTER_ANGLE, order);
	filter.roty = new Filter(FilterType::FILTER_ANGLE, order);
	filter.rotz = new Filter(FilterType::FILTER_ANGLE, order);

/*
	int outlier_order = 6;
	outliers.posx = new MADFilter(FilterType::FILTER_TIMESERIES, outlier_order);
	outliers.posy = new MADFilter(FilterType::FILTER_TIMESERIES, outlier_order);
	outliers.posz = new MADFilter(FilterType::FILTER_TIMESERIES, outlier_order);

	outliers.rotx = new MADFilter(FilterType::FILTER_ANGLE, outlier_order);
	outliers.roty = new MADFilter(FilterType::FILTER_ANGLE, outlier_order);
	outliers.rotz = new MADFilter(FilterType::FILTER_ANGLE, outlier_order);
	*/
	mPlane = NULL;
	o = cv::Mat::zeros(3, 1, CV_32F);
	ocw = cv::Mat::zeros(3, 1, CV_32F);

	mTcw = (cv::Mat_<float>(4, 4) <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1);

	pcorrect = (cv::Mat_<float>(4, 4) <<
		1, 0, 0, 0,
		0, -1, 0, 0,
		0, 0, -1, 0,
		0, 0, 0, 1);


	prevmTcw = cv::Mat::eye(4, 4, CV_32F);

	pos_thresh = 0.05f;
	ang_thresh = 0.5f;
}

SlamUtils::~SlamUtils()
{
	mSLAM->Shutdown();

	if (mPlane)
		delete mPlane;
}

void SlamUtils::Initialize(VizPlayout *_playout)
{
	string mapfolder = "";
	mSLAM = new ORB_SLAM2::System("/home/siddharth/workspace/catkin_ws/src/bm_capture/config/slamvoc.bin", "/home/siddharth/workspace/catkin_ws/src/bm_capture/config/qtracker_config.yaml", ORB_SLAM2::System::MONOCULAR, true);// , true);// , true, mapfolder);

	//osmap = new ORB_SLAM2::Osmap(*mSLAM);

	playout = _playout;

	cv::FileStorage fSettings("/home/siddharth/workspace/catkin_ws/src/bm_capture/config/qtracker_config.yaml", cv::FileStorage::READ);

	mRGB = static_cast<bool>((int)fSettings["Camera.RGB"]);
	mFPS = fSettings["Camera.fps"];

	mT = 1000 / mFPS;

	mFx = fSettings["Camera.fx"];
	mFy = fSettings["Camera.fy"];
	mCx = fSettings["Camera.cx"];
	mCy = fSettings["Camera.cy"];

	//SetCameraCalibration(fx, fy, cx, cy);

	mK = cv::Mat::eye(3, 3, CV_32F);
	mK.at<float>(0, 0) = mFx;
	mK.at<float>(1, 1) = mFy;
	mK.at<float>(0, 2) = mCx;
	mK.at<float>(1, 2) = mCy;

	mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
	mDistCoef.at<float>(0) = fSettings["Camera.k1"];
	mDistCoef.at<float>(1) = fSettings["Camera.k2"];
	mDistCoef.at<float>(2) = fSettings["Camera.p1"];
	mDistCoef.at<float>(3) = fSettings["Camera.p2"];
	const float k3 = fSettings["Camera.k3"];

	if (k3 != 0)
	{
		mDistCoef.resize(5);
		mDistCoef.at<float>(4) = k3;
	}

	mbPlane = false;
}

void SlamUtils::TrackFrame(void *ptrHostVideoBuffer)
{

	if (ptrHostVideoBuffer == NULL)
	{		std::cout<<"Incoming Frame was NULL :: TrackFrame"<<std::endl;
			return;
	}

	if (!bSimulate)
	{


		cv::Mat image(1080, 1920, CV_8UC4, ptrHostVideoBuffer);

		auto current_time = std::chrono::steady_clock::now();
		auto duration_nano = std::chrono::duration<double, std::nano>(current_time.time_since_epoch());
		double timestamp = duration_nano.count();

		//if (image.empty())
		//	return;
		{
			unique_lock<mutex> lock(slamlock);

			mTcw = mSLAM->TrackMonocular(image, timestamp);

			int aux = mSLAM->GetTrackingState();

			if (aux != mTrackingState)
			{
				if (aux <= 1)
				{
					std::cout << "Not Initialized" << std::endl;
					if (mPlane)
						delete mPlane;


				}
				else if (aux == 2)
					std::cout << "Started Tracking" << std::endl;
				else if (aux == 3)
					std::cout << "!Lost Tracking!" << std::endl;

				mTrackingState = aux;
			}

		}

		{
			unique_lock<mutex> lock2(imlock);
			image.copyTo(currentImage);
		}


		if (!mTcw.empty() && mSLAM) {


			mTcw = pcorrect * mTcw;
			//vKeys = mSLAM->GetTrackedKeyPointsUn();

			if (mbPlane == true) {
				vMPs = mSLAM->GetTrackedMapPoints();
				mPlane = DetectPlane(mTcw, vMPs, 50);

				if (mPlane)
				{
					//mPlane->mTcw = (pcorrect * mPlane->mTcw);
					ocw = mPlane->o;
					o = -(mPlane->mTcw.rowRange(0, 3).colRange(0, 3).t()) * mPlane->o;
					//std::cout << "Plane found : \t" << o << std::endl;
					playout->sendPlanePos(-o.at<float>(0)*1000.0f, -o.at<float>(1)*1000.0f, o.at<float>(2)*1000.0f);
				}
				else
					std::cout << "No plane found" << std::endl;

				mbPlane = false;
			}

		}
		
	}


}

/*= cv::Mat::eye(4, 4, CV_32F);
cv::Mat rx = cv::Mat::eye(4, 4, CV_32F);
cv::Mat ry = cv::Mat::eye(4, 4, CV_32F);
cv::Mat rz = cv::Mat::eye(4, 4, CV_32F);*/
float theta = 0;

void SlamUtils::UpdatePose(VizPlayout *mPlayout)
{

	if (bSimulate) {

		theta += CV_PI / 50;

		{
			//unique_lock<mutex> lock(lSimlock);

			if (anim_tx)
				tx = sin(theta);
			if (anim_ty)
				ty = sin(theta);
			if (anim_tz)
				tz = sin(theta);

			if (anim_ax)
			{
				xrad += CV_PI / 200.0;
				xang = toDEG(xrad);
			}
			else
				xrad = toRAD(xang);

			if (anim_ay)
			{
				yrad += CV_PI / 200.0;
				yang = toDEG(yrad);
			}
			else
				yrad = toRAD(yang);

			if (anim_az)
			{
				zrad += CV_PI / 200.0;
				zang = toDEG(zrad);
			}
			else
				zrad = toRAD(zang);

		}

		rx = (cv::Mat_<float>(3, 3) <<
			1, 0, 0,
			0, cos(xrad), sin(xrad),
			0, -sin(xrad), cos(xrad));

		ry = (cv::Mat_<float>(3, 3) <<
			cos(yrad), 0, -sin(yrad),
			0, 1, 0,
			sin(yrad), 0, cos(yrad));

		rz = (cv::Mat_<float>(3, 3) <<
			cos(zrad), sin(zrad), 0,
			-sin(zrad), cos(zrad), 0,
			0, 0, 1);


		mTcw = cv::Mat::eye(4, 4, CV_32F);

		Rwc = ry * rx * rz;
		twc = (cv::Mat_<float>(3, 1) << tx, ty, tz);

		mTcw.rowRange(0, 3).colRange(0, 3) = Rwc.t();

		mTcw.rowRange(0, 3).col(3) = -Rwc.t()*twc;
		mTcw = pcorrect * mTcw;
	}
	else {

		theta = 0;
		tx = ty = tz = 0.0;
		xang = yang = zang = 0.0;
		xrad = yrad = zrad = 0.0;

	}



	if (!mTcw.empty()) {

		//bool bUpdate = false;
		//bool bOutlier = false;

		
		//	float T1 = atan2(glm_Tcw[1][1], -glm_Tcw[1][0]);
		//	float C2 = sqrt(glm_Tcw[0][2] * glm_Tcw[0][2] + glm_Tcw[2][2] * glm_Tcw[2][2]);
		//	float T2 = atan2(C2, glm_Tcw[1][2]);
		//	float S1 = sin(T1);
		//	float C1 = cos(T1);
		//	float T3 = atan2(C1*glm_Tcw[0][0] + S1 * glm_Tcw[0][1], C1*glm_Tcw[2][0] + S1 * glm_Tcw[2][1]);
		//	ezang = T1;
		//	exang = T2;
		//	eyang = T3;
		//
		//
		//	mTcw = mTcw.t();

		//	float T1 = atan2(mTcw.at<float>(1,1), -mTcw.at<float>(1,0));
		//	float C2 = sqrt(mTcw.at<float>(0,2) * mTcw.at<float>(0,2) + mTcw.at<float>(2,2) * mTcw.at<float>(2,2));
		//	float T2 = atan2(C2, mTcw.at<float>(1,2));
		//	float S1 = sin(T1);
		//	float C1 = cos(T1);
		//	float T3 = atan2(C1*mTcw.at<float>(0,0) + S1 * mTcw.at<float>(0,1), C1*mTcw.at<float>(2,0) + S1 * mTcw.at<float>(2,1));
		//	ezang = T1;
		//	exang = T2;
		//	eyang = T3;

		//	mTcw = mTcw.t();

		Rwc = mTcw.rowRange(0, 3).colRange(0, 3).t(); // Rotation information
		twc = -Rwc * mTcw.rowRange(0, 3).col(3);      // Translation information

		cv::Mat Twc = cv::Mat::eye(4, 4, CV_32F);

		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
			{
				//if (matval(mTcw, i, j) != matval(prevmTcw, i, j))
				//bUpdate = true;

				//if (i < 3 && j < 3)
				matval(Twc, i, j) = matval(Rwc, i, j);
			}

		matval(Twc, 0, 3) = matval(twc, 0, 0);
		matval(Twc, 1, 3) = matval(twc, 0, 1);
		matval(Twc, 2, 3) = matval(twc, 0, 2);

		//std::cout << Twc << std::endl;

		MapRender::setCameraWorldPos(Twc);

		float exang, eyang, ezang;

		{
			unique_lock<mutex> lock(camlock);
			fromCV2GLM(mTcw, &glm_Tcw);
		}

		glm::extractEulerAngleZXY(glm_Tcw, ezang, exang, eyang);

/*
	100Hz
	{
		isOUtlier? = MADFilter.input([x, y, z, xang, yang, zang]);

		if(isOUtlier)
			Correction logic?
			if(yes)
				correct value
			else	
				reject value


	if(corrected)
		if(isFreshValueFromFrame?)
			Kalman Filter
		else
			interpolate using velocity information from KF
	}
*/
#define outlier_thresh 0.05f
#define angle_thresh 0.5f

	//	if (((fabs(twc.at<float>(0, 0) - rawtx) >= outlier_thresh) || (fabs(twc.at<float>(0, 1) - rawty) >= outlier_thresh) || (fabs(twc.at<float>(0, 2) - rawtz) >= outlier_thresh)))
//		{
//			bOutlier = true;
//			std::cout << "outlier POS: " << fabs(twc.at<float>(0, 0) - rawtx) << " " << fabs(twc.at<float>(0, 1) - rawty) << " " << fabs(twc.at<float>(0, 2) - rawtz) << " " << std::endl;
//		}

		
		//////if(bUpdate &&(outliers.rotx->input(exang) || outliers.roty->input(eyang) || outliers.rotz->input(ezang)))

		////if ( (diff_ang(rawxang, exang) >= ang_thresh) || (diff_ang(rawyang, eyang) >= ang_thresh) || (diff_ang(rawzang, ezang) >= ang_thresh))
		
		//if ((fabs(exang - rawxang) >= angle_thresh) || (fabs(eyang - rawyang) >= angle_thresh) || (fabs(ezang - rawzang) >= angle_thresh))

		bool bX = false;
		bool bY = false;
		bool bZ = false;
/*

		if( fabs(exang + rawxang) < angle_thresh )// || (fabs(eyang) - fabs(rawyang) < angle_thresh) || (fabs(ezang) - fabs(rawzang) < angle_thresh))
		{	
			
			//bOutlier = true;
			//std::cout << "outlier ANG: " << diff_ang(rawxang, exang) << " " << diff_ang(rawyang, eyang) << " " << diff_ang(rawzang, ezang) << " " << std::endl;
			std::cout << "outlier ANG X: " << exang << " " << rawxang << " " << exang + rawxang << std::endl;
			bX = true;
			
		}

		//if (fabs(eyang) - fabs(rawyang) < angle_thresh)
		{
			//bOutlier = true;
			//std::cout << "outlier ANG: " << diff_ang(rawxang, exang) << " " << diff_ang(rawyang, eyang) << " " << diff_ang(rawzang, ezang) << " " << std::endl;
			//std::cout << "outlier ANG Y: " << eyang << " " << rawyang << " " << eyang + rawyang << " " << eyang - rawyang <<  std::endl;

		}
		
		if ( fabs(ezang + rawzang - CV_PI) < angle_thresh)
		{
			
			//bOutlier = true;
			//std::cout << "outlier ANG: " << diff_ang(rawxang, exang) << " " << diff_ang(rawyang, eyang) << " " << diff_ang(rawzang, ezang) << " " << std::endl;
			std::cout << "outlier ANG Z: " << ezang << " " << rawzang << " " << ezang + rawzang - CV_PI << std::endl;
			bZ = true;

		}
		
*/
/*
		if (bUpdate && !bOutlier)
		{
		
			//std::cout << "normal"<<std::endl;
			curtx = filter.posx->input(twc.at<float>(0, 0));
			curty = filter.posy->input(twc.at<float>(0, 1));
			curtz = filter.posz->input(twc.at<float>(0, 2));

			curxang = filter.rotx->input(exang);
			curyang = filter.roty->input(eyang);
			curzang = filter.rotz->input(ezang);

			vx = curtx - prevtx;
			vy = curty - prevty;
			vz = curtz - prevtz;

			wx = curxang - prevxang;
			wy = curyang - prevyang;
			wz = curzang - prevzang;

		}

		else //if(!bUpdate && bOutlier)
		{
			//std::cout << "interpolating" << std::endl;

			curtx = curtx + vx / 2;
			curty = curty + vy / 2;
			curtz = curtz + vz / 2;

			curxang = curxang + wx / 2;
			curyang = curyang + wy / 2;
			curzang = curzang + wz / 2;

		}
		
		
		else
		{
			curtx = curtx + vx;
			curty = curty + vy;
			curtz = curtz + vz;

			curxang = curxang + wx;
			curyang = curyang + wy;
			curzang = curzang + wz;
		}
		

		prevtx = curtx;
		prevty = curty;
		prevtz = curtz;

		prevxang = curxang;
		prevyang = curyang;
		prevzang = curzang;

		rawtx = twc.at<float>(0, 0);
		rawty = twc.at<float>(0, 1);
		rawtz = twc.at<float>(0, 2);

		rawxang = exang;
		rawyang = eyang;
		rawzang = ezang;
*/
		//bInterpolate = false;

		{
			unique_lock<mutex> lock(mPlayout->mMutexPlayout);
			
			/*if (bInterpolate) 
			{

				mPlayout->mTrackingData.posx = curtx + vx / 2;
				mPlayout->mTrackingData.posy = curty + vy / 2;
				mPlayout->mTrackingData.posz = curtz + vz / 2;

				mPlayout->mTrackingData.rotx = curxang + wx / 2;
				mPlayout->mTrackingData.roty = curyang + wy / 2;
				mPlayout->mTrackingData.rotz = curzang + wz / 2;
			}*/
			//else if (!bInterpolate)
			//{
			mPlayout->mTrackingData.posx = filter.posx->input(twc.at<float>(0, 0));
			mPlayout->mTrackingData.posy = filter.posy->input(twc.at<float>(0, 1));
			mPlayout->mTrackingData.posz = filter.posz->input(twc.at<float>(0, 2));

			mPlayout->mTrackingData.rotx = filter.rotx->input(exang);
			mPlayout->mTrackingData.roty = filter.roty->input(eyang);
			mPlayout->mTrackingData.rotz = filter.rotz->input(ezang);
		//}

		}



		//bInterpolate = !bInterpolate;
		//prevmTcw = mTcw.clone();

	}
	
}

void SlamUtils::ActivateLocalizationMode()
{
	mSLAM->ActivateLocalizationMode();
	mLocalizationMode = true;
}

void SlamUtils::DeactivateLocalizationMode()
{
	mSLAM->DeactivateLocalizationMode();
	mLocalizationMode = false;
}

void SlamUtils::Reset() {
	unique_lock<mutex> lock(slamlock);
	mSLAM->Shutdown();
	delete mSLAM;
	mSLAM = new ORB_SLAM2::System("/home/siddharth/workspace/catkin_ws/src/bm_capture/config/slamvoc.bin", "/home/siddharth/workspace/catkin_ws/src/bm_capture/config/qtracker_config.yaml", ORB_SLAM2::System::MONOCULAR, true);// , true);// , true, mapfolder);
}

void SlamUtils::SaveMap(string filename)
{
	//unique_lock<mutex> lock(slamlock);
	//mSLAM->SaveMap(filename);
}

void SlamUtils::LoadMap(string filename)
{
	//unique_lock<mutex> lock(slamlock);
	//mSLAM->Shutdown();
	//delete mSLAM;
	//mSLAM = new ORB_SLAM2::System("slamvoc.bin", "qtracker_config.yaml", ORB_SLAM2::System::MONOCULAR, true, true, filename);// , true, mapfolder);
	////mSLAM->LoadMap(filename);
}

void SlamUtils::getState(bool & trackingmode, int & trackstate, int & mappoints, int & numKFs, int & matches, int & VOmatches)
{
	mSLAM->getCurrentTrackingStatus(trackingmode, trackstate, mappoints, numKFs, matches, VOmatches);
}

void SlamUtils::setSimulate(bool val) {
	bSimulate = val;
}

bool SlamUtils::getSimulate() {
	return bSimulate;
}

float adeg, bdeg, diff, diff_comp, retval;


float SlamUtils::diff_ang(float a, float b)
{
	//ang_diff(a, b);
	/*
	adeg = a * 180.0 / CV_PI;
	bdeg = b * 180.0f / CV_PI;

	diff = fabs(bdeg - adeg);
	diff_comp = 360.f - diff;

	if (diff < diff_comp)
		retval =  diff;
	else
		retval = diff_comp;

	return retval;*/

	return (180.f - fabsf(fmodf(fabsf(toDEG(a) - toDEG(b)), 360.0) - 180.0f));
}

glm::mat4 SlamUtils::getViewMatrix() {

	unique_lock<mutex> lock(camlock);

	glm::mat4 aux(glm_Tcw);
	return aux;
	//fromCV2GLM(mTcw, &glm_Tcw);
	//return glm_Tcw;
}

cv::Mat SlamUtils::getCurrentImage()
{
	unique_lock<mutex> lock(imlock);

	return currentImage.clone();
}

void SlamUtils::triggerPlane() {
	mbPlane = true;
}


Plane::Plane(const std::vector<MapPoint *> &vMPs, const cv::Mat &Tcw) : mvMPs(vMPs), mTcw(Tcw.clone())
{
	rang = -3.14f / 2 + ((float)rand() / RAND_MAX) * 3.14f;
	Recompute();
}

void Plane::Recompute()
{
	const int N = mvMPs.size();

	// Recompute plane with all points
	cv::Mat A = cv::Mat(N, 4, CV_32F);
	A.col(3) = cv::Mat::ones(N, 1, CV_32F);

	o = cv::Mat::zeros(3, 1, CV_32F);

	int nPoints = 0;
	for (int i = 0; i < N; i++)
	{
		MapPoint *pMP = mvMPs[i];
		if (!pMP->isBad())
		{
			cv::Mat Xw = pMP->GetWorldPos();
			o += Xw;
			A.row(nPoints).colRange(0, 3) = Xw.t();
			nPoints++;
		}
	}
	A.resize(nPoints);

	cv::Mat u, w, vt;
	cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

	float a = vt.at<float>(3, 0);
	float b = vt.at<float>(3, 1);
	float c = vt.at<float>(3, 2);

	o = o * (1.0f / nPoints);
	const float f = 1.0f / sqrt(a * a + b * b + c * c);

	// Compute XC just the first time
	if (XC.empty())
	{
		cv::Mat Oc = -mTcw.colRange(0, 3).rowRange(0, 3).t() * mTcw.rowRange(0, 3).col(3);
		XC = Oc - o;
	}

	if ((XC.at<float>(0) * a + XC.at<float>(1) * b + XC.at<float>(2) * c) > 0)
	{
		a = -a;
		b = -b;
		c = -c;
	}

	const float nx = a * f;
	const float ny = b * f;
	const float nz = c * f;

	n = (cv::Mat_<float>(3, 1) << nx, ny, nz);

	cv::Mat up = (cv::Mat_<float>(3, 1) << 0.0f, 1.0f, 0.0f);

	cv::Mat v = up.cross(n);
	const float sa = cv::norm(v);
	const float ca = up.dot(n);
	const float ang = atan2(sa, ca);
	Tpw = cv::Mat::eye(4, 4, CV_32F);

	Tpw.rowRange(0, 3).colRange(0, 3) = ExpSO3(v * ang / sa) * ExpSO3(up * rang);
	o.copyTo(Tpw.col(3).rowRange(0, 3));

	/*
	glTpw.m[0] = Tpw.at<float>(0,0);
	glTpw.m[1] = Tpw.at<float>(1,0);
	glTpw.m[2] = Tpw.at<float>(2,0);
	glTpw.m[3]  = 0.0;

	glTpw.m[4] = Tpw.at<float>(0,1);
	glTpw.m[5] = Tpw.at<float>(1,1);
	glTpw.m[6] = Tpw.at<float>(2,1);
	glTpw.m[7]  = 0.0;

	glTpw.m[8] = Tpw.at<float>(0,2);
	glTpw.m[9] = Tpw.at<float>(1,2);
	glTpw.m[10] = Tpw.at<float>(2,2);
	glTpw.m[11]  = 0.0;

	glTpw.m[12] = Tpw.at<float>(0,3);
	glTpw.m[13] = Tpw.at<float>(1,3);
	glTpw.m[14] = Tpw.at<float>(2,3);
	glTpw.m[15]  = 1.0;

	*/
}

Plane::Plane(const float &nx, const float &ny, const float &nz, const float &ox, const float &oy, const float &oz)
{
	n = (cv::Mat_<float>(3, 1) << nx, ny, nz);
	o = (cv::Mat_<float>(3, 1) << ox, oy, oz);

	cv::Mat up = (cv::Mat_<float>(3, 1) << 0.0f, 1.0f, 0.0f);

	cv::Mat v = up.cross(n);
	const float s = cv::norm(v);
	const float c = up.dot(n);
	const float a = atan2(s, c);
	Tpw = cv::Mat::eye(4, 4, CV_32F);
	const float rang = -3.14f / 2 + ((float)rand() / RAND_MAX) * 3.14f;
	cout << rang;
	Tpw.rowRange(0, 3).colRange(0, 3) = ExpSO3(v * a / s) * ExpSO3(up * rang);
	o.copyTo(Tpw.col(3).rowRange(0, 3));

	/*
	glTpw.m[0] = Tpw.at<float>(0,0);
	glTpw.m[1] = Tpw.at<float>(1,0);
	glTpw.m[2] = Tpw.at<float>(2,0);
	glTpw.m[3]  = 0.0;

	glTpw.m[4] = Tpw.at<float>(0,1);
	glTpw.m[5] = Tpw.at<float>(1,1);
	glTpw.m[6] = Tpw.at<float>(2,1);
	glTpw.m[7]  = 0.0;

	glTpw.m[8] = Tpw.at<float>(0,2);
	glTpw.m[9] = Tpw.at<float>(1,2);
	glTpw.m[10] = Tpw.at<float>(2,2);
	glTpw.m[11]  = 0.0;

	glTpw.m[12] = Tpw.at<float>(0,3);
	glTpw.m[13] = Tpw.at<float>(1,3);
	glTpw.m[14] = Tpw.at<float>(2,3);
	glTpw.m[15]  = 1.0;
*/
}

void SlamUtils::sendObjPosition(string slug, float x, float y, float z)
{
	playout->sendSlugPos(slug, x, y, z);
}

Plane *SlamUtils::DetectPlane(const cv::Mat Tcw, const std::vector<MapPoint *> &vMPs, const int iterations)
{
	// Retrieve 3D points
	vector<cv::Mat> vPoints;
	vPoints.reserve(vMPs.size());
	vector<MapPoint *> vPointMP;
	vPointMP.reserve(vMPs.size());

	for (size_t i = 0; i < vMPs.size(); i++)
	{
		MapPoint *pMP = vMPs[i];
		if (pMP)
		{
			if (pMP->Observations() > 5)
			{
				vPoints.push_back(pMP->GetWorldPos());
				vPointMP.push_back(pMP);
			}
		}
	}

	const int N = vPoints.size();

	if (N < 50)
		return NULL;

	// Indices for minimum set selection
	vector<size_t> vAllIndices;
	vAllIndices.reserve(N);
	vector<size_t> vAvailableIndices;

	for (int i = 0; i < N; i++)
	{
		vAllIndices.push_back(i);
	}

	float bestDist = 1e10;
	vector<float> bestvDist;

	//RANSAC
	for (int n = 0; n < iterations; n++)
	{
		vAvailableIndices = vAllIndices;

		cv::Mat A(3, 4, CV_32F);
		A.col(3) = cv::Mat::ones(3, 1, CV_32F);

		// Get min set of points
		for (short i = 0; i < 3; ++i)
		{
			int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size() - 1);

			int idx = vAvailableIndices[randi];

			A.row(i).colRange(0, 3) = vPoints[idx].t();

			vAvailableIndices[randi] = vAvailableIndices.back();
			vAvailableIndices.pop_back();
		}

		cv::Mat u, w, vt;
		cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

		const float a = vt.at<float>(3, 0);
		const float b = vt.at<float>(3, 1);
		const float c = vt.at<float>(3, 2);
		const float d = vt.at<float>(3, 3);

		vector<float> vDistances(N, 0);

		const float f = 1.0f / sqrt(a * a + b * b + c * c + d * d);

		for (int i = 0; i < N; i++)
		{
			vDistances[i] = fabs(vPoints[i].at<float>(0) * a + vPoints[i].at<float>(1) * b + vPoints[i].at<float>(2) * c + d) * f;
		}

		vector<float> vSorted = vDistances;
		sort(vSorted.begin(), vSorted.end());

		int nth = max((int)(0.2 * N), 20);
		const float medianDist = vSorted[nth];

		if (medianDist < bestDist)
		{
			bestDist = medianDist;
			bestvDist = vDistances;
		}
	}

	// Compute threshold inlier/outlier
	const float th = 1.4 * bestDist;
	vector<bool> vbInliers(N, false);
	int nInliers = 0;
	for (int i = 0; i < N; i++)
	{
		if (bestvDist[i] < th)
		{
			nInliers++;
			vbInliers[i] = true;
		}
	}

	vector<MapPoint *> vInlierMPs(nInliers, NULL);
	int nin = 0;
	for (int i = 0; i < N; i++)
	{
		if (vbInliers[i])
		{
			vInlierMPs[nin] = vPointMP[i];
			nin++;
		}
	}

	return new Plane(vInlierMPs, Tcw);
}

cv::Mat Plane::ExpSO3(const float &x, const float &y, const float &z)
{
	cv::Mat I = cv::Mat::eye(3, 3, CV_32F);
	const float d2 = x * x + y * y + z * z;
	const float d = sqrt(d2);
	cv::Mat W = (cv::Mat_<float>(3, 3) << 0, -z, y,
		z, 0, -x,
		-y, x, 0);
	if (d < eps)
		return (I + W + 0.5f * W * W);
	else
		return (I + W * sin(d) / d + W * W * (1.0f - cos(d)) / d2);
}

cv::Mat Plane::ExpSO3(const cv::Mat &v)
{
	return ExpSO3(v.at<float>(0), v.at<float>(1), v.at<float>(2));
}


Filter::Filter(FilterType _type, int _order) {

	type = _type;
	order = _order;

	for (int i = 0; i < order; i++) {
		inputs.push_back(0);
		inputs_aux.push_back(0);
		coeff.push_back(1.0);
	}


}

Filter::~Filter() {}

/*
void Filter::changeOrder(int _order) {

unique_lock<mutex> lock(mMutex);


order = _order;
inputs.resize(order, 0);
inputs_aux.resize(order, 0);
coeff.resize(order, 0);


for (int i = 0; i < order; i++) {
inputs.push_back(0);
inputs_aux.push_back(0);
coeff.push_back(1.0);
}


}
*/

float Filter::input(float val, float auxval) {

	unique_lock<mutex> lock(mMutex);


	inputs.pop_front();
	inputs_aux.pop_front();

	inputs.push_back(val);
	inputs_aux.push_back(auxval);

	//std::cout << "DEBUG: size = " << inputs.size() << " [ ";
	output = 0.0;

	double avgCos = 0, avgSin = 0;


	switch (type) {
	case FilterType::FILTER_ANGLE:

		for (int i = 0; i < order; i++) {

			avgSin += sin(inputs.at(i));
			avgCos += cos(inputs.at(i));//inputs_aux.at(i);

		}

		output = remap_angle(atan2(avgSin, avgCos) * 180.0 / CV_PI);


		break;

	case FilterType::FILTER_TIMESERIES:

		for (int i = 0; i < order; i++) {
			output += inputs.at(i);
		}

		output = output / (float)order;

		break;

	}

	return output;

}

MADFilter::MADFilter(FilterType _type, int _order)
{
	type = _type;
	order = _order;
	
	//inputs.resize(order);
	inputs.push_back(0.01f);
	/*
	for (int i = 0; i < order; i++) {
		inputs.push_back(0);
		inputs_aux.push_back(0);
		sorted.push_back(1.0);
	}
	*/


}

MADFilter::~MADFilter()
{

}

float MADFilter::input(float val, float auxval)
{
	//unique_lock<mutex> lock(mMutex);
	int num=0;

	if (inputs.size() < order)
		num = inputs.size();
	else
		num = order;

	sorted.clear();

	sorted = inputs;

	if(sorted.size() > 0)
	std::sort(sorted.begin(), sorted.end());
/*
	std::cout << " Inputs = ";
	
	for (int i = 0; i < num; i++) {
		std::cout << inputs.at(i);
	}
	std::cout << std::endl;
	
	std::cout << " Sorted Inputs = ";
	for (int i = 0; i < num; i++) {
		std::cout << sorted.at(i);
	}
	std::cout << std::endl;
	*/
	float median = 0.0f;

	if (sorted.size() > 0) 
		median = sorted.at(num/2);

	//std::cout << " median = "<< median<< std::endl;

	//output = 0.0;

	float MADSin = 0, MADCos = 0;
	float medSin = 0, medCos = 0;

	switch (type) {
	case FilterType::FILTER_ANGLE:

		////medSin = sin(median);
		////medCos = cos(median);

		//for (int i = 0; i < order; i++) {

		//	//MADSin += sin(inputs.at(i)) - medSin;
		//	//MADCos += cos(inputs.at(i)) - medCos;//inputs_aux.at(i);

		//}

		////remap_angle(atan2(avgSin, avgCos) * 180.0 / CV_PI);


		//MAD /= order;

		//if (val >= 2 * MAD)
		//	outlier = true;
		//else
		//{
		//	outlier = false;
		//	inputs.pop_front();
		//	inputs.push_back(val);
		//}

		for (int i = 0; i < num; i++) {

			MAD += fabs(inputs.at(i) - median);

		}

		MAD /= num;

		if (fabs(val - median) >= 4 * MAD)
			outlier = true;
		else
		{
			outlier = false;
			if (inputs.size() >= order)
			{
				inputs.pop_front();
			}
			inputs.push_back(val);
		}

		break;

	case FilterType::FILTER_TIMESERIES:

		for (int i = 0; i < num; i++) {
			
			MAD += fabs(inputs.at(i) - median);

		}

		MAD /= num;

		if (fabs(val - median) >= 4 * MAD)
			outlier = true;
		else
		{
			outlier = false;
			if (inputs.size() >= order)
			{
				inputs.pop_front();
			}

			inputs.push_back(val);
		}

		break;

	}

	return outlier;
}


