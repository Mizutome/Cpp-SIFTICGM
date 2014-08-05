#ifndef CGVM_H
#define CGVM_H

# include "surflib.h"
# include "opencv2/core/core_c.h"
# include "opencv2/core/core.hpp"
# include "opencv2/imgproc/imgproc_c.h"
# include "opencv2/imgproc/imgproc.hpp"
# include "opencv2/calib3d/calib3d.hpp"
# include "opencv2/objdetect/objdetect.hpp"
# include "opencv2/legacy/compat.hpp"
# include <opencv/highgui.h>
# include <opencv2/nonfree/features2d.hpp>
# include <opencv2/photo/photo.hpp>
# include <opencv2/nonfree/nonfree.hpp>
# include <opencv2/gpu/gpu.hpp>
# include <iostream>
# include <vector>
# include <map>
# include <sys/time.h>

using namespace cv;
using namespace std;

typedef vector<KeyPoint> KpVec;


namespace DepthPIRF
{

struct Vector2D
{
	double X;
	double Y;
	double Slope;
};

struct DistanceRelation
{
    float dis;
    int idx;
};


void SURF(IplImage& img,IpVec& FeatureVec);
void FAST(IplImage& img,KpVec& FeatureVec);

class CGVM
{
	static int cgsum;
public:
	CGVM();
	static Vector2D GetVector(Ipoint& p0,Ipoint& p1);
	static Vector2D GetVector(Point2f p0,Ipoint p1);
	static Point2f GetCG(Ipoint p0,Ipoint p1);
	static Point2f GetCG(Point2f p0,Ipoint p1);
	static Vector2D GetVector(KeyPoint& p0,KeyPoint& p1);
	static Vector2D GetVector(Point2f p0,KeyPoint p1);
	static Point2f GetCG(KeyPoint p0,KeyPoint p1);
	static Point2f GetCG(Point2f p0,KeyPoint p1);
	static double GetLength(Vector2D V);
	static double GetAngle(Vector2D V0,Vector2D V1);
	static double VectorCompare(Vector2D V0,Vector2D V1);
    static void disTable(vector<vector<float> >& table, IpVec& Fea, double* ave=new double(0));
    static void EstimateQueue(const vector<vector<float> >& table, int idx, IpVec& FeaVec, vector<DistanceRelation>& OQ);
	static bool CGVMx(Point2f& CG,Point2f& CG1,IpVec& ipts0,IpVec& ipts1,IpVec& ipts0x,IpVec& ipts1x,IpVec& ipts0y, IpVec& ipts1y, int loops, int GoodSetNum,double t1,double t2);
	static bool CGVMcos(Point2f& CG,Point2f& CG1,IpVec& ipts0,IpVec& ipts1,IpVec& ipts0x,IpVec& ipts1x,IpVec& ipts0y, IpVec& ipts1y, int loops, int GoodSetNum,double t1,double t2,double& diff);
	static bool CGVMCorner(Point2f& CG,Point2f& CG1,KpVec& ipts0,KpVec& ipts1,KpVec& ipts0x,KpVec& ipts1x, KpVec& ipts0y,KpVec& ipts1y,int loops, int GoodSetNum,double t1,double t2);
    static bool ICGM (Point2f& CG,Point2f& CG1,IpVec& ipts0,IpVec& ipts1,const vector<vector<float> >& table0,const vector<vector<float> >& table1,IpVec& ipts0x,IpVec& ipts1x, IpVec& ipts0y,IpVec& ipts1y,int GoodSetNum,double t1,double t2,double& diff);
};


}
#endif // CGVM_H
