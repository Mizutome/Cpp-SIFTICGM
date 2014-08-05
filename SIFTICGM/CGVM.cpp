#include "CGVM.h"

using namespace DepthPIRF;
using namespace cv;
using namespace std;

CGVM::CGVM()
{
}

int CGVM::cgsum=0;

void DepthPIRF::SURF(IplImage& img,IpVec& FeatureVec)
{
    surfDetDes(&img, FeatureVec, false, 4, 4, 1, 0.0001f);
}

void DepthPIRF::FAST(IplImage& img,KpVec& FeatureVec)
{
    FastFeatureDetector detector(50);

    detector.detect( Mat(&img), FeatureVec );

    for(int i=0;i<FeatureVec.size();i++) FeatureVec[i].class_id=i;
}

Vector2D CGVM::GetVector(Ipoint& p0,Ipoint& p1)
{
    Vector2D v;
    v.X=p1.x-p0.x;
    v.Y=p1.y-p0.y;
    v.Slope=v.Y/v.X;
    return v;
}

Vector2D CGVM::GetVector(Point2f p0,Ipoint p1)
{
    Vector2D v;
    v.X=p1.x-p0.x;
    v.Y=p1.y-p0.y;
    v.Slope=v.Y/v.X;
    return v;
}

Point2f CGVM::GetCG(Ipoint p0,Ipoint p1)
{
    Point2f p;
    p.x=(p0.x+p1.x)/2;
    p.y=(p0.y+p1.y)/2;
    cgsum=2;
    return p;
}

Point2f CGVM::GetCG(Point2f p0,Ipoint p1)
{
    Point2f p;
    double rp=(double)1/cgsum;
    double rp1=(double)cgsum/(cgsum+1);
    p.x=(p0.x+rp*p1.x)*rp1;
    p.y=(p0.y+rp*p1.y)*rp1;
    cgsum++;
    return p;
}

Vector2D CGVM::GetVector(KeyPoint& p0,KeyPoint& p1)
{
    Vector2D v;
    v.X=p1.pt.x-p0.pt.x;
    v.Y=p1.pt.y-p0.pt.y;
    v.Slope=v.Y/v.X;
    return v;
}

Vector2D CGVM::GetVector(Point2f p0,KeyPoint p1)
{
    Vector2D v;
    v.X=p1.pt.x-p0.x;
    v.Y=p1.pt.y-p0.y;
    v.Slope=v.Y/v.X;
    return v;
}

Point2f CGVM::GetCG(KeyPoint p0,KeyPoint p1)
{
    Point2f p;
    p.x=(p0.pt.x+p1.pt.x)/2;
    p.y=(p0.pt.y+p1.pt.y)/2;
    cgsum=2;
    return p;
}

Point2f CGVM::GetCG(Point2f p0,KeyPoint p1)
{
    Point2f p;
    double rp=(double)1/cgsum;
    double rp1=(double)cgsum/(cgsum+1);
    p.x=(p0.x+rp*p1.pt.x)*rp1;
    p.y=(p0.y+rp*p1.pt.y)*rp1;
    cgsum++;
    return p;
}

double CGVM::GetLength(Vector2D V)
{
    double l;
    l=sqrt(pow(V.X,2)+pow(V.Y,2));
    return l;
}

double CGVM::GetAngle(Vector2D V0,Vector2D V1)
{
    double a;
    double b,c;
    b=(double)V0.X*V1.X+(double)V0.Y*V1.Y;
    c=(double)GetLength(V0)*GetLength(V1);
    if(b==c) return 0;
    a=acos(b/c);
    if((V0.X*V1.Y-V0.Y*V1.X)<0) a=-a;
    //	cout<<b<<" "<<c<<" aaa "<<a<<endl;
    return a;
}

double CGVM::VectorCompare(Vector2D V0,Vector2D V1)
{
    double l,a,b;
    l=sqrt(pow((V0.X-V1.X),2)+pow((V0.Y-V1.Y),2));
    a=CGVM::GetLength(V0);
    b=CGVM::GetLength(V1);
    l=(2*l)/(a+b);
    return l;
}

//bool CGVM::CGVMCorner(Point2f& CG,Point2f& CG1,KpVec& ipts0,KpVec& ipts1,KpVec& ipts0x,KpVec& ipts1x, KpVec& ipts0y,KpVec& ipts1y,int loops, int GoodSetNum,double t1,double t2)
//{
//	map<int,KeyPoint> iptsm0,iptsm1;
//	for(int i=0;i<ipts0.size();i++ ) iptsm0[i]=ipts0[i];
//	for(int i=0;i<ipts1.size();i++ ) iptsm0[i]=ipts1[i];
//	vector<int> TestSet;
//	vector<bool> GoodPoints;
//	GoodPoints.resize(ipts0.size());
//	struct timeval tpstart;
//	gettimeofday(&tpstart,NULL);
//	for(int i;i<ipts0.size();++i) GoodPoints[i]=true;
//	TestSet.resize(100);
//	bool GoodSet=false;
//	int SetSize=GoodSetNum-1;
//	int loop=0;
//	int loopX=0;
//	double diffSlope=0;
//	double sc;
//	int SUM;
//
//	srand(tpstart.tv_usec);
//
//	CG=Point2f(0,0);
//	CG1=Point2f(0,0);
//	Vector2D Testv,Testv1;
//	double Testx,Testx1;
//	bool GetGoodSet=true;
//	sc=0;
//
//	vector<Vector2D> vecVec;
//
//	ipts0x.clear();
//	ipts1x.clear();
//	ipts0y.clear();
//	ipts1y.clear();
//
//
//
//	//    for(int i=0;i<ipts0.size();++i)
//	//    {
//	//        Testv=CGVM::GetVector(CG,ipts0[i]);
//	//        Testx=CGVM::GetLength(Testv);
//	//        Testv1=CGVM::GetVector(CG1,ipts1[i]);
//	//        Testx1=CGVM::GetLength(Testv1);
//
//	//        Vector2D Testk;
//
//	//        Testk.X=Testv.X-Testv1.X;
//
//	//        Testk.Y=Testv.Y-Testv1.Y;
//
//	//        vecVec.push_back(Testk);
//
//	//        cout<<"x "<<Testk.X<<" y "<<Testv1.Y<<endl;
//	//    }
//
//	//    CvScalar color_tab[5] =
//	//    { CV_RGB (255, 0, 0), CV_RGB (0, 255, 0), CV_RGB (100, 100, 255), CV_RGB (255, 0, 255), CV_RGB (255, 255, 0) };
//
//	//    IplImage *img = cvCreateImage (cvSize (5184, 3456), IPL_DEPTH_8U, 3);
//
//	//    cvZero (img);
//
//	//    for(int i=0;i<vecVec.size();++i)
//	//    {
//	//        CvPoint ipt;
//	//        ipt.x=vecVec[i].X+2529;
//	//        ipt.y=vecVec[i].Y+1728;
//	//        cvCircle (img, ipt, 5, color_tab[1], CV_FILLED, CV_AA, 0);
//	//    }
//
//	//    cvSaveImage("z.png",img);
//
//	//    return false;
//
//
//
//
//
//	while(!GoodSet)
//	{
//		for(int i=0;i<=SetSize;i++)
//		{
//			bool noRepeat=false;
//			while(!noRepeat)
//			{
//				TestSet[i]=rand()%ipts0.size();
//				if(!i) break;
//				else
//				{
//					for(int j=0;j<=SetSize;j++)
//					{
//						if(i==j) ;
//						else if(TestSet[i]==TestSet[j]) break;
//						else
//						{
//							noRepeat=true;
//							break;
//						}
//					}
//				}
//			}
//		}
//		CG=CGVM::GetCG(ipts0[TestSet[0]],ipts0[TestSet[1]]);
//		CG1=CGVM::GetCG(ipts1[TestSet[0]],ipts1[TestSet[1]]);
//		Vector2D Testv,Testv1;
//		double Testx,Testx1;
//		bool GetGoodSet=true;
//		sc=0;
//		diffSlope=0;
//		for(int i=2;i<=SetSize;++i)
//		{
//			Testv=CGVM::GetVector(CG,ipts0[TestSet[i]]);
//			Testx=CGVM::GetLength(Testv);
//			Testv1=CGVM::GetVector(CG1,ipts1[TestSet[i]]);
//			Testx1=CGVM::GetLength(Testv1);
//			sc+=CGVM::VectorCompare(Testv,Testv1);
//			//            cout<<atan(Testv.Slope)-atan(Testv1.Slope)<<endl;
//			if(i==2) diffSlope=abs(atan(Testv.Slope)-atan(Testv1.Slope));
//			if(CGVM::VectorCompare(Testv,Testv1)>=0.07||abs(abs(atan(Testv.Slope)-atan(Testv1.Slope))-diffSlope)>=t1)
//			{
//				GetGoodSet=false;
//				break;
//			}
//			else
//			{
//				CG=CGVM::GetCG(CG,ipts0[TestSet[i]]);
//				CG1=CGVM::GetCG(CG1,ipts1[TestSet[i]]);
//				diffSlope+=abs(atan(Testv.Slope)-atan(Testv1.Slope));
//				diffSlope/=i;
//				SUM=i;
//			}
//		}
//		GoodSet=GetGoodSet;
//
//		if(++loop>100)
//		{
//			loop=0;
//			t1+=t1*0.05;
//			t2=t1;
//		}
//
//		if(++loopX>1000)
//		{
//			return false;
//		}
//	}
//
//	//    cout<<"loop "<<loop<<endl;
//
//	sc/=(SetSize-2);
//
//	//    cout<<"SC!!!! "<<sc<<endl;
//
//	//    cout<<"CG!!!!!!!"<<CG.x<<" "<<CG.y<<" "<<CG1.x<<" "<<CG1.y<<endl;
//
//	for(int i=0;i<ipts0.size();++i)
//	{
//		bool NeedTest=true;
//		for(int k=0;k<=SetSize;k++) if(TestSet[k]==i) NeedTest=false;
//		if(!NeedTest) continue;
//		Vector2D v,v1;
//		double x,x1;
//		v=CGVM::GetVector(CG,ipts0[i]);
//		x=CGVM::GetLength(v);
//		v1=CGVM::GetVector(CG1,ipts1[i]);
//		x1=CGVM::GetLength(v1);
//
//		Vector2D Testk;
//
//		double m;
//
//		Testk.X=v.X-v1.X;
//
//		Testk.Y=v.Y-v1.Y;
//
//		Testk.Slope=Testk.Y/Testk.X;
//
//		vecVec.push_back(Testk);
//
//		//        Vector2D Testkx;
//
//		//        Testkx.Slope=Testk.Slope;
//
//		//        double l=(CGVM::VectorCompare(v,v1)-sc)*1728;
//
//		//        double a=atan(Testkx.Slope);
//
//		//        Testkx.X=l*cos(a)*m;
//
//		//        Testkx.Y=l*sin(a);
//
//
//
//
//
//		CvPoint px0,px1;
//		px0.x=CG.x;
//		px0.y=CG.y;
//		px1.x=ipts0[i].x;
//		px1.y=ipts0[i].y;
//		//        cvDrawLine(p0,px0,px1,CV_RGB( 255, 0, 0 ));
//		//        cout<<"x "<<v.X<<" y "<<v.Y<<" Slope "<<atan(v.Slope)<<" x1 "<<v1.X<<" y1 "<<v1.Y<<" Slope1 "<<atan(v1.Slope)<<"   "<<abs(1-(x/x1))<<endl;
//
//		if(CGVM::VectorCompare(v,v1)-sc>=0.03||abs(abs(atan(v.Slope)-atan(v1.Slope))-diffSlope)>=t2)
//			//        if(abs(atan(v.Slope)-atan(v1.Slope))>=0.03)
//		{
//			GoodPoints[i]=false;
//			//            cout<<CGVM::VectorCompare(v,v1)-sc<<" Moved!"<<endl;
//		}
//		else
//		{
//			CG=CGVM::GetCG(CG,ipts0[i]);
//			CG1=CGVM::GetCG(CG1,ipts1[i]);
//			diffSlope+=abs(atan(v.Slope)-atan(v1.Slope));
//			diffSlope/=++SUM;
//
//
//			//            cout<<vecVec.back().X<<" "<<vecVec.back().Y<<endl;
//			//            cout<<"CG!!"<<CG.x<<" "<<CG.y<<" "<<CG1.x<<" "<<CG1.y<<"   "<<abs(1-(x/x1))<<endl;
//		}
//	}
//
//	IpVec ip0,ip1;
//
//	for(int i;i<ipts0.size();++i)
//	{
//		if(GoodPoints[i])
//		{
//			ip0.push_back(ipts0[i]);
//			ip1.push_back(ipts1[i]);
//		}
//		else
//		{
//			ipts0y.push_back(ipts0[i]);
//			ipts1y.push_back(ipts1[i]);
//		}
//	}
//
//	ipts0x=ip0;
//	ipts1x=ip1;
//
//	//    CvScalar color_tab[5] =
//	//    { CV_RGB (255, 0, 0), CV_RGB (0, 255, 0), CV_RGB (100, 100, 255), CV_RGB (255, 0, 255), CV_RGB (255, 255, 0) };
//
//	//    IplImage *img = cvCreateImage (cvSize (5184*5, 3456*5), IPL_DEPTH_8U, 3);
//
//	//    cvZero (img);
//
//	//    for(int i=0;i<vecVec.size();++i)
//	//    {
//	//        CvPoint ipt;
//	//        ipt.x=vecVec[i].X*5+2529*5;
//	//        ipt.y=vecVec[i].Y*5+1728*5;
//	//        cvCircle (img, ipt, 5, color_tab[1], CV_FILLED, CV_AA, 0);
//	//    }
//
//	//    cvSaveImage("x.png",img);
//
//	//    cout<<"Sizee: "<<ipts0x.size()<<endl;
//	//    cout<<"Sizey: "<<ipts0y.size()<<endl;
//
//	return true;
//}

bool CGVM::CGVMx(Point2f& CG,Point2f& CG1,IpVec& ipts0,IpVec& ipts1,IpVec& ipts0x,IpVec& ipts1x, IpVec& ipts0y,IpVec& ipts1y,int loops, int GoodSetNum,double t1,double t2)
{
    vector<int> TestSet;
    vector<bool> GoodPoints;
    GoodPoints.resize(ipts0.size());
    struct timeval tpstart;
    gettimeofday(&tpstart,NULL);
    for(int i;i<ipts0.size();++i) GoodPoints[i]=true;
    TestSet.resize(100);
    bool GoodSet=false;
    int SetSize=GoodSetNum-1;
    int loop=0;
    int loopX=0;
    double diffSlope=0;
    double sc;
    int SUM;

    srand(tpstart.tv_usec);

    CG=Point2f(0,0);
    CG1=Point2f(0,0);
    Vector2D Testv,Testv1;
    double Testx,Testx1;
    bool GetGoodSet=true;
    sc=0;

    vector<Vector2D> vecVec;

    ipts0x.clear();
    ipts1x.clear();
    ipts0y.clear();
    ipts1y.clear();



    //    for(int i=0;i<ipts0.size();++i)
    //    {
    //        Testv=CGVM::GetVector(CG,ipts0[i]);
    //        Testx=CGVM::GetLength(Testv);
    //        Testv1=CGVM::GetVector(CG1,ipts1[i]);
    //        Testx1=CGVM::GetLength(Testv1);

    //        Vector2D Testk;

    //        Testk.X=Testv.X-Testv1.X;

    //        Testk.Y=Testv.Y-Testv1.Y;

    //        vecVec.push_back(Testk);

    //        cout<<"x "<<Testk.X<<" y "<<Testv1.Y<<endl;
    //    }

    //    CvScalar color_tab[5] =
    //    { CV_RGB (255, 0, 0), CV_RGB (0, 255, 0), CV_RGB (100, 100, 255), CV_RGB (255, 0, 255), CV_RGB (255, 255, 0) };

    //    IplImage *img = cvCreateImage (cvSize (5184, 3456), IPL_DEPTH_8U, 3);

    //    cvZero (img);

    //    for(int i=0;i<vecVec.size();++i)
    //    {
    //        CvPoint ipt;
    //        ipt.x=vecVec[i].X+2529;
    //        ipt.y=vecVec[i].Y+1728;
    //        cvCircle (img, ipt, 5, color_tab[1], CV_FILLED, CV_AA, 0);
    //    }

    //    cvSaveImage("z.png",img);

    //    return false;





    while(!GoodSet)
    {
        for(int i=0;i<=SetSize;i++)
        {
            bool noRepeat=false;
            while(!noRepeat)
            {
                TestSet[i]=rand()%ipts0.size();
                if(!i) break;
                else
                {
                    for(int j=0;j<=SetSize;j++)
                    {
                        if(i==j) ;
                        else if(TestSet[i]==TestSet[j]) break;
                        else
                        {
                            noRepeat=true;
                            break;
                        }
                    }
                }
            }
        }
        CG=CGVM::GetCG(ipts0[TestSet[0]],ipts0[TestSet[1]]);
        CG1=CGVM::GetCG(ipts1[TestSet[0]],ipts1[TestSet[1]]);
        Vector2D Testv,Testv1;
        double Testx,Testx1;
        bool GetGoodSet=true;
        sc=0;
        diffSlope=0;
        for(int i=2;i<=SetSize;++i)
        {
            Testv=CGVM::GetVector(CG,ipts0[TestSet[i]]);
            Testx=CGVM::GetLength(Testv);
            Testv1=CGVM::GetVector(CG1,ipts1[TestSet[i]]);
            Testx1=CGVM::GetLength(Testv1);
            sc+=CGVM::VectorCompare(Testv,Testv1);
            //            cout<<atan(Testv.Slope)-atan(Testv1.Slope)<<endl;
            if(i==2) diffSlope=abs(atan(Testv.Slope)-atan(Testv1.Slope));
            if(CGVM::VectorCompare(Testv,Testv1)>=0.07||abs(abs(atan(Testv.Slope)-atan(Testv1.Slope))-diffSlope)>=t1)
            {
                GetGoodSet=false;
                break;
            }
            else
            {
                CG=CGVM::GetCG(CG,ipts0[TestSet[i]]);
                CG1=CGVM::GetCG(CG1,ipts1[TestSet[i]]);
                diffSlope+=abs(atan(Testv.Slope)-atan(Testv1.Slope));
                diffSlope/=i;
                SUM=i;
            }
        }
        GoodSet=GetGoodSet;

        if(++loop>100)
        {
            loop=0;
            t1+=t1*0.05;
            t2=t1;
        }

        if(++loopX>1000)
        {
            return false;
        }
    }

    //    cout<<"loop "<<loop<<endl;

    sc/=(SetSize-2);

    //    cout<<"SC!!!! "<<sc<<endl;

    //    cout<<"CG!!!!!!!"<<CG.x<<" "<<CG.y<<" "<<CG1.x<<" "<<CG1.y<<endl;

    for(int i=0;i<ipts0.size();++i)
    {
        bool NeedTest=true;
        for(int k=0;k<=SetSize;k++) if(TestSet[k]==i) NeedTest=false;
        if(!NeedTest) continue;
        Vector2D v,v1;
        double x,x1;
        v=CGVM::GetVector(CG,ipts0[i]);
        x=CGVM::GetLength(v);
        v1=CGVM::GetVector(CG1,ipts1[i]);
        x1=CGVM::GetLength(v1);

        Vector2D Testk;

        double m;

        Testk.X=v.X-v1.X;

        Testk.Y=v.Y-v1.Y;

        Testk.Slope=Testk.Y/Testk.X;

        vecVec.push_back(Testk);

        //        Vector2D Testkx;

        //        Testkx.Slope=Testk.Slope;

        //        double l=(CGVM::VectorCompare(v,v1)-sc)*1728;

        //        double a=atan(Testkx.Slope);

        //        Testkx.X=l*cos(a)*m;

        //        Testkx.Y=l*sin(a);





        CvPoint px0,px1;
        px0.x=CG.x;
        px0.y=CG.y;
        px1.x=ipts0[i].x;
        px1.y=ipts0[i].y;
        //        cvDrawLine(p0,px0,px1,CV_RGB( 255, 0, 0 ));
        //        cout<<"x "<<v.X<<" y "<<v.Y<<" Slope "<<atan(v.Slope)<<" x1 "<<v1.X<<" y1 "<<v1.Y<<" Slope1 "<<atan(v1.Slope)<<"   "<<abs(1-(x/x1))<<endl;

        if(CGVM::VectorCompare(v,v1)-sc>=0.03||abs(abs(atan(v.Slope)-atan(v1.Slope))-diffSlope)>=t2)
            //        if(abs(atan(v.Slope)-atan(v1.Slope))>=0.03)
        {
            GoodPoints[i]=false;
            //            cout<<CGVM::VectorCompare(v,v1)-sc<<" Moved!"<<endl;
        }
        else
        {
            CG=CGVM::GetCG(CG,ipts0[i]);
            CG1=CGVM::GetCG(CG1,ipts1[i]);
            diffSlope+=abs(atan(v.Slope)-atan(v1.Slope));
            diffSlope/=++SUM;


            //            cout<<vecVec.back().X<<" "<<vecVec.back().Y<<endl;
            //            cout<<"CG!!"<<CG.x<<" "<<CG.y<<" "<<CG1.x<<" "<<CG1.y<<"   "<<abs(1-(x/x1))<<endl;
        }
    }

    IpVec ip0,ip1;

    for(int i;i<ipts0.size();++i)
    {
        if(GoodPoints[i])
        {
            ip0.push_back(ipts0[i]);
            ip1.push_back(ipts1[i]);
        }
        else
        {
            ipts0y.push_back(ipts0[i]);
            ipts1y.push_back(ipts1[i]);
        }
    }

    ipts0x=ip0;
    ipts1x=ip1;

    //    CvScalar color_tab[5] =
    //    { CV_RGB (255, 0, 0), CV_RGB (0, 255, 0), CV_RGB (100, 100, 255), CV_RGB (255, 0, 255), CV_RGB (255, 255, 0) };

    //    IplImage *img = cvCreateImage (cvSize (5184*5, 3456*5), IPL_DEPTH_8U, 3);

    //    cvZero (img);

    //    for(int i=0;i<vecVec.size();++i)
    //    {
    //        CvPoint ipt;
    //        ipt.x=vecVec[i].X*5+2529*5;
    //        ipt.y=vecVec[i].Y*5+1728*5;
    //        cvCircle (img, ipt, 5, color_tab[1], CV_FILLED, CV_AA, 0);
    //    }

    //    cvSaveImage("x.png",img);

    //    cout<<"Sizee: "<<ipts0x.size()<<endl;
    //    cout<<"Sizey: "<<ipts0y.size()<<endl;

    return true;
}

bool CGVM::CGVMcos(Point2f& CG,Point2f& CG1,IpVec& ipts0,IpVec& ipts1,IpVec& ipts0x,IpVec& ipts1x, IpVec& ipts0y,IpVec& ipts1y,int loops, int GoodSetNum,double t1,double t2,double& diff)
{
    vector<int> TestSet;
    vector<bool> GoodPoints;
    GoodPoints.resize(ipts0.size());
    struct timeval tpstart;
    gettimeofday(&tpstart,0);
    for(int i=0;i<ipts0.size();++i) GoodPoints[i]=true;
    TestSet.resize(100);
    bool GoodSet=false;
    int SetSize=GoodSetNum-1;
    int loop=0;
    int loopX=0;
    double diffSlope=0;
    double sc;
    int SUM;

    srand(tpstart.tv_usec);

    CG=Point2f(0,0);
    CG1=Point2f(0,0);
    Vector2D Testv,Testv1;
    double Testx,Testx1;
    bool GetGoodSet=true;
    sc=0;

    vector<Vector2D> vecVec;

    ipts0x.clear();
    ipts1x.clear();
    ipts0y.clear();
    ipts1y.clear();



    //    for(int i=0;i<ipts0.size();++i)
    //    {
    //        Testv=CGVM::GetVector(CG,ipts0[i]);
    //        Testx=CGVM::GetLength(Testv);
    //        Testv1=CGVM::GetVector(CG1,ipts1[i]);
    //        Testx1=CGVM::GetLength(Testv1);

    //        Vector2D Testk;

    //        Testk.X=Testv.X-Testv1.X;

    //        Testk.Y=Testv.Y-Testv1.Y;

    //        vecVec.push_back(Testk);

    //        cout<<"x "<<Testk.X<<" y "<<Testv1.Y<<endl;
    //    }

    //    CvScalar color_tab[5] =
    //    { CV_RGB (255, 0, 0), CV_RGB (0, 255, 0), CV_RGB (100, 100, 255), CV_RGB (255, 0, 255), CV_RGB (255, 255, 0) };

    //    IplImage *img = cvCreateImage (cvSize (5184, 3456), IPL_DEPTH_8U, 3);

    //    cvZero (img);

    //    for(int i=0;i<vecVec.size();++i)
    //    {
    //        CvPoint ipt;
    //        ipt.x=vecVec[i].X+2529;
    //        ipt.y=vecVec[i].Y+1728;
    //        cvCircle (img, ipt, 5, color_tab[1], CV_FILLED, CV_AA, 0);
    //    }

    //    cvSaveImage("z.png",img);

    //    return false;





    while(!GoodSet)
    {
        SUM=0;
        for(int i=0;i<=SetSize;i++)
        {
            bool noRepeat=false;
            while(!noRepeat)
            {
                TestSet[i]=rand()%ipts0.size();
                if(!i) break;
                else
                {
                    for(int j=0;j<=SetSize;j++)
                    {
                        if(i==j) ;
                        else if(TestSet[i]==TestSet[j]) break;
                        else
                        {
                            noRepeat=true;
                            break;
                        }
                    }
                }
            }
        }
        CG=CGVM::GetCG(ipts0[TestSet[0]],ipts0[TestSet[1]]);
        CG1=CGVM::GetCG(ipts1[TestSet[0]],ipts1[TestSet[1]]);
        Vector2D Testv,Testv1;
        double Testx,Testx1;
        bool GetGoodSet=true;
        sc=0;
        diffSlope=0;
        for(int i=2;i<=SetSize;++i)
        {
            Testv=CGVM::GetVector(CG,ipts0[TestSet[i]]);
            Testx=CGVM::GetLength(Testv);
            Testv1=CGVM::GetVector(CG1,ipts1[TestSet[i]]);
            Testx1=CGVM::GetLength(Testv1);
            sc+=CGVM::VectorCompare(Testv,Testv1);
            //            cout<<atan(Testv.Slope)-atan(Testv1.Slope)<<endl;
            if(i==2) diffSlope=GetAngle(Testv,Testv1);
            if(CGVM::VectorCompare(Testv,Testv1)>=0.5||abs(GetAngle(Testv,Testv1)-diffSlope)>=t1)
            {
                GetGoodSet=false;
                break;
            }
            else
            {
                CG=CGVM::GetCG(CG,ipts0[TestSet[i]]);
                CG1=CGVM::GetCG(CG1,ipts1[TestSet[i]]);
                diffSlope=diffSlope*(i-1)+GetAngle(Testv,Testv1);
                diffSlope/=i;
                //				cout<<"diffSlope"<<i<<": "<<diffSlope<<" GetAngle(Testv,Testv1) "<<GetAngle(Testv,Testv1)<<endl;
                SUM=i;
            }
        }
        GoodSet=GetGoodSet;

        if(++loop>1000)
        {
            loop=0;
            t1+=t1*0.05;
            t2=t1;
        }

        if(++loopX>10000)
        {
            return false;
        }
    }

    //	cout<<"diffSlopex: "<<diffSlope<<endl;

    //    cout<<"loop "<<loop<<endl;

    sc/=(SetSize-2);

    //    cout<<"SC!!!! "<<sc<<endl;

    //    cout<<"CG!!!!!!!"<<CG.x<<" "<<CG.y<<" "<<CG1.x<<" "<<CG1.y<<endl;

    for(int i=0;i<ipts0.size();++i)
    {
        bool NeedTest=true;
        for(int k=0;k<=SetSize;k++) if(TestSet[k]==i) NeedTest=false;
        if(!NeedTest) continue;
        Vector2D v,v1;
        double x,x1;
        v=CGVM::GetVector(CG,ipts0[i]);
        x=CGVM::GetLength(v);
        v1=CGVM::GetVector(CG1,ipts1[i]);
        x1=CGVM::GetLength(v1);

        Vector2D Testk;

        double m;

        Testk.X=v.X-v1.X;

        Testk.Y=v.Y-v1.Y;

        Testk.Slope=Testk.Y/Testk.X;

        vecVec.push_back(Testk);

        //        Vector2D Testkx;

        //        Testkx.Slope=Testk.Slope;

        //        double l=(CGVM::VectorCompare(v,v1)-sc)*1728;

        //        double a=atan(Testkx.Slope);

        //        Testkx.X=l*cos(a)*m;

        //        Testkx.Y=l*sin(a);





        CvPoint px0,px1;
        px0.x=CG.x;
        px0.y=CG.y;
        px1.x=ipts0[i].x;
        px1.y=ipts0[i].y;
        //        cvDrawLine(p0,px0,px1,CV_RGB( 255, 0, 0 ));
        //        cout<<"x "<<v.X<<" y "<<v.Y<<" Slope "<<atan(v.Slope)<<" x1 "<<v1.X<<" y1 "<<v1.Y<<" Slope1 "<<atan(v1.Slope)<<"   "<<abs(1-(x/x1))<<endl;

        if(CGVM::VectorCompare(v,v1)>=0.5||abs(GetAngle(v,v1)-diffSlope)>=t2)
            //        if(abs(atan(v.Slope)-atan(v1.Slope))>=0.03)
        {
            GoodPoints[i]=false;
            //            cout<<CGVM::VectorCompare(v,v1)-sc<<" Moved!"<<endl;
        }
        else
        {
            CG=CGVM::GetCG(CG,ipts0[i]);
            CG1=CGVM::GetCG(CG1,ipts1[i]);
            diffSlope=diffSlope*SUM+GetAngle(v,v1);
            diffSlope/=++SUM;


            //            cout<<vecVec.back().X<<" "<<vecVec.back().Y<<endl;
            //            cout<<"CG!!"<<CG.x<<" "<<CG.y<<" "<<CG1.x<<" "<<CG1.y<<"   "<<abs(1-(x/x1))<<endl;
        }
    }

    IpVec ip0,ip1;

    for(int i;i<ipts0.size();++i)
    {
        if(GoodPoints[i])
        {
            ip0.push_back(ipts0[i]);
            ip1.push_back(ipts1[i]);
        }
        else
        {
            ipts0y.push_back(ipts0[i]);
            ipts1y.push_back(ipts1[i]);
        }
    }

    ipts0x=ip0;
    ipts1x=ip1;

    //    CvScalar color_tab[5] =
    //    { CV_RGB (255, 0, 0), CV_RGB (0, 255, 0), CV_RGB (100, 100, 255), CV_RGB (255, 0, 255), CV_RGB (255, 255, 0) };

    //    IplImage *img = cvCreateImage (cvSize (5184*5, 3456*5), IPL_DEPTH_8U, 3);

    //    cvZero (img);

    //    for(int i=0;i<vecVec.size();++i)
    //    {
    //        CvPoint ipt;
    //        ipt.x=vecVec[i].X*5+2529*5;
    //        ipt.y=vecVec[i].Y*5+1728*5;
    //        cvCircle (img, ipt, 5, color_tab[1], CV_FILLED, CV_AA, 0);
    //    }

    //    cvSaveImage("x.png",img);

    //    cout<<"Sizee: "<<ipts0x.size()<<endl;
    //    cout<<"Sizey: "<<ipts0y.size()<<endl;

    //	cout<<"diffSlope: "<<diffSlope<<endl;

    diff=diffSlope;

    return true;
}

//生成有序的相对距离的deque
void CGVM::EstimateQueue(const vector<vector<float> >& table, int idx, IpVec& FeaVec, vector<DistanceRelation>& OQ)
{
    Ipoint Fea=FeaVec[idx];
    OQ.reserve(FeaVec.size());
    for(int i=0;i<FeaVec.size();i++)
    {
        if(i!=idx)
        {
            DistanceRelation tmpDR;
            tmpDR.dis=table[Fea.clusterIndex][FeaVec[i].clusterIndex];
            tmpDR.idx=i;
            if(!OQ.size()) OQ.push_back(tmpDR);
            else
            {
                if(tmpDR.dis<=OQ.front().dis)
                    OQ.insert(OQ.begin(),tmpDR);
                else if(tmpDR.dis>=OQ.back().dis)
                    OQ.push_back(tmpDR);
                else
                {
                    for(vector<DistanceRelation>::iterator it=OQ.begin()+1;it!=OQ.end();++it)
                    {
                        if(tmpDR.dis>=(it-1)->dis&&tmpDR.dis<=it->dis)
                        {
                            OQ.insert(it,tmpDR);
                            break;
                        }
                    }
                }
            }
        }
    }
}

//通过clusterIndex进行索引的特征点相对距离矩阵
void CGVM::disTable(vector<vector<float> >& table, IpVec& Fea, double* ave)
{
    double num=0;
    *ave=0;
    table.resize(Fea.size());
    for(int i=0;i<table.size();i++) table[i].resize(Fea.size());
    for(int i=0;i<Fea.size()-1;i++)
    {
        for(int j=i+1;j<Fea.size();j++)
        {
            double d;
            d=sqrt(pow((Fea[i].x-Fea[j].x),2)+pow((Fea[i].y-Fea[j].y),2));
            table[i][j]=d;
            table[j][i]=d;
            *ave+=d;
            num++;
        }
    }
    *ave/=num;
}

//通过最近点计算
bool CGVM::ICGM(Point2f& CG,Point2f& CG1,IpVec& ipts0,IpVec& ipts1,const vector<vector<float> >& table0,const vector<vector<float> >& table1,IpVec& ipts0x,IpVec& ipts1x, IpVec& ipts0y,IpVec& ipts1y,int GoodSetNum,double t1,double t2,double& diff)
{
    if(ipts0.size()<=GoodSetNum) return false;
    vector<int> TestSet;
    vector<bool> GoodPoints;
    GoodPoints.resize(ipts0.size());
    struct timeval tpstart,tpend;
    float timeuse;
    gettimeofday(&tpstart,NULL);

    vector<DistanceRelation> OrderedQueue0;
    for(int i=0;i<ipts0.size();++i) GoodPoints[i]=true;
    TestSet.resize(GoodSetNum);
    bool GoodSet=false;
    int loop=0;
    int loopX=0;
    double diffSlope=0;
    double sc;
    int SUM;

    srand(tpstart.tv_usec);

    CG=Point2f(0,0);
    CG1=Point2f(0,0);
    sc=0;

    vector<Vector2D> vecVec;

    ipts0x.clear();
    ipts1x.clear();
    ipts0y.clear();
    ipts1y.clear();

    while(!GoodSet)
    {
        SUM=0;
        TestSet[0]=rand()%ipts0.size();
        OrderedQueue0.clear();
        EstimateQueue(table0, TestSet[0], ipts0, OrderedQueue0);
        for(int i=1;i<GoodSetNum;i++)
        {
            TestSet[i]=OrderedQueue0[i-1].idx;
        }
        CG=CGVM::GetCG(ipts0[TestSet[0]],ipts0[TestSet[1]]);
        CG1=CGVM::GetCG(ipts1[TestSet[0]],ipts1[TestSet[1]]);
        Vector2D Testv,Testv1;
        double Testx,Testx1;
        bool GetGoodSet=true;
        sc=0;
        diffSlope=0;
        for(int i=2;i<GoodSetNum;++i)
        {
            Testv=CGVM::GetVector(CG,ipts0[TestSet[i]]);
            Testx=CGVM::GetLength(Testv);
            Testv1=CGVM::GetVector(CG1,ipts1[TestSet[i]]);
            Testx1=CGVM::GetLength(Testv1);
            sc+=CGVM::VectorCompare(Testv,Testv1);
            //            cout<<atan(Testv.Slope)-atan(Testv1.Slope)<<endl;
            if(i==2) diffSlope=GetAngle(Testv,Testv1);
            if(CGVM::VectorCompare(Testv,Testv1)>=0.3||abs(GetAngle(Testv,Testv1)-diffSlope)>=t1)
            {
                GetGoodSet=false;
                break;
            }
            else
            {
                CG=CGVM::GetCG(CG,ipts0[TestSet[i]]);
                CG1=CGVM::GetCG(CG1,ipts1[TestSet[i]]);
                diffSlope=diffSlope*(i-1)+GetAngle(Testv,Testv1);
                diffSlope/=i;
                //				cout<<"diffSlope"<<i<<": "<<diffSlope<<" GetAngle(Testv,Testv1) "<<GetAngle(Testv,Testv1)<<endl;
                SUM=i;
            }
        }
        GoodSet=GetGoodSet;

        if(++loop>1000)
        {
            loop=0;
            t1+=t1*0.05;
            t2=t1;
        }

        if(++loopX>10000)
        {
            return false;
        }
    }

    //	cout<<"diffSlopex: "<<diffSlope<<endl;

    //    cout<<"loop "<<loop<<endl;

    sc/=(GoodSetNum-1);

    //    cout<<"SC!!!! "<<sc<<endl;

    //    cout<<"CG!!!!!!!"<<CG.x<<" "<<CG.y<<" "<<CG1.x<<" "<<CG1.y<<endl;

    for(int i=0;i<ipts0.size();++i)
    {
        bool NeedTest=true;
        for(int k=0;k<GoodSetNum;k++) if(TestSet[k]==i) NeedTest=false;
        if(!NeedTest) continue;
        Vector2D v,v1;
        double x,x1;
        v=CGVM::GetVector(CG,ipts0[i]);
        x=CGVM::GetLength(v);
        v1=CGVM::GetVector(CG1,ipts1[i]);
        x1=CGVM::GetLength(v1);

        Vector2D Testk;

        Testk.X=v.X-v1.X;

        Testk.Y=v.Y-v1.Y;

        Testk.Slope=Testk.Y/Testk.X;

        vecVec.push_back(Testk);

        CvPoint px0,px1;
        px0.x=CG.x;
        px0.y=CG.y;
        px1.x=ipts0[i].x;
        px1.y=ipts0[i].y;
        //        cvDrawLine(p0,px0,px1,CV_RGB( 255, 0, 0 ));
        //        cout<<"x "<<v.X<<" y "<<v.Y<<" Slope "<<atan(v.Slope)<<" x1 "<<v1.X<<" y1 "<<v1.Y<<" Slope1 "<<atan(v1.Slope)<<"   "<<abs(1-(x/x1))<<endl;

        if(CGVM::VectorCompare(v,v1)>=0.3||abs(GetAngle(v,v1)-diffSlope)>=t2)
            //        if(abs(atan(v.Slope)-atan(v1.Slope))>=0.03)
        {
            GoodPoints[i]=false;
            //            cout<<CGVM::VectorCompare(v,v1)-sc<<" Moved!"<<endl;
        }
        else
        {
            CG=CGVM::GetCG(CG,ipts0[i]);
            CG1=CGVM::GetCG(CG1,ipts1[i]);
            diffSlope=diffSlope*SUM+GetAngle(v,v1);
            diffSlope/=++SUM;


            //            cout<<vecVec.back().X<<" "<<vecVec.back().Y<<endl;
            //            cout<<"CG!!"<<CG.x<<" "<<CG.y<<" "<<CG1.x<<" "<<CG1.y<<"   "<<abs(1-(x/x1))<<endl;
        }
    }

    IpVec ip0,ip1;

    for(int i=0;i<ipts0.size();++i)
    {
        if(GoodPoints[i])
        {
            ip0.push_back(ipts0[i]);
            ip1.push_back(ipts1[i]);
        }
        else
        {
            ipts0y.push_back(ipts0[i]);
            ipts1y.push_back(ipts1[i]);
        }
    }

    ipts0x=ip0;
    ipts1x=ip1;


    diff=diffSlope;

    gettimeofday(&tpend,NULL);
    timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec)+tpend.tv_usec-tpstart.tv_usec;
    timeuse/=1000000;
    cout<<"**********ICGM is Done!**********  "<<timeuse<<"s "<<endl;

    return true;
}
