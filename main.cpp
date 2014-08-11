/*
  Detects SIFT features in two images and finds matches between them.

  Copyright (C) 2006-2012  Rob Hess <rob@iqengines.com>

  @version 1.1.2-20100521
*/

#include "sift.h"
#include "imgfeatures.h"
#include "kdtree.h"
#include "utils.h"
#include "xform.h"

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#include <stdio.h>
#include <surflib.h>
#include <iostream>
#include "CGVM.h"

using namespace std;

using namespace DepthPIRF;


/* the maximum number of keypoint NN candidates to check during BBF search */
#define KDTREE_BBF_MAX_NN_CHKS 200

/* threshold on squared ratio of distances between NN and 2nd NN */
#define NN_SQ_DIST_RATIO_THR 0.49


int main( int argc, char** argv )
{
    IplImage* img1, * img2, * stacked;

    img1=cvLoadImage("IMG_0666.JPG");

    img2=cvLoadImage("IMG_0664.JPG");

    IpVec ipts1,ipts2;

    IpPairVec matches;

    vector<vector<float> > table0,table1;

    surfDetDes(img1,ipts1);

    surfDetDes(img2,ipts2);

    CGVM::disTable(table0, ipts1);

    getMatches(ipts1,ipts2,matches);

    ipts1.clear();
    ipts2.clear();



    for(int i=0;i<matches.size();i++)
    {
        ipts1.push_back(matches[i].first);
        ipts2.push_back(matches[i].second);
    }

    IpVec ipts0x,ipts1x,ipts0y,ipts1y;

    Point2f CG,CG1;



    double diff=0;

    int GSN=10;

    CGVM::ICGM(CG,CG1,ipts1,ipts2,table0,table1,ipts0x,ipts1x,ipts0y,ipts1y,GSN,0.12,0.12,diff);

    drawIpoints(img1,ipts0x);
    drawIpoints(img2,ipts1x);

    cvShowImage("SIFT1",img1);
    cvShowImage("SIFT2",img2);

    cvWaitKey( 0 );

    return 0;
}
