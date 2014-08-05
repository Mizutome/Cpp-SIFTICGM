#ifndef SURFLIB_H
#define SURFLIB_H

#include "ipoint.h"
#include "sift.h"
#include "imgfeatures.h"
#include "kdtree.h"
#include "utils.h"
#include "xform.h"

using namespace std;

/* the maximum number of keypoint NN candidates to check during BBF search */
#define KDTREE_BBF_MAX_NN_CHKS 200

/* threshold on squared ratio of distances between NN and 2nd NN */
#define NN_SQ_DIST_RATIO_THR 0.49

inline void surfDetDes(IplImage *img,  /* image to find Ipoints in */
                       std::vector<Ipoint> &ipts, /* reference to vector of Ipoints */
                       bool upright = false, /* run in rotation invariant mode? */
                       int octaves = 0, /* number of octaves to calculate */
                       int intervals = 0, /* number of intervals per octave */
                       int init_sample = 0, /* initial sampling step */
                       float thres = 0 /* blob response threshold */)
{
    int n;


    feature *feat;

    n=sift_features( img, &feat );

    ipts.clear();

    for(int i=0;i<n;i++)
        ipts.push_back(Ipoint(feat[i]));

    for(int i=0;i<ipts.size();i++)
    {
        ipts[i].fea.Index=i;
        ipts[i].SetCor();
        ipts[i].SetIndex();
    }

    free(feat);
}

inline feature* vevtorTOpointer(IpVec& ipts1)
{
    struct feature* feat1;
    feat1=new feature[ipts1.size()];
    for(int i=0;i<ipts1.size();i++)
    {
        feat1[i]=ipts1[i].fea;
    }
    return feat1;
}

inline void getMatches(IpVec &ipts1, IpVec &ipts2, IpPairVec &matches)
{
    struct feature *feat1,*feat2,*feat;
    feat1=vevtorTOpointer(ipts1);
    feat2=vevtorTOpointer(ipts2);
    struct feature** nbrs;
    struct kd_node* kd_root;
    int n1=ipts1.size(),n2=ipts2.size();
    CvPoint pt1, pt2;
    int  k, i, m = 0;
    double d0, d1;
    matches.clear();
    kd_root = kdtree_build( feat2, n2 );
    for( int i = 0; i < n1; i++ )
    {
        feat = feat1 + i;
        k = kdtree_bbf_knn( kd_root, feat, 2, &nbrs, KDTREE_BBF_MAX_NN_CHKS );
        if( k == 2 )
        {
            d0 = descr_dist_sq( feat, nbrs[0] );
            d1 = descr_dist_sq( feat, nbrs[1] );
            if( d0 < d1 * NN_SQ_DIST_RATIO_THR )
            {
                pt1 = cvPoint( cvRound( feat->x ), cvRound( feat->y ) );
                pt2 = cvPoint( cvRound( nbrs[0]->x ), cvRound( nbrs[0]->y ) );
                feat1[i].fwd_match = nbrs[0];
                m++;
                Ipoint iptsx;
                for(int z=0;z<ipts2.size();z++)
                {
                    bool get=false;
                    if(ipts2[z].fea.Index==nbrs[0]->Index)
                    {
                        iptsx=ipts2[z];
                        get=true;
                    }
                    if(get) break;
                }
                matches.push_back(make_pair(ipts1[i],iptsx));
            }
        }
        free( nbrs );
    }
    cout<<"matches: "<<m<<endl;

    kdtree_release( kd_root );
    free( feat1 );
    free( feat2 );

}

inline void drawIpoints(IplImage *img, vector<Ipoint> &ipts)
{
    struct feature* features=vevtorTOpointer(ipts);
    int n=ipts.size();
    free(features);
    draw_features( img, features, n );
}


#endif // SURFLIB_H
