#ifndef IPOINT_H
#define IPOINT_H

#include "imgfeatures.h"
#include <vector>
#include <iostream>

using namespace std;

class Ipoint; // Pre-declaration
typedef std::vector<Ipoint> IpVec;
typedef std::vector<std::pair<Ipoint, Ipoint> > IpPairVec;

class Ipoint
{

public:
    Ipoint();
    ~Ipoint();

    Ipoint(feature feaP);

    feature fea;

    float x, y;

    void SetCor();

    void SetIndex();

    int clusterIndex;
};

#endif // IPOINT_H
