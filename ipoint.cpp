#include "ipoint.h"

Ipoint::Ipoint(feature feaP)
{
    fea=feaP;
}

Ipoint::Ipoint()
{
}

Ipoint::~Ipoint()
{
}


void Ipoint::SetCor()
{
    x=fea.x;
    y=fea.y;
}

void Ipoint::SetIndex()
{
    clusterIndex=fea.Index;
}

