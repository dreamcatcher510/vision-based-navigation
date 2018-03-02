/*
Wenhao FU
*/

#ifndef MYDSPFEAT_H
#define MYDSPFEAT_H

#include <cv.h>

extern "C"{
#include "sift.h"
#include "imgfeatures.h"
#include "utils.h"
#include "kdtree.h"
}

int myDspFeat( IplImage* img, struct feature* features, int n, char* win_name, char* out_img_name );

#endif
