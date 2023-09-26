#ifndef ROADWAYPAINTER_H
#define ROADWAYPAINTER_H

#include <string.h>
#include <cmath>
#include <float.h>
#include "smartpainterdef.h"

class SmartRgbImage
{
public:
    SmartRgbImage();
    SmartRgbImage(unsigned char *imgData, int _width, int _height);
    ~SmartRgbImage();

    int mWidth;
    int mHeight;
    int mStep;

    unsigned char *mData;
};

class SMARTPAINTER_SHARED_EXPORT RoadwayPainter
{
public:
    RoadwayPainter();
    ~RoadwayPainter();

    static bool paintRoadway(void *_roadwayParam, unsigned char * _rgbImageData, int _width_, int _height, bool maskMode = false);
    static void imageGrayToRGB(const unsigned char *gray, unsigned char *rgb, int _width, int _height);
protected:
    static void outputParamFromMToMm(void *_roadwayParam);
    static void getBeginAndEndImageIndex(int &_bH, int &_eH, int _arrayIndex[], int closeH);
    static bool paintLane(SmartRgbImage &_img, int _bH, int _eH, int _lEdge[], int _rEdge[], int _alpha, int _color[], bool maskMode = false);
    static bool calculateImageIndex(void *_lens, float _worldX, float _worldY, float & _imageX, float & _imageY);
    static void getImageCurveIndex(void *_lens, void *_boundary, float _worldY, float & _imageX, float & _imageY);
    static void calculateCurveIndex(void *_lens, void *_boundary, SmartRgbImage &_image, float _by, float _ey, int _boundaryIndex[]);
    static void doCurveInterpolation(int _bH, int _eH, int _curveEquation[]);
    static void getLineEquation(float _x1, float _y1, float _x2, float _y2, float &_k, float &_b);
    static void drawLine(SmartRgbImage &_img, int _bh, int _eh, float _k, float _b,int _color[]);
};

#endif // ROADWAYPAINTER_H
