#include <iostream>
#include <math.h>
#include <string>

#include "camera_handler.h"
#include "satpext.h"
#include "frameid.h"
#include "LdwDataInterface.h"
#include "obstacleData.h"
#include "disparityconvertor.h"

#ifdef _WIN64
static const std::string k3dPointByPointFilePath = "D:/3d_pointbypoint.txt";
static const std::string k3dByLookUpTableFilePath = "D:/3d_by_lookuptable.txt";
static const std::string kRotationMartrixFilePath = "D:/rotation_matrix.txt";
#else
static const std::string kHomeDir = getenv("HOME") ? getenv("HOME") : "/var";
static const std::string k3dPointByPointFilePath = kHomeDir + "/3d_pointbypoint.txt";
static const std::string k3dByLookUpTableFilePath = kHomeDir + "/3d_by_lookuptable.txt";
static const std::string kRotationMartrixFilePath = kHomeDir + "/rotation_matrix.txt";
#endif

static const int kDisparityCount = 81;

MyCameraHandler::MyCameraHandler(std::string name):
    mName(name),
    mIsCalibParamReady(false)
{
    mIsLookupTableGenerated = false;
    ros::NodeHandle nh;
    PubString = nh.advertise<std_msgs::String>("/apollo/zkhy_obs", 1);
    PubImage = nh.advertise<sensor_msgs::Image>("/apollo/sensor/camera/obstacle/front_6mm", 1);
}

void MyCameraHandler::setStereoCalibParams(StereoCalibrationParameters &params)
{
    mStereoCalibrationParameters = params;
    mIsCalibParamReady = true;
    printf("************ Camera params ************\n");
    printf("Focus: %e pixel\n", params.focus);
    printf("Optical center X: %e pixel\n", params.cx);
    printf("Optical center Y: %e pixel\n", params.cy);
    printf("R-vector roll: %e rad\n", params.RRoll);
    printf("R-vector pitch: %e rad\n", params.RPitch);
    printf("R-vector yaw: %e rad\n", params.RYaw);
    printf("Translation x: %e mm\n", params.Tx);
    printf("Translation y: %e mm\n", params.Ty);
    printf("Translation z: %e mm\n", params.Tz);
    printf("**************************************\n");
}

void MyCameraHandler::setRotationMatrix(RotationMatrix &rotationMatrix)
{
    mRotationMatrix = rotationMatrix;
    FILE * fp = nullptr;
    fp = fopen(kRotationMartrixFilePath.data(), "wb+");
    if (!fp) {
        std::cout << kRotationMartrixFilePath << " file not open" << std::endl;
        return;
    }

    fprintf(fp, "Real3DToImage:\n");
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 4; col++) {
            fprintf(fp, "%e\t", mRotationMatrix.real3DToImage[col + 4 * row]);
        }
        fprintf(fp, "\n");
    }

    fprintf(fp, "ImageToReal3D:\n");
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            fprintf(fp, "%e\t", mRotationMatrix.imageToReal3D[col + 3 * row]);
        }
        fprintf(fp, "\n");
    }

    fclose(fp);
}

void MyCameraHandler::handleRawFrame(const RawImageFrame *rawFrame)
{
//        std::cout << mName
//                  << ", got image, id: " << rawFrame->frameId
//                  << " , time stamp: " << rawFrame->time
//                  << std::endl;

    // put you image processing logic here.    eg:
    processFrame(rawFrame->frameId, (char*)rawFrame + sizeof(RawImageFrame),
                 rawFrame->dataSize, rawFrame->width, rawFrame->height, rawFrame->format);
}

void MyCameraHandler::processFrame(int frameId, char *image, uint32_t dataSize, int width, int height, int frameFormat)
{
    switch (frameId){
    case FrameId::Lane:
    {
        LdwDataPack *ldwDataPack = (LdwDataPack *)image;
        std::cout << "ldw degree is: " << ldwDataPack->roadway.left_Lane.left_Boundary.degree << std::endl;
    }
        break;
    case FrameId::Obstacle:
    {
        //std_msgs::Float64MultiString MsgFloat64String;
        std_msgs::StringPtr MsgFloat64String(new std_msgs::String);
        int blockNum = ((int *)image)[0];
        OutputObstacles *obsDataPack = (OutputObstacles *) (((int *)image) + 2);
        for(int i=0;i<blockNum;i++)
        {
            /*printf("lx=%f\n",obsDataPack->real3DLeftX);
            printf("rx=%f\n",obsDataPack->real3DRightX);
            printf("cx=%f\n",obsDataPack->real3DCenterX);
            printf("uh=%f\n",obsDataPack->real3DUpY);
            printf("dh=%f\n",obsDataPack->real3DLowY);
            printf("ad=%f\n",obsDataPack->avgDistanceZ);
            printf("distance=%f\n",obsDataPack->avgDistanceZ);*/
            double width = obsDataPack[i].real3DRightX - obsDataPack[i].real3DLeftX;
            double height = obsDataPack[i].real3DLowY - obsDataPack[i].real3DUpY;
            double Distance = obsDataPack[i].avgDistanceZ;
            double x = obsDataPack[i].real3DCenterX;
            obsDataPack[i].real3DCenterX = -1.0 * obsDataPack[i].real3DCenterX;
            double y = sqrt(x * x + Distance * Distance);
            //printf("id:%d x=%06f y=%06f width=%06f height=%06f,lenth=%ld\n",obsDataPack[i].trackId,x,y,width,height,sizeof(*obsDataPack));
        }
        MsgFloat64String->data.assign(reinterpret_cast<const char *>(obsDataPack), sizeof(*obsDataPack) * blockNum);
        PubString.publish(MsgFloat64String);
        //std::cout << "blockNum is: " << blockNum << std::endl;
    }
        break;
    case FrameId::Disparity:
    {
        int bitNum = DisparityConvertor::getDisparityBitNum(frameFormat);
        if (mIsCalibParamReady) {
            // you can choose any of these methods
            handleDisparityByLookupTable((unsigned char *)image, width, height, bitNum);
            //handleDisparityPointByPoint((unsigned char *)image, width, height, bitNum);
        }
    }
	break;
    case FrameId::LeftCamera:
    {
//        int bitNum = DisparityConvertor::getDisparityBitNum(frameFormat);
        static unsigned char *rgbBuf = new unsigned char[width * height * 3];
        static float *floatData = new float[width * height];

	static int uu = 0;
	uu ++;

//        DisparityConvertor::convertDisparity2FloatFormat(image, width, height, bitNum, floatData);
//        DisparityConvertor::convertDisparity2RGB(floatData, width, height,
//                                                 kMinValidDisparityValue, kMaxValidDisparityValue, rgbBuf);

	if(uu == 1){
	ImageMsg.header.stamp = ros::Time::now();
	ImageMsg.encoding = "yuyv";
	ImageMsg.height = height;
	ImageMsg.width = width;
	ImageMsg.step = 2 * width;
	ImageMsg.data.resize(width * height * 2);
	memcpy(&ImageMsg.data[0], image, width * height * 2);
	ImageMsg.is_bigendian = 0;
	PubImage.publish(ImageMsg);

	uu =0;
}
    }
        break;
    default:
        break;
    }
}

void MyCameraHandler::handleDisparityPointByPoint(unsigned char *image, int width, int height, int bitNum)
{
    // get the disparity coordinate
    std::cout << "width: " << width << ", height: " << height << std::endl;

    // convert disparity format to float:
    static float *floatData = new float[width * height];
    DisparityConvertor::convertDisparity2FloatFormat(image, width, height, bitNum, floatData);

    // get the 3D point cloud data, and save one to file for verification
    static int index = 0;
    index ++;
    FILE * fp = nullptr;
    if (index == 1) {
        fp = fopen(k3dPointByPointFilePath.data(), "wb+");
        if (!fp) {
            std::cout << k3dPointByPointFilePath << " file not open" << std::endl;
            return;
        }
    }

    for(int posY = 0; posY < height; posY++) {
        for(int posX = 0; posX < width; posX++) {
            float x, y, z;
            DisparityConvertor::getPointXYZDistance(image, width, height, bitNum, mStereoCalibrationParameters.Tx,
                                                    mStereoCalibrationParameters.focus, mStereoCalibrationParameters.cx,
                                                    mStereoCalibrationParameters.cy, posX, posY, x, y, z);

            if((fabs(x) > 200000.0f)||(fabs(y) > 200000.0f)||(fabs(z) > 200000.0f)) {
                x = 0.0f;
                y = 0.0f;
                z = 0.0f;
            }
            if (index == 1) {
                fprintf(fp, "%f %f %f\n", x, y, z);
            }
        }
    }
    if (index == 1) {
        fclose(fp);
    }
}

void MyCameraHandler::handleDisparityByLookupTable(unsigned char *image, int width, int height, int bitNum)
{
    std::cout << "width: " << width << ", height: " << height << std::endl;

    // generate X Y Z lookup table, only once is OK
    static float *lookUpTableX = new float[kDisparityCount*(int)pow(2, bitNum)*width];
    static float *lookUpTableY = new float[kDisparityCount*(int)pow(2, bitNum)*height];
    static float *lookUpTableZ = new float[kDisparityCount*(int)pow(2, bitNum)];
    if(!mIsLookupTableGenerated) {
        DisparityConvertor::generateLookUpTableX(width, bitNum, mStereoCalibrationParameters.Tx, mStereoCalibrationParameters.cx, lookUpTableX);
        DisparityConvertor::generateLookUpTableY(height, bitNum, mStereoCalibrationParameters.Tx, mStereoCalibrationParameters.cy, lookUpTableY);
        DisparityConvertor::generateLookUpTableZ(bitNum, mStereoCalibrationParameters.Tx, mStereoCalibrationParameters.focus, lookUpTableZ);
        mIsLookupTableGenerated = true;
    }

    // get X Y Z distance according to the lookup table
    float *xDistance = new float[width*height];
    DisparityConvertor::getWholeXDistanceByLookupTable(image, width, height, bitNum, lookUpTableX, xDistance);
    float *yDistance = new float[width*height];
    DisparityConvertor::getWholeYDistanceByLookupTable(image, width, height, bitNum, lookUpTableY, yDistance);
    float *zDistance = new float[width*height];
    DisparityConvertor::getWholeZDistanceByLookupTable(image, width, height, lookUpTableZ, zDistance);

    // get the 3D point cloud data, and save one to file for verification
    static int index = 0;
    index ++;
    FILE * fp = nullptr;
    if (index == 1) {
        fp = fopen(k3dByLookUpTableFilePath.data(), "wb+");
        if (!fp) {
            std::cout << k3dByLookUpTableFilePath << " file not open" << std::endl;
            return;
        }
    }
    for(int i = 0; i < width*height; i++) {
            float x, y, z;
            x = xDistance[i];
            y = yDistance[i];
            z = zDistance[i];
            if((fabs(x) > 200000.0f)||(fabs(y) > 200000.0f)||(fabs(z) > 200000.0f)) {
                x = 0.0f;
                y = 0.0f;
                z = 0.0f;
            }
            if (index == 1) {
                fprintf(fp, "%f %f %f %d\n", x, y, z, i);
            }
    }
    if (index == 1) {
        fclose(fp);
    }

    delete [] xDistance;
    delete [] yDistance;
    delete [] zDistance;
}

void MyCameraHandler::handleUpdateFinished(Result result)
{
    mUpgradeResult = result;
    if (!mUpgradeResult.successed) {
        std::cout <<"Update failed with warning:"<<mUpgradeResult.warning << std::endl;
        std::cout << "If you want to update firmware, please press 'u' to try again !!! " << std::endl;
    }
}
