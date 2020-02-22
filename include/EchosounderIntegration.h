#ifndef ECHOSOUNDERINTEGRATION_H
#define ECHOSOUNDERINTEGRATION_H


#include <opencv2/opencv.hpp>
#include<string>

namespace ORB_SLAM2
{

class EchosounderIntegration
{
public:
    EchosounderIntegration(const std::string &strSettingFile);

    void SetEchosounderDistance(float echosounderDistance, int echosounderConfidence);

    bool MatchEchosounderReading(cv::KeyPoint potentialPoint, int imageRows);
    // bool MatchEchosounderReading(int potentialPoint);

    float GetEchosounderDepthRatio(cv::Point3f targetPoint);

    cv::Point2f GetRectifiedPixelPoint(cv::Point3f cameraPoint);

    float GetEchosounderDistance(){return esDist;}

    bool IsEsConfident(){return esConfidence >= threshConfidence;}

    void SaveEsEdgeConstraint(int echosounderMatchIndex){this->esTargetMapIndex = echosounderMatchIndex;}

    int GetEsEdgeIndex(){return this->esTargetMapIndex;}


private:

    float esDist;
    int esConfidence;
    int threshConfidence = 100;

    int esTargetMapIndex;

    float esRadius;
    float esAngle;
    int originalHeight;
    cv::Point2f rectifiedEsCenter;
    cv::Point2f rectifiedEsEdge;
    cv::Mat cameraProjection;
    cv::Point3f esPosition;
    cv::Point3f esDirection;

};

}// namespace ORB_SLAM

#endif // ECHOSOUNDERINTEGRATION_H
