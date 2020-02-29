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

    void SetEchosounderDistance(const float &echosounderDistance, const int &echosounderConfidence);

    bool MatchEchosounderReading(const cv::KeyPoint &potentialPoint, const int &imageRows);
    // bool MatchEchosounderReading(int potentialPoint);

    float GetEchosounderDepthRatio(const cv::Mat &targetPointMat);

    cv::Point2f GetRectifiedPixelPoint(const cv::Point3f &cameraPoint);

    float GetEchosounderDistance(){return esDist;}

    bool IsEsConfident(){return isEchosounderUsed && esDist > 0.0 && esConfidence >= threshConfidence;}

    void SaveEsEdgeConstraint(const int &echosounderMatchIndex){this->esTargetMapIndex = echosounderMatchIndex;}

    int GetEsEdgeIndex(){return this->esTargetMapIndex;}

    /* Project sonar measurement to get the 3D point in the camera reference frame. */
    cv::Point3f ProjectSonarPoint();

    /* Transform sonar point to the world reference frame. */
    cv::Point3f transformSonarPointToWorld(const cv::Point3f &echosounder_point, const cv::Mat &Twc);

    bool isEchosounderUsed;
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
    cv::Mat cTe; // Transformation matrix: parent camera, child echosounder.

};

}// namespace ORB_SLAM

#endif // ECHOSOUNDERINTEGRATION_H
