#include "EchosounderIntegration.h"

#include<iostream>
#include <opencv2/opencv.hpp>
#include<string>

namespace ORB_SLAM2
{

EchosounderIntegration::EchosounderIntegration(const std::string &strSettingFile)
{
    cv::FileStorage fSettings(strSettingFile, cv::FileStorage::READ);

    this->esPosition = cv::Point3f(fSettings["Echosounder.position.x"], fSettings["Echosounder.position.y"], fSettings["Echosounder.position.z"]);
    this->esDirection = cv::Point3f(fSettings["Echosounder.direction.x"], fSettings["Echosounder.direction.y"], fSettings["Echosounder.direction.z"]);
    this->esAngle = fSettings["Echosounder.angle"];

    this->cameraProjection = (cv::Mat_<float>(3,4) <<
            fSettings["Camera.proj00"], 0.0, fSettings["Camera.proj02"], 0.0,
            0.0, fSettings["Camera.proj11"], fSettings["Camera.proj12"], 0.0,
            0.0, 0.0, 1.0, 0.0);
    this->originalHeight =  fSettings["Camera.height"];

    // Normalize directional vector
    float magnitude = sqrt(pow(this->esDirection.x, 2) + pow(this->esDirection.y, 2) + pow(this->esDirection.z, 2));
    this->esDirection = this->esDirection / magnitude;

    // Calculate rectified point of echosounder direcitonal vector in camera frame
    // ProjectCenterPoint()

    std::cout << "\nEchosounder parameters:" << std::endl;
    std::cout << "- position: " << this->esPosition << std::endl;
    std::cout << "- direction: " << this->esDirection << std::endl;
    std::cout << "- angle: " << this->esAngle << std::endl;
    // std::cout << "- rectified point: " << pixel_x << " " << pixel_y << std::endl;

    std::cout << "\nExtra camera parameters:" << std::endl;
    std::cout << "- projection matrix: " << this->cameraProjection << std::endl;
}


void EchosounderIntegration::SetEchosounderDistance(const float echosounderDistance, const int echosounderConfidence)
{
    // TO DO: does this need to be dvided because the image frame size is cut in half?
    this->esDist = echosounderDistance / 2.0;
    this->esConfidence = echosounderConfidence;

    // Calculate echosounder radius here
    this->esRadius = this->esDist * tan(this->esAngle);

    // Calculate rectified echosounder direction vector point
    cv::Point3f centerEsPoint = this->esPosition + this->esDist * this->esDirection;
    this->rectifiedEsCenter = GetRectifiedPixelPoint(centerEsPoint);

    cv::Point3f edgeCirclePoint = centerEsPoint + cv::Point3f(this->esRadius, 0.0, 0.0);
    this->rectifiedEsEdge = GetRectifiedPixelPoint(edgeCirclePoint);
}


cv::Point2f EchosounderIntegration::GetRectifiedPixelPoint(const cv::Point3f cameraPoint)
{
    cv::Mat pointMat = (cv::Mat_<float>(4, 1) << cameraPoint.x, cameraPoint.y, cameraPoint.z, 0.000000);
    cv::Mat pointRectify = this->cameraProjection * pointMat;
    float pixel_x = pointRectify.at<float>(0,0) / pointRectify.at<float>(2,0);
    float pixel_y =  pointRectify.at<float>(1,0) / pointRectify.at<float>(2,0);
    return cv::Point2f(pixel_x, pixel_y);
}


bool EchosounderIntegration::MatchEchosounderReading(const cv::KeyPoint potentialFeaturePoint, const int imageRows)
{
    float imageResize = this->originalHeight / imageRows;

    cv::Point2f resizeEsCenter = this->rectifiedEsCenter / imageResize;
    cv::Point2f resizeEsEdge = this->rectifiedEsEdge / imageResize;

    float rectifiedRadius = resizeEsEdge.x - resizeEsCenter.x;
    float distBetweenPoints = sqrt(pow(potentialFeaturePoint.pt.x - resizeEsCenter.x, 2) + pow(potentialFeaturePoint.pt.y - resizeEsCenter.y, 2));

    if(distBetweenPoints <= rectifiedRadius)
    {
        return true;
    }
    return false;
}


float EchosounderIntegration::GetEchosounderDepthRatio(const cv::Mat targetPointMat)
{
    // Optimize position of map point such that its distance from the echosounder approximately
    //      matches with the echosounder's reading.

    float distError = 100.0;
    float deltaError = 0.05;
    int maxIterations = 20000;
    int curIteration = 0;

    cv::Point3f targetPoint = cv::Point3f(targetPointMat.at<float>(0), targetPointMat.at<float>(1), targetPointMat.at<float>(2));
    cv::Point3f optTargetPoint = targetPoint;

    while((deltaError <= fabs(distError)) && curIteration < maxIterations)
    {
        // Calculate distance of map point to echosounder
        float curDist = norm(optTargetPoint - this->esPosition);
        // Error between current calculated distance and the echosoudner reading
        distError = curDist - this->esDist;

        if(deltaError < fabs(distError))
        {
            if(0 < distError)
            {
                optTargetPoint = optTargetPoint - 0.01 * optTargetPoint / norm(optTargetPoint);
            }
            else
            {
                optTargetPoint = optTargetPoint + 0.01 * optTargetPoint / norm(optTargetPoint);
            }
        }
        curIteration++;
    }

    float firstDist = norm(targetPoint);
    float finalDist = norm(optTargetPoint);

    float depthRatio = finalDist / firstDist;
    std::cout << "\n\nEchosounder: " << this->esDist << std::endl;
    std::cout << "First dist: " << firstDist << std::endl;
    std::cout << "Final dist: " << finalDist << std::endl;
    std::cout << "Depth ratio: " << depthRatio << std::endl;

    return depthRatio;
}

}  //namespace ORB_SLAM
