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

void EchosounderIntegration::SetEchosounderDistance(float echosounderDistance, int echosounderConfidence)
{
    this->esDist = echosounderDistance / 2.0;
    this->esConfidence = echosounderConfidence;

    // Calculate echosounder radius here
    this->esRadius = this->esDist * tan(this->esAngle);

    // Calculate rectified echosounder direction vector point
    cv::Point3f centerEsPoint = this->esPosition + this->esDist * this->esDirection;
    this->rectifiedEsCenter = GetRectifiedPixelPoint(centerEsPoint);

    cv::Point3f edgeCirclePoint = centerEsPoint + cv::Point3f(this->esRadius, 0.0, 0.0);
    this->rectifiedEsEdge = GetRectifiedPixelPoint(edgeCirclePoint);


    // std::cout << "es center: " << this->rectifiedEsCenter.x << " " << this->rectifiedEsCenter.y << std::endl;
}

cv::Point2f EchosounderIntegration::GetRectifiedPixelPoint(cv::Point3f cameraPoint)
{
    cv::Mat pointMat = (cv::Mat_<float>(4, 1) << cameraPoint.x, cameraPoint.y, cameraPoint.z, 0.000000);
    cv::Mat pointRectify = this->cameraProjection * pointMat;
    float pixel_x = pointRectify.at<float>(0,0) / pointRectify.at<float>(2,0);
    float pixel_y =  pointRectify.at<float>(1,0) / pointRectify.at<float>(2,0);
    return cv::Point2f(pixel_x, pixel_y);
}

bool EchosounderIntegration::MatchEchosounderReading(cv::KeyPoint potentialFeaturePoint, int imageRows)
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


float EchosounderIntegration::GetEchosounderDepthRatio(cv::Point3f targetPoint)
{
    float depthError = 100.0;
    float deltaError = 0.05;
    int maxIterations = 20000;
    int curIteration = 0;
    cv::Point3f optTargetPoint = targetPoint;
    while((deltaError <= abs(depthError)) && curIteration < maxIterations)
    {
        float curDepth = sqrt(pow(optTargetPoint.x - this->esPosition.x, 2) + pow(optTargetPoint.y - this->esPosition.y, 2) + pow(optTargetPoint.z - this->esPosition.z, 2));
        depthError = curDepth - this->esDist;
        // std::cout << "depth error: " << depthError << std::endl;
        if(deltaError < depthError || deltaError < -depthError)
        {
            // std::cout << "updating target point" << std::endl;
            float magnitude = sqrt(pow(optTargetPoint.x, 2) + pow(optTargetPoint.y, 2) + pow(optTargetPoint.z, 2));
            if(0 < depthError)
            {
                optTargetPoint = optTargetPoint - 0.01 * optTargetPoint / magnitude;
            }
            else
            {
                optTargetPoint = optTargetPoint + 0.01 * optTargetPoint / magnitude;
            }
        }
        curIteration++;
    }
    // std::cout << "iterations: " << curIteration << std::endl;
    // std::cout << "optimized target point: " << optTargetPoint.x << " " << optTargetPoint.y << " " << optTargetPoint.z << std::endl;


    float firstDist = sqrt(pow(targetPoint.x, 2) + pow(targetPoint.y, 2) + pow(targetPoint.z, 2));
    float finalDist = sqrt(pow(optTargetPoint.x, 2) + pow(optTargetPoint.y, 2) + pow(optTargetPoint.z, 2));

    float depthRatio = firstDist / finalDist;
    std::cout << "depth ratio: " << depthRatio << std::endl;

    return depthRatio;
}

}  //namespace ORB_SLAM
