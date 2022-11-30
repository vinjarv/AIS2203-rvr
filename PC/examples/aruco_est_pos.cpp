
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <iostream>


/*
 * Chance to piCam params!!!
 */

// Camera intrinsic parameters
struct {
    double FX = 656.6992227;
    double FY = 658.4488084;
    double CX = 350.5463367;
    double CY = 248.9997307;
}Intr;

// Distortion coefficients
struct {
    double K1 = 0.1063427;
    double K2 = 0.6210248;
    double K3 = -3.0955403;
    double P1 = -0.0031847;
    double P2 = 0.0127446;
}Dist;

cv::Mat cameraMatrix = (cv::Mat1d(3,3) << Intr.FX, 0, Intr.CX, 0, Intr.FY, Intr.CY, 0, 0, 1);
cv::Mat distCoeff = (cv::Mat1d(1, 5) << Dist.K1, Dist.K2, Dist.P1, Dist.P2, Dist.K3);



class ArucoDetection {
private:
    cv::Mat _camMat;
    cv::Mat _distCoeffs;
    float _markerSize = 50.0f;
    std::vector<int> _ids;
    std::vector<std::vector<cv::Point2f>> _corners, _rejCand;
    std::vector<cv::Vec3d> _rotVecs, _transVecs;
    cv::aruco::PREDEFINED_DICTIONARY_NAME _dict = cv::aruco::DICT_6X6_50;

    bool _identifyArucos(cv::Mat& img){
        cv::Ptr<cv::aruco::Dictionary> dictPtr = cv::aruco::getPredefinedDictionary(_dict);
        cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
        cv::aruco::detectMarkers(img, dictPtr, _corners, _ids, params, _rejCand);
        if (_ids.empty()){
            return false;
        }
        else{
            return true;
        }
    }
public:
    // Constructor
    ArucoDetection(cv::Mat& camMat, cv::Mat& distCoeffs): _camMat(camMat), _distCoeffs(distCoeffs){}

    cv::Mat getArucoImg(cv::Mat& inputImg){
        cv::Mat outputImg;
        _identifyArucos(inputImg);
        outputImg = inputImg.clone();
        if (_identifyArucos(inputImg)){
            cv::aruco::drawDetectedMarkers(outputImg, _corners, _ids);
        }
        return outputImg;
    }

    cv::Vec<double, 3> getRelativePosition(cv::Mat& img, int& arucoId){
        double zMinVal;
        cv::Vec<double, 3> zMinPos;
        _identifyArucos(img);
        if (_identifyArucos(img)){
            cv::aruco::estimatePoseSingleMarkers(_corners, _markerSize, _camMat, _distCoeffs, _rotVecs, _transVecs);
            //find closest marker
            zMinVal = _transVecs[0][2];
            zMinPos = _transVecs[0];
            for (int i = 0; i < _ids.size(); i++){
                if (_transVecs[i][2] < zMinVal){
                    zMinVal = _transVecs[i][2];
                    zMinPos = _transVecs[i];
                    arucoId = _ids[i];
                }
            }
            return zMinPos; //mm
        }
        else{
            return {0,0,0};
        }
    }

    void setCameraMatrix(cv::Mat& inputMatrix){_camMat = inputMatrix;}
    void setDistortionCoefficients(cv::Mat& inputVector){_distCoeffs = inputVector;}
    void setMarkerSize(float& size){_markerSize = size;}
    void setDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME& inputDict) {_dict = inputDict;}

    auto getCameraMatrix(){return _camMat;}
    auto getDistortionCoefficients(){return _distCoeffs;}
    auto getMarkerSize(){return _markerSize;}
    auto getIds(){return _ids;}
    auto getCorners(){return _corners;}
    auto getRejectedCandidates(){return _rejCand;}
    auto getRotationVectors(){return _rotVecs;}
    auto getTranslationVectors(){return _transVecs;}
    auto getDictionary(){return _dict;}
};




int main(){
    /*
     * TESTCODE:
     */

    //inputs:
    cv::Mat inImg = cv::imread(R"(C:\Users\MathiasM\Documents\AIS2203 Sanntid\Aruco_test_img\aruco_1.jpg)");
    int id;

    ArucoDetection Aruco(cameraMatrix, distCoeff);
    auto outImg = Aruco.getArucoImg(inImg);
    auto Pos = Aruco.getRelativePosition(inImg, id);

    // Print and show
    std::cout << "pos: " << "\t" << Pos << "\t"<< std::endl;
    while (true){
        cv::imshow("outIm", outImg);
        if (cv::waitKey(30) == 27) break; //Esc
    }
    cv::destroyAllWindows();
}









