
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>


/**
 * This class expects a 3x3 intrinsic camera matrix of type cv::Mat and a 5x1 distortion coefficient row vector of type cv::Mat when
 * object is initialized.
 *
 * As an example:
 *

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
 */



/** Class that handles aruco marker detection, identification and position estimation
 *
 */
class ArucoDetection {
private:
    cv::Mat _camMat;
    cv::Mat _distCoeffs;
    float _markerSize = 50.0f; //mm
    std::vector<int> _ids;
    std::vector<std::vector<cv::Point2f>> _corners, _rejCand;
    std::vector<cv::Vec3d> _rotVecs, _transVecs;
    cv::aruco::PREDEFINED_DICTIONARY_NAME _dict = cv::aruco::DICT_6X6_50;

    /** Private member function that takes in image as parameter and updates private variables if one or more aruco markers is found in img.
     * Return true if aruco marker found.
     *
     * @param img
     */
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

    /** Private member function that return the element number of vector _ids of the closest aruco marker.
     *
     */
    int _findClosest(){
        double zMinVal = _transVecs[0][2];
        int idNum = 0;
        for (int i = 0; i < _ids.size(); i++){
            if (_transVecs[i][2] < zMinVal){
                zMinVal = _transVecs[i][2];
                idNum = i;
            }
        }
        return idNum;
    }
public:
    /** Constructor that takes in intrinsic camera matrix and distortion coefficients as parameters and initialize to private variables.
     *
     * @param camMat
     * @param distCoeffs
     */
    ArucoDetection(cv::Mat& camMat, cv::Mat& distCoeffs): _camMat(camMat), _distCoeffs(distCoeffs){}

    /** Public member function that takes in images as parameter and returns image marked with identified aruco markers
     *
     * @param inputImg
     */
    cv::Mat getArucoImg(cv::Mat& inputImg){
        cv::Mat outputImg;
        outputImg = inputImg.clone();
        if (_identifyArucos(inputImg)){
            cv::aruco::drawDetectedMarkers(outputImg, _corners, _ids);
        }
        return outputImg;
    }

    /** Public member function that takes in an image as parameter and returns relative position to closest aruco marker in millimeters.
     * Returns zero vector if no aruco marker found.
     *
     * @param img
     */
    cv::Vec<double, 3> getRelativePosition(cv::Mat& img){
        int idNum;
        if (_identifyArucos(img)){
            cv::aruco::estimatePoseSingleMarkers(_corners, _markerSize, _camMat, _distCoeffs, _rotVecs, _transVecs);
            idNum = _findClosest();
            return _transVecs[idNum]; //mm
        }
        else{
            return {0,0,0};
        }
    }

    /** Public member function that takes in an image and a references to aruco marker id as parameters.
     * Returns relative position to closest aruco marker in millimeters, and updates aruco marker id accordingly.
     * Returns zero vector if no aruco marker found.
     *
     * @param img
     * @param arucoId
     */
    cv::Vec<double, 3> getRelativePosition(cv::Mat& img, int& arucoId){
        int idNum;
        if (_identifyArucos(img)){
            cv::aruco::estimatePoseSingleMarkers(_corners, _markerSize, _camMat, _distCoeffs, _rotVecs, _transVecs);
            idNum = _findClosest();
            arucoId = _ids[idNum];
            return _transVecs[idNum]; //mm
        }
        else{
            return {0,0,0};
        }
    }

    /** Public setters
     *
     */
    void setCameraMatrix(cv::Mat& inputMatrix){_camMat = inputMatrix;}
    void setDistortionCoefficients(cv::Mat& inputVector){_distCoeffs = inputVector;}
    void setMarkerSize(float& size){_markerSize = size;}
    void setDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME& inputDict) {_dict = inputDict;}

    /** Public getters
     *
     */
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