#pragma once

#include <ctime>
#include <cassert>
#include <cmath>
#include <utility>
#include <vector>
#include <algorithm> 
#include <cstdlib>
#include <memory>
#include <iostream>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "scancontext/nanoflann.hpp"
#include "scancontext/KDTreeVectorOfVectorsAdaptor.h"
#include "aloam_velodyne/tic_toc.h"

using namespace Eigen;
using namespace nanoflann;

using std::cout;
using std::endl;
using std::make_pair;

using std::atan2;
using std::cos;
using std::sin;

using SCPointType = pcl::PointXYZI; // using xyz only. but a user can exchange the original bin encoding function (i.e., max hegiht) to max intensity (for detail, refer 20 ICRA Intensity Scan Context)
using KeyMat = std::vector<std::vector<float> >;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor< KeyMat, float >;


// namespace SC2
// {

void coreImportTest ( void );


// sc param-independent helper functions 
float xy2theta( const float & _x, const float & _y );
MatrixXd circshift( MatrixXd &_mat, int _num_shift );
std::vector<float> eig2stdvec( MatrixXd _eigmat );


class SCManager
{
public: 
    SCManager( ) = default; // reserving data space (of std::vector) could be considered. but the descriptor is lightweight so don't care.

    Eigen::MatrixXd makeScancontext( pcl::PointCloud<SCPointType> & _scan_down );
    Eigen::MatrixXd makeRingkeyFromScancontext( Eigen::MatrixXd &_desc );
    Eigen::MatrixXd makeSectorkeyFromScancontext( Eigen::MatrixXd &_desc );

    int fastAlignUsingVkey ( MatrixXd & _vkey1, MatrixXd & _vkey2 ); 
    double distDirectSC ( MatrixXd &_sc1, MatrixXd &_sc2 ); // "d" (eq 5) in the original paper (IROS 18)
    std::pair<double, int> distanceBtnScanContext ( MatrixXd &_sc1, MatrixXd &_sc2 ); // "D" (eq 6) in the original paper (IROS 18)

    // User-side API
    void makeAndSaveScancontextAndKeys( pcl::PointCloud<SCPointType> & _scan_down );
    std::pair<int, float> detectLoopClosureID( void ); // int: nearest node index, float: relative yaw  

    // for ltslam 
    // User-side API for multi-session
    void saveScancontextAndKeys( Eigen::MatrixXd _scd );
    std::pair<int, float> detectLoopClosureIDBetweenSession ( std::vector<float>& curr_key,  Eigen::MatrixXd& curr_desc);

    const Eigen::MatrixXd& getConstRefRecentSCD(void);

public:
    // hyper parameters ()
    double PC_MAX_RADIUS = 80.0; // 80 meter max in the original paper (IROS 18)
    double SC_DIST_THRES = 0.2; // 0.4-0.6 is good choice for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold / if not, recommend 0.1-0.15
    // const double SC_DIST_THRES = 0.7; // 0.4-0.6 is good choice for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold / if not, recommend 0.1-0.15
    int    NUM_EXCLUDE_RECENT = 30;
    int          tree_making_period_conter = 0;

    // setter
    void setSCdistThres(double _new_thres);
    void setMaximumRadius(double _max_r);

    // data 
    std::vector<double> polarcontexts_timestamp_; // optional.
    std::vector<Eigen::MatrixXd> polarcontexts_;
    std::vector<Eigen::MatrixXd> polarcontext_invkeys_;
    std::vector<Eigen::MatrixXd> polarcontext_vkeys_;

    KeyMat polarcontext_invkeys_mat_;
    KeyMat polarcontext_invkeys_to_search_;
    std::unique_ptr<InvKeyTree> polarcontext_tree_;

    bool is_tree_batch_made = false;
    std::unique_ptr<InvKeyTree> polarcontext_tree_batch_;

}; // SCManager

// } // namespace SC2
