#include "utility.h"
#include "sc_liorf_localization/msg/cloud_info.hpp"
#include "sc_liorf_localization/srv/save_map.hpp"
// <!-- sc_liorf_localization_yjz_lucky_boy -->
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <experimental/filesystem>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <pcl/registration/gicp.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include "Scancontext.h"
#include <experimental/filesystem>
#include <rcpputils/filesystem_helper.hpp>  
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/benchmark/read_points.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/registration_helper.hpp>
#include <pclomp/ndt_omp.h>
using namespace gtsam;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;

enum class SCInputType 
{ 
    SINGLE_SCAN_FULL, 
    SINGLE_SCAN_FEAT, 
    MULTI_SCAN_FEAT 
}; 

class mapOptimization : public ParamServer
{

public:
    // gtsam
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    rclcpp::Subscription<sc_liorf_localization::msg::CloudInfo>::SharedPtr subCloud;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subGPS;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subLoop;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMappedROS;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurround;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryGlobal;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryIncremental;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubKeyPoses;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubHistoryKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubIcpKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrame;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudRegisteredRaw;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubLoopConstraintEdge;
    rclcpp::Publisher<sc_liorf_localization::msg::CloudInfo>::SharedPtr pubSLAMInfo;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubGpsOdom;

    rclcpp::Service<sc_liorf_localization::srv::SaveMap>::SharedPtr srvSaveMap;
    std::deque<nav_msgs::msg::Odometry> gpsQueue;
    sc_liorf_localization::msg::CloudInfo cloudInfo;

    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
    
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;


    //addded**********************************by gc
    std::mutex mtxWin;
    std::vector<PointType> win_cloudKeyPoses3D;
    std::vector<PointTypePose> win_cloudKeyPoses6D;

    std::vector<pcl::PointCloud<PointType>::Ptr> win_cornerCloudKeyFrames;
    std::vector<pcl::PointCloud<PointType>::Ptr> win_surfCloudKeyFrames;

    //added***********************************by gc


    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast; // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; // downsampled corner featuer set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS; // downsampled surf featuer set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriCornerVec; // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterLocalMapSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization
    
    rclcpp::Time timeLaserInfoStamp;
    double timeLaserInfoCur;

    float transformTobeMapped[6];

    std::mutex mtx;
    std::mutex mtxLoopInfo;
    std::mutex mtxtranformOdomToWorld;
    std::mutex mtx_general;

    bool isDegenerate = false;
    cv::Mat matP;

    int winSize = 30;
    int laserCloudCornerFromMapDSNum = 0;
    int laserCloudCornerLastDSNum = 0;

    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    map<int, int> loopIndexContainer; // from new to old
    vector<pair<int, int>> loopIndexQueue;
    vector<gtsam::Pose3> loopPoseQueue;
    // vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
    vector<gtsam::SharedNoiseModel> loopNoiseQueue;
    deque<std_msgs::msg::Float64MultiArray> loopInfoVec;

    nav_msgs::msg::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;

    /*************added by gc*****************/
    pcl::PointCloud<PointType>::Ptr cloudGlobalMap;
    pcl::PointCloud<PointType>::Ptr cloudGlobalMapDS;
    pcl::PointCloud<PointType>::Ptr cloudScanForInitialize;
    pcl::PointCloud<PointType>::Ptr laserCloudRaw; // zxl
    pcl::PointCloud<PointType>::Ptr laserCloudRawDS; // zxl
    Eigen::MatrixXd matrix_poses;

    double laserCloudRawTime;//zxl
    pcl::VoxelGrid<PointType> downSizeFilterSC;//zxl

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subIniPoseFromRviz;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudInWorld;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMapWorld;
    //ros::Publisher fortest_publasercloudINWorld;

    float transformInTheWorld[6];// the pose in the world, i.e. the prebuilt map
    float tranformOdomToWorld[6];
    int globalLocaSkipFrames = 3;
    int frameNum = 1;
    int fileCount = 0;
    bool aLoopIsClosed = false;
    bool globalLocalizeInitialiized = false;

    enum InitializedFlag
    {
        NonInitialized,
        Initializing,
        Initialized
    };

    enum globalLocalizedFlag
    {
        NoGlobalLocalized,
        GlobalLocalized
    };

    enum InitSCLoopClosure
    {
        InitSCLoopClosured,
        NoinitSCLoopClosured
    };

    InitializedFlag initializedFlag;
    globalLocalizedFlag globalLocalizedFlag;
    InitSCLoopClosure initSCLoopClosure;
    // geometry_msgs::PoseStamped poseOdomToMap;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubOdomToMapPose;

    /*************added by gc******************/

    //zxl
    std::string loadSCDDirectory;
    std::string loadNodePCDDirectory;
    std::string loadPosesDirectory;
    SCManager scManager;//create a SCManager  object
    double timeLaserCloudInfoLast;
    double timeLastProcessing = -1;
    GeographicLib::LocalCartesian gps_trans_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> br;
    mapOptimization(const rclcpp::NodeOptions & options) : ParamServer("sc_liorf_localization_mapOptimization", options)
    {
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        subIniPoseFromRviz = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", QosPolicy(history_policy, reliability_policy),
                    std::bind(&mapOptimization::initialpose_callback, this, std::placeholders::_1));
        // subCloud = create_subscription<sc_liorf_localization::msg::CloudInfo>("sc_liorf_localization/feature/cloud_info", QosPolicy(history_policy, reliability_policy),
        //             std::bind(&mapOptimization::laserCloudInfoHandler, this, std::placeholders::_1));
        subCloud = create_subscription<sc_liorf_localization::msg::CloudInfo>("sc_liorf_localization/feature/cloud_info", qos_lidar,
                    std::bind(&mapOptimization::laserCloudInfoHandler, this, std::placeholders::_1));

        subGPS = create_subscription<sensor_msgs::msg::NavSatFix>(gpsTopic, 200,
                    std::bind(&mapOptimization::gpsHandler, this, std::placeholders::_1));

        // (新增)扩展发布者
        pubMapWorld = create_publisher<sensor_msgs::msg::PointCloud2>("sc_liorf_localization/mapping/cloud_map_map",1);       
        pubLaserCloudInWorld = create_publisher<sensor_msgs::msg::PointCloud2>("sc_liorf_localization/mapping/lasercloud_in_world", 1); 
        pubOdomToMapPose = create_publisher<geometry_msgs::msg::PoseStamped>("sc_liorf_localization/mapping/pose_odomTo_map", 1);       
        // pubOdomToMapPose = create_publisher<geometry_msgs::msg::PoseStamped>("sc_liorf_localization/mapping/pose_odomTo_map", 1);

        // pubOdomAftMappedROS = create_publisher<nav_msgs::msg::Odometry>("sc_liorf_localization/mapping/odometry", 1); 
        pubKeyPoses = create_publisher<sensor_msgs::msg::PointCloud2>("sc_liorf_localization/mapping/trajectory", 1);
        pubLaserCloudSurround = create_publisher<sensor_msgs::msg::PointCloud2>("sc_liorf_localization/mapping/map_global", 1);
        pubLaserOdometryGlobal = create_publisher<nav_msgs::msg::Odometry>("sc_liorf_localization/mapping/odometry", 1);

        pubLaserOdometryIncremental = create_publisher<nav_msgs::msg::Odometry>("sc_liorf_localization/mapping/odometry_incremental", qos);
        pubPath = create_publisher<nav_msgs::msg::Path>("sc_liorf_localization/mapping/path", 1);
        pubHistoryKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>("sc_liorf_localization/mapping/icp_loop_closure_history_cloud", 1);
        pubIcpKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>("sc_liorf_localization/mapping/icp_loop_closure_corrected_cloud", 1);
        pubLoopConstraintEdge = create_publisher<visualization_msgs::msg::MarkerArray>("/sc_liorf_localization/mapping/loop_closure_constraints", 1);
        pubRecentKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>("sc_liorf_localization/mapping/map_local", 1);
        pubRecentKeyFrame = create_publisher<sensor_msgs::msg::PointCloud2>("sc_liorf_localization/mapping/cloud_registered", 1);
        pubCloudRegisteredRaw = create_publisher<sensor_msgs::msg::PointCloud2>("sc_liorf_localization/mapping/cloud_registered_raw", 1);
        pubSLAMInfo = create_publisher<sc_liorf_localization::msg::CloudInfo>("sc_liorf_localization/mapping/slam_info", qos);
        pubGpsOdom = create_publisher<nav_msgs::msg::Odometry>("sc_liorf_localization/mapping/gps_odom", qos);

        srvSaveMap = create_service<sc_liorf_localization::srv::SaveMap>("sc_liorf_localization/save_map", 
                        std::bind(&mapOptimization::saveMapService, this, std::placeholders::_1, std::placeholders::_2 ));

        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize); 
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterLocalMapSurf.setLeafSize(surroundingKeyframeMapLeafSize, surroundingKeyframeMapLeafSize, surroundingKeyframeMapLeafSize);
        downSizeFilterICP.setLeafSize(loopClosureICPSurfLeafSize, loopClosureICPSurfLeafSize, loopClosureICPSurfLeafSize);
        downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

        br = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        loadSCDDirectory = savePCDDirectory + "SCDs/";    
        loadNodePCDDirectory =  savePCDDirectory + "Scans/"; 

        for (const auto& entry : std::experimental::filesystem::directory_iterator(loadSCDDirectory)) {
            if (is_regular_file(entry)) fileCount++;
        }
        std::cout << "Total number of Scan Context files: " << fileCount << std::endl;

        for(int i = 0; i < fileCount ; ++i) {
            std::string filename = padZeros(i); 
            std::string scd_path = loadSCDDirectory + filename + ".scd";
            
            Eigen::MatrixXd load_sc;           
            loadSCD(scd_path, load_sc);         
            
            // 提取环向/扇区特征描述子
            Eigen::MatrixXd ringkey = scManager.makeRingkeyFromScancontext(load_sc);    
            Eigen::MatrixXd sectorkey = scManager.makeSectorkeyFromScancontext(load_sc);
            std::vector<float> polarcontext_invkey_vec = eig2stdvec(ringkey);         

            scManager.polarcontexts_.push_back(load_sc);               
            scManager.polarcontext_invkeys_.push_back(ringkey);        
            scManager.polarcontext_vkeys_.push_back(sectorkey);         
            scManager.polarcontext_invkeys_mat_.push_back(polarcontext_invkey_vec); 
        }

        loadPosesDirectory = savePCDDirectory + "optimized_poses.txt"; 
        loadPoses(loadPosesDirectory, matrix_poses); 

        downSizeFilterSC.setLeafSize(0.5, 0.5, 0.5); 
        allocateMemory(); 
    }

    void allocateMemory()
    {
        laserCloudRaw.reset(new pcl::PointCloud<PointType>()); // zxl
        laserCloudRawDS.reset(new pcl::PointCloud<PointType>()); // zxl
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        cloudGlobalMap.reset(new pcl::PointCloud<PointType>());//addded by gc
	    cloudGlobalMapDS.reset(new pcl::PointCloud<PointType>());//added
        cloudScanForInitialize.reset(new pcl::PointCloud<PointType>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudOriCornerVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelCornerVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriCornerFlag.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        for (int i = 0; i < 6; ++i){
            transformTobeMapped[i] = 0;
        }
        for (int i = 0; i < 6; ++i){
            transformInTheWorld[i] = 0;
        }

        for (int i = 0; i < 6; ++i){
            tranformOdomToWorld[i] = 0;
        }
        initializedFlag = NonInitialized;
        initSCLoopClosure = NoinitSCLoopClosured;
        globalLocalizedFlag = NoGlobalLocalized;
        // std::cout<<" initializedFlag == " << initializedFlag <<std::endl;
        // std::cout<<" globalLocalizedFlag == " << globalLocalizedFlag <<std::endl;
        cloudGlobalLoad();//added by gc
        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
    }

    void laserCloudInfoHandler(const sc_liorf_localization::msg::CloudInfo::SharedPtr msgIn)
    {
        timeLaserInfoStamp = msgIn->header.stamp;
        auto &stamp = msgIn->header.stamp;
        timeLaserInfoCur = stamp.sec + static_cast<double>(stamp.nanosec) * 1e-9;

        // laserCloudRawTime = cloudInfo.header.stamp.toSec();        
        // extract info and feature cloud
        cloudInfo = *msgIn;

        pcl::fromROSMsg(msgIn->cloud_corner,  *laserCloudCornerLast);
        pcl::fromROSMsg(msgIn->cloud_surface, *laserCloudSurfLast);
        pcl::fromROSMsg(msgIn->cloud_deskewed,  *laserCloudRaw); // giseop 

        if((initializedFlag == NonInitialized || initializedFlag == Initializing)) 
        {

            if((cloudScanForInitialize->points.size() == 0) && (initSCLoopClosure != InitSCLoopClosured))
            {
                downsampleCurrentScan();

                mtx_general.lock();
                *cloudScanForInitialize += *laserCloudCornerLastDS;  
                *cloudScanForInitialize += *laserCloudSurfLastDS;  
                mtx_general.unlock();

                laserCloudCornerLastDS->clear();
                laserCloudSurfLastDS->clear();
                laserCloudCornerLastDSNum = 0;
                laserCloudSurfLastDSNum = 0;

                transformTobeMapped[0] = cloudInfo.imurollinit;
                transformTobeMapped[1] = cloudInfo.imupitchinit;
                transformTobeMapped[2] = cloudInfo.imuyawinit;
                if (!useImuHeadingInitialization) 
                    transformTobeMapped[2] = 0;

                pcl::PointCloud<PointType>::Ptr thisRawCloudKeyFrame(new pcl::PointCloud<PointType>());
                pcl::copyPointCloud(*laserCloudRawDS, *thisRawCloudKeyFrame);  
                scManager.makeAndSaveScancontextAndKeys(*thisRawCloudKeyFrame); 
                performSCLoopClosure(); 
            }
            return; 
        }

        // 正常SLAM流程 
        frameNum++; 

        std::lock_guard<std::mutex> lock(mtx);

        static double timeLastProcessing = -1;
        if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval)
        {
            timeLastProcessing = timeLaserInfoCur;

            updateInitialGuess();

            extractSurroundingKeyFrames();

            downsampleCurrentScan();

            scan2MapOptimization();

            saveKeyFramesAndFactor();

            // correctPoses();

            publishOdometry();

            publishFrames();
        }
    }


    void gpsHandler(const sensor_msgs::msg::NavSatFix::SharedPtr gpsMsg)
    {
        if (gpsMsg->status.status != 0)
            return;

        Eigen::Vector3d trans_local_;
        static bool first_gps = false;
        if (!first_gps) {
            first_gps = true;
            gps_trans_.Reset(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude);
        }

        gps_trans_.Forward(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude, trans_local_[0], trans_local_[1], trans_local_[2]);

        nav_msgs::msg::Odometry gps_odom;
        gps_odom.header = gpsMsg->header;
        gps_odom.header.frame_id = "map";
        gps_odom.pose.pose.position.x = trans_local_[0];
        gps_odom.pose.pose.position.y = trans_local_[1];
        gps_odom.pose.pose.position.z = trans_local_[2];
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(0.0, 0.0, 0.0);
        geometry_msgs::msg::Quaternion quat_msg;
        tf2::convert(quat_tf, quat_msg);
        gps_odom.pose.pose.orientation = quat_msg;
        pubGpsOdom->publish(gps_odom);
        gpsQueue.push_back(gps_odom);
    }

    /*************added by gc*****Todo: (1) ICP or matching point to edge and surface?  (2) global_pcd or whole keyframes************/
    void cloudGlobalLoad()
    {
        pcl::io::loadPCDFile( savePCDDirectory + "cloudGlobal.pcd", *cloudGlobalMap);

        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(cloudGlobalMap);
        downSizeFilterICP.filter(*cloud_temp);
        *cloudGlobalMapDS = *cloud_temp;

        std::cout << "test 0.01  the size of global cloud: " << cloudGlobalMap->points.size() << std::endl;
        std::cout << "test 0.02  the size of global map after filter: " << cloudGlobalMapDS->points.size() << std::endl;
    }

    void pointAssociateToMap(PointType const * const pi, PointType * const po)
    {
        po->x = transPointAssociateToMap(0,0) * pi->x + transPointAssociateToMap(0,1) * pi->y + transPointAssociateToMap(0,2) * pi->z + transPointAssociateToMap(0,3);
        po->y = transPointAssociateToMap(1,0) * pi->x + transPointAssociateToMap(1,1) * pi->y + transPointAssociateToMap(1,2) * pi->z + transPointAssociateToMap(1,3);
        po->z = transPointAssociateToMap(2,0) * pi->x + transPointAssociateToMap(2,1) * pi->y + transPointAssociateToMap(2,2) * pi->z + transPointAssociateToMap(2,3);
        po->intensity = pi->intensity;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
        
        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }

    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                                  gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
    }

    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), 
                                  gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
    { 
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    Eigen::Affine3f trans2Affine3f(float transformIn[])
    {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

    PointTypePose trans2PointTypePose(float transformIn[])
    {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll  = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw   = transformIn[2];
        return thisPose6D;
    }

    













    bool saveMapService(const std::shared_ptr<sc_liorf_localization::srv::SaveMap::Request> req,
                                std::shared_ptr<sc_liorf_localization::srv::SaveMap::Response> res)
    {
      string saveMapDirectory;

      cout << "****************************************************" << endl;
      cout << "Saving map to pcd files ..." << endl;
      if(req->destination.empty()) saveMapDirectory = std::getenv("HOME") + savePCDDirectory;
      else saveMapDirectory = std::getenv("HOME") + req->destination;
      cout << "Save destination: " << saveMapDirectory << endl;
      // create directory and remove old files;
      int unused = system((std::string("exec rm -r ") + saveMapDirectory).c_str());
      unused = system((std::string("mkdir -p ") + saveMapDirectory).c_str());
      // save key frame transformations
      pcl::io::savePCDFileBinary(saveMapDirectory + "/trajectory.pcd", *cloudKeyPoses3D);
      pcl::io::savePCDFileBinary(saveMapDirectory + "/transformations.pcd", *cloudKeyPoses6D);
      // extract global point cloud map

      pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
      for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++) {
          *globalSurfCloud   += *transformPointCloud(surfCloudKeyFrames[i],    &cloudKeyPoses6D->points[i]);
          cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";
      }

      if(req->resolution != 0)
      {
        cout << "\n\nSave resolution: " << req->resolution << endl;
        // down-sample and save surf cloud
        downSizeFilterSurf.setInputCloud(globalSurfCloud);
        downSizeFilterSurf.setLeafSize(req->resolution, req->resolution, req->resolution);
        downSizeFilterSurf.filter(*globalSurfCloudDS);
        pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloudDS);
      }
      else
      {

        // save surf cloud
        pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloud);
      }

      // save global point cloud map
      *globalMapCloud += *globalSurfCloud;

      int ret = pcl::io::savePCDFileBinary(saveMapDirectory + "/GlobalMap.pcd", *globalMapCloud);
      res->success = ret == 0;

      downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);

      cout << "****************************************************" << endl;
      cout << "Saving map to pcd files completed\n" << endl;

      return true;
    }

    void publishGlobalMap()
    {
        if (pubLaserCloudSurround->get_subscription_count() == 0)
            return;

        if (cloudKeyPoses3D->points.empty() == true)
            return;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());;
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

        // kd-tree to find near key frames to visualize
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        // search near key frames to visualize
        mtx.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
        // downsample near selected key frames
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
        for(auto& pt : globalMapKeyPosesDS->points)
        {
            kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
            pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
        }

        // extract visualized and downsampled key frames
        for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i){
            if (common_lib_->pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
                continue;
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
        }
        // downsample visualized points
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
        publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, odometryFrame);
    }





    void performSCLoopClosure()
    {
        int loopKeyCur = scManager.polarcontexts_.size() - 1; 
        int subsequent_invkeys_mat = loopKeyCur - fileCount;

        // find keys
        auto detectResult = scManager.detectLoopClosureID(subsequent_invkeys_mat); 
        // auto detectResult = scManager.detectLoopClosureID(); 
        // int loopKeyCur = scManager.polarcontexts_.size() - 1; 
        int loopKeyPre = detectResult.first;
        float yawDiffRad = detectResult.second; 

        if( loopKeyPre == -1 || loopKeyPre >= fileCount)
        {
            std::cout << "No loop found!" << std::endl;
            cloudScanForInitialize->clear(); 
            return;
        }

        std::cout << "SC loop found! between " << loopKeyCur << " and " << loopKeyPre << "." << std::endl; 

        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        
        int base_key = 0; 
        Eigen::MatrixXd pose_temp = matrix_poses.row(loopKeyPre); 
        Eigen::Matrix<double, 3, 1> t_temp; 
        Eigen::Matrix<double, 3, 3> R_temp; 
        t_temp << pose_temp(3), pose_temp(7), pose_temp(11); 
        R_temp << pose_temp(0), pose_temp(1), pose_temp(2), pose_temp(4), pose_temp(5), pose_temp(6), pose_temp(8), pose_temp(9), pose_temp(10); 

        Eigen::Vector3d eulerAngle_temp = R_temp.eulerAngles(0, 1, 2);
        
        PointTypePose pointTypePose_temp;
        pointTypePose_temp.x = t_temp(0);
        pointTypePose_temp.y = t_temp(1);
        pointTypePose_temp.z = t_temp(2);
        pointTypePose_temp.roll = eulerAngle_temp(0);
        pointTypePose_temp.pitch = eulerAngle_temp(1);
        pointTypePose_temp.yaw = eulerAngle_temp(2);

        *cureKeyframeCloud += *transformPointCloud(laserCloudRawDS, &pointTypePose_temp);

        loopFindNearKeyframesWithRespectTo(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum, base_key); 

        if (cureKeyframeCloud->size() < 450 || prevKeyframeCloud->size() < 1000)
        {
            cloudScanForInitialize->clear(); 
            std::cout << "cureKeyframeCloud or prevKeyframeCloud are too little!" << std::endl;
            std::cout << "cureKeyframeCloud->size() == " <<cureKeyframeCloud->size()<< " prevKeyframeCloud->size() == "<< prevKeyframeCloud->size()  << std::endl;
            return;
        }

        // NDT Settings
        pcl::NormalDistributionsTransform<PointType, PointType> ndt_multires;
        ndt_multires.setTransformationEpsilon(0.001); 
        ndt_multires.setResolution(0.6); 
        ndt_multires.setStepSize(0.1); 

 
        ndt_multires.setInputSource(cureKeyframeCloud);
        ndt_multires.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result_0(new pcl::PointCloud<PointType>());
        ndt_multires.align(*unused_result_0); 

        // ICP Settings
        pcl::PointCloud<pcl::PointCovariance>::Ptr target = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZI>, pcl::PointCloud<pcl::PointCovariance>>(*prevKeyframeCloud, 0.20);
        pcl::PointCloud<pcl::PointCovariance>::Ptr source = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZI>, pcl::PointCloud<pcl::PointCovariance>>(*cureKeyframeCloud, 0.20);

        // Estimate covariances of points.
        const int num_neighbors = small_num_neighbors;
        small_gicp::estimate_covariances_omp(*target, small_num_neighbors, small_gicp_reduction_num_threads);
        small_gicp::estimate_covariances_omp(*source, small_num_neighbors, small_gicp_reduction_num_threads);

        // Create KdTree for target and source.
        auto target_tree = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(target, small_gicp::KdTreeBuilderOMP(small_gicp_reduction_num_threads));
        auto source_tree = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(source, small_gicp::KdTreeBuilderOMP(small_gicp_reduction_num_threads));
        small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> small_gicp;
        small_gicp.reduction.num_threads = small_gicp_reduction_num_threads;
        small_gicp.rejector.max_dist_sq = 20.0;
        small_gicp.criteria.rotation_eps = small_gicp_criteria_rotation_eps;
        small_gicp.criteria.translation_eps = small_gicp_criteria_translation_eps;
        small_gicp.optimizer.max_iterations = small_gicp_optimizer_max_iterations;

        Eigen::Isometry3d eigen_isometry;
        eigen_isometry.matrix() = ndt_multires.getFinalTransformation().cast<double>();
        auto small_gicp_result = small_gicp.align(*target, *source, *target_tree, eigen_isometry);

        if (!small_gicp_result.converged || small_gicp_result.error>=small_gicp_result_max_error) {
        std::cout << "SMALL_GICP fitness test failed "<< " Reject this SC loop." << std::endl;
        cout<<"small_gicp_result.converged =="<<small_gicp_result.converged<<" error == "<<small_gicp_result.error <<std::endl;
        cloudScanForInitialize->clear(); 
        return;
        }
        else 
        {
        std::cout << "SMALL_GICP fitness test passed " <<" Add this SC loop." <<std::endl;
        cout<<"small_gicp_result.converged =="<<small_gicp_result.converged<<" error == "<<small_gicp_result.error <<std::endl;
        initSCLoopClosure = InitSCLoopClosured; 
        }

        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = small_gicp_result.T_target_source.cast<float>();
        Eigen::Affine3f loopKeyPreTransformInTheWorld = pclPointToAffine3f(pointTypePose_temp); 
        Eigen::Affine3f loopKeyCurTransformInTheWorld = loopKeyPreTransformInTheWorld * correctionLidarFrame; 

        pcl::getTranslationAndEulerAngles(loopKeyCurTransformInTheWorld, x, y, z, roll, pitch, yaw);


        yaw += yawDiffRad;
        transformInTheWorld[0] = roll;
        transformInTheWorld[1] = pitch;
        transformInTheWorld[2] = yaw;
        transformInTheWorld[3] = x;
        transformInTheWorld[4] = y;
        transformInTheWorld[5] = z;

        std::cout << "The adjusted pose loop is: x" << x << " y" << y 
                << " z" << z << " yaw: " << yaw << std::endl;

    }

    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum, const int& loop_index)
    {
        // extract near keyframes
        nearKeyframes->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize )
                continue;

            int select_loop_index = (loop_index != -1) ? loop_index : key + i;
            *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear],   &copy_cloudKeyPoses6D->points[select_loop_index]);
        }

        if (nearKeyframes->empty())
            return;

        // downsample near keyframes
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }

    void updateInitialGuess()
    {

        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

        static Eigen::Affine3f lastImuTransformation; 

        if (cloudKeyPoses3D->points.empty()) // No key pose, initialize
        {
            transformTobeMapped[0] = cloudInfo.imurollinit;
            transformTobeMapped[1] = cloudInfo.imupitchinit;
            transformTobeMapped[2] = cloudInfo.imuyawinit;

            if (!useImuHeadingInitialization) // If not using IMU heading for initialization
                transformTobeMapped[2] = 0;

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imurollinit, cloudInfo.imupitchinit, cloudInfo.imuyawinit);
            return; 
        }

        static bool lastImuPreTransAvailable = false;      
        static Eigen::Affine3f lastImuPreTransformation;  

        // Use IMU pre-integration estimation for pose guess 
        if (cloudInfo.odomavailable == true) 
        {

            Eigen::Affine3f transBack = pcl::getTransformation(
                cloudInfo.initialguessx, cloudInfo.initialguessy, cloudInfo.initialguessz,
                cloudInfo.initialguessroll, cloudInfo.initialguesspitch, cloudInfo.initialguessyaw);

            if (!lastImuPreTransAvailable) 
            {
                lastImuPreTransformation = transBack; 
                lastImuPreTransAvailable = true;       
            }
            else
            {

                Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
                
                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
                Eigen::Affine3f transFinal = transTobe * transIncre;
                
                pcl::getTranslationAndEulerAngles(
                    transFinal,
                    transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                    transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]  
                );

                lastImuPreTransformation = transBack; 
                lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imurollinit, cloudInfo.imupitchinit, cloudInfo.imuyawinit);
                return; 
            }
        }

        // Use IMU incremental estimation for pose guess (only rotation)
        if (cloudInfo.imuavailable == true && imuType) 
        {
            Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imurollinit, cloudInfo.imupitchinit, cloudInfo.imuyawinit);
            
            Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;

            pcl::getTranslationAndEulerAngles(
                transFinal,
                transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]  
            );

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imurollinit, cloudInfo.imupitchinit, cloudInfo.imuyawinit); 
            return;
        }
    }

    void extractForLoopClosure()
    {
        pcl::PointCloud<PointType>::Ptr cloudToExtract(new pcl::PointCloud<PointType>());
        int numPoses = win_cloudKeyPoses3D.size();
        for (int i =  numPoses-1; i >=0; --i)
        {
            cloudToExtract->push_back(win_cloudKeyPoses3D[i]);

        }
        extractCloud(cloudToExtract);

    }

    void extractNearby()
    {
        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // extract all the nearby key poses and downsample them
        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
        kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
        for (int i = 0; i < (int)pointSearchInd.size(); ++i)
        {
            int id = pointSearchInd[i];
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
        }

        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
        for(auto& pt : surroundingKeyPosesDS->points)
        {
            kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
            pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
        }

        // also extract some latest key frames in case the robot rotates in one position
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses-1; i >= 0; --i)
        {
            if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
                surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(surroundingKeyPosesDS);
    }

    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
    {
        std::vector<pcl::PointCloud<PointType>> laserCloudCornerSurroundingVec;
        std::vector<pcl::PointCloud<PointType>> laserCloudSurfSurroundingVec;

        laserCloudCornerSurroundingVec.resize(cloudToExtract->size());
        laserCloudSurfSurroundingVec.resize(cloudToExtract->size());

        // Extract surrounding map
    #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < (int)cloudToExtract->size(); ++i)
        {
            PointTypePose thisPose6D = win_cloudKeyPoses6D[i];
            laserCloudCornerSurroundingVec[i] = *transformPointCloud(win_cornerCloudKeyFrames[i], &thisPose6D);
            laserCloudSurfSurroundingVec[i] = *transformPointCloud(win_surfCloudKeyFrames[i], &thisPose6D);
        }

        // Fuse the map
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear();
        for (int i = 0; i < (int)cloudToExtract->size(); ++i)
        {
            *laserCloudCornerFromMap += laserCloudCornerSurroundingVec[i];
            *laserCloudSurfFromMap += laserCloudSurfSurroundingVec[i];
        }

        // Downsample the surrounding corner key frames (or map)
        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();

        // Downsample the surrounding surf key frames (or map)
        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();
    }

    void extractSurroundingKeyFrames()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return; 
        
        // if (loopClosureEnableFlag == true)
        // {
            extractForLoopClosure();    
        // } else {
        //     extractNearby();
        // }

        // extractNearby();
    }

    void downsampleCurrentScan()
    {
        //zxl
        laserCloudRawDS->clear();
        downSizeFilterSC.setInputCloud(laserCloudRaw);
        downSizeFilterSC.filter(*laserCloudRawDS);

        // Downsample cloud from current scan
        laserCloudCornerLastDS->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

        laserCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
    }

    void cornerOptimization()
    {
        updatePointAssociateToMap();

        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudCornerLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudCornerLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));
                    
            if (pointSearchSqDis[4] < 1.0) {
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++) {
                    cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                    cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                    cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                }
                cx /= 5; cy /= 5;  cz /= 5;

                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++) {
                    float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                    float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                    float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                    a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
                    a22 += ay * ay; a23 += ay * az;
                    a33 += az * az;
                }
                a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

                matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
                matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
                matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;

                cv::eigen(matA1, matD1, matV1);

                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

                    float x0 = pointSel.x;
                    float y0 = pointSel.y;
                    float z0 = pointSel.z;
                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                    float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                                    + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                                    + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)) * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                    float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                    float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                              + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                    float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                               - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                               + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float ld2 = a012 / l12;

                    float s = 1 - 0.9 * fabs(ld2);

                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;

                    if (s > 0.1) {
                        laserCloudOriCornerVec[i] = pointOri;
                        coeffSelCornerVec[i] = coeff;
                        laserCloudOriCornerFlag[i] = true;
                    }
                }
            }
        }
    }

    void updatePointAssociateToMap()
    {
        transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
    }

    void surfOptimization()
    {
        updatePointAssociateToMap();

        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudSurfLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudSurfLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel); 
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;

            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();
            if (pointSearchSqDis[4] < 1.0) {
                for (int j = 0; j < 5; j++) {
                    matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }

                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps; pb /= ps; pc /= ps; pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++) {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                             pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid) {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x
                            + pointOri.y * pointOri.y + pointOri.z * pointOri.z));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    if (s > 0.1) {
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
                        laserCloudOriSurfFlag[i] = true;
                    }
                }
            }
        }
    }

    void loopFindNearKeyframesWithRespectTo(pcl::PointCloud<PointType>::Ptr& nearKeyframes, 
                                        const int& key,      
                                        const int& searchNum,  
                                        const int _wrt_key)    
    {
        // extract near keyframes
        nearKeyframes->clear();  
        int cloudSize = scManager.polarcontexts_.size();  

        for (int i = -searchNum; i <= searchNum; ++i) 
        {
            int keyNear = key + i; 
            
            if (keyNear < 0)
                continue;
            if (keyNear >= fileCount)
                break;

            pcl::PointCloud<PointType>::Ptr scan_temp(new pcl::PointCloud<PointType>());

            pcl::io::loadPCDFile(loadNodePCDDirectory + padZeros(keyNear) + ".pcd", *scan_temp);

            Eigen::MatrixXd pose_temp = matrix_poses.row(keyNear);  

            Eigen::Matrix<double,3,1> t_temp;  
            Eigen::Matrix<double,3,3> R_temp; 
            t_temp << pose_temp(3), pose_temp(7), pose_temp(11);  
            R_temp << pose_temp(0), pose_temp(1), pose_temp(2),   
                    pose_temp(4), pose_temp(5), pose_temp(6),
                    pose_temp(8), pose_temp(9), pose_temp(10);

            Eigen::Vector3d eulerAngle_temp = R_temp.eulerAngles(0, 1, 2);

            PointTypePose pointTypePose_temp;
            pointTypePose_temp.x = t_temp(0);     
            pointTypePose_temp.y = t_temp(1);      
            pointTypePose_temp.z = t_temp(2);     
            pointTypePose_temp.roll = eulerAngle_temp(0);  
            pointTypePose_temp.pitch = eulerAngle_temp(1); 
            pointTypePose_temp.yaw = eulerAngle_temp(2);   


            *nearKeyframes += *transformPointCloud(scan_temp, &pointTypePose_temp);
        }

        if (nearKeyframes->empty())
            return;

        // downsample near keyframes
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes); 
        downSizeFilterICP.filter(*cloud_temp);          
        *nearKeyframes = *cloud_temp;                    

    }

    void combineOptimizationCoeffs()
    {
        // combine corner coeffs
        for (int i = 0; i < laserCloudCornerLastDSNum; ++i){
            if (laserCloudOriCornerFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriCornerVec[i]);
                coeffSel->push_back(coeffSelCornerVec[i]);
            }
        }

        // combine surf coeffs
        for (int i = 0; i < laserCloudSurfLastDSNum; ++i){
            if (laserCloudOriSurfFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                coeffSel->push_back(coeffSelSurfVec[i]);
            }
        }
        // reset flag for next iteration
        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
    }

    bool LMOptimization(int iterCount)
    {
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // lidar <- camera      ---     camera <- lidar
        // x = z                ---     x = y
        // y = x                ---     y = z
        // z = y                ---     z = x
        // roll = yaw           ---     roll = pitch
        // pitch = roll         ---     pitch = yaw
        // yaw = pitch          ---     yaw = roll

        // lidar -> camera
        float srx = sin(transformTobeMapped[2]);
        float crx = cos(transformTobeMapped[2]);
        float sry = sin(transformTobeMapped[1]);
        float cry = cos(transformTobeMapped[1]);
        float srz = sin(transformTobeMapped[0]);
        float crz = cos(transformTobeMapped[0]);

        int laserCloudSelNum = laserCloudOri->size();
        if (laserCloudSelNum < 50) {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

        PointType pointOri, coeff;

        for (int i = 0; i < laserCloudSelNum; i++) {
            // lidar -> camera
            pointOri.x = laserCloudOri->points[i].x;
            pointOri.y = laserCloudOri->points[i].y;
            pointOri.z = laserCloudOri->points[i].z;
            // lidar -> camera
            coeff.x = coeffSel->points[i].x;
            coeff.y = coeffSel->points[i].y;
            coeff.z = coeffSel->points[i].z;
            coeff.intensity = coeffSel->points[i].intensity;
            // in camera
/*             float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                      + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                      + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

            float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                      + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                      + ((-cry*crz - srx*sry*srz)*pointOri.x 
                      + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

            float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                      + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                      + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
             */

            float arx = (-srx * cry * pointOri.x - (srx * sry * srz + crx * crz) * pointOri.y + (crx * srz - srx * sry * crz) * pointOri.z) * coeff.x
                      + (crx * cry * pointOri.x - (srx * crz - crx * sry * srz) * pointOri.y + (crx * sry * crz + srx * srz) * pointOri.z) * coeff.y;

            float ary = (-crx * sry * pointOri.x + crx * cry * srz * pointOri.y + crx * cry * crz * pointOri.z) * coeff.x
                      + (-srx * sry * pointOri.x + srx * sry * srz * pointOri.y + srx * cry * crz * pointOri.z) * coeff.y
                      + (-cry * pointOri.x - sry * srz * pointOri.y - sry * crz * pointOri.z) * coeff.z;

            float arz = ((crx * sry * crz + srx * srz) * pointOri.y + (srx * crz - crx * sry * srz) * pointOri.z) * coeff.x
                      + ((-crx * srz + srx * sry * crz) * pointOri.y + (-srx * sry * srz - crx * crz) * pointOri.z) * coeff.y
                      + (cry * crz * pointOri.y - cry * srz * pointOri.z) * coeff.z;
              
            // camera -> lidar
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = ary;
            matA.at<float>(i, 2) = arx;
            matA.at<float>(i, 3) = coeff.x;
            matA.at<float>(i, 4) = coeff.y;
            matA.at<float>(i, 5) = coeff.z;
            matB.at<float>(i, 0) = -coeff.intensity;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {

            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate)
        {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        float deltaR = sqrt(
                            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.05 && deltaT < 0.05) {
            return true; // converged
        }
        return false; // keep optimizing
    }

    void scan2MapOptimization()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum && laserCloudSurfLastDSNum > surfFeatureMinValidNum)
        {
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            for (int iterCount = 0; iterCount < 20; iterCount++)
            {
                laserCloudOri->clear();
                coeffSel->clear();

                cornerOptimization();
                surfOptimization();

                combineOptimizationCoeffs();

                if (LMOptimization(iterCount) == true)
                    break;              
            }

            transformUpdate();
        } else {
            RCLCPP_WARN(get_logger(), "Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
        }
    }

    void transformUpdate()
    {
        if (cloudInfo.imuavailable == true && imuType)
        {
            if (std::abs(cloudInfo.imupitchinit) < 1.4)
            {
                double imuWeight = imuRPYWeight;
                tf2::Quaternion imuQuaternion;
                tf2::Quaternion transformQuaternion;
                double rollMid, pitchMid, yawMid;

                // slerp roll
                transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
                imuQuaternion.setRPY(cloudInfo.imurollinit, 0, 0);
                tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[0] = rollMid;

                // slerp pitch
                transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
                imuQuaternion.setRPY(0, cloudInfo.imupitchinit, 0);
                tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[1] = pitchMid;
            }
        }

        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);

        incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
    }

    float constraintTransformation(float value, float limit)
    {
        if (value < -limit)
            value = -limit;
        if (value > limit)
            value = limit;

        return value;
    }

    bool saveFrame()
    {
        if (cloudKeyPoses3D->points.empty())
            return true;

        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
            abs(pitch) < surroundingkeyframeAddingAngleThreshold && 
            abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
            sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)
            return false;

        return true;
    }

    void addOdomFactor()
    {
        if (cloudKeyPoses3D->points.empty())
        {
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Eigen::VectorXd(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished());
// rad*rad, meter*meter
            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
        }else{
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Eigen::VectorXd(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
            gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
        }
    }

    void addGPSFactor()
    {
        if (gpsQueue.empty())
            return;

        // wait for system initialized and settles down
        if (cloudKeyPoses3D->points.empty())
            return;
        else
        {
            if (common_lib_->pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
                return;
        }

        // pose covariance small, no need to correct
        if (poseCovariance(3,3) < poseCovThreshold && poseCovariance(4,4) < poseCovThreshold)
            return;

        // last gps position
        static PointType lastGPSPoint;

        while (!gpsQueue.empty())
        {
            if (ROS_TIME(gpsQueue.front().header.stamp) < timeLaserInfoCur - 0.2)
            {
                // message too old
                gpsQueue.pop_front();
            }
            else if (ROS_TIME(gpsQueue.front().header.stamp) > timeLaserInfoCur + 0.2)
            {
                // message too new
                break;
            }
            else
            {
                nav_msgs::msg::Odometry thisGPS = gpsQueue.front();
                gpsQueue.pop_front();

                // GPS too noisy, skip
                float noise_x = thisGPS.pose.covariance[0];
                float noise_y = thisGPS.pose.covariance[7];
                float noise_z = thisGPS.pose.covariance[14];
                if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                    continue;

                float gps_x = thisGPS.pose.pose.position.x;
                float gps_y = thisGPS.pose.pose.position.y;
                float gps_z = thisGPS.pose.pose.position.z;
                if (!useGpsElevation)
                {
                    gps_z = transformTobeMapped[5];
                    noise_z = 0.01;
                }

                // GPS not properly initialized (0,0,0)
                if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                    continue;

                // Add GPS every a few meters
                PointType curGPSPoint;
                curGPSPoint.x = gps_x;
                curGPSPoint.y = gps_y;
                curGPSPoint.z = gps_z;
                if (common_lib_->pointDistance(curGPSPoint, lastGPSPoint) < 5.0)
                    continue;
                else
                    lastGPSPoint = curGPSPoint;

                gtsam::Vector Vector3(3);
                Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
                noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
                gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
                gtSAMgraph.add(gps_factor);

                aLoopIsClosed = true;
                break;
            }
        }
    }

    void addLoopFactor()
    {
        if (loopIndexQueue.empty())
            return;

        for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
        {
            int indexFrom = loopIndexQueue[i].first;
            int indexTo = loopIndexQueue[i].second;
            gtsam::Pose3 poseBetween = loopPoseQueue[i];
            // gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
            auto noiseBetween = loopNoiseQueue[i];
            gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
        }

        loopIndexQueue.clear();
        loopPoseQueue.clear();
        loopNoiseQueue.clear();
        aLoopIsClosed = true;
    }

    void saveKeyFramesAndFactor()
    {
        if (saveFrame() == false)
            return;

        // odom factor
        addOdomFactor();

        // gps factor
        addGPSFactor();

        // loop factor
        addLoopFactor();

        // cout << "****************************************************" << endl;
        // gtSAMgraph.print("GTSAM Graph:\n");

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        if (aLoopIsClosed == true)
        {
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
        }

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        //save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
        // cout << "****************************************************" << endl;
        // isamCurrentEstimate.print("Current estimate: ");

        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
        thisPose6D.roll  = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw   = latestEstimate.rotation().yaw();
        thisPose6D.time = timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);
        mtxWin.lock();

        win_cloudKeyPoses3D.push_back(thisPose3D);
        win_cloudKeyPoses6D.push_back(thisPose6D);
		if(win_cloudKeyPoses3D.size() > winSize)
		{
			win_cloudKeyPoses3D.erase(win_cloudKeyPoses3D.begin());
			win_cloudKeyPoses6D.erase(win_cloudKeyPoses6D.begin());
		}

        // cout << "****************************************************" << endl;
        // cout << "Pose covariance:" << endl;
        // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);

        // save updated transform
        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();

        // save all the received edge and surf points
        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());

        pcl::copyPointCloud(*laserCloudCornerLastDS,  *thisCornerKeyFrame);
        pcl::copyPointCloud(*laserCloudSurfLastDS,    *thisSurfKeyFrame);

        // save key frame cloud
        cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);
        // The following code is copy from sc-lio-sam
        // Scan Context loop detector - giseop
        // - SINGLE_SCAN_FULL: using downsampled original point cloud (/full_cloud_projected + downsampling)
        // - SINGLE_SCAN_FEAT: using surface feature as an input point cloud for scan context (2020.04.01: checked it works.)
        // - MULTI_SCAN_FEAT: using NearKeyframes (because a MulRan scan does not have beyond region, so to solve this issue ... )
        const SCInputType sc_input_type = SCInputType::SINGLE_SCAN_FULL; // change this 

        if( sc_input_type == SCInputType::SINGLE_SCAN_FULL )
        {
            pcl::PointCloud<PointType>::Ptr thisRawCloudKeyFrame(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *thisRawCloudKeyFrame);

            scManager.makeAndSaveScancontextAndKeys(*thisRawCloudKeyFrame);
        }  
        else if (sc_input_type == SCInputType::SINGLE_SCAN_FEAT)
        { 
            scManager.makeAndSaveScancontextAndKeys(*thisSurfKeyFrame); 
        }
        else if (sc_input_type == SCInputType::MULTI_SCAN_FEAT)
        { 
            pcl::PointCloud<PointType>::Ptr multiKeyFrameFeatureCloud(new pcl::PointCloud<PointType>());
            loopFindNearKeyframes(multiKeyFrameFeatureCloud, cloudKeyPoses6D->size() - 1, historyKeyframeSearchNum, -1);
            scManager.makeAndSaveScancontextAndKeys(*multiKeyFrameFeatureCloud); 
        }

        /*added    gc*/

        
            win_cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
            win_surfCloudKeyFrames.push_back(thisSurfKeyFrame);
        	if(win_surfCloudKeyFrames.size() > winSize)
		{
			win_cornerCloudKeyFrames.erase(win_cornerCloudKeyFrames.begin());
			win_surfCloudKeyFrames.erase(win_surfCloudKeyFrames.begin());
		}
		
        
        mtxWin.unlock();
        /*added    gc*/
        // save path for visualization
        updatePath(thisPose6D);
    }

    void correctPoses()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (aLoopIsClosed == true)
        {
            // clear map cache
            laserCloudMapContainer.clear();
            // clear path
            globalPath.poses.clear();
            // update key poses
            int numPoses = isamCurrentEstimate.size();
            for (int i = 0; i < numPoses; ++i)
            {
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

                updatePath(cloudKeyPoses6D->points[i]);
            }

            aLoopIsClosed = false;
        }
    }

    void updatePath(const PointTypePose& pose_in)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        rclcpp::Time t(static_cast<uint32_t>(pose_in.time * 1e9));
        pose_stamped.header.stamp = t;
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        tf2::Quaternion q;
        q.setRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }

    void publishOdometry()
    {
        // Publish odometry for ROS (global)
        nav_msgs::msg::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = odometryFrame;
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        // Ref: http://wiki.ros.org/tf2/Tutorials/Migration/DataConversions
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        geometry_msgs::msg::Quaternion quat_msg;
        tf2::convert(quat_tf, quat_msg);
        laserOdometryROS.pose.pose.orientation = quat_msg;
        pubLaserOdometryGlobal->publish(laserOdometryROS);
        
        // Publish TF
        quat_tf.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        tf2::Transform t_odom_to_lidar = tf2::Transform(quat_tf, tf2::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        tf2::TimePoint time_point = tf2_ros::fromRclcpp(timeLaserInfoStamp);
        tf2::Stamped<tf2::Transform> temp_odom_to_lidar(t_odom_to_lidar, time_point, odometryFrame);
        geometry_msgs::msg::TransformStamped trans_odom_to_lidar;
        tf2::convert(temp_odom_to_lidar, trans_odom_to_lidar);
        trans_odom_to_lidar.child_frame_id = "lidar_link";
        br->sendTransform(trans_odom_to_lidar);

        // Publish odometry for ROS (incremental)
        static bool lastIncreOdomPubFlag = false;
        static nav_msgs::msg::Odometry laserOdomIncremental; // incremental odometry msg
        static Eigen::Affine3f increOdomAffine; // incremental odometry in affine
        if (lastIncreOdomPubFlag == false)
        {
            lastIncreOdomPubFlag = true;
            laserOdomIncremental = laserOdometryROS;
            increOdomAffine = trans2Affine3f(transformTobeMapped);
        } else {
            Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
            increOdomAffine = increOdomAffine * affineIncre;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles (increOdomAffine, x, y, z, roll, pitch, yaw);
            if (cloudInfo.imuavailable == true && imuType)
            {
                if (std::abs(cloudInfo.imupitchinit) < 1.4)
                {
                    double imuWeight = 0.1;
                    tf2::Quaternion imuQuaternion;
                    tf2::Quaternion transformQuaternion;
                    double rollMid, pitchMid, yawMid;

                    // slerp roll
                    transformQuaternion.setRPY(roll, 0, 0);
                    imuQuaternion.setRPY(cloudInfo.imurollinit, 0, 0);
                    tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    roll = rollMid;

                    // slerp pitch
                    transformQuaternion.setRPY(0, pitch, 0);
                    imuQuaternion.setRPY(0, cloudInfo.imupitchinit, 0);
                    tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    pitch = pitchMid;
                }
            }
            laserOdomIncremental.header.stamp = timeLaserInfoStamp;
            laserOdomIncremental.header.frame_id = odometryFrame;
            laserOdomIncremental.child_frame_id = "odom_mapping";
            laserOdomIncremental.pose.pose.position.x = x;
            laserOdomIncremental.pose.pose.position.y = y;
            laserOdomIncremental.pose.pose.position.z = z;
            tf2::Quaternion quat_tf;
            quat_tf.setRPY(roll, pitch, yaw);
            geometry_msgs::msg::Quaternion quat_msg;
            tf2::convert(quat_tf, quat_msg);
            laserOdomIncremental.pose.pose.orientation = quat_msg;
            if (isDegenerate)
                laserOdomIncremental.pose.covariance[0] = 1;
            else
                laserOdomIncremental.pose.covariance[0] = 0;
        }
        pubLaserOdometryIncremental->publish(laserOdomIncremental);
    }

    void publishFrames()
    {
        if (cloudKeyPoses3D->points.empty())
            return;
        // publish key poses
        publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);
        // Publish surrounding key frames
        publishCloud(pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, odometryFrame);
        // publish registered key frame
        if (pubRecentKeyFrame->get_subscription_count() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut += *transformPointCloud(laserCloudCornerLastDS,  &thisPose6D);
            *cloudOut += *transformPointCloud(laserCloudSurfLastDS,    &thisPose6D);
            publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, odometryFrame);
        }

        //added *****************by gc
        if(pubLaserCloudInWorld->get_subscription_count() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudInBase(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr cloudOutInWorld(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6DInOdom = trans2PointTypePose(transformTobeMapped);
            Eigen::Affine3f T_thisPose6DInOdom = pclPointToAffine3f(thisPose6DInOdom);
            mtxtranformOdomToWorld.lock();
            PointTypePose pose_Odom_Map = trans2PointTypePose(tranformOdomToWorld);
            mtxtranformOdomToWorld.unlock();
            Eigen::Affine3f T_pose_Odom_Map = pclPointToAffine3f(pose_Odom_Map);

            Eigen::Affine3f T_poseInMap = T_pose_Odom_Map * T_thisPose6DInOdom;
            *cloudInBase += *laserCloudCornerLastDS;
            *cloudInBase += *laserCloudSurfLastDS;
            pcl::transformPointCloud(*cloudInBase, *cloudOutInWorld, T_poseInMap.matrix());
            publishCloud(pubLaserCloudInWorld, cloudOutInWorld, timeLaserInfoStamp, "map");
        }


        // publish registered high-res raw cloud
        if (pubCloudRegisteredRaw->get_subscription_count() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);
            publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish path
        if (pubPath->get_subscription_count() != 0)
        {
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath->publish(globalPath);
        }
        // publish SLAM infomation for 3rd-party usage
        static int lastSLAMInfoPubSize = -1;
        if (pubSLAMInfo->get_subscription_count() != 0)
        {
            // if (lastSLAMInfoPubSize != cloudKeyPoses6D->size())
            // {
            //     sc_liorf_localization::msg::CloudInfo slamInfo;
            //     slamInfo.header.stamp = timeLaserInfoStamp;
            //     pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            //     *cloudOut += *laserCloudSurfLastDS;
            //     slamInfo.key_frame_cloud = publishCloud(rclcpp::Publisher(), cloudOut, timeLaserInfoStamp, lidarFrame);
            //     slamInfo.key_frame_poses = publishCloud(rclcpp::Publisher(), cloudKeyPoses6D, timeLaserInfoStamp, odometryFrame);
            //     pcl::PointCloud<PointType>::Ptr localMapOut(new pcl::PointCloud<PointType>());
            //     *localMapOut += *laserCloudSurfFromMapDS;
            //     slamInfo.key_frame_map = publishCloud(rclcpp::Publisher(), localMapOut, timeLaserInfoStamp, odometryFrame);
            //     pubSLAMInfo->publish(slamInfo);
            //     lastSLAMInfoPubSize = cloudKeyPoses6D->size();
            // }
        }
    }

    void globalLocalizeThread()
    {
        while (rclcpp::ok())   
        {
                
            if((initializedFlag == NonInitialized) && (initSCLoopClosure == InitSCLoopClosured))
            {
                ICPLocalizeInitialize();
                // rclcpp::sleep_for(std::chrono::milliseconds(100));
            }

            else if(initializedFlag == Initialized)
            {
                if (globalLocalizedFlag == NoGlobalLocalized)
                {
                    std::cout<<"begin the first globalLocalization " <<std::endl;
                    ICPscanMatchGlobal();
                    rclcpp::sleep_for(std::chrono::milliseconds(100));
                }

                else
                {
                    ICPscanMatchGlobal();
                    rclcpp::sleep_for(std::chrono::milliseconds(2500));
                    
                }
            }
        }
    }

    void ICPLocalizeInitialize()
    {

            pcl::PointCloud<PointType>::Ptr laserCloudIn(new pcl::PointCloud<PointType>());

            mtx_general.lock();
            *laserCloudIn += *cloudScanForInitialize;
            // *laserCloudIn = *cloudScanForInitialize;
            cloudScanForInitialize->clear();
            mtx_general.unlock();
            
            //publishCloud(&fortest_publasercloudINWorld, laserCloudIn, timeLaserInfoStamp, "map");

            if(laserCloudIn->points.size() == 0)
                return;
            
            if(transformInTheWorld[0] == 0 && transformInTheWorld[1] == 0 && transformInTheWorld[2] == 0 && transformInTheWorld[3] == 0 && transformInTheWorld[4] == 0 && transformInTheWorld[5] == 0)
                return;
            // cloudScanForInitialize->clear();
            std::cout << "the size of incoming lasercloud: " << laserCloudIn->points.size() << std::endl;


            //seeting small_gicp
            pcl::PointCloud<pcl::PointCovariance>::Ptr target = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZI>, pcl::PointCloud<pcl::PointCovariance>>(*cloudGlobalMapDS, 0.20);
            pcl::PointCloud<pcl::PointCovariance>::Ptr source = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZI>, pcl::PointCloud<pcl::PointCovariance>>(*laserCloudIn, 0.20);

            // Estimate covariances of points.
            const int num_neighbors = small_num_neighbors;
            small_gicp::estimate_covariances_omp(*target, small_num_neighbors, small_gicp_reduction_num_threads);
            small_gicp::estimate_covariances_omp(*source, small_num_neighbors, small_gicp_reduction_num_threads);

            // Create KdTree for target and source.
            auto target_tree = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(target, small_gicp::KdTreeBuilderOMP(small_gicp_reduction_num_threads));
            auto source_tree = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(source, small_gicp::KdTreeBuilderOMP(small_gicp_reduction_num_threads));
            small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> small_gicp;
            small_gicp.reduction.num_threads = small_gicp_reduction_num_threads;
            small_gicp.rejector.max_dist_sq = 10.0;
            small_gicp.criteria.rotation_eps = small_gicp_criteria_rotation_eps;
            small_gicp.criteria.translation_eps = small_gicp_criteria_translation_eps;
            small_gicp.optimizer.max_iterations = small_gicp_optimizer_max_iterations;

            // seeting ndt_multires
            pcl::NormalDistributionsTransform<PointType, PointType> ndt_multires;
            ndt_multires.setResolution(ndtResolution);           
            ndt_multires.setTransformationEpsilon(ndtTransformationEpsilon);
            ndt_multires.setStepSize(ndtStepSize);       
            ndt_multires.setInputSource(laserCloudIn);
            ndt_multires.setInputTarget(cloudGlobalMapDS);
            pcl::PointCloud<PointType>::Ptr unused_result_1(new pcl::PointCloud<PointType>());

            PointTypePose thisPose6DInWorld = trans2PointTypePose(transformInTheWorld);     // transformInTheWorld由回环提供
            Eigen::Affine3f T_thisPose6DInWorld = pclPointToAffine3f(thisPose6DInWorld);
            ndt_multires.align(*unused_result_1, T_thisPose6DInWorld.matrix());     

            pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());

            Eigen::Isometry3d eigen_isometry;
            eigen_isometry.matrix() = ndt_multires.getFinalTransformation().cast<double>();
            auto small_gicp_result = small_gicp.align(*target, *source, *target_tree, eigen_isometry);
            if (!small_gicp_result.converged || small_gicp_result.error>=small_gicp_result_max_error) 
            {
                std::cout << "Initializing Fail" <<std::endl;
                cout<<"small_gicp_result.converged =="<<small_gicp_result.converged<<" error == "<<small_gicp_result.error <<std::endl;
                initSCLoopClosure = NoinitSCLoopClosured;
                return;
            }
            else 
            {
                RCLCPP_INFO(this->get_logger(), "Initializing succeeded");
                cout<<" error == "<<small_gicp_result.error <<std::endl;
                initializedFlag = Initialized;
                
                PointTypePose thisPose6DInOdom = trans2PointTypePose(transformTobeMapped);      //transformTobeMapped来自里程计
                Eigen::Affine3f T_thisPose6DInOdom = pclPointToAffine3f(thisPose6DInOdom);

                Eigen::Affine3f T_thisPose6DInMap;

                T_thisPose6DInMap = small_gicp_result.T_target_source.cast<float>();
                float x_g, y_g, z_g, R_g, P_g, Y_g;
                pcl::getTranslationAndEulerAngles (T_thisPose6DInMap, x_g, y_g, z_g, R_g, P_g, Y_g);
                transformInTheWorld[0] = R_g;
                transformInTheWorld[1] = P_g;
                transformInTheWorld[2] = Y_g;
                transformInTheWorld[3] = x_g;
                transformInTheWorld[4] = y_g;
                transformInTheWorld[5] = z_g;

                // Eigen::Affine3f transOdomToMap = T_thisPose6DInMap;
                Eigen::Affine3f transOdomToMap = T_thisPose6DInMap * T_thisPose6DInOdom.inverse();     // T_thisPose6DInMap：map->world   T_thisPose6DInOdom：odom->world
                float deltax, deltay, deltaz, deltaR, deltaP, deltaY;
                pcl::getTranslationAndEulerAngles (transOdomToMap, deltax, deltay, deltaz, deltaR, deltaP, deltaY);

                mtxtranformOdomToWorld.lock();
                    //renew tranformOdomToWorld
                tranformOdomToWorld[0] = deltaR;
                tranformOdomToWorld[1] = deltaP;
                tranformOdomToWorld[2] = deltaY;
                tranformOdomToWorld[3] = deltax;
                tranformOdomToWorld[4] = deltay;
                tranformOdomToWorld[5] = deltaz;

                mtxtranformOdomToWorld.unlock();
                
                std::cout << "the pose of odom relative to Map: x" << tranformOdomToWorld[3] << " y" << tranformOdomToWorld[4]
                        << " z" << tranformOdomToWorld[5] <<std::endl;
                // publishCloud(pubLaserCloudInWorld, unused_result, timeLaserInfoStamp, "map");
                // publishCloud(pubMapWorld, cloudGlobalMapDS, timeLaserInfoStamp, "map");

                geometry_msgs::msg::PoseStamped pose_odomTo_map;
                tf2::Quaternion q_odomTo_map;
                q_odomTo_map.setRPY(deltaR, deltaP, deltaY);

                pose_odomTo_map.header.stamp = timeLaserInfoStamp;
                pose_odomTo_map.header.frame_id = "map";
                pose_odomTo_map.pose.position.x = deltax;
                pose_odomTo_map.pose.position.y = deltay;
                pose_odomTo_map.pose.position.z = deltaz;
                pose_odomTo_map.pose.orientation.x = q_odomTo_map.x();
                pose_odomTo_map.pose.orientation.y = q_odomTo_map.y();
                pose_odomTo_map.pose.orientation.z = q_odomTo_map.z();
                pose_odomTo_map.pose.orientation.w = q_odomTo_map.w();
                pubOdomToMapPose->publish(pose_odomTo_map);
            }
        //cloudScanForInitialize.reset(new pcl::PointCloud<PointType>());

    }

    void ICPscanMatchGlobal()
    {

        // if(initializedFlag == NonInitialized)
        // {
        //     ICPLocalizeInitialize();
        //     return;
        // }

        if (cloudKeyPoses3D->points.empty() == true)
            return;

        mtxWin.lock();
        int latestFrameIDGlobalLocalize;
        latestFrameIDGlobalLocalize = win_cloudKeyPoses3D.size() - 1;

        pcl::PointCloud<PointType>::Ptr latestCloudIn(new pcl::PointCloud<PointType>());
        *latestCloudIn += *transformPointCloud(win_cornerCloudKeyFrames[latestFrameIDGlobalLocalize], &win_cloudKeyPoses6D[latestFrameIDGlobalLocalize]);
        *latestCloudIn += *transformPointCloud(win_surfCloudKeyFrames[latestFrameIDGlobalLocalize],   &win_cloudKeyPoses6D[latestFrameIDGlobalLocalize]);
        std::cout << "the size of input cloud: " << latestCloudIn->points.size() << std::endl;

        mtxWin.unlock();

        pcl::NormalDistributionsTransform<PointType, PointType> ndt;      
        ndt.setTransformationEpsilon(ndtTransformationEpsilon);  
        ndt.setResolution(ndtResolution); 

        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(icpMaxCorrespondenceDistance * 0.9);  
        icp.setMaximumIterations(icpMaximumIterations);
        icp.setTransformationEpsilon(icpTransformationEpsilon);  
        icp.setEuclideanFitnessEpsilon(icpEuclideanFitnessEpsilon); 
        icp.setRANSACIterations(0); 

        // Calculating the transformation from odom to world
        mtxtranformOdomToWorld.lock();
        Eigen::Affine3f transodomToWorld_init = pcl::getTransformation(tranformOdomToWorld[3], tranformOdomToWorld[4], tranformOdomToWorld[5], tranformOdomToWorld[0], tranformOdomToWorld[1], tranformOdomToWorld[2]);
        mtxtranformOdomToWorld.unlock();

        Eigen::Matrix4f matricInitGuess = transodomToWorld_init.matrix();

        // Perform NDT in coarse resolution
        ndt.setInputSource(latestCloudIn);  
        ndt.setInputTarget(cloudGlobalMapDS);  
        pcl::PointCloud<PointType>::Ptr unused_result_0(new pcl::PointCloud<PointType>());
        ndt.align(*unused_result_0, matricInitGuess);  

        // Use NDT outcome as the initial guess for ICP
        icp.setInputSource(latestCloudIn);  
        icp.setInputTarget(cloudGlobalMapDS); 
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result, ndt.getFinalTransformation()); 

        std::cout << "ICP converg flag:" << icp.hasConverged() << ". Fitness score: " << icp.getFitnessScore() << std::endl << std::endl;

        Eigen::Affine3f transodomToWorld_New;
        transodomToWorld_New = icp.getFinalTransformation();  
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transodomToWorld_New, x, y, z, roll, pitch, yaw);  

        mtxtranformOdomToWorld.lock();
        tranformOdomToWorld[0] = roll;
        tranformOdomToWorld[1] = pitch;
        tranformOdomToWorld[2] = yaw;
        tranformOdomToWorld[3] = x;
        tranformOdomToWorld[4] = y;
        tranformOdomToWorld[5] = z;
        mtxtranformOdomToWorld.unlock();

        publishCloud(pubMapWorld, cloudGlobalMapDS, timeLaserInfoStamp, "map");

        if (icp.hasConverged() == true && icp.getFitnessScore() < historyKeyframeFitnessScore)
        {
            std::cout<<" globalLocalization succeeded! " << std::endl;
            globalLocalizedFlag = GlobalLocalized;

            // Publish the pose from odom to map frame
            geometry_msgs::msg::PoseStamped pose_odomTo_map;
            tf2::Quaternion q_odomTo_map;
            q_odomTo_map.setRPY(roll, pitch, yaw);  
            geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(q_odomTo_map);

            pose_odomTo_map.header.stamp = timeLaserInfoStamp;  
            pose_odomTo_map.header.frame_id = "map";  
            pose_odomTo_map.pose.position.x = x;
            pose_odomTo_map.pose.position.y = y;
            pose_odomTo_map.pose.position.z = z;
            pose_odomTo_map.pose.orientation.x = q_odomTo_map.x();
            pose_odomTo_map.pose.orientation.y = q_odomTo_map.y();
            pose_odomTo_map.pose.orientation.z = q_odomTo_map.z();
            pose_odomTo_map.pose.orientation.w = q_odomTo_map.w();
            pubOdomToMapPose->publish(pose_odomTo_map); 
        }
    }


    void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_msg)
    {
        if(initializedFlag == Initialized)
            return;

        float x = pose_msg->pose.pose.position.x;
        float y = pose_msg->pose.pose.position.y;
        float z = pose_msg->pose.pose.position.z;

        tf2::Quaternion q_global;
        double roll_global, pitch_global, yaw_global;
        
        q_global.setX(pose_msg->pose.pose.orientation.x);
        q_global.setY(pose_msg->pose.pose.orientation.y);
        q_global.setZ(pose_msg->pose.pose.orientation.z);
        q_global.setW(pose_msg->pose.pose.orientation.w);
        
        tf2::Matrix3x3(q_global).getRPY(roll_global, pitch_global, yaw_global);

        transformInTheWorld[0] = roll_global;
        transformInTheWorld[1] = pitch_global;
        transformInTheWorld[2] = yaw_global;
        transformInTheWorld[3] = x;
        transformInTheWorld[4] = y;
        transformInTheWorld[5] = z;

        PointTypePose thisPose6DInWorld = trans2PointTypePose(transformInTheWorld);
        Eigen::Affine3f T_thisPose6DInWorld = pclPointToAffine3f(thisPose6DInWorld);
        
        PointTypePose thisPose6DInOdom = trans2PointTypePose(transformTobeMapped);
        Eigen::Affine3f T_thisPose6DInOdom = pclPointToAffine3f(thisPose6DInOdom);

        Eigen::Affine3f T_OdomToMap = T_thisPose6DInWorld * T_thisPose6DInOdom.inverse();
        
        float delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw;
        pcl::getTranslationAndEulerAngles(T_OdomToMap, delta_x, delta_y, delta_z, 
                                        delta_roll, delta_pitch, delta_yaw);


        mtxtranformOdomToWorld.lock();
        tranformOdomToWorld[0] = delta_roll;
        tranformOdomToWorld[1] = delta_pitch;
        tranformOdomToWorld[2] = delta_yaw;
        tranformOdomToWorld[3] = delta_x;
        tranformOdomToWorld[4] = delta_y;
        tranformOdomToWorld[5] = delta_z;
        mtxtranformOdomToWorld.unlock(); 

        initializedFlag = NonInitialized;
    }

};



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::SingleThreadedExecutor exec;

    auto MO = std::make_shared<mapOptimization>(options);
    exec.add_node(MO);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Map Optimization Started.\033[0m");

    // std::thread loopthread(&mapOptimization::loopClosureThread, MO);
    // std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, MO);
    std::thread localizeInWorldThread(&mapOptimization::globalLocalizeThread, MO);

    exec.spin();

    rclcpp::shutdown();

    // loopthread.join();
    // visualizeMapThread.join();
    localizeInWorldThread.join();

    return 0;
}



