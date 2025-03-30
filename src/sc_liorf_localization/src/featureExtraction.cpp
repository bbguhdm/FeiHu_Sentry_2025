#include "utility.h"
#include "sc_liorf_localization/msg/cloud_info.hpp"

struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

class FeatureExtraction : public ParamServer
{

public:

    rclcpp::Subscription<sc_liorf_localization::msg::CloudInfo>::SharedPtr subLaserCloudInfo;

    rclcpp::Publisher<sc_liorf_localization::msg::CloudInfo>::SharedPtr pubLaserCloudInfo;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCornerPoints;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfacePoints;

    pcl::PointCloud<PointType>::Ptr extractedCloud;
    pcl::PointCloud<PointType>::Ptr cornerCloud;
    pcl::PointCloud<PointType>::Ptr surfaceCloud;

    pcl::VoxelGrid<PointType> downSizeFilter;

    sc_liorf_localization::msg::CloudInfo cloudInfo;
    std_msgs::msg::Header cloudHeader;

    std::vector<smoothness_t> cloudSmoothness;
    float *cloudCurvature;
    int *cloudNeighborPicked;
    int *cloudLabel;

    FeatureExtraction(const rclcpp::NodeOptions & options) :
        ParamServer("sc_liorf_localization_featureExtraction", options)
    {
        // subLaserCloudInfo = create_subscription<sc_liorf_localization::msg::CloudInfo>(
        //     "sc_liorf_localization/deskew/cloud_info", QosPolicy(history_policy, reliability_policy),
        //     std::bind(&FeatureExtraction::laserCloudInfoHandler, this, std::placeholders::_1));

        // pubLaserCloudInfo = create_publisher<sc_liorf_localization::msg::CloudInfo>(
        //     "sc_liorf_localization/feature/cloud_info", QosPolicy(history_policy, reliability_policy));
        // pubCornerPoints = create_publisher<sensor_msgs::msg::PointCloud2>(
        //     "sc_liorf_localization/feature/cloud_corner", QosPolicy(history_policy, reliability_policy));
        // pubSurfacePoints = create_publisher<sensor_msgs::msg::PointCloud2>(
        //     "sc_liorf_localization/feature/cloud_surface", QosPolicy(history_policy, reliability_policy));

        subLaserCloudInfo = create_subscription<sc_liorf_localization::msg::CloudInfo>(
            "sc_liorf_localization/deskew/cloud_info", qos,
            std::bind(&FeatureExtraction::laserCloudInfoHandler, this, std::placeholders::_1));

        pubLaserCloudInfo = create_publisher<sc_liorf_localization::msg::CloudInfo>(
            "sc_liorf_localization/feature/cloud_info", qos_lidar);
        pubCornerPoints = create_publisher<sensor_msgs::msg::PointCloud2>(
            "sc_liorf_localization/feature/cloud_corner", 1);
        pubSurfacePoints = create_publisher<sensor_msgs::msg::PointCloud2>(
            "sc_liorf_localization/feature/cloud_surface", 1);

        initializationValue();
    }

    void initializationValue()
    {
        cloudSmoothness.resize(N_SCAN*Horizon_SCAN);

        downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

        extractedCloud.reset(new pcl::PointCloud<PointType>());
        cornerCloud.reset(new pcl::PointCloud<PointType>());
        surfaceCloud.reset(new pcl::PointCloud<PointType>());

        cloudCurvature = new float[N_SCAN*Horizon_SCAN];
        cloudNeighborPicked = new int[N_SCAN*Horizon_SCAN];
        cloudLabel = new int[N_SCAN*Horizon_SCAN];
    }

    void laserCloudInfoHandler(const sc_liorf_localization::msg::CloudInfo::SharedPtr msgIn)
    {

        cloudInfo = *msgIn; // new cloud info
        cloudHeader = msgIn->header; // new cloud header
        pcl::fromROSMsg(msgIn->cloud_deskewed, *extractedCloud); // new cloud for extraction

        calculateSmoothness();

        markOccludedPoints();

        extractFeatures();

        publishFeatureCloud();

    }

    // 计算点云中各点的平滑度（曲率），用于后续特征提取
    void calculateSmoothness()
    {
        // 获取当前提取点云的总点数
        int cloudSize = extractedCloud->points.size();
        

        if (sensor == SensorType::LIVOX)
        {
            for (int i = 5; i < cloudSize - 5; i++)
            {
                // float diffRange = cloudInfo.point_range[i-5] + cloudInfo.point_range[i-4]
                //                 + cloudInfo.point_range[i-3] + cloudInfo.point_range[i-2]
                //                 + cloudInfo.point_range[i-1] - cloudInfo.point_range[i] * 10
                //                 + cloudInfo.point_range[i+1] + cloudInfo.point_range[i+2]
                //                 + cloudInfo.point_range[i+3] + cloudInfo.point_range[i+4]
                //                 + cloudInfo.point_range[i+5];

                float diffRange = 
                                cloudInfo.point_range[i-2]  + cloudInfo.point_range[i-1] - cloudInfo.point_range[i] * 4
                                + cloudInfo.point_range[i+1] + cloudInfo.point_range[i+2];    

                cloudCurvature[i] = diffRange*diffRange;//diffX * diffX + diffY * diffY + diffZ * diffZ;

                cloudNeighborPicked[i] = 0;
                cloudLabel[i] = 0;
                // cloudSmoothness for sorting
                cloudSmoothness[i].value = cloudCurvature[i];
                cloudSmoothness[i].ind = i;
            }

        }
        else if (sensor == SensorType::VELODYNE)
        {
            // 遍历点云，跳过前5和后5个点以避免越界
            for (int i = 5; i < cloudSize - 5; i++)
            {
                // 计算当前点与周围10个点的距离差异总和
                // 公式：前5点 + 后5点的距离之和 - 当前点距离 × 10
                // 相当于当前点与周围点平均值的差异的10倍
                float diffRange = cloudInfo.point_range[i-5] + cloudInfo.point_range[i-4]
                                + cloudInfo.point_range[i-3] + cloudInfo.point_range[i-2]
                                + cloudInfo.point_range[i-1] - cloudInfo.point_range[i] * 10
                                + cloudInfo.point_range[i+1] + cloudInfo.point_range[i+2]
                                + cloudInfo.point_range[i+3] + cloudInfo.point_range[i+4]
                                + cloudInfo.point_range[i+5];

                // 曲率计算：差异平方作为曲率估计（类似方差计算）
                // 曲率越大表示该点周围变化越剧烈（边缘特征）
                cloudCurvature[i] = diffRange * diffRange;

                // 初始化标记：0表示该点尚未被选为特征点
                cloudNeighborPicked[i] = 0;
                // 初始化特征标签：0-未分类，后续可能标记为平面/边缘特征
                cloudLabel[i] = 0;
                
                // 将曲率值和索引存入结构体，用于后续按曲率排序
                cloudSmoothness[i].value = cloudCurvature[i];
                cloudSmoothness[i].ind = i; // 保存原始索引以便后续映射
        }

        }
    }

    // 标记被遮挡点和平行光束干扰点，避免在特征提取中使用这些不可靠的点
    void markOccludedPoints()
    {
        int cloudSize = extractedCloud->points.size();
        
        if (sensor == SensorType::LIVOX)
        {
            for (int i = 5; i < cloudSize - 6; ++i)
            {
                // occluded points
                float depth1 = cloudInfo.point_range[i];
                float depth2 = cloudInfo.point_range[i+1];
                int columnDiff = std::abs(int(cloudInfo.point_col_ind[i+1] - cloudInfo.point_col_ind[i]));
                if (columnDiff < 10){
                    // 10 pixel diff in range image
                    if (depth1 - depth2 > 0.3){
                        // cloudNeighborPicked[i - 5] = 1;
                        // cloudNeighborPicked[i - 4] = 1;
                        // cloudNeighborPicked[i - 3] = 1;
                        // cloudNeighborPicked[i - 2] = 1;
                        cloudNeighborPicked[i - 1] = 1;
                        cloudNeighborPicked[i] = 1;
                    }else if (depth2 - depth1 > 0.3){
                        cloudNeighborPicked[i + 1] = 1;
                        cloudNeighborPicked[i + 2] = 1;
                        // cloudNeighborPicked[i + 3] = 1;
                        // cloudNeighborPicked[i + 4] = 1;
                        // cloudNeighborPicked[i + 5] = 1;
                        // cloudNeighborPicked[i + 6] = 1;
                    }
                }
                // parallel beam
                float diff1 = std::abs(float(cloudInfo.point_range[i-1] - cloudInfo.point_range[i]));
                float diff2 = std::abs(float(cloudInfo.point_range[i+1] - cloudInfo.point_range[i]));

                if (diff1 > 0.1 * cloudInfo.point_range[i] && diff2 > 0.1 * cloudInfo.point_range[i])
                    cloudNeighborPicked[i] = 1;
            }
        }

        else if (sensor == SensorType::VELODYNE)
        {
            // 遍历点云，前后各保留足够点数用于邻域比较（索引范围5到cloudSize-6）
            for (int i = 5; i < cloudSize - 6; ++i)
            {
                /* 第一部分：处理被遮挡点 --------------------------------*/
                float depth1 = cloudInfo.point_range[i];     // 当前点的深度值
                float depth2 = cloudInfo.point_range[i+1];   // 下一个相邻点的深度值
                // 计算在距离图像（range image）中的列索引差（同一扫描线相邻点）
                int columnDiff = std::abs(int(cloudInfo.point_col_ind[i+1] - cloudInfo.point_col_ind[i]));

                // 当相邻点在距离图像中位于相近列时（同一扫描线附近）
                if (columnDiff < 10){
                    // 情况1：当前点明显比后点近（后点被当前点遮挡）
                    if (depth1 - depth2 > 0.3){  // 深度差阈值0.3m
                        // 标记当前点及前5个点为遮挡区域（避免选择边缘附近点）
                        cloudNeighborPicked[i - 5] = 1;  // 向前标记5个点
                        cloudNeighborPicked[i - 4] = 1;
                        cloudNeighborPicked[i - 3] = 1;
                        cloudNeighborPicked[i - 2] = 1;
                        cloudNeighborPicked[i - 1] = 1;
                        cloudNeighborPicked[i] = 1;
                    }
                    // 情况2：后点明显比当前点近（当前点被后续点遮挡）
                    else if (depth2 - depth1 > 0.3){
                        // 标记后6个点为遮挡区域
                        cloudNeighborPicked[i + 1] = 1;  // 向后标记6个点
                        cloudNeighborPicked[i + 2] = 1;
                        cloudNeighborPicked[i + 3] = 1;
                        cloudNeighborPicked[i + 4] = 1;
                        cloudNeighborPicked[i + 5] = 1;
                        cloudNeighborPicked[i + 6] = 1;
                    }
                }

                /* 第二部分：处理平行光束干扰 ----------------------------*/
                // 计算当前点与前后相邻点的深度差绝对值
                float diff1 = std::abs(cloudInfo.point_range[i-1] - cloudInfo.point_range[i]);
                float diff2 = std::abs(cloudInfo.point_range[i+1] - cloudInfo.point_range[i]);

                // 如果前后深度差均超过当前点深度的2%，判定为平行光束干扰
                //（激光束与物体表面近似平行时，测量结果不可靠）
                if (diff1 > 0.02 * cloudInfo.point_range[i] && 
                    diff2 > 0.02 * cloudInfo.point_range[i]){
                    cloudNeighborPicked[i] = 1;  // 直接标记当前点为不可用
                }
            }
        }

    }

    // 特征提取函数：从原始点云中提取角点（边缘）和平面点特征
    void extractFeatures()
    {
        // 清空角点和平面点云，准备存储新提取的特征
        cornerCloud->clear();
        surfaceCloud->clear();

        // 声明局部点云变量，用于处理单个扫描线的平面点（原始和降采样后）
        pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

        // 遍历所有激光扫描线（例如16线或32线激光雷达）
        for (int i = 0; i < N_SCAN; i++)
        {

            surfaceCloudScan->clear(); // 清空当前扫描线的平面点缓存

            // 将每条扫描线分为6个子区域，分别处理以均匀提取特征
            for (int j = 0; j < 6; j++)
            {
                // 计算当前子区域的起始(sp)和结束(ep)索引
                // 通过线性插值将扫描线分成6段，确保特征均匀分布
                int sp = (cloudInfo.start_ring_index[i] * (6 - j) + cloudInfo.end_ring_index[i] * j) / 6;
                int ep = (cloudInfo.start_ring_index[i] * (5 - j) + cloudInfo.end_ring_index[i] * (j + 1)) / 6 - 1;

                if (sp >= ep)
                    continue; // 跳过无效区间

                // 按曲率值从低到高排序当前子区域的点（后续反向遍历选高曲率点）
                std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());

                // 角点提取：从后向前遍历排序后的点（曲率从高到低）
                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--)
                {
                    int ind = cloudSmoothness[k].ind; // 获取原始点云索引
                    
                    // 检查该点是否未被选取且曲率超过角点阈值
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold)
                    {
                        largestPickedNum++;
                        if (largestPickedNum <= 20) { // 最多选20个角点，避免过密集
                            cloudLabel[ind] = 1;      // 标记为角点
                            cornerCloud->push_back(extractedCloud->points[ind]);
                        } else {
                            break; // 达到数量限制后跳出
                        }

                        cloudNeighborPicked[ind] = 1; // 标记该点已被选取
                        
                        // 向前标记相邻点（防止后续重复选取邻近点）
                        for (int l = 1; l <= 4; l++) {
                            // 检查列号差异，超过10说明跨列（可能不同物体）
                            int columnDiff = std::abs(int(cloudInfo.point_col_ind[ind + l] - cloudInfo.point_col_ind[ind + l - 1]));
                            if (columnDiff > 10) break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        // 向后标记相邻点
                        for (int l = -1; l >= -4; l--) {
                            int columnDiff = std::abs(int(cloudInfo.point_col_ind[ind + l] - cloudInfo.point_col_ind[ind + l + 1]));
                            if (columnDiff > 10) break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }
                // 平面点提取：从前往后遍历，选择低曲率点
                for (int k = sp; k <= ep; k++)
                {
                    int ind = cloudSmoothness[k].ind;
                    // 检查未被选取且曲率低于平面阈值
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold)
                    {
                        cloudLabel[ind] = -1;      // 标记为平面点
                        cloudNeighborPicked[ind] = 1;

                        // 同样标记相邻点防止重复选取
                        for (int l = 1; l <= 4; l++) {
                            int columnDiff = std::abs(int(cloudInfo.point_col_ind[ind + l] - cloudInfo.point_col_ind[ind + l - 1]));
                            if (columnDiff > 10) break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -4; l--) {
                            int columnDiff = std::abs(int(cloudInfo.point_col_ind[ind + l] - cloudInfo.point_col_ind[ind + l + 1]));
                            if (columnDiff > 10) break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }
                // 收集未被标记为角点的点作为候选平面点（包括平面点和未分类点）
                for (int k = sp; k <= ep; k++)
                {
                    if (cloudLabel[k] <= 0) { // label<=0包含平面点(-1)和未分类点(0)
                        surfaceCloudScan->push_back(extractedCloud->points[k]);
                    }
                }
            }

            // 对当前扫描线的平面点进行降采样（减少数据量）
            surfaceCloudScanDS->clear();
            downSizeFilter.setInputCloud(surfaceCloudScan);
            downSizeFilter.filter(*surfaceCloudScanDS); // 使用体素滤波或类似方法

            *surfaceCloud += *surfaceCloudScanDS; // 将降采样后的点云合并到总平面点云
        }
    }


    void freeCloudInfoMemory()
    {
        cloudInfo.start_ring_index.clear();
        cloudInfo.end_ring_index.clear();
        cloudInfo.point_col_ind.clear();
        cloudInfo.point_range.clear();
    }

    void publishFeatureCloud()
    {
        // free cloud info memory
        freeCloudInfoMemory();
        // save newly extracted features
        cloudInfo.cloud_corner = publishCloud(pubCornerPoints,  cornerCloud,  cloudHeader.stamp, lidarFrame);
        cloudInfo.cloud_surface = publishCloud(pubSurfacePoints, surfaceCloud, cloudHeader.stamp, lidarFrame);
        // publish to mapOptimization
        pubLaserCloudInfo->publish(cloudInfo);
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::SingleThreadedExecutor exec;

    auto FE = std::make_shared<FeatureExtraction>(options);

    exec.add_node(FE);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Feature Extraction Started.\033[0m");

    exec.spin();

    rclcpp::shutdown();
    return 0;
}









