#include "utils/common.h"
#include "utils/math_tools.h"
#include "utils/timer.h"
#include "factors/LidarKeyframeFactor.h"

class LidarOdometry {
private:
    int odom_pub_cnt = 0;
    ros::NodeHandle nh;

    ros::Subscriber sub_edge;
    ros::Subscriber sub_surf;
    ros::Subscriber sub_full_cloud;

    ros::Publisher pub_edge;
    ros::Publisher pub_surf;
    ros::Publisher pub_full_cloud, pub_full_cloud_map;
    ros::Publisher pub_odom;
    ros::Publisher pub_each_odom;
    ros::Publisher pub_path;

    std_msgs::Header cloud_header; // 点云消息头
    nav_msgs::Odometry odom; // 里程计消息
    nav_msgs::Path path; // 路径消息

    // // 点云指针
    pcl::PointCloud<PointType>::Ptr edge_features;
    pcl::PointCloud<PointType>::Ptr surf_features;
    pcl::PointCloud<PointType>::Ptr full_cloud;

    pcl::PointCloud<PointType>::Ptr surf_last_ds;

    // k-D 树指针，用于最近邻搜索
    pcl::KdTreeFLANN<PointType >::Ptr kd_tree_surf_last;

    // pose representation: [quaternion: w, x, y, z | transition: x, y, z]
    double abs_pose[7];   //absolute pose from current frame to the first  从当前帧到第一帧的绝对位姿
    double rel_pose[7];   //relative pose between two frames 两帧之间的相对位姿

    bool new_edge = false;
    bool new_surf = false;
    bool new_full_cloud = false;

    double time_new_surf = 0;
    double time_new_full_points = 0;
    double time_new_edge = 0;

    bool system_initialized;  // 系统初始化标志

    int surf_res_cnt;  // 平面点特征计数器

    int max_num_iter;  // 最大迭代次数
    int scan_match_cnt; // 扫描匹配计数器

    pcl::PointCloud<PointPoseInfo>::Ptr pose_info_cloud_frame; //pose of each frame  每帧的位姿信息 //revise
    pcl::PointCloud<PointXYZI>::Ptr pose_cloud_frame; //position of each frame 每帧的位置

    deque<pcl::PointCloud<PointType>::Ptr> surf_frames; // 存储平面点云帧

    deque<pcl::PointCloud<PointType>::Ptr> recent_surf_frames;  // 存储最新的平面点云帧

    pcl::PointCloud<PointType>::Ptr surf_from_map;     //surf_from_map 包含原始的高密度点云数据，用于生成高精度的局部地图
    pcl::PointCloud<PointType>::Ptr surf_from_map_ds;  // surf_from_map_ds 是降采样后的低密度点云数据，用于加速匹配和优化过程。

    pcl::PointCloud<PointType>::Ptr surf_current_pts;
    pcl::PointCloud<PointType>::Ptr surf_normal;

    pcl::VoxelGrid<PointType> down_size_filter_surf;  // 用于降采样
    pcl::VoxelGrid<PointType> down_size_filter_surf_map;  // 用于地图的降采样

    int latest_frame_idx;  // 最近帧的索引

    bool kf = true; // 关键帧标志
    int kf_num = 0; // 关键帧数目

    Eigen::Vector3d trans_last_kf = Eigen::Vector3d::Zero(); // 上一关键帧的平移
    Eigen::Quaterniond quat_last_kF = Eigen::Quaterniond::Identity(); // 上一关键帧的旋转

    string frame_id = "GLIO";
    bool if_to_deskew; // 是否去畸变标志
    double runtime = 0; // 运行时间

public:
    LidarOdometry(): nh("~") {
        initializeParameters();  // 初始化参数
        allocateMemory(); // 分配内存

        // 订阅点云话题 revise
        sub_edge = nh.subscribe<sensor_msgs::PointCloud2>("/edge_features", 100, &LidarOdometry::laserCloudLessSharpHandler, this);
        sub_surf = nh.subscribe<sensor_msgs::PointCloud2>("/surf_features", 100, &LidarOdometry::laserCloudLessFlatHandler, this);
        sub_full_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/lidar_cloud_cutted", 100, &LidarOdometry::FullPointCloudHandler, this);

        // 订阅点云话题
        pub_edge = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100);
        pub_surf = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100);
        pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 100);
        pub_each_odom = nh.advertise<nav_msgs::Odometry>("/each_odom", 100);
        pub_path = nh.advertise<nav_msgs::Path>("/path", 100);
        pub_full_cloud = nh.advertise<sensor_msgs::PointCloud2>("/full_point_cloud", 100);
        pub_full_cloud_map = nh.advertise<sensor_msgs::PointCloud2>("/full_point_cloud_map", 100);

    }

    ~LidarOdometry(){}


    //allocateMemory()这个函数用于分配各种点云指针的内存。
    void allocateMemory() {
        edge_features.reset(new pcl::PointCloud<PointType>());
        surf_features.reset(new pcl::PointCloud<PointType>());
        full_cloud.reset(new pcl::PointCloud<PointType>());

        kd_tree_surf_last.reset(new pcl::KdTreeFLANN<PointType>());

        pose_info_cloud_frame.reset(new pcl::PointCloud<PointPoseInfo>());
        pose_cloud_frame.reset(new pcl::PointCloud<PointXYZI>());

        surf_from_map.reset(new pcl::PointCloud<PointType>());
        surf_from_map_ds.reset(new pcl::PointCloud<PointType>());
        surf_last_ds.reset(new pcl::PointCloud<PointType>());

        surf_current_pts.reset(new pcl::PointCloud<PointType>());
        surf_normal.reset(new pcl::PointCloud<PointType>());
    }

    //用于初始化参数，如果参数没有在配置文件中设置，则使用默认值。revise
    void initializeParameters() {
        // Load parameters from yaml
        if (!getParameter("/common/frame_id", frame_id)) {
            ROS_WARN("frame_id not set, use default value: lili_om");
            frame_id = "lili_om";
        }

        if (!getParameter("/lidar_odometry/if_to_deskew", if_to_deskew)) {
            ROS_WARN("if_to_deskew not set, use default value: true");
            if_to_deskew = true;
        }

        if (!getParameter("/lidar_odometry/max_num_iter", max_num_iter)) {
            ROS_WARN("maximal iteration number not set, use default value: 50");
            max_num_iter = 15;
        }

        if (!getParameter("/lidar_odometry/scan_match_cnt", scan_match_cnt)) {
            ROS_WARN("number of scan matching not set, use default value: 1");
            scan_match_cnt = 1;
        }

        latest_frame_idx = 0;

        odom.header.frame_id = frame_id;

        system_initialized = false;

        abs_pose[0] = 1;
        rel_pose[0] = 1;

        for (int i = 1; i < 7; ++i) {
            abs_pose[i] = 0;
            rel_pose[i] = 0;
        }
        surf_res_cnt = 0;

        down_size_filter_surf.setLeafSize(0.4, 0.4, 0.4);
        down_size_filter_surf_map.setLeafSize(0.4, 0.4, 0.4);
    }

    //处理点云中的边缘特征（角点），保存处理后的点云数据和时间戳。
    void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &pointCloudIn) {
        time_new_edge = pointCloudIn->header.stamp.toSec(); // 保存时间戳
        pcl::fromROSMsg(*pointCloudIn, *edge_features); // 将ROS消息转换为PCL点云
        new_edge = true; // 标记为新的边缘特征点云
    }

    //处理点云中的平面特征（平面点），保存处理后的点云数据和时间戳。
    void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &pointCloudIn) {
        time_new_surf = pointCloudIn->header.stamp.toSec();
        pcl::fromROSMsg(*pointCloudIn, *surf_features);
        new_surf = true; // 标记为新的平面特征点云
    }

    //处理完整的点云数据，保存处理后的点云数据和时间戳。
    void FullPointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &pointCloudIn) {
        time_new_full_points = pointCloudIn->header.stamp.toSec();
        cloud_header = pointCloudIn->header; //// 保存点云消息头
        pcl::fromROSMsg(*pointCloudIn, *full_cloud);
        new_full_cloud = true;
    }

    //对点云数据进行去畸变处理，根据时间和位姿插值计算每个点的实际位置。 revise
    void undistortion(const pcl::PointCloud<PointType>::Ptr &pcloud, const Eigen::Vector3d trans, const Eigen::Quaterniond quat) {
        double dt = 0.1;
        for (auto &pt : pcloud->points) {
            int line = int(pt.intensity);  // 获取点所属的扫描线
            double dt_i = pt.intensity - line;  //计算点的时间偏移
            double ratio_i = dt_i / dt;  // 计算插值比例

            if(ratio_i > 1)   // 如果比例大于1，则设置为1
                ratio_i = 1;

            Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
            Eigen::Quaterniond q_si = q0.slerp(ratio_i, quat);  // 根据比例插值计算四元数

            Eigen::Vector3d t_si = ratio_i * trans;  // 根据比例插值计算平移
            Eigen::Vector3d pt_i(pt.x, pt.y, pt.z);
            Eigen::Vector3d pt_s = q_si * pt_i + t_si;  // 计算去畸变后的点位置

            pt.x = pt_s.x();
            pt.y = pt_s.y();
            pt.z = pt_s.z();
        }
    }

    //用于检查系统的初始化状态，并发布点云数据。
    void checkInitialization() {
        // 将边缘特征点云转换为ROS消息并发布
        sensor_msgs::PointCloud2 msgs;
        pcl::toROSMsg(*edge_features, msgs);
        msgs.header.stamp = cloud_header.stamp;
        msgs.header.frame_id = frame_id;
        pub_edge.publish(msgs);

        // 将平面特征点云转换为ROS消息并发布
        pcl::toROSMsg(*surf_features, msgs);
        msgs.header.stamp = cloud_header.stamp;
        msgs.header.frame_id = frame_id;
        pub_surf.publish(msgs);

        // 将完整点云转换为ROS消息并发布
        pcl::toROSMsg(*full_cloud, msgs);
        msgs.header.stamp = cloud_header.stamp;
        msgs.header.frame_id = frame_id;
        pub_full_cloud.publish(msgs);

        // 标记系统已初始化
        system_initialized = true;
    }

    //该函数用于将一个点从局部坐标系转换到全局坐标系 revise
    void transformPoint(PointType const *const pi, PointType *const po) {

        // 构建四元数和位移向量
        Eigen::Quaterniond quaternion(abs_pose[0],
                abs_pose[1],
                abs_pose[2],
                abs_pose[3]);
        Eigen::Vector3d transition(abs_pose[4],
                abs_pose[5],
                abs_pose[6]);

        // 将输入点从局部坐标系转换到全局坐标系
        Eigen::Vector3d ptIn(pi->x, pi->y, pi->z);
        Eigen::Vector3d ptOut = quaternion * ptIn + transition;


        Eigen::Vector3d normIn(pi->normal_x, pi->normal_y, pi->normal_z);
        Eigen::Vector3d normOut = quaternion * normIn;

        // 设置输出点的坐标和强度
        po->x = ptOut.x();
        po->y = ptOut.y();
        po->z = ptOut.z();
        po->intensity = pi->intensity;
        po->curvature = pi->curvature;
        po->normal_x = normOut.x();
        po->normal_y = normOut.y();
        po->normal_z = normOut.z();
    }

    //该函数用于将一组点（点云）从局部坐标系转换到全局坐标系。revise
    pcl::PointCloud<PointType>::Ptr transformCloud(const pcl::PointCloud<PointType>::Ptr &cloudIn, PointPoseInfo * PointInfoIn) {
        
        // 创建输出点云revise
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        // 获取四元数和位移向量
        Eigen::Quaterniond quaternion(PointInfoIn->qw,
                                      PointInfoIn->qx,
                                      PointInfoIn->qy,
                                      PointInfoIn->qz);
        Eigen::Vector3d transition(PointInfoIn->x,
                                   PointInfoIn->y,
                                   PointInfoIn->z);

        // 调整点云大小
        int numPts = cloudIn->points.size();
        cloudOut->resize(numPts);

        // 将每个点从局部坐标系转换到全局坐标系
        for (int i = 0; i < numPts; ++i)
        {
            Eigen::Vector3d ptIn(cloudIn->points[i].x, cloudIn->points[i].y, cloudIn->points[i].z);
            Eigen::Vector3d ptOut = quaternion * ptIn + transition;


            Eigen::Vector3d normIn(cloudIn->points[i].normal_x, cloudIn->points[i].normal_y, cloudIn->points[i].normal_z);
            Eigen::Vector3d normOut = quaternion * normIn;

            // 设置输出点的坐标和强度
            PointType pt;
            pt.x = ptOut.x();
            pt.y = ptOut.y();
            pt.z = ptOut.z();
            pt.intensity = cloudIn->points[i].intensity;
            pt.curvature = cloudIn->points[i].curvature;
            pt.normal_x = normOut.x();
            pt.normal_y = normOut.y();
            pt.normal_z = normOut.z();
            cloudOut->points[i] = pt;
        }
        return cloudOut;
    }

    //构建本地地图，整合最近的平面特征帧到当前的平面特征点云中。
    void buildLocalMap() {
        surf_from_map->clear();
        // Initialization
        // 如果是初始状态，仅有一个位姿帧
        if (pose_cloud_frame->points.size() <= 1) {
            //ROS_INFO("Initialization for odometry local map");
            *surf_from_map += *surf_features;
            return;
        }

        // If already more then 20 frames, pop the frames at the beginning
        // 如果最近的平面特征帧数少于20，则继续添加新的帧
        if (recent_surf_frames.size() < 20) {
            int i = pose_cloud_frame->points.size() - 1;
            recent_surf_frames.push_back(transformCloud(surf_frames.back(), &pose_info_cloud_frame->points[i]));

        } else {
            // 否则，移除最早的一帧并添加最新的一帧
            if (latest_frame_idx != pose_cloud_frame->points.size() - 1) {
                recent_surf_frames.pop_front();
                latest_frame_idx = pose_cloud_frame->points.size() - 1;
                recent_surf_frames.push_back(transformCloud(surf_frames.back(), &pose_info_cloud_frame->points[latest_frame_idx]));
            }
        }

        // 将最近的所有平面特征帧整合到当前的平面特征点云中
        for (int i = 0; i < recent_surf_frames.size(); ++i)
            *surf_from_map += *recent_surf_frames[i];
    }

    //用于清理点云数据
    void clearCloud() {
        surf_from_map->clear();
        surf_from_map_ds->clear();
        edge_features->clear();
        surf_features->clear();
        full_cloud->clear();
        // 保持surf_frames队列最多有20个帧
        while(surf_frames.size() > 20) {
            surf_frames.pop_front();
        }

    }

    //该函数用于对点云进行降采样
    void downSampleCloud() {

        // 对地图中的平面特征点云进行降采样
        down_size_filter_surf_map.setInputCloud(surf_from_map);
        down_size_filter_surf_map.filter(*surf_from_map_ds);

        surf_last_ds->clear();

        // 对当前帧的平面特征点云进行降采样
        down_size_filter_surf.setInputCloud(surf_features);
        down_size_filter_surf.filter(*surf_last_ds);
    }

    //该函数用于保存位姿信息和当前帧的点云数据
    void savePoses() {
        PointXYZI tmpPose;
        // 保存当前帧的绝对位姿信息到pose_cloud_frame
        tmpPose.x = abs_pose[4];
        tmpPose.y = abs_pose[5];
        tmpPose.z = abs_pose[6];
        tmpPose.intensity = pose_cloud_frame->points.size();
        pose_cloud_frame->push_back(tmpPose);

        // 保存当前帧的详细位姿信息到pose_info_cloud_frame
        PointPoseInfo tmpPoseInfo;
        tmpPoseInfo.x = abs_pose[4];
        tmpPoseInfo.y = abs_pose[5];
        tmpPoseInfo.z = abs_pose[6];
        tmpPoseInfo.qw = abs_pose[0];
        tmpPoseInfo.qx = abs_pose[1];
        tmpPoseInfo.qy = abs_pose[2];
        tmpPoseInfo.qz = abs_pose[3];
        tmpPoseInfo.idx = pose_cloud_frame->points.size();
        tmpPoseInfo.time = time_new_surf;
        pose_info_cloud_frame->push_back(tmpPoseInfo);

        // 将当前帧的降采样后的平面特征点云保存到surf_frames
        pcl::PointCloud<PointType>::Ptr surfEachFrame(new pcl::PointCloud<PointType>());

        pcl::copyPointCloud(*surf_last_ds, *surfEachFrame);

        surf_frames.push_back(surfEachFrame);
    }

    //该函数用于找到当前帧中的平面特征点在地图中的对应点，以便进行后续的位姿优化 revise scan to map
    /*************
    1.将当前帧的点转换到全局坐标系下：使用之前估计的位姿（abs_pose），将当前帧中的点转换到全局坐标系下。
    2.寻找最近的点：使用KD树（kd_tree_surf_last），在地图中找到当前点最近的5个点。
    3.拟合平面：使用QR分解法，通过这5个点拟合出一个平面，并计算这个平面的法向量和质心。
    4.验证平面有效性：检查拟合的平面是否有效，即这5个点是否足够平面。
    5.计算点到平面的距离并加权：计算当前点到拟合平面的距离，并根据距离加权。若判断为有效点，则将有效的点和其对应的法向量保存起来。 
    ***********/

    void findCorrespondingSurfFeatures() {
        surf_res_cnt = 0;

        for (int i = 0; i < surf_last_ds->points.size(); ++i) {
            PointType point_sel;
            transformPoint(&surf_last_ds->points[i], &point_sel);  // 转换当前帧点到全局坐标系下
            vector<int> point_search_idx;
            vector<float> point_search_dists;
            kd_tree_surf_last->nearestKSearch(point_sel, 5, point_search_idx, point_search_dists); // 在地图中找到最近的5个点

            Eigen::Matrix<double, 5, 3> matA0;
            Eigen::Matrix<double, 5, 1> matB0 = - Eigen::Matrix<double, 5, 1>::Ones();

            if (point_search_dists[4] < 1.0) {  // 确保最近的5个点都在一定范围内
                PointType center;

                for (int j = 0; j < 5; ++j) {
                    matA0(j, 0) = surf_from_map_ds->points[point_search_idx[j]].x;
                    matA0(j, 1) = surf_from_map_ds->points[point_search_idx[j]].y;
                    matA0(j, 2) = surf_from_map_ds->points[point_search_idx[j]].z;
                }

                // Get the norm of the plane using linear solver based on QR composition
                // 使用QR分解法求解平面的法向量
                Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                double normInverse = 1 / norm.norm();
                norm.normalize(); // get the unit norm

                // Compute the centroid of the plane
                // 计算平面的质心
                center.x = matA0.col(0).sum() / 5.0;
                center.y = matA0.col(1).sum() / 5.0;
                center.z = matA0.col(2).sum() / 5.0;

                // Make sure that the plan is fit
                // 检查拟合的平面是否有效
                bool planeValid = true;
                for (int j = 0; j < 5; ++j) {
                    if (fabs(norm.x() * surf_from_map_ds->points[point_search_idx[j]].x +
                             norm.y() * surf_from_map_ds->points[point_search_idx[j]].y +
                             norm.z() * surf_from_map_ds->points[point_search_idx[j]].z + normInverse) > 0.06) {
                        planeValid = false;
                        break;
                    }
                }

                // if one eigenvalue is significantly larger than the other two
                if (planeValid) {
                    // 计算点到平面的距离
                    float pd = norm.x() * point_sel.x + norm.y() * point_sel.y + norm.z() * point_sel.z + normInverse;
                    float weight = 1 - 0.9 * fabs(pd) / sqrt(sqrt(point_sel.x * point_sel.x + point_sel.y * point_sel.y + point_sel.z * point_sel.z));

                    if(weight > 0.4) {
                        // 保存当前点和其对应的法向量
                        PointType normal;
                        normal.x = weight * norm.x();
                        normal.y = weight * norm.y();
                        normal.z = weight * norm.z();
                        normal.intensity = weight * normInverse;
                        surf_current_pts->push_back(surf_last_ds->points[i]);
                        surf_normal->push_back(normal);
                        ++surf_res_cnt;
                    }
                }
            }
        }
    }

    //该函数用于初始化或更新当前帧的绝对位姿。need to know
    void poseInitialization() {
        Eigen::Quaterniond q0(abs_pose[0],
                abs_pose[1],
                abs_pose[2],
                abs_pose[3]);
        Eigen::Vector3d t0(abs_pose[4],
                abs_pose[5],
                abs_pose[6]);

        Eigen::Quaterniond dq(rel_pose[0],
                rel_pose[1],
                rel_pose[2],
                rel_pose[3]);
        Eigen::Vector3d dt(rel_pose[4],
                rel_pose[5],
                rel_pose[6]);
        t0 = q0 * dt + t0; //?
        q0 = q0 * dq;

        abs_pose[0] = q0.w();
        abs_pose[1] = q0.x();
        abs_pose[2] = q0.y();
        abs_pose[3] = q0.z();

        abs_pose[4] = t0.x();
        abs_pose[5] = t0.y();
        abs_pose[6] = t0.z();
    }


    //该函数的主要目的是计算相对位姿（rel_pose），即当前帧与上一关键帧之间的相对旋转和平移。
    void computeRelative() {
        Eigen::Quaterniond quaternion1;
        Eigen::Vector3d transition1;
        //获取上一关键帧的位姿：
        if(pose_info_cloud_frame->points.empty()) {
            // 如果是第一帧，初始化为单位四元数和零向量
            quaternion1 = Eigen::Quaterniond::Identity();
            transition1 = Eigen::Vector3d::Zero();
        } else {
            // 否则从上一关键帧中提取位姿
            int max_idx = pose_info_cloud_frame->points.size();
            quaternion1 = Eigen::Quaterniond(pose_info_cloud_frame->points[max_idx-2].qw,
                    pose_info_cloud_frame->points[max_idx-2].qx,
                    pose_info_cloud_frame->points[max_idx-2].qy,
                    pose_info_cloud_frame->points[max_idx-2].qz);
            transition1 = Eigen::Vector3d (pose_info_cloud_frame->points[max_idx-2].x,
                    pose_info_cloud_frame->points[max_idx-2].y,
                    pose_info_cloud_frame->points[max_idx-2].z);
        }

        // 提取当前帧的绝对位姿
        Eigen::Quaterniond quaternion2(abs_pose[0],
                abs_pose[1],
                abs_pose[2],
                abs_pose[3]);
        Eigen::Vector3d transition2(abs_pose[4],
                abs_pose[5],
                abs_pose[6]);

        // 计算相对旋转和平移
        Eigen::Quaterniond quaternion_r = quaternion1.inverse() * quaternion2;
        Eigen::Vector3d transition_r = quaternion1.inverse() *(transition2 - transition1);

         // 更新相对位姿数组
        rel_pose[0] = quaternion_r.w();
        rel_pose[1] = quaternion_r.x();
        rel_pose[2] = quaternion_r.y();
        rel_pose[3] = quaternion_r.z();
        rel_pose[4] = transition_r.x();
        rel_pose[5] = transition_r.y();
        rel_pose[6] = transition_r.z();
    }

    //通过 Ceres Solver 优化当前帧的位姿，使其与地图中提取的平面特征点对齐，从而更新当前帧的绝对位姿。
    /*******************
    该函数通过使用 Ceres Solver 进行非线性优化，逐步调整当前帧的位姿，使其与地图中的平面特征点对齐，从而实现高精度的位姿估计。
    同时，通过检测平移和旋转的变化量，判断是否需要添加新的关键帧，以保持位姿估计的稳定性和准确性。
    *******************/
    void updateTransformationWithCeres() {
//        Timer t_utc("updateTransformationWithCeres");
        // Make sure there is enough feature points in the sweep
        //1.检查特征点数量
        if (surf_from_map_ds->points.size() < 10) {
            ROS_WARN("Not enough feature points from the map");
            return;
        }
        //2.设置 KD-Tree：将地图中的特征点设置到 KD-Tree 中，以便后续进行最近邻搜索。
        kd_tree_surf_last->setInputCloud(surf_from_map_ds);

        //3.初始化位姿增量：将当前帧的绝对位姿初始化为位姿增量 transformInc。
        double transformInc[7] = {abs_pose[0],
                                  abs_pose[1],
                                  abs_pose[2],
                                  abs_pose[3],
                                  abs_pose[4],
                                  abs_pose[5],
                                  abs_pose[6]};

        //4.设置迭代次数，根据帧的数量，决定匹配次数
        int match_cnt;
        if(pose_info_cloud_frame->points.size() < 2)
            match_cnt = 8;
        else
            match_cnt = scan_match_cnt;
//        Timer t_loo("LiDAROdometryOptimization");

        //5.构建 Ceres 优化问题：为每次迭代创建 Ceres 优化问题。设置损失函数为 HuberLoss，并添加四元数和位移参数块。
        /**********
        
        整个优化过程如下：(1)初始化 Ceres 优化问题，包括损失函数和参数块。
        (2)找到当前帧中的平面特征点在地图中的对应点。
        (3)为每个对应点对添加残差块到 Ceres 优化问题中。
        (4)配置求解器选项并求解优化问题。
        (5)统一四元数表示，确保四元数的 w 分量为正。
        这个过程通过最小化点到平面的距离残差，优化当前帧相对于地图的位姿。通过迭代优化，可以不断提高位姿估计的精度。     
        ************/

        for (int iter_cnt = 0; iter_cnt < match_cnt; iter_cnt++) {
            ceres::LossFunction *lossFunction = new ceres::HuberLoss(0.1); //损失函数：我们使用了 Huber 损失函数（HuberLoss）。Huber 损失函数对异常值具有较好的鲁棒性，当残差较小时，Huber 损失函数是平方损失函数，当残差较大时，Huber 损失函数是线性损失函数。
            ceres::LocalParameterization *quatParameterization = new ceres:: QuaternionParameterization(); //对于四元数（Quaternion），我们使用了 Ceres 自带的 QuaternionParameterization 来保证四元数的约束，即四元数的模为1。
            ceres::Problem problem;
            problem.AddParameterBlock(transformInc, 4, quatParameterization);  //我们向优化问题中添加了两个参数块：一个是四元数（4维），另一个是位移（3维）。这两个参数块组合在一起描述了我们的位姿变化。
            problem.AddParameterBlock(transformInc + 4, 3);

            //6.寻找对应的平面特征：调用 findCorrespondingSurfFeatures 函数，找到当前帧中的平面特征点在地图中的对应点。
            findCorrespondingSurfFeatures();

            //7.添加残差块：为每个找到的对应点对，添加残差块到 Ceres 优化问题中。
            //历找到的所有对应点对，并为每个对应点对添加一个残差块到 Ceres 优化问题中。残差块的作用是定义一个误差函数，该误差函数反映了当前估计的参数和实际观察值之间的差异。
            for (int i = 0; i < surf_res_cnt; ++i) {
                Eigen::Vector3d currentPt(surf_current_pts->points[i].x,
                                          surf_current_pts->points[i].y,
                                          surf_current_pts->points[i].z);
                Eigen::Vector3d norm(surf_normal->points[i].x,
                                     surf_normal->points[i].y,
                                     surf_normal->points[i].z);
                double normInverse = surf_normal->points[i].intensity;

                //LidarPlaneNormIncreFactor::Create：这是一个自定义的工厂函数，用于创建一个残差块。
                //这个残差块表示的是点到平面的距离残差，使用当前点 currentPt，平面法向量 norm 和法向量的逆 normInverse 来构建残差。
                ceres::CostFunction *costFunction = LidarPlaneNormIncreFactor::Create(currentPt, norm, normInverse);
                problem.AddResidualBlock(costFunction, lossFunction, transformInc, transformInc + 4); //problem.AddResidualBlock：将残差块添加到优化问题中，同时指定损失函数 lossFunction 和两个参数块（四元数和位移）。
            }

            //8.求解优化问题：配置求解器选项并求解优化问题。
            ceres::Solver::Options solverOptions;
            solverOptions.linear_solver_type = ceres::DENSE_QR;  //线性求解器类型：DENSE_QR，这是一种适用于小规模问题的求解器。
            solverOptions.max_num_iterations = max_num_iter;  //最大迭代次数：设置为 max_num_iter。
            solverOptions.max_solver_time_in_seconds = 0.015;  //最大求解时间：限制为 0.015 秒。
            solverOptions.minimizer_progress_to_stdout = false;
            solverOptions.check_gradients = false;
            solverOptions.gradient_check_relative_precision = 1e-2;

            ceres::Solver::Summary summary;  
            ceres::Solve( solverOptions, &problem, &summary );  //调用 ceres::Solve 来实际求解问题，并将结果存储在 summary 中。

            //9.统一四元数表示：确保四元数的 w 分量为正，以便统一表示。
            if(transformInc[0] < 0) {
                Eigen::Quaterniond tmpQ(transformInc[0],
                        transformInc[1],
                        transformInc[2],
                        transformInc[3]);
                tmpQ = unifyQuaternion(tmpQ);
                transformInc[0] = tmpQ.w();
                transformInc[1] = tmpQ.x();
                transformInc[2] = tmpQ.y();
                transformInc[3] = tmpQ.z();
            }

            surf_current_pts->clear();
            surf_normal->clear();

            //10.更新绝对位姿：用优化后的位姿增量更新当前帧的绝对位姿
            abs_pose[0] = transformInc[0];
            abs_pose[1] = transformInc[1];
            abs_pose[2] = transformInc[2];
            abs_pose[3] = transformInc[3];
            abs_pose[4] = transformInc[4];
            abs_pose[5] = transformInc[5];
            abs_pose[6] = transformInc[6];
        }
//        t_loo.tic_toc();

        //    double ratio_u = double(surfResCount) / double(surfLastDS->points.size());

        //11.检查关键帧:根据平移和旋转的变化量，判断是否需要添加新的关键帧。
        Eigen::Vector3d transCur = Eigen::Vector3d(abs_pose[4],
                abs_pose[5],
                abs_pose[6]);
        Eigen::Quaterniond quatCur = Eigen::Quaterniond(abs_pose[0],
                abs_pose[1],
                abs_pose[2],
                abs_pose[3]);

        double dis = (transCur - trans_last_kf).norm();
        double ang = 2 * acos((quat_last_kF.inverse() * quatCur).w());
        if(((dis > 0.2 || ang > 0.1) && (pose_cloud_frame->points.size() - kf_num > 1) || (pose_cloud_frame->points.size() - kf_num > 2)) || pose_cloud_frame->points.size() <= 1){
            kf = true;
            trans_last_kf = Eigen::Vector3d(abs_pose[4],
                    abs_pose[5],
                    abs_pose[6]);
            quat_last_kF = Eigen::Quaterniond(abs_pose[0],
                    abs_pose[1],
                    abs_pose[2],
                    abs_pose[3]);
        } else
            kf = false;

//        t_utc.tic_toc();
    }

    //这个函数发布累计的里程计信息，以及保存并发布整个路径信息。
    void publishOdometry() {
        odom.header.stamp = cloud_header.stamp;
        odom.pose.pose.orientation.w = abs_pose[0];
        odom.pose.pose.orientation.x = abs_pose[1];
        odom.pose.pose.orientation.y = abs_pose[2];
        odom.pose.pose.orientation.z = abs_pose[3];
        odom.pose.pose.position.x = abs_pose[4];
        odom.pose.pose.position.y = abs_pose[5];
        odom.pose.pose.position.z = abs_pose[6];
        pub_odom.publish(odom);

        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header = odom.header;
        poseStamped.pose = odom.pose.pose;
        poseStamped.header.stamp = odom.header.stamp;
        path.header.stamp = odom.header.stamp;
        path.poses.push_back(poseStamped);
        path.header.frame_id = frame_id;
        pub_path.publish(path);
    }

    //这个函数发布当前帧相对于上一帧的相对里程计信息。
    void publishEachOdometry() {
        nav_msgs::Odometry eachOdom;
        eachOdom.header.frame_id = frame_id;

        eachOdom.header.stamp = cloud_header.stamp;
        eachOdom.pose.pose.orientation.w = rel_pose[0];
        eachOdom.pose.pose.orientation.x = rel_pose[1];
        eachOdom.pose.pose.orientation.y = rel_pose[2];
        eachOdom.pose.pose.orientation.z = rel_pose[3];
        eachOdom.pose.pose.position.x = rel_pose[4];
        eachOdom.pose.pose.position.y = rel_pose[5];
        eachOdom.pose.pose.position.z = rel_pose[6];
        pub_each_odom.publish(eachOdom);
    }

    //这个函数发布处理后的点云数据。
    void publishCloudLast() {
        Eigen::Vector3d trans(rel_pose[4], rel_pose[5], rel_pose[6]);
        Eigen::Quaterniond quat(1, 0, 0, 0);

        //去畸变
        if(if_to_deskew) {
            undistortion(surf_features, trans, quat);
            undistortion(edge_features, trans, quat);
            undistortion(full_cloud, trans, quat);
        }

        //发布特征点云：将 edge_features、surf_features 和 full_cloud 转换为 ROS 消息并发布
        sensor_msgs::PointCloud2 msgs;

        pcl::toROSMsg(*edge_features, msgs);
        msgs.header.stamp = cloud_header.stamp;
        msgs.header.frame_id = frame_id;
        pub_edge.publish(msgs);

        pcl::toROSMsg(*surf_features, msgs);
        msgs.header.stamp = cloud_header.stamp;
        msgs.header.frame_id = frame_id;
        pub_surf.publish(msgs);

        pcl::toROSMsg(*full_cloud, msgs);
        msgs.header.stamp = cloud_header.stamp;
        msgs.header.frame_id = frame_id;
        pub_full_cloud.publish(msgs);
        pcl::PointCloud<PointType>::Ptr full_cloud_map;
        full_cloud_map.reset(new pcl::PointCloud<PointType>());

        // transform full resolution input cloud to map
        //转换点云：将完整分辨率的输入点云转换到地图坐标系下，并将其转换为 ROS 消息发布。
        size_t laser_full_cloud_size = full_cloud->points.size();
        for (int i = 0; i < laser_full_cloud_size; i++) {
            PointType point_sel;
            transformPoint(&full_cloud->points[i], &point_sel);
            full_cloud_map->points.emplace_back(point_sel);
        }
        pcl::toROSMsg(*full_cloud_map, msgs);
        msgs.header.stamp = cloud_header.stamp;
        msgs.header.frame_id = frame_id;
        pub_full_cloud_map.publish(msgs);    
    }

    /*******
    这段代码定义了run函数，这是整个激光雷达里程计模块的核心执行流程。
    它负责处理新接收到的数据，初始化系统，并更新和发布当前帧的位姿信息。
     *******/
    void run() {
        //1. 数据同步和检查:检查新接收到的平面特征点云 (new_surf)、完整点云 (new_full_cloud) 和边缘特征点云 (new_edge) 是否存在，并确保它们的时间戳差异在0.1秒以内。这是为了确保数据是同步的
        if (new_surf && new_full_cloud && new_edge
                && abs(time_new_full_points - time_new_surf) < 0.1
                && abs(time_new_full_points - time_new_edge) < 0.1) {
            new_surf = false;
            new_edge = false;
            new_full_cloud = false;
        } else
            return;

        //如果系统未初始化，调用savePoses()和checkInitialization()进行初始化。
        if (!system_initialized) {
            savePoses();
            checkInitialization();
            return;
        }
        //初始化当前帧的位姿。这里的poseInitialization()是将相对位姿转换为绝对位姿。
        poseInitialization();
//        Timer t_odm("LidarOdometry");
        //构建当前帧的局部地图，确保有足够的环境信息用于匹配和优化。
        buildLocalMap();
        //对当前帧和局部地图的点云进行下采样，减少计算量并去除冗余点。
        downSampleCloud();
        //调用updateTransformationWithCeres()函数，利用Ceres优化库进行位姿优化。
        updateTransformationWithCeres();
        //将当前帧的优化位姿保存起来。
        savePoses();
        //计算当前帧相对于上一关键帧的相对位姿。
        computeRelative();
        //如果当前帧被确定为关键帧 (kf)，发布里程计和点云信息，并更新关键帧计数。
        if(kf) {
            kf_num = pose_cloud_frame->points.size();
            publishOdometry();
            publishCloudLast();
        }
        publishEachOdometry();
        clearCloud();
//        cout<<"odom_pub_cnt: "<<++odom_pub_cnt<<endl;
//        t_odm.tic_toc();
//        runtime += t_odm.toc();
//        cout<<"Odometry average run time: "<<runtime / odom_pub_cnt<<endl;
    }
};

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);  //始化Google的日志系统，以便在程序中使用glog进行日志记录。
    ros::init(argc, argv, "GLIO");

    LidarOdometry LO;

    ROS_INFO("\033[1;32m---->\033[0m Lidar Odometry Started.");

    ros::Rate rate(200);

    while (ros::ok()) {
        ros::spinOnce();
        LO.run();
        rate.sleep();
    }

    ros::spin();
    return 0;
}
