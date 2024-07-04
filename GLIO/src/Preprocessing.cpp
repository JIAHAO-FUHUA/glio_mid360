#include "utils/common.h"
#include "utils/timer.h"
#include "utils/math_tools.h"

#define  PI  3.1415926535

class Preprocessing {
private:
    // float cloudCurvature[400000];
    // int cloudSortInd[400000];
    // int cloudNeighborPicked[400000];
    // int cloudLabel[400000];
    // int ds_rate = 2;
    // double ds_v = 0.4;

    // bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]);}

    ros::NodeHandle nh;

    ros::Subscriber sub_Lidar_cloud;
    ros::Subscriber sub_imu;

    ros::Publisher pub_surf; //lidar detect the surface
    ros::Publisher pub_edge; //lidar detect the edge
    ros::Publisher pub_cutted_cloud; //处理后的点云发布者

    int pre_num = 0;

    // pcl::PointCloud<PointType> lidar_cloud_in; // 输入的激光雷达点云。

    pcl::PointCloud<PointXYZINormal> lidar_cloud_in; //livox revise
    pcl::PointCloud<PointXYZINormal> lidar_cloud_cutted;

    std_msgs::Header cloud_header;  //点云消息头。

    vector<sensor_msgs::ImuConstPtr> imu_buf;  //IMU数据缓冲区
    int idx_imu = 0;   //IMU数据索引
    double current_time_imu = -1;  //当前IMU时间

    Eigen::Vector3d gyr_0;    //初始陀螺仪数据
    Eigen::Quaterniond qIMU = Eigen::Quaterniond::Identity();  //IMU四元数姿态
    //Eigen::Quaterniond q_iMU = Eigen::Quaterniond::Identity();
    //Eigen::Vector3d rIMU = Eigen::Vector3d::Zero();
    bool first_imu = false;
    string imu_topic;   //IMU数据话题

    std::deque<sensor_msgs::PointCloud2> cloud_queue;  //点云数据队列
    sensor_msgs::PointCloud2 current_cloud_msg;   //当前点云消息
    double time_scan_next;                        // 下一次扫描时间

    //int N_SCANS = 64;
    //revise livox
    int N_SCANS = 4;
    int H_SCANS = 4000;

    double qlb0, qlb1, qlb2, qlb3; //四元数的分量
    Eigen::Quaterniond q_lb;

    string frame_id = "GLIO";  //帧ID

    // "/points_raw", "/velodyne_pcl_gen/cloud", "/data_raw/lidar"
    string lidar_topic = "/livox_ros_points";  //revise  激光雷达数据话题

    double runtime = 0;

    //livox
    //double edge_thres, surf_thres;

    double edgeThreshold = 0;  //边缘点阈值
    double surfThreshold = 0;  //平面点阈值
    Eigen::Matrix4f lidar_extrinsic;  //激光雷达外参矩阵


public:
    Preprocessing():
        nh("~"){
        //// 初始化激光雷达的外参矩阵
        // lidar_extrinsic << 0.999945, 0.0104382,0.00142715, 0.00634028,
        //                     -0.00005845,0.845067, -0.534599, -0.354308,
        //                     -0.0067067, 0.534558, 0.845105, 0.163907,
        //                     0         , 0       , 0       ,  1;

        // // revise
        // lidar_extrinsic << 0.999945, 0.0104382,0.00142715, 0.00634028,                            
        //                   -0.00805845,0.845067, -0.534599, -0.354308,                           
        //                   -0.00678627, 0.534558, 0.845105, 0.163907,                           
        //                   0         , 0       , 0       ,  1;

        lidar_extrinsic << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
        // 获取参数
        // 获取参数
        if (!getParameter("/lidar_odometry/lidar_topic", lidar_topic)) {
            ROS_WARN("lidar_topic not set, use default value: /livox_ros_points");
            lidar_topic = "/livox_ros_points";
        }

        if (!getParameter("/lidar_odometry/line_num", N_SCANS)) {
            ROS_WARN("line_num not set, use default value: 4");
            N_SCANS = 4;
        }

        // if (!getParameter("/lidar_odometry/ds_rate", ds_rate)) {
        //     ROS_WARN("ds_rate not set, use default value: 1");
        //     ds_rate = 1;
        // }

        if (!getParameter("/common/frame_id", frame_id)) {
            ROS_WARN("frame_id not set, use default value: lili_odom");
            frame_id = "lili_odom";
        }

        if (!getParameter("/IMU/imu_topic", imu_topic)) {
            ROS_WARN("imu_topic not set, use default value: /livox/imu");
            imu_topic = "/imu/data";
        }

        //extrinsic parameters 表示激光雷达在车体坐标系中的姿态
        if (!getParameter("/Estimator/ql2b_w", qlb0)) {
            ROS_WARN("ql2b_w not set, use default value: 1");
            qlb0 = 1;
        }

        if (!getParameter("/Estimator/ql2b_x", qlb1)) {
            ROS_WARN("ql2b_x not set, use default value: 0");
            qlb1 = 0;
        }

        if (!getParameter("/Estimator/ql2b_y", qlb2)) {
            ROS_WARN("ql2b_y not set, use default value: 0");
            qlb2 = 0;
        }

        if (!getParameter("/Estimator/ql2b_z", qlb3)) {
            ROS_WARN("ql2b_z not set, use default value: 0");
            qlb3 = 0;
        }

        if (!getParameter("/lidar_odometry/edgeThreshold", edgeThreshold)) {
            ROS_WARN("edgeThreshold not set, use default value: 1.0");
            edgeThreshold = 1.0;
        }

        if (!getParameter("/lidar_odometry/surfThreshold", surfThreshold)) {
            ROS_WARN("surfThreshold not set, use default value: 0.1");
            surfThreshold = 0.1;
        }

        q_lb = Eigen::Quaterniond(qlb0, qlb1, qlb2, qlb3);

         // 订阅激光雷达点云数据
        sub_Lidar_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/livox_ros_points", 100, &Preprocessing::cloudHandler, this);

        // 订阅IMU数据
        sub_imu = nh.subscribe<sensor_msgs::Imu>(imu_topic, 200, &Preprocessing::imuHandler, this);

        // 发布处理后的点云数据
        pub_surf = nh.advertise<sensor_msgs::PointCloud2>("/surf_features", 100);
        pub_edge = nh.advertise<sensor_msgs::PointCloud2>("/edge_features", 100);
        pub_cutted_cloud = nh.advertise<sensor_msgs::PointCloud2>("/lidar_cloud_cutted", 100);
    }

    ~Preprocessing(){}

    //移除距离小于给定阈值的点云点
    //cloud_in：输入的点云，cloud_out：输出的点云， thres：距离阈值，小于该阈值的点将被移除。
    template <typename PointT>
    void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                                pcl::PointCloud<PointT> &cloud_out, float thres)
    {
        // 检查 cloud_in 和 cloud_out 是否是同一个对象，如果不是，则复制 cloud_in 的头信息和大小
        if (&cloud_in != &cloud_out)
        {
            cloud_out.header = cloud_in.header;
            cloud_out.points.resize(cloud_in.points.size());
        }

        size_t j = 0;
        //遍历输入点云的每个点，计算其到原点的距离平方，如果距离小于阈值的平方，则跳过该点
        //将剩余的点复制到输出点云中，并调整输出点云的大小。
        for (size_t i = 0; i < cloud_in.points.size(); ++i) {
            if (cloud_in.points[i].x * cloud_in.points[i].x +
                    cloud_in.points[i].y * cloud_in.points[i].y +
                    cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
                continue;
            cloud_out.points[j] = cloud_in.points[i];
            j++;
        }
        if (j != cloud_in.points.size()) {
            cloud_out.points.resize(j);
        }

        //设置输出点云的高度、宽度和密集性标志。
        cloud_out.height = 1;
        cloud_out.width = static_cast<uint32_t>(j);
        cloud_out.is_dense = true;
    }

    //这个函数用于计算给定点的深度（到原点的距离）。
    template <typename PointT>
    double getDepth(PointT pt) {
        return sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
    }

    //用于去除点云数据中的畸变，主要是由于激光雷达在扫描过程中自身的运动引起的畸变
    //pt：输入的点，包含坐标和强度信息。 quat：四元数，用于表示姿态
   PointXYZINormal undistortion(PointXYZINormal pt, const Eigen::Quaterniond quat) {
        double dt = 0.1;
        int line = int(pt.intensity);
        double dt_i = pt.intensity - line;

        double ratio_i = dt_i / dt;
        if(ratio_i >= 1.0) {
            ratio_i = 1.0;
        }

        //使用球面线性插值（SLERP）计算当前时刻的四元数 q_si。
        Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
        Eigen::Quaterniond q_si = q0.slerp(ratio_i, qIMU);

        Eigen::Vector3d pt_i(pt.x, pt.y, pt.z);
        //使用四元数 q_si 进行坐标变换，去除畸变。
        q_si = q_lb * q_si * q_lb.inverse();   //maybe need to revise
        Eigen::Vector3d pt_s = q_si * pt_i;

        //返回去畸变后的点。
        PointXYZINormal p_out;
        p_out.x = pt_s.x();
        p_out.y = pt_s.y();
        p_out.z = pt_s.z();
        p_out.intensity = pt.intensity;
        p_out.curvature = pt.curvature;
        return p_out;
    }

    //这个方法用于根据IMU的角速度数据计算旋转增量，并更新IMU的姿态四元数
    void solveRotation(double dt, Eigen::Vector3d angular_velocity)
    {
        Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity); //计算未加权的角速度 un_gyr
        qIMU *= deltaQ(un_gyr * dt);   //根据角速度计算旋转增量，并更新IMU的姿态四元数 qIMU。
        gyr_0 = angular_velocity;  //更新角速度 gyr_0。
    }

    //这个方法用于将点云从激光雷达坐标系转换到车体坐标系。revise
    PointXYZINormal TransfromPointToBody(const PointXYZINormal & pi){
        Eigen::Vector3f p(pi.x,pi.y,pi.z);
        p = lidar_extrinsic.block(0,0,3,3) * p;  //将点的坐标从激光雷达坐标系转换到车体坐标系。
        PointXYZINormal p0;
        p0.x = p.x();
        p0.y = p.y();
        p0.z = p.z();
        p0.intensity = pi.intensity;
        p0.curvature = pi.curvature;
        //p0.ring = pi.ring;  //revise
        //p0.timestamp = pi.intensity;
        return p0;
    }


    //这个方法用于处理IMU（惯性测量单元）数据，并根据IMU的角速度更新系统的姿态四元数。该方法考虑了时间戳对齐和插值，以提高姿态估计的精度。
    //t_cur：当前时间。
    void processIMU(double t_cur)
    {
        double rx = 0, ry = 0, rz = 0;  // 初始化角速度变量
        int i = idx_imu;    // 从上一次处理的IMU数据索引开始
        if(i >= imu_buf.size())
            i--;   // 确保索引在有效范围内
        
        //遍历IMU缓冲区中的数据，直到当前时间 t_cur。
        while(imu_buf[i]->header.stamp.toSec() < t_cur) {

            double t = imu_buf[i]->header.stamp.toSec();  // 获取IMU数据的时间戳
            if (current_time_imu < 0)
                current_time_imu = t;   // 初始化当前IMU时间
            double dt = t - current_time_imu;  // 计算时间差
            current_time_imu = imu_buf[i]->header.stamp.toSec();  // 更新当前IMU时间

            // 获取当前IMU的角速度
            rx = imu_buf[i]->angular_velocity.x;
            ry = imu_buf[i]->angular_velocity.y;
            rz = imu_buf[i]->angular_velocity.z;
            // 计算旋转并更新姿态四元数
            solveRotation(dt, Eigen::Vector3d(rx, ry, rz));
            i++;   // 移动到下一个IMU数据
            if(i >= imu_buf.size())
                break;  // 超出缓冲区范围，退出循环
        }

        // 如果还有未处理的IMU数据
        if(i < imu_buf.size()) {
            double dt1 = t_cur - current_time_imu;  // 当前时间到最后一个IMU数据时间的差
            double dt2 = imu_buf[i]->header.stamp.toSec() - t_cur;  // 下一个IMU数据时间到当前时间的差

            // 计算插值权重
            double w1 = dt2 / (dt1 + dt2);
            double w2 = dt1 / (dt1 + dt2);
            // 插值角速度
            rx = w1 * rx + w2 * imu_buf[i]->angular_velocity.x;
            ry = w1 * ry + w2 * imu_buf[i]->angular_velocity.y;
            rz = w1 * rz + w2 * imu_buf[i]->angular_velocity.z;

            // 计算旋转并更新姿态四元数
            solveRotation(dt1, Eigen::Vector3d(rx, ry, rz));
        }
        current_time_imu = t_cur;  // 更新当前IMU时间到最新的时间
        idx_imu = i;  // 更新IMU数据索引
    }

    //处理IMU（惯性测量单元）数据，并将其存储在缓冲区中，同时初始化某些状态变量
    void imuHandler(const sensor_msgs::ImuConstPtr& ImuIn)
    {

        // 将IMU数据存储在缓冲区中
        imu_buf.push_back(ImuIn);
//        sensor_msgs::ImuPtr raw_imu_msg (new sensor_msgs::Imu(*ImuIn));
//        //imu_in.angular_velocity.x/180*3.1415926, imu_in.angular_velocity.y/180*3.1415926, imu_in.angular_velocity.z/180*3.141592
//        raw_imu_msg->angular_velocity.x =  raw_imu_msg->angular_velocity.x/180*3.1415926;
//
//
//        raw_imu_msg->angular_velocity.x =  raw_imu_msg->angular_velocity.x/180*3.1415926;
//        raw_imu_msg->angular_velocity.y =  raw_imu_msg->angular_velocity.y/180*3.1415926;
//        raw_imu_msg->angular_velocity.z =  raw_imu_msg->angular_velocity.z/180*3.1415926;
//        double correct_stamp  = gps2utc(2153, ImuIn->header.stamp.toSec());
//        raw_imu_msg->header.stamp = ros::Time(correct_stamp);
//
//        imu_buf.push_back(raw_imu_msg);

        // 保持缓冲区大小不超过3000个数据点，超出部分置为空
        if(imu_buf.size() > 3000)
            imu_buf[imu_buf.size() - 3001] = nullptr;

        // 初始化当前IMU时间
        if (current_time_imu < 0)
            current_time_imu = ImuIn->header.stamp.toSec();

        // 初始化角速度，如果是第一次接收到IMU数据
        if (!first_imu)
        {
            first_imu = true;
            double rx = 0, ry = 0, rz = 0;
            rx = ImuIn->angular_velocity.x;
            ry = ImuIn->angular_velocity.y;
            rz = ImuIn->angular_velocity.z;
            Eigen::Vector3d angular_velocity(rx, ry, rz);
            gyr_0 = angular_velocity;
        }
    }

    //获取特定角度对应的扫描线ID，主要用于HLD32激光雷达。
    //angle：输入的角度。 SCANID：输出的扫描线ID。
    bool getScanIDHLD32(double angle, int& SCANID)
    {
        //
        std::vector<double> angle2ID;
        angle2ID.push_back(-30.67); // 1st ring
        angle2ID.push_back(-9.33);
        angle2ID.push_back(-29.33);
        angle2ID.push_back(-8.00);
        angle2ID.push_back(-28.00);
        angle2ID.push_back(-6.67);
        angle2ID.push_back(-26.67);
        angle2ID.push_back(-5.33); 

        angle2ID.push_back(-25.33);
        angle2ID.push_back(-4.00);
        angle2ID.push_back(-24.00);
        angle2ID.push_back(-2.67);
        angle2ID.push_back(-22.67);
        angle2ID.push_back(-1.33);
        angle2ID.push_back(-21.33);
        angle2ID.push_back(0.00); 

        angle2ID.push_back(-20.00);
        angle2ID.push_back(1.33);
        angle2ID.push_back(-18.67);
        angle2ID.push_back(2.67);
        angle2ID.push_back(-17.33);
        angle2ID.push_back(4.00);
        angle2ID.push_back(-16.00);
        angle2ID.push_back(5.33); 

        angle2ID.push_back(-14.67);
        angle2ID.push_back(6.67);
        angle2ID.push_back(-13.33);
        angle2ID.push_back(8.0);
        angle2ID.push_back(-12.00);
        angle2ID.push_back(9.33);
        angle2ID.push_back(-10.67);
        angle2ID.push_back(10.67);

        //遍历 angle2ID，找到与输入角度 angle 最接近的扫描线角度，并记录对应的扫描线ID SCANID
        int length = angle2ID.size();
        int diff = 10000;
        for(int i = 0; i < length; i++)
        {
            double diffTmp = fabs(angle2ID[i] - angle);
            if(diffTmp < diff)
            {
                SCANID = i;
            }
        }
    }
    
    //将GPS时间转换为UTC时间。GPS时间和UTC时间之间有一个已知的偏移量，方法通过将GPS周和GPS秒转换为秒数，然后加上一个常数偏移量来实现转换。
    double gps2utc(double gps_week, double gps_second){
        
        return (gps_week * 604800.0 + gps_second - 18.0) + 315964800.0;
    }

    //处理激光雷达点云数据，执行去除闭合点云、计算点云曲率、挑选特征点等操作，并发布处理后的点云数据。 revise
    void cloudHandler( const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {
        // cache point cloud
        //ROS_WARN("Waiting for laserCloudMsg");
        cloud_queue.push_back(*laserCloudMsg);
//        sensor_msgs::PointCloud2Ptr correctlaserCloudMsg (new sensor_msgs::PointCloud2(*laserCloudMsg));
//        // for date 2021/04/16 03:20:54.000
//        double correct_stamp  = gps2utc(2153, laserCloudMsg->header.stamp.toSec());
//        correctlaserCloudMsg->header.stamp = ros::Time(correct_stamp);
//        cloud_queue.push_back(*correctlaserCloudMsg);
        // 保持点云队列大小不超过2
        if (cloud_queue.size() <= 2)
            return;
        else {
            current_cloud_msg = cloud_queue.front();
            cloud_queue.pop_front();

            cloud_header = current_cloud_msg.header;
            cloud_header.frame_id = frame_id;
            time_scan_next = cloud_queue.front().header.stamp.toSec();
        }

        //ROS_WARN("Waiting for laserCloudMsg11111");
        //检查IMU数据是否存在：
        int tmpIdx = 0;
        if(idx_imu > 0)
            tmpIdx = idx_imu - 1;
        //如果IMU数据缓冲区为空，或者最早的IMU数据时间戳大于下一次扫描时间，则打印警告信息并返回。
        if (imu_buf.empty() || imu_buf[tmpIdx]->header.stamp.toSec() > time_scan_next) {
            ROS_WARN("Waiting for IMU data ...");
            return;
        }

//        Timer t_pre("LidarPreprocessing");
//        Timer t_pre_1("LidarPreprocessing_1");
//        Timer t_pre_2("LidarPreprocessing_2");
//        Timer t_pre_3("LidarPreprocessing_3");
//        Timer t_pre_4("LidarPreprocessing_4");

        //点云预处理
        // std::vector<int> scanStartInd(N_SCANS, 0);
        // std::vector<int> scanEndInd(N_SCANS, 0);

//        pcl::PointCloud<PointXYZIRT> lidar_cloud_in;
//        pcl::PointCloud<PointType> lidar_cloud_in; //revise
        pcl::fromROSMsg(current_cloud_msg, lidar_cloud_in);


        lidar_cloud_cutted.clear(); //livox

        std::vector<int> indices;

        //使用 pcl::removeNaNFromPointCloud 去除NaN点。
        pcl::removeNaNFromPointCloud(lidar_cloud_in, lidar_cloud_in, indices);

        //调用 removeClosedPointCloud 去除距离小于3米的点。
        removeClosedPointCloud(lidar_cloud_in, lidar_cloud_in, 0.1);  //revise


        //计算点云扫描的起始角度 startOri 和结束角度 endOri。
        int cloudSize = lidar_cloud_in.points.size();

        //revise
        // float startOri = -atan2(lidar_cloud_in.points[0].y, lidar_cloud_in.points[0].x);
        // float endOri = -atan2(lidar_cloud_in.points[cloudSize - 1].y,
        //         lidar_cloud_in.points[cloudSize - 1].x) +
        //         2 * M_PI;

        // if (endOri - startOri > 3 * M_PI)
        //     endOri -= 2 * M_PI;

        // else if (endOri - startOri < M_PI)
        //     endOri += 2 * M_PI;


        //处理IMU数据
        //调用 processIMU 方法处理IMU数据，更新姿态四元数 qIMU
        //如果姿态四元数包含NaN值，则重置为单位四元数。
        if(first_imu)
            processIMU(time_scan_next);
        if(isnan(qIMU.w()) || isnan(qIMU.x()) || isnan(qIMU.y()) || isnan(qIMU.z())) {
            qIMU = Eigen::Quaterniond::Identity();
        }
//        t_pre_1.tic_toc();
        //ROS_WARN("Waiting for laserCloudMsg12222");
        //点云去畸变和特征提取 revise
        // bool halfPassed = false;
        // int count = cloudSize;
        // PointType point;
        // PointType point_undis;
        // std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS); // revise

        //livox

        /*
        PointXYZINormal point;
        PointXYZINormal point_undis;
        PointXYZINormal mat[N_SCANS][H_SCANS];
        double t_interval = 0.1 / (H_SCANS - 1);
        pcl::PointCloud<PointXYZINormal>::Ptr surf_features(new pcl::PointCloud<PointXYZINormal>());
        pcl::PointCloud<PointXYZINormal>::Ptr edge_features(new pcl::PointCloud<PointXYZINormal>());

        //遍历每个点，计算其扫描ID和相对时间。 revise
        for (int i = 0; i < cloudSize; i++) {
//            PointXYZIRT point_transform;
//
//            point_transform = TransfromPointToBody(lidar_cloud_in.points[i]);
//            point.x = point_transform.x;
//            point.y = point_transform.y;
//            point.z = point_transform.z;
//            point.intensity = 0.1 * point_transform.intensity;
            point.x = lidar_cloud_in.points[i].x;
            point.y = lidar_cloud_in.points[i].y;
            point.z = lidar_cloud_in.points[i].z;
            point.intensity = 0.1 * lidar_cloud_in.points[i].intensity;

            point.curvature = lidar_cloud_in.points[i].curvature;


            float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
            int scanID = 0;

            //revise
            if (N_SCANS == 16) {
                scanID = int((angle + 15) / 2 + 0.5);
                if (scanID > (N_SCANS - 1) || scanID < 0) {
                    count--;
                    continue;
                }
            }
            else if (N_SCANS == 32) {
                scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
                // std::cout << "Scan ID: " << scanID << std::endl;
                // std::cout << "angle:   " << angle << std::endl;
                // getScanIDHLD32(angle, scanID);
                // std::cout << "Scan ID-M: " << scanID << std::endl;
                // std::cout << "angle-M:   " << angle << std::endl;
                if (scanID > (N_SCANS - 1) || scanID < 0) {
                    count--;
                    continue;
                }
            }
//            else if (N_SCANS == 40) {
//                scanID = lidar_cloud_in.points[i].ring;
//                // std::cout << "Scan ID: " << scanID << std::endl;
//                // std::cout << "angle:   " << angle << std::endl;
//                // getScanIDHLD32(angle, scanID);
//                // std::cout << "Scan ID-M: " << scanID << std::endl;
//                // std::cout << "angle-M:   " << angle << std::endl;
//                if (scanID > (N_SCANS - 1) || scanID < 0) {
//                    count--;
//                    continue;
//                }
//            }
            else if (N_SCANS == 64) {
                if (angle >= -8.83)
                    scanID = int((2 - angle) * 3.0 + 0.5);
                else
                    scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

                // use [0 50]  > 50 remove outlies
                if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0) {
                    count--;
                    continue;
                }
            }
            else {
                printf("wrong scan number\n");
                ROS_BREAK();
            }

            float ori = -atan2(point.y, point.x);
            if (!halfPassed) {
                if (ori < startOri - M_PI / 2)
                    ori += 2 * M_PI;
                else if (ori > startOri + M_PI * 3 / 2)
                    ori -= 2 * M_PI;

                if (ori - startOri > M_PI)
                    halfPassed = true;
            }
            else {
                ori += 2 * M_PI;
                if (ori < endOri - M_PI * 3 / 2)
                    ori += 2 * M_PI;
                else if (ori > endOri + M_PI / 2)
                    ori -= 2 * M_PI;
            }

            float relTime = (ori - startOri) / (endOri - startOri);
//            point.intensity = scanID;
            point.intensity = scanID + 0.1 * relTime;

            //调用 undistortion 方法去除点云畸变。
            //将去畸变后的点云按照扫描ID存储在 laserCloudScans 中。
            point_undis = undistortion(point, qIMU);
//            laserCloudScans[scanID].push_back(point);
            laserCloudScans[scanID].push_back(point_undis);
        }
//        std::cout << "cloudSize: " << cloudSize << std::endl;
//        t_pre_2.tic_toc();
        cloudSize = count;
        // printf("points size %d \n", cloudSize);

        pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
        for (int i = 0; i < N_SCANS; i++) {
            scanStartInd[i] = laserCloud->size() + 5;
            *laserCloud += laserCloudScans[i];
            scanEndInd[i] = laserCloud->size() - 6;
        }

        //遍历每个点，计算其曲率并存储在 cloudCurvature 中。
        for (int i = 5; i < cloudSize - 5; i++) {
            float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
            float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
            float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

            cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
            cloudSortInd[i] = i;
            cloudNeighborPicked[i] = 0;
            cloudLabel[i] = 0;
        }
//        t_pre_3.tic_toc();

        //遍历每个扫描，选择边缘特征点和平面特征点，并将其存储在相应的点云中。
        pcl::PointCloud<PointType> cornerPointsSharp;
        pcl::PointCloud<PointType> cornerPointsLessSharp;
        pcl::PointCloud<PointType> surfPointsFlat;
        pcl::PointCloud<PointType> surfPointsLessFlat;

        for (int i = 0; i < N_SCANS; i++) {
            if( scanEndInd[i] - scanStartInd[i] < 6 || i % ds_rate != 0)
                continue;
            pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
            for (int j = 0; j < 6; j++) {
                int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;
                int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

                auto bound_comp = bind(&Preprocessing::comp, this, _1, _2);
                std::sort(cloudSortInd + sp, cloudSortInd + ep + 1, bound_comp);

                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--) {
                    int ind = cloudSortInd[k];

                    //select the edge features
                    if (cloudNeighborPicked[ind] == 0 &&
                            cloudCurvature[ind] > edgeThreshold) { // original 2.0

                        largestPickedNum++;
                        if (largestPickedNum <= 2) {
                            cloudLabel[ind] = 2;
                            cornerPointsSharp.push_back(laserCloud->points[ind]);
                            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                        }
                        else if (largestPickedNum <= 10) {
                            cloudLabel[ind] = 1;
                            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                        }
                        else
                            break;

                        cloudNeighborPicked[ind] = 1;

                        for (int l = 1; l <= 5; l++) {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                int smallestPickedNum = 0;
                for (int k = sp; k <= ep; k++) {
                    int ind = cloudSortInd[k];

                    if(laserCloud->points[ind].x*laserCloud->points[ind].x+laserCloud->points[ind].y*laserCloud->points[ind].y+laserCloud->points[ind].z*laserCloud->points[ind].z < 0.25)
                        continue;

                    //select the plannar features
                    if (cloudNeighborPicked[ind] == 0 &&
                            cloudCurvature[ind] < surfThreshold) { // 0.1 original

                        cloudLabel[ind] = -1;
                        surfPointsFlat.push_back(laserCloud->points[ind]);

                        smallestPickedNum++;
                        if (smallestPickedNum >= 4)
                            break;

                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++) {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++) {
                    if(laserCloud->points[k].x*laserCloud->points[k].x+laserCloud->points[k].y*laserCloud->points[k].y+laserCloud->points[k].z*laserCloud->points[k].z < 0.25)
                        continue;
                    if (cloudLabel[k] <= 0)
                        surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }

            //使用 pcl::VoxelGrid 对平面特征点进行下采样。
            pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
            pcl::VoxelGrid<PointType> downSizeFilter;
            downSizeFilter.setInputCloud(surfPointsLessFlatScan);
            downSizeFilter.setLeafSize(ds_v, ds_v, ds_v);
            downSizeFilter.filter(surfPointsLessFlatScanDS);

            surfPointsLessFlat += surfPointsLessFlatScanDS;
        }

        */
        //        t_pre_4.tic_toc();

        PointXYZINormal point;
        PointXYZINormal point_undis;
        //ROS_WARN("Waiting for laserCloudMsg11233");
        PointXYZINormal mat[N_SCANS][H_SCANS];
        //ROS_WARN("Waiting for laserCloudMsg12345");
        double t_interval = 0.1 / (H_SCANS - 1);
       //ROS_WARN("Waiting for laserCloudMsg1233");
        pcl::PointCloud<PointXYZINormal>::Ptr surf_features(new pcl::PointCloud<PointXYZINormal>());
        pcl::PointCloud<PointXYZINormal>::Ptr edge_features(new pcl::PointCloud<PointXYZINormal>());

        //ROS_WARN("Waiting for laserCloudMsg33");
        for (int i = 0; i < cloudSize; i++)
        {
            point.x = lidar_cloud_in.points[i].x;
            point.y = lidar_cloud_in.points[i].y;
            point.z = lidar_cloud_in.points[i].z;
            point.intensity = lidar_cloud_in.points[i].intensity;
            point.curvature = lidar_cloud_in.points[i].curvature;

            int scan_id = 0;
            if (N_SCANS == 4)
                scan_id = (int)point.intensity;
            if (scan_id < 0)
                continue;

            point_undis = undistortion(point, qIMU);
            lidar_cloud_cutted.push_back(point_undis);

            double dep = point_undis.x * point_undis.x + point_undis.y * point_undis.y + point_undis.z * point_undis.z;
            if (dep > 40000.0 || dep < 4.0 || point_undis.curvature < 0.05 || point_undis.curvature > 25.45)
                continue;
            int col = int(round((point_undis.intensity - scan_id) / t_interval));
            if (col >= H_SCANS || col < 0)
                continue;
            if (mat[scan_id][col].curvature != 0)
                continue;
            mat[scan_id][col] = point_undis;
        }

        //ROS_WARN("Waiting for laserCloudMsg4");
        for (int i = 5; i < H_SCANS - 12; i = i + 6)
        {
            vector<Eigen::Vector3d> near_pts;
            Eigen::Vector3d center(0, 0, 0);
            int num = 24;
            for (int j = 0; j < 6; j++)
            {
                for (int k = 0; k < N_SCANS; k++)
                {
                    if (mat[k][i + j].curvature <= 0)
                    {
                        num--;
                        continue;
                    }
                    Eigen::Vector3d pt(mat[k][i + j].x,
                                       mat[k][i + j].y,
                                       mat[k][i + j].z);
                    center += pt;
                    near_pts.push_back(pt);
                }
            }
            if (num < 15)
                continue;
            center /= num;
            // Covariance matrix
            Eigen::Matrix3d matA1 = Eigen::Matrix3d::Zero();
            for (int j = 0; j < near_pts.size(); j++)
            {
                Eigen::Vector3d zero_mean = near_pts[j] - center;
                matA1 += (zero_mean * zero_mean.transpose());
            }
            //ROS_WARN("Waiting for laserCloudMsg5");
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(matA1);

            vector<int> idsx_edge;
            vector<int> idsy_edge;
            for (int k = 0; k < N_SCANS; k++)
            {
                double max_s = 0;
                double max_s1 = 0;
                int idx = i;
                for (int j = 0; j < 6; j++)
                {
                    if (mat[k][i + j].curvature <= 0)
                    {
                        continue;
                    }
                    double g1 = getDepth(mat[k][i + j - 4]) + getDepth(mat[k][i + j - 3]) +
                                getDepth(mat[k][i + j - 2]) + getDepth(mat[k][i + j - 1]) - 8 * getDepth(mat[k][i + j]) +
                                getDepth(mat[k][i + j + 1]) + getDepth(mat[k][i + j + 2]) + getDepth(mat[k][i + j + 3]) +
                                getDepth(mat[k][i + j + 4]);

                    g1 = g1 / (8 * getDepth(mat[k][i + j]) + 1e-3);

                    if (g1 > 0.06)
                    {
                        if (g1 > max_s)
                        {
                            max_s = g1;
                            idx = i + j;
                        }
                    }
                    else if (g1 < -0.06)
                    {
                        if (g1 < max_s1)
                        {
                        }
                    }
                }
                if (max_s != 0)
                {
                    idsx_edge.push_back(k);
                    idsy_edge.push_back(idx);
                }
            }

            vector<Eigen::Vector3d> near_pts_edge;
            Eigen::Vector3d center_edge(0, 0, 0);
            for (int j = 0; j < idsx_edge.size(); j++)
            {
                Eigen::Vector3d pt(mat[idsx_edge[j]][idsy_edge[j]].x,
                                   mat[idsx_edge[j]][idsy_edge[j]].y,
                                   mat[idsx_edge[j]][idsy_edge[j]].z);
                center_edge += pt;
                near_pts_edge.push_back(pt);
            }
            center_edge /= idsx_edge.size();
            // Covariance matrix
            Eigen::Matrix3d matA_edge = Eigen::Matrix3d::Zero();
            for (int j = 0; j < near_pts_edge.size(); j++)
            {
                Eigen::Vector3d zero_mean = near_pts_edge[j] - center_edge;
                matA_edge += (zero_mean * zero_mean.transpose());
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver_edge(matA_edge);

            if (eigen_solver_edge.eigenvalues()[2] > edgeThreshold * eigen_solver_edge.eigenvalues()[1] && idsx_edge.size() > 3)
            {
                Eigen::Vector3d unitDirection = eigen_solver_edge.eigenvectors().col(2);
                for (int j = 0; j < idsx_edge.size(); j++)
                {
                    if (mat[idsx_edge[j]][idsy_edge[j]].curvature <= 0 && mat[idsx_edge[j]][idsy_edge[j]].intensity <= 0)
                        continue;
                    mat[idsx_edge[j]][idsy_edge[j]].normal_x = unitDirection.x();
                    mat[idsx_edge[j]][idsy_edge[j]].normal_y = unitDirection.y();
                    mat[idsx_edge[j]][idsy_edge[j]].normal_z = unitDirection.z();

                    edge_features->points.push_back(mat[idsx_edge[j]][idsy_edge[j]]);
                    mat[idsx_edge[j]][idsy_edge[j]].curvature *= -1;
                }
            }

            if (eigen_solver.eigenvalues()[0] < surfThreshold * eigen_solver.eigenvalues()[1])
            {
                Eigen::Vector3d unitDirection = eigen_solver.eigenvectors().col(0);
                for (int j = 0; j < 6; j++)
                {
                    for (int k = 0; k < N_SCANS; k++)
                    {
                        if (mat[k][i + j].curvature <= 0)
                        {
                            continue;
                        }
                        mat[k][i + j].normal_x = unitDirection.x();
                        mat[k][i + j].normal_y = unitDirection.y();
                        mat[k][i + j].normal_z = unitDirection.z();

                        surf_features->points.push_back(mat[k][i + j]);
                        mat[k][i + j].curvature *= -1;
                    }
                }
            }
        }

        //
        sensor_msgs::PointCloud2 surf_features_msg;
        pcl::toROSMsg(*surf_features, surf_features_msg);
        surf_features_msg.header.stamp = cloud_header.stamp;
        surf_features_msg.header.frame_id = frame_id;
        pub_surf.publish(surf_features_msg);

        sensor_msgs::PointCloud2 edge_features_msg;
        pcl::toROSMsg(*edge_features, edge_features_msg);
        edge_features_msg.header.stamp = cloud_header.stamp;
        edge_features_msg.header.frame_id = frame_id;
        pub_edge.publish(edge_features_msg);

        sensor_msgs::PointCloud2 cloud_cutted_msg;
        pcl::toROSMsg(lidar_cloud_cutted, cloud_cutted_msg);
        cloud_cutted_msg.header.stamp = cloud_header.stamp;
        cloud_cutted_msg.header.frame_id = frame_id;
        pub_cutted_cloud.publish(cloud_cutted_msg);



        // // 发布处理后的点云
        // sensor_msgs::PointCloud2 laserCloudOutMsg;
        // pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
        // laserCloudOutMsg.header.stamp = current_cloud_msg.header.stamp;
        // laserCloudOutMsg.header.frame_id = frame_id;
        // pub_cutted_cloud.publish(laserCloudOutMsg);

        // sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
        // pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
        // cornerPointsLessSharpMsg.header.stamp = current_cloud_msg.header.stamp;
        // cornerPointsLessSharpMsg.header.frame_id = frame_id;
        // pub_edge.publish(cornerPointsLessSharpMsg);

        // sensor_msgs::PointCloud2 surfPointsLessFlat2;
        // pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
        // surfPointsLessFlat2.header.stamp = current_cloud_msg.header.stamp;
        // surfPointsLessFlat2.header.frame_id = frame_id;
        // pub_surf.publish(surfPointsLessFlat2);

        // 重置IMU状态
        qIMU = Eigen::Quaterniond::Identity();
        //rIMU = Eigen::Vector3d::Zero();
//        t_pre.tic_toc();
//        runtime += t_pre.toc();
        //cout<<"pre_num: "<<++pre_num<<endl;
        //cout<<"Preprocessing average run time: "<<runtime / pre_num<<endl;
    }




};

int main(int argc, char** argv) {
    ros::init(argc, argv, "GLIO");
    //创建 Preprocessing 类的实例 Pre。该实例将订阅激光雷达和IMU数据，并进行点云数据预处理。
    Preprocessing Pre;
    ROS_INFO("\033[1;32m---->\033[0m Preprocessing Started.");

    ros::spin();
    return 0;
}
