// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <chisel_ros/ChiselServer.h>
#include <open_chisel/truncation/QuadraticTruncator.h>
#include <open_chisel/truncation/InverseTruncator.h>

int main(int argc, char **argv)
{
    ROS_INFO("Starting up chisel node.");
    ros::init(argc, argv, "Chisel");
    ros::NodeHandle nh("~");
    int chunkSizeX, chunkSizeY, chunkSizeZ;
    double voxelResolution;
    double truncationDistScale;
    int weight;
    bool useCarving;
    bool useColor;
    double carvingDist;
    std::string depthImageTopic;
    std::string depthImageInfoTopic;
    std::string depthImageTransform;
    std::string colorImageTopic;
    std::string colorImageInfoTopic;
    std::string colorImageTransform;
    std::string baseTransform;
    std::string meshTopic;
    std::string chunkBoxTopic;
    double nearPlaneDist;
    double farPlaneDist;
    chisel_ros::ChiselServer::FusionMode mode;
    std::string modeString;
    std::string pointCloudTopic;

    // param_name:参数服务器中的参数名 
    // param_val：将该参数名的值读取到param_val 
    // default_val：指定默认的值。
    nh.param("chunk_size_x", chunkSizeX, 32);
    nh.param("chunk_size_y", chunkSizeY, 32);
    nh.param("chunk_size_z", chunkSizeZ, 32);
    nh.param("truncation_scale", truncationDistScale, 8.0);
    nh.param("integration_weight", weight, 1);
    nh.param("use_voxel_carving", useCarving, true);
    nh.param("use_color", useColor, true);
    nh.param("carving_dist_m", carvingDist, 0.05);
    nh.param("voxel_resolution_m", voxelResolution, 0.03);
    nh.param("near_plane_dist", nearPlaneDist, 0.05);
    nh.param("far_plane_dist", farPlaneDist, 5.0);
    // stereo_mapper/depth..
    nh.param("depth_image_topic", depthImageTopic, std::string("/depth_image"));
    nh.param("point_cloud_topic", pointCloudTopic, std::string("/camera/depth_registered/points"));
    // 内参和图像大小
    nh.param("depth_image_info_topic", depthImageInfoTopic, std::string("/depth_camera_info"));
    // 深度坐标系
    nh.param("depth_image_transform", depthImageTransform, std::string("/camera_depth_optical_frame"));
    // stereo_mapper/image..
    nh.param("color_image_topic", colorImageTopic, std::string("/color_image"));
    // 内参和图像大小
    nh.param("color_image_info_topic", colorImageInfoTopic, std::string("/color_camera_info"));
    // 色彩坐标系
    nh.param("color_image_transform", colorImageTransform, std::string("/camera_rgb_optical_frame"));
    // 参考系
    nh.param("base_transform", baseTransform, std::string("/camera_link"));
    // 发布tpoic
    nh.param("mesh_topic", meshTopic, std::string("full_mesh"));
    nh.param("chunk_box_topic", chunkBoxTopic, std::string("chunk_boxes"));
    // 模式
    nh.param("fusion_mode", modeString, std::string("DepthImage"));

    if (modeString == "DepthImage")
    {
        ROS_INFO("Mode depth image");
        mode = chisel_ros::ChiselServer::FusionMode::DepthImage;
    }
    else if (modeString == "PointCloud")
    {
        ROS_INFO("Mode point cloud");
        mode = chisel_ros::ChiselServer::FusionMode::PointCloud;
    }
    else
    {
        ROS_ERROR("Unrecognized fusion mode %s. Recognized modes: \"DepthImage\", \"PointCloud\"\n", modeString.c_str());
        return -1;
    }

    ROS_INFO("Subscribing.");
    
    // create server
    chisel_ros::ChiselServerPtr server(new chisel_ros::ChiselServer(nh, chunkSizeX, chunkSizeY, chunkSizeZ, voxelResolution, useColor, mode));

    //chisel::TruncatorPtr truncator(new chisel::QuadraticTruncator(truncationDistScale));
    // create 截断？
    chisel::TruncatorPtr truncator(new chisel::InverseTruncator(truncationDistScale));
    
    // 设置融合参数 截断方式，加权方式，雕刻，雕刻距离
    server->SetupProjectionIntegrator(truncator, static_cast<uint16_t>(weight), useCarving, carvingDist);

    server->SetNearPlaneDist(nearPlaneDist);
    server->SetFarPlaneDist(farPlaneDist);

    // 订阅节点，深度与色彩坐标系有区别
    if (depthImageTransform == colorImageTransform)
    {
        server->SubscribeAll(depthImageTopic, depthImageInfoTopic,
                             colorImageTopic, colorImageInfoTopic,
                             depthImageTransform,
                             pointCloudTopic);
        //server->SubscribePointCloud(pointCloudTopic);
    }
    else
    {
        server->SubscribeDepthImage(depthImageTopic, depthImageInfoTopic, depthImageTransform);
        server->SubscribeColorImage(colorImageTopic, colorImageInfoTopic, colorImageTransform);
    }
    
    // 设置发布节点
    server->SetupDepthPosePublisher("last_depth_pose");
    server->SetupDepthFrustumPublisher("last_depth_frustum");

    server->SetupColorPosePublisher("last_color_pose");
    server->SetupColorFrustumPublisher("last_color_frustum");
    
    // 四种通讯方式：Topic(话题)、Service(服务)、Parameter Serivce(参数服务器)、Actionlib(动作库)
    // 区别：
    // topic:节点A发布，节点B订阅（不相关）
    // service:节点A请求服务S，服务S相应（相关）
    server->AdvertiseServices();

    server->SetBaseTransform(baseTransform);
    // 设置发布topic
    server->SetupMeshPublisher(meshTopic);
    server->SetupChunkBoxPublisher(chunkBoxTopic);
    ROS_INFO("Beginning to loop.");

    ros::spin();
}
