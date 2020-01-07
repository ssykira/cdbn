#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/PoseStamped.h"
#include <image_transport/image_transport.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>

#include "piiq_msgs/DetectComponents.h"
#include "componentdetection.h"
#include <iostream>

ComponentDetection componDetec;
ros::Publisher pub, pub0;
image_transport::Publisher pub_img;

std::string templatesPath = ros::package::getPath("piiq_components")+"/config/axle_base/dataset_axle_base/";
std::string cameraFilepath = ros::package::getPath("piiq_components")+"/config/axle_base/test_camera_calib_bas.yml";
std::string modelFilepath = ros::package::getPath("piiq_components")+"/config/axle_base/axle_base_new.stl";
std::string paramFilepath = ros::package::getPath("piiq_components")+"/config/axle_base/params_setting_bas.yml";
std::string resImgFilepath = ros::package::getPath("piiq_components")+"/config/axle_base/res_image.png";
std::string resImgFilepath0 = ros::package::getPath("piiq_components")+"/config/axle_base/src_image.png";
std::string resImgFilepath1 = ros::package::getPath("piiq_components")+"/config/axle_base/edge_image.png";
std::string resImgFilepath2 = ros::package::getPath("piiq_components")+"/config/axle_base/rect_image.png";

//std::string templatesPath = ros::package::getPath("piiq_components")+"/config/box/dataset_box/";
//std::string cameraFilepath = ros::package::getPath("piiq_components")+"/config/box/test_camera_calib_bas.yml";
//std::string modelFilepath = ros::package::getPath("piiq_components")+"/config/box/box_new.stl";
//std::string paramFilepath = ros::package::getPath("piiq_components")+"/config/box/params_setting_bas.yml";
//std::string resImgFilepath = ros::package::getPath("piiq_components")+"/config/box/res_image.png";
//std::string resImgFilepath0 = ros::package::getPath("piiq_components")+"/config/box/src_image.png";
//std::string resImgFilepath1 = ros::package::getPath("piiq_components")+"/config/box/edge_image.png";


Eigen::Quaterniond eulerAnglesToQuaternion(const Eigen::Vector3d& rpy) 
{
    double roll=rpy(0);
    double pitch=rpy(1);
    double yaw=rpy(2); 
    double cosRoll = cosf(roll * 0.5f); 
    double sinRoll = sinf(roll * 0.5f);

    double cosPitch = cosf(pitch * 0.5f);
    double sinPitch = sinf(pitch * 0.5f);

    double cosHeading = cosf(yaw * 0.5f);
    double sinHeading = sinf(yaw * 0.5f);

    double w = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    double x = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    double y = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    double z = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

    Eigen::Quaterniond q(w,x,y,z); 
    return q;
}


bool handle_detect_components(piiq_msgs::DetectComponents::Request &req,
                                piiq_msgs::DetectComponents::Response &res)
{
    cv::Mat img_src = cv_bridge::toCvCopy(req.camera_data.rgb_l, sensor_msgs::image_encodings::BGR8)->image;
    cv::cvtColor(img_src, img_src, CV_BGR2GRAY);
    std::cout<<img_src.size<<std::endl;
    cv::imwrite(resImgFilepath0, img_src);
    ROS_INFO("%s",req.camera_mat);
    //ROS_INFO(req.distcoe);
    /*
    string camera_filename = generateYAMLFilename(cameraFilepath);
    FileStorage fs(camera_filename, FileStorage::READ );
    Mat_<double> camera_matrix(3,3);
    Mat_<double> distcoeffs(1,5);

    //cv::Mat camera_matrix , distcoeffs;
    //camera_matrix = cv::Mat(3, 3, CV_64F);
    //distcoeffs = cv::Mat(1, 5, CV_64F);
    fs["fx"]>>camera_matrix.at<double>(0,0);
    fs["fy"]>>camera_matrix.at<double>(1,1);
    fs["cx"]>>camera_matrix.at<double>(0,2);
    fs["cy"]>>camera_matrix.at<double>(1,2);
    camera_matrix(2,2) = 1;
    fs["dist_px"]>>distcoeffs.at<double>(0,2);
    fs["dist_py"]>>distcoeffs.at<double>(0,3);
    fs["dist_k1"]>>distcoeffs.at<double>(0,0);
    fs["dist_k2"]>>distcoeffs.at<double>(0,1);
    fs["dist_k3"]>>distcoeffs.at<double>(0,4);
    fs.release(); */

/*    camera_matrix.at<double>(0,0)=1629.3722201123494;
    camera_matrix.at<double>(1,1)=1629.5272527504253;
    camera_matrix.at<double>(0,2)=553.38658244979558;
    camera_matrix.at<double>(1,2)=472.91046906022740;
    distcoeffs.at<double>(0,2)=-0.0011506381329986334;
    distcoeffs.at<double>(0,3)=0.00065151747243398777;
    distcoeffs.at<double>(0,0)=-0.42160873818241146;
    distcoeffs.at<double>(0,1)=0.15224731040966030;
    distcoeffs.at<double>(0,4)=0.36599939891950084;
*/
    Mat_<double> camera_matrix(3,3);
    Mat_<double> distcoeffs(1,5);
    camera_matrix.at<double>(0,0)=req.camera_mat[0];
    camera_matrix.at<double>(1,1)=req.camera_mat[4];
    camera_matrix.at<double>(0,2)=req.camera_mat[2];
    camera_matrix.at<double>(1,2)=req.camera_mat[5];
    camera_matrix(2,2) = 1;
    distcoeffs.at<double>(0,2)=req.distcoe[2];
    distcoeffs.at<double>(0,3)=req.distcoe[3];
    distcoeffs.at<double>(0,0)=req.distcoe[0];
    distcoeffs.at<double>(0,1)=req.distcoe[1];
    distcoeffs.at<double>(0,4)=req.distcoe[4];

    cv:: Mat dst, newCameraMatrix;
//    blur(img_src, img_src, Size(5, 5)); 
    //dst = img_src ;
    cv:: undistort(img_src, dst, camera_matrix,distcoeffs,newCameraMatrix);

//    img_src = dst ;
//    Ptr<CLAHE> clahe = createCLAHE();
//    clahe ->apply(img_src,img_src);
//    equalizeHist( img_src, img_src );

//    cv::filter2D(img_src, img_src, CV_8UC3, kernel);
    blur(dst, dst, Size(3, 3));                         //模糊去噪
    //equalizeHist( dst, dst );
    
    cv::imwrite(resImgFilepath2, dst);
    bool res_out;
    ROS_INFO("!!!!!!");
    std::cout<<img_src.size<<std::endl;
    res_out = componDetec.onObjDetection(dst);
    ROS_INFO("???????!");
    std::cout<<componDetec.img_res.size<<std::endl;

    
    cv::imwrite(resImgFilepath, componDetec.img_res);
    cv::imwrite(resImgFilepath1, componDetec.img_edge);

    

    sensor_msgs::Image img_show;
    cv_bridge::CvImage(std_msgs::Header(), "bgr8", componDetec.img_res).toImageMsg(img_show);
    pub_img.publish(img_show);

    ROS_INFO("Image Saved!");

    if(!res_out)
    {
        ROS_INFO("Recive Frames Failed!\n");
        res.gripper_pose = {0.,0.,0.,0.,0.,0.};
        //return false;
        return true;
    }

    // select top r_vec and t_vec
    if(componDetec.line2d.matchGood.size()==0)
    {
        ROS_INFO("No Component Detected!\n");
        res.gripper_pose = {0.,0.,0.,0.,0.,0.};
        //return false;
        return true;
    }

    // Convert Rotation and Translation to ROS::TF:
	MatchRes detectRes = componDetec.line2d.matchGood[0];
	cv::Mat r_vec_, t_vec_;
	r_vec_ = detectRes.r_vec.clone();
	t_vec_ = detectRes.t_vec.clone();

    // Break up Pose to translation and rotation
    double t_vec_comp[3] = {t_vec_.at<double>(0, 0),t_vec_.at<double>(1, 0),t_vec_.at<double>(2, 0)};
    double r_vec_comp[3] = {r_vec_.at<double>(0, 0),r_vec_.at<double>(1, 0),r_vec_.at<double>(2, 0)};

    // acquire transfermation Matrix
    double rot_qut_comp[4] = {0};
    ceres::AngleAxisToQuaternion<double> ( r_vec_comp, rot_qut_comp );

    printf("\nCompo Position in camera:  %lf  %lf  %lf\n", t_vec_comp[0], t_vec_comp[1], t_vec_comp[2]);


    // Coordinate transformation: From Camera to RobotBase
    tf::Pose compo_Pose;
    compo_Pose.setOrigin( tf::Vector3(t_vec_comp[0], t_vec_comp[1], t_vec_comp[2]) );
 	compo_Pose.setRotation( tf::Quaternion(rot_qut_comp[1], rot_qut_comp[2], rot_qut_comp[3], rot_qut_comp[0]) );


    tf::Stamped<tf::Pose> stam_in(compo_Pose, ros::Time(), "camera");
    tf::Stamped<tf::Pose> stam_out;

    tf::TransformListener tf_transf;
//    tf::StampedTransform tf_cam2word;// ***********************test lookuptransform


    while (1){
        tf::StampedTransform transform;
        try
        {
          //tf_transf.waitForTransform("/world", "/camera", ros::Time(0), ros::Duration(0.1));
          tf_transf.lookupTransform("/world", "/camera", ros::Time(0), transform);
          tf::Vector3 tfvec_cam2word = transform.getOrigin();

          /**** Detail: because the tool0_controller frame from ur_driver may set to ZERO, 0.086205 is eye to tool0_controller trans. ***/
          if(abs(tfvec_cam2word.m_floats[2] - 0.086205) < 0.001 ){
                printf("\nCurrent Continue. \n");
                ros::Duration(0.1).sleep();
                continue;
          }
          printf("\nPosition Translation:  %f  %f  %f\n", tfvec_cam2word.m_floats[0], tfvec_cam2word.m_floats[1], tfvec_cam2word.m_floats[2]);

          tf_transf.transformPose("world", stam_in, stam_out);
          break;
        }
        catch (tf::TransformException &ex)
        {
          ROS_WARN("Current :  %s",ex.what());
          ros::Duration(0.1).sleep();
          continue;
        }
    }

    tf::Vector3 tfvec_comp_RB = stam_out.getOrigin();
    tf::Quaternion tfqua_comp_RB = stam_out.getRotation();

    {
//       printf("\nCompo Position in world:  %lf  %lf  %lf\n", tfvec_comp_RB.m_floats[0], tfvec_comp_RB.m_floats[1], tfvec_comp_RB.m_floats[2]);

       geometry_msgs::PoseStamped pub_pose0;

       pub_pose0.pose.position.x=tfvec_comp_RB.m_floats[0];
       pub_pose0.pose.position.y=tfvec_comp_RB.m_floats[1];
       pub_pose0.pose.position.z=tfvec_comp_RB.m_floats[2];
       pub_pose0.pose.orientation.x= tfqua_comp_RB.x();
       pub_pose0.pose.orientation.y= tfqua_comp_RB.y();
       pub_pose0.pose.orientation.z= tfqua_comp_RB.z();
       pub_pose0.pose.orientation.w= tfqua_comp_RB.getW();

       pub_pose0.header.stamp=ros::Time::now();
       pub_pose0.header.frame_id="world";
       pub0.publish(pub_pose0);
    }


    printf("\nCompo Pose in world:  %lf  %lf  %lf  %lf  %lf  %lf  %lf\n",
           tfvec_comp_RB.m_floats[0], tfvec_comp_RB.m_floats[1], tfvec_comp_RB.m_floats[2],
           tfqua_comp_RB.getW(), tfqua_comp_RB.x(), tfqua_comp_RB.y(), tfqua_comp_RB.z());

    double rotMat_comp_RB[9] = {0};
    double rot_qut_comp_RB[4] = {tfqua_comp_RB.getW(), tfqua_comp_RB.x(), tfqua_comp_RB.y(), tfqua_comp_RB.z()};
    ceres::QuaternionToRotation<double> (rot_qut_comp_RB, rotMat_comp_RB);
    double tl_comp_RB[3] = {tfvec_comp_RB.m_floats[0], tfvec_comp_RB.m_floats[1], tfvec_comp_RB.m_floats[2]};
    cv::Mat rotat_trans = cv::Mat(3, 3, CV_64F, rotMat_comp_RB);
    cv::Mat tl_trans = cv::Mat(3, 1, CV_64F, tl_comp_RB);
    cv::Mat tf_comp_RB = cv::Mat::eye(4, 4, CV_64F);
    rotat_trans.copyTo(tf_comp_RB(Rect(0,0,3,3)));
    tl_trans.copyTo(tf_comp_RB(Rect(3,0,1,3)));

    Eigen::Vector3d x_axis_vec, y_axis_vec, z_axis_vec;
    x_axis_vec = Eigen::Vector3d(tf_comp_RB.at<double>(0,0), tf_comp_RB.at<double>(1,0), tf_comp_RB.at<double>(2,0));
    y_axis_vec = Eigen::Vector3d(tf_comp_RB.at<double>(0,1), tf_comp_RB.at<double>(1,1), tf_comp_RB.at<double>(2,1));
    z_axis_vec = Eigen::Vector3d(tf_comp_RB.at<double>(0,2), tf_comp_RB.at<double>(1,2), tf_comp_RB.at<double>(2,2));

    // Judge which status the component is
    Eigen::Vector3d Z_axis_RB(0,0,1); 
    double moud_x = sqrt(pow(x_axis_vec(0), 2) + pow(x_axis_vec(1), 2) + pow(x_axis_vec(2), 2));
    double moud_y = sqrt(pow(y_axis_vec(0), 2) + pow(y_axis_vec(1), 2) + pow(y_axis_vec(2), 2));
    double moud_z = sqrt(pow(z_axis_vec(0), 2) + pow(z_axis_vec(1), 2) + pow(z_axis_vec(2), 2));
    double ang_x_Z = acos(x_axis_vec.dot(Z_axis_RB)/moud_x)*180./M_PI;
    double ang_y_Z = acos(y_axis_vec.dot(Z_axis_RB)/moud_y)*180./M_PI;
    double ang_z_Z = acos(z_axis_vec.dot(Z_axis_RB)/moud_z)*180./M_PI;

    cv::Mat tf_gripper_RB = cv::Mat::eye(4, 4, CV_64F);
    
    if(ang_x_Z<45 || ang_x_Z>=135) // if component is standing sideways
    {
        if(ang_x_Z>=135) // rotation 90 by Y axis
        {
            cv::Mat tf_Cc_Cg = cv::Mat::eye(4, 4, CV_64F);
            tf_Cc_Cg.at<double>(0,3) = -0.000;
            tf_Cc_Cg.at<double>(0,0) = 0;
            tf_Cc_Cg.at<double>(2,2) = 0;
            tf_Cc_Cg.at<double>(0,2) = 1;
            tf_Cc_Cg.at<double>(2,0) = -1;
            tf_gripper_RB = tf_comp_RB * tf_Cc_Cg;
        }
        else  // rotation -90 by Y axis
        {
            cv::Mat tf_Cc_Cg = cv::Mat::eye(4, 4, CV_64F);
            tf_Cc_Cg.at<double>(0,3) = 0.000;
            tf_Cc_Cg.at<double>(0,0) = 0;
            tf_Cc_Cg.at<double>(2,2) = 0;
            tf_Cc_Cg.at<double>(0,2) = -1;
            tf_Cc_Cg.at<double>(2,0) = 1;
            tf_gripper_RB = tf_comp_RB * tf_Cc_Cg;
        }

        res.gripper_type = 1;   // Component is standing sideways
    }
    else if(ang_y_Z<45 || ang_y_Z>=135) // if component is standing
    {
        if(ang_y_Z>=135) // rotation -90 by X axis
        {
            cv::Mat tf_Cc_Cg = cv::Mat::eye(4, 4, CV_64F);
            tf_Cc_Cg.at<double>(1,3) = -0.015;
            tf_Cc_Cg.at<double>(1,1) = 0;
            tf_Cc_Cg.at<double>(2,2) = 0;
            tf_Cc_Cg.at<double>(1,2) = 1;
            tf_Cc_Cg.at<double>(2,1) = -1;
            tf_gripper_RB = tf_comp_RB * tf_Cc_Cg;

            res.gripper_type = 2;   // Component is standing up
        }
        else  // rotation 90 by Y axis and 90 by X axis, Dxyz
        {
            cv::Mat tf_Cc_Cg = cv::Mat::eye(4, 4, CV_64F);
            tf_Cc_Cg.at<double>(1,3) = 0.015;
            tf_Cc_Cg.at<double>(0,0) = 0;
            tf_Cc_Cg.at<double>(1,1) = 0;
            tf_Cc_Cg.at<double>(2,2) = 0;
            tf_Cc_Cg.at<double>(0,1) = 1;
            tf_Cc_Cg.at<double>(1,2) = -1;
            tf_Cc_Cg.at<double>(2,0) = -1;
            tf_gripper_RB = tf_comp_RB * tf_Cc_Cg;

            res.gripper_type = 1;   // Component is standing down
        }
    }
    else //if(ang_z_Z<45 || ang_z_Z>=135) // if component is lying
    {
        if(ang_z_Z>=125)
        {
            cv::Mat tf_Cc_Cg = cv::Mat::eye(4, 4, CV_64F);
            tf_Cc_Cg.at<double>(1,3) -= 0.005;
            tf_gripper_RB = tf_comp_RB * tf_Cc_Cg;
        }
        else if(ang_z_Z<55)// rotation 180 by Y axis
        {
            cv::Mat tf_Cc_Cg = cv::Mat::eye(4, 4, CV_64F);
            tf_Cc_Cg.at<double>(1,3) = -0.005;
            tf_Cc_Cg.at<double>(0,0) = -1;
            tf_Cc_Cg.at<double>(2,2) = -1;
            tf_gripper_RB = tf_comp_RB * tf_Cc_Cg;
        }

        res.gripper_type = 0;   // Component is lying
    }

    // Judge whether the tf_Matrix is Correct or not
    Eigen::Vector3d x_axis_gr, y_axis_gr, z_axis_gr;
    x_axis_gr = Eigen::Vector3d(tf_gripper_RB.at<double>(0,0), tf_gripper_RB.at<double>(1,0), tf_gripper_RB.at<double>(2,0));
    y_axis_gr = Eigen::Vector3d(tf_gripper_RB.at<double>(0,1), tf_gripper_RB.at<double>(1,1), tf_gripper_RB.at<double>(2,1));
    z_axis_gr = Eigen::Vector3d(tf_gripper_RB.at<double>(0,2), tf_gripper_RB.at<double>(1,2), tf_gripper_RB.at<double>(2,2));
    
    moud_z = sqrt(pow(z_axis_gr(0), 2) + pow(z_axis_gr(1), 2) + pow(z_axis_gr(2), 2));
    ang_z_Z = acos(z_axis_gr.dot(Z_axis_RB)/moud_z)*180./M_PI;

    double pos_x = tf_gripper_RB.at<double>(0,3);
    double pos_y = tf_gripper_RB.at<double>(1,3);
    double pos_z = tf_gripper_RB.at<double>(2,3);

    if(ang_z_Z<125 || pos_x > 0.200 || pos_x < -0.350 || pos_y > -0.450 || pos_y < -1.000)
    {
        ROS_INFO("Tranfermat component pose Error!");
        res.gripper_pose = {0.,0.,0.,0.,0.,0.};
        //return false;
        return true;
    }
    
    // makesure Z axis is negative
    if(z_axis_gr(2) >= 0)
        z_axis_gr = -z_axis_gr;

    // makesure Y axis is positive
    if(y_axis_gr(1) <= 0)
        y_axis_gr = -y_axis_gr;
    x_axis_gr = y_axis_gr.cross(z_axis_gr);

    // convert rotationMatrix to RPY angle
    double thetax = atan2(y_axis_gr(2), z_axis_gr(2));
    double thetay = atan2(-x_axis_gr(2), sqrt(pow(x_axis_gr(0), 2) + pow(x_axis_gr(1), 2)));
    double thetaz = atan2(x_axis_gr(1), x_axis_gr(0));

    res.gripper_pose = {pos_x, pos_y, pos_z, thetax, thetay, thetaz};

    printf("\nPose:  %lf  %lf  %lf  %lf  %lf  %lf\n", res.gripper_pose[0], res.gripper_pose[1], res.gripper_pose[2],
    res.gripper_pose[3], res.gripper_pose[4], res.gripper_pose[5]);

    //if visualize is true then convert pose into ros msg and publish
    if(req.visualize)
    {
       Eigen::Quaterniond q;
       Eigen::Vector3d grasp_eulerRPY(thetax,thetay,thetaz);
       q = eulerAnglesToQuaternion(grasp_eulerRPY);

       geometry_msgs::PoseStamped pub_pose;
    
       pub_pose.pose.position.x=tf_gripper_RB.at<double>(0,3);
       pub_pose.pose.position.y=tf_gripper_RB.at<double>(1,3);
       pub_pose.pose.position.z=tf_gripper_RB.at<double>(2,3);
       pub_pose.pose.orientation.x=q.x();
       pub_pose.pose.orientation.y=q.y();
       pub_pose.pose.orientation.z=q.z();
       pub_pose.pose.orientation.w=q.w();
    
       pub_pose.header.stamp=ros::Time::now();
       pub_pose.header.frame_id="world";
       pub.publish(pub_pose);

       ROS_INFO("Pose Published");
    }

    ROS_INFO("Gripper Pose Computed!!!");

    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "components_vision");

    ros::NodeHandle node;

    pub = node.advertise<geometry_msgs::PoseStamped>("/piiq/gripper_pose", 10);
    pub0 = node.advertise<geometry_msgs::PoseStamped>("/piiq/component_pose", 10);

    image_transport::ImageTransport it(node);
    pub_img = it.advertise("camera/image_res", 1);

    componDetec.setDetectFilePathes(templatesPath, cameraFilepath, modelFilepath);

    bool flag0 = componDetec.loadParameters(paramFilepath);
    if(!flag0)
    {
        ROS_INFO("\nLoad Parameters Failed!");
        return -1;
    }
    ROS_INFO("\nDetector Initialization Start!");
    bool flag1 = componDetec.onInitialDetector();
    if(!flag1)
    {
        ROS_INFO("\nDetector Initialization Failed!");
        return -2;
    }

    ros::ServiceServer service = node.advertiseService("/piiq/components_vision", handle_detect_components);
    ROS_INFO("Ready to detect components ...");
    ros::spin();

    return 0;
}
