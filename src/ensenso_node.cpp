// ROS headers
#include <ros/ros.h>
#include <ros/service.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <std_srvs/Trigger.h>
// Conversions
#include <eigen_conversions/eigen_msg.h>

// Image transport
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL headers
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// Ensenso grabber
#include <ensenso/ensenso_grabber.h>
// Services
#include <ensenso/Lights.h>
#include <ensenso/CapturePattern.h>
#include <ensenso/ComputeCalibration.h>
#include <ensenso/ConfigureStreaming.h>
#include <ensenso/GridSpacing.h>
#include <ensenso/InitCalibration.h>
#include <ensenso/SetBool.h>
#include <ensenso/GetPC.h>
#include <ensenso/GetMono.h>
#include <ensenso/GetCalError.h>
#include <ensenso/UpdateOffset.h>


// Typedefs
typedef std::pair<pcl::PCLImage, pcl::PCLImage> PairOfImages;
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;


class EnsensoNode
{
  private:
    // ROS
    ros::NodeHandle                   nh_, nh_private_;
    ros::ServiceServer                calibrate_srv_;
    ros::ServiceServer                mono_calibrate_srv_;
    ros::ServiceServer                capture_srv_;
    ros::ServiceServer                mono_capture_srv_;
    ros::ServiceServer                checkcalib_srv_;
    ros::ServiceServer                grid_spacing_srv_;
    ros::ServiceServer                init_cal_srv_;
    ros::ServiceServer                init_mono_cal_srv_;
    ros::ServiceServer                lights_srv_;
    ros::ServiceServer                start_srv_;
    ros::ServiceServer                mono_start_srv_;
    ros::ServiceServer                single_pc_;
    ros::ServiceServer                single_pc_trigger_;
    ros::ServiceServer                triggered_pc_;
    ros::ServiceServer                single_mono_;
    ros::ServiceServer                configure_srv_;

    ros::ServiceServer                reset_link_srv_;
  
    // Images
    image_transport::CameraPublisher  l_raw_pub_;
    image_transport::CameraPublisher  r_raw_pub_;
    image_transport::CameraPublisher  mono_raw_pub_;

    image_transport::Publisher        mono_rectified_pub_;
    image_transport::Publisher        l_rectified_pub_;
    image_transport::Publisher        r_rectified_pub_;
    // Point cloud
    bool                              point_cloud_;
    ros::Publisher                    cloud_pub_;
    // Camera info
    ros::Publisher                    linfo_pub_;
    ros::Publisher                    rinfo_pub_;

    // TF
    std::string                       camera_frame_id_;
    std::string                       mono_camera_frame_id_;
    // Ensenso grabber
    boost::signals2::connection       connection_;
    boost::signals2::connection       connection2_;
    boost::signals2::connection       mono_connection_;
    pcl::EnsensoGrabber::Ptr          ensenso_ptr_;

  bool pc_camera_configuration, mono_camera_configuration;
  bool sim;
  
  public:
     EnsensoNode(): 
      nh_private_("~")
    {
      // Read parameters
      std::string serial, mono_serial;
      pc_camera_configuration = mono_camera_configuration = false;
      sim = false;
      
      nh_private_.param(std::string("sim"), sim, false);
      if (!nh_private_.hasParam("sim"))
        ROS_WARN_STREAM("Parameter [~sim] not found, using default: " << sim);

      nh_private_.param(std::string("serial"), serial, std::string("0"));
      if (!nh_private_.hasParam("serial"))
        ROS_WARN_STREAM("Parameter [~serial] not found, using default: " << serial);
      nh_private_.param(std::string("mono_serial"), mono_serial, std::string("0"));
      if (!nh_private_.hasParam("mono_serial"))
        ROS_WARN_STREAM("Parameter [~mono_serial] not found, using default: " << mono_serial);

      nh_private_.param("camera_frame_id", camera_frame_id_, std::string("ensenso_optical_frame"));
      if (!nh_private_.hasParam("camera_frame_id"))
        ROS_WARN_STREAM("Parameter [~camera_frame_id] not found, using default: " << camera_frame_id_);

      nh_private_.param("mono_camera_frame_id", mono_camera_frame_id_, std::string("ueye_optical_frame"));
      if (!nh_private_.hasParam("mono_camera_frame_id"))
        ROS_WARN_STREAM("Parameter [~mono_camera_frame_id] not found, using default: " << mono_camera_frame_id_);
      // Booleans
      // bool front_light, projector;
      // nh_private_.param("front_light", front_light, false);
      // if (!nh_private_.hasParam("front_light"))
      //   ROS_WARN_STREAM("Parameter [~front_light] not found, using default: " << front_light);
      // nh_private_.param("projector", projector, false);
      // if (!nh_private_.hasParam("projector"))
      //   ROS_WARN_STREAM("Parameter [~projector] not found, using default: " << projector);
      nh_private_.param("point_cloud", point_cloud_, true);
      if (!nh_private_.hasParam("point_cloud"))
        ROS_WARN_STREAM("Parameter [~point_cloud] not found, using default: " << point_cloud_);
      // Advertise topics
      image_transport::ImageTransport it(nh_);
      l_raw_pub_ = it.advertiseCamera("left/image_raw", 1);
      r_raw_pub_ = it.advertiseCamera("right/image_raw", 1);
      l_rectified_pub_ = it.advertise("left/image_rect", 1);
      r_rectified_pub_ = it.advertise("right/image_rect", 1);

      //      image_transport::ImageTransport it_mono(nh_);
      mono_raw_pub_ = it.advertiseCamera("/ueye/image_raw", 1);
      mono_rectified_pub_ = it.advertise("/ueye/image_rect", 1);
      
      cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2 >("depth/points", 1); 
      linfo_pub_=nh_.advertise<sensor_msgs::CameraInfo> ("left/camera_info", 1, true); // Latched
      rinfo_pub_=nh_.advertise<sensor_msgs::CameraInfo> ("right/camera_info", 1, true);

      bool started;

      do {
        started=true;
        ROS_WARN_STREAM_THROTTLE(1.0,"Trying to open uEye devices");
        
        // Initialize Ensenso
        ensenso_ptr_.reset(new pcl::EnsensoGrabber);
        //      ensenso_ptr_->openTcpPort();
        if (!sim) {
          if (serial != "0") {
            started &= ensenso_ptr_->openDevice(serial);
          }
          
          if (mono_serial != "0") {
            started &= ensenso_ptr_->mono_openDevice(mono_serial);
          }
          
          if (started)
            break;

          ROS_INFO_STREAM("Restarting ueyeethdrc service");

          ensenso_ptr_->closeTcpPort();
          ensenso_ptr_->closeDevices();

          std::string command="sudo service ueyeethdrc stop && sleep 1 && sudo service ueyeethdrc start && sleep 1";
          int rc = system(command.c_str());
          
          if (rc != 0) {
            ROS_FATAL("Could not initialize Ueye Ethernet service");
            return;
          }
          ros::Duration(0.5).sleep();
        }
      } while (!started);
        
      //      ensenso_ptr_->configureCapture();
      //      ensenso_ptr_->enableProjector(projector);
      //      ensenso_ptr_->enableFrontLight(front_light);
      // Start ensenso grabber
      ensenso::ConfigureStreaming::Request req;
      ensenso::ConfigureStreaming::Response res;
      req.cloud = point_cloud_;
      req.images = true;
      configureStreamingCB(req, res);

      mono_configureStreaming();
      //      ensenso_ptr_->start();
      // Advertise services
      capture_srv_ = nh_.advertiseService("stereo_capture_pattern", &EnsensoNode::capturePatternCB, this);
      mono_capture_srv_ = nh_.advertiseService("mono_capture_pattern", &EnsensoNode::mono_capturePatternCB, this);
      calibrate_srv_ = nh_.advertiseService("compute_stereo_calibration", &EnsensoNode::computeCalibrationCB, this);
      mono_calibrate_srv_ = nh_.advertiseService("compute_mono_calibration", &EnsensoNode::computeMonoCalibrationCB, this);
      checkcalib_srv_ =nh_.advertiseService("check_stereo_calibration", &EnsensoNode::checkCalibrationCB, this);
      init_cal_srv_ = nh_.advertiseService("init_stereo_calibration", &EnsensoNode::initCalibrationCB, this);
      init_mono_cal_srv_ = nh_.advertiseService("init_mono_calibration", &EnsensoNode::initMonoCalibrationCB, this);
      start_srv_ = nh_.advertiseService("start_stereo_streaming", &EnsensoNode::startStreamingCB, this);
      mono_start_srv_ = nh_.advertiseService("start_mono_streaming", &EnsensoNode::mono_startStreamingCB, this);
      single_pc_ = nh_.advertiseService("get_single_pc", &EnsensoNode::getSinglePCCB, this);
      single_pc_trigger_ = nh_.advertiseService("trigger_single_pc", &EnsensoNode::triggerSinglePCCB, this);
      triggered_pc_ = nh_.advertiseService("get_triggered_pc", &EnsensoNode::getTriggeredPCCB, this);
      single_mono_ = nh_.advertiseService("get_single_mono", &EnsensoNode::getSingleMonoCB, this);

      configure_srv_ = nh_.advertiseService("configure_streaming", &EnsensoNode::configureStreamingCB, this);
      grid_spacing_srv_ = nh_.advertiseService("grid_spacing", &EnsensoNode::gridSpacingCB, this);

      lights_srv_ = nh_.advertiseService("lights", &EnsensoNode::lightsCB, this);

      reset_link_srv_ = nh_.advertiseService("reset_mono_link", &EnsensoNode::mono_resetLink, this);

      mono_camera_configuration=false;
      pc_camera_configuration=false;

      pcl::PointCloud<pcl::PointXYZ> pc;
      ensenso_ptr_->grabSingleCloud(pc);
      pcl::PCLImage img;
      ensenso_ptr_->grabSingleMono(img);
      
    }
    
    ~EnsensoNode()
    {
      if (!sim) {
        ensenso_ptr_->closeTcpPort();
        ensenso_ptr_->closeDevices();
      }
    }
    
    bool lightsCB(ensenso::Lights::Request& req, ensenso::Lights::Response &res)
    {
      ensenso_ptr_->enableProjector(req.projector);
      ensenso_ptr_->enableFrontLight(req.front_light);
      res.success = true;
      return true;
    }

  
    bool capturePatternCB(ensenso::CapturePattern::Request& req, ensenso::CapturePattern::Response &res)
    {

      if (sim) {
        res.success = true;
        res.pattern_count = 1;
        return true;
      }
      
      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();
      if (req.clear_buffer)
      {
        if (!ensenso_ptr_->clearCalibrationPatternBuffer ()) {
          res.success = false;
          res.pattern_count = 0;
          return true;
        }
      }
      res.pattern_count = ensenso_ptr_->captureCalibrationPattern();
      res.success = (res.pattern_count > 0);
      if (res.success && res.pattern_count == 1)
        {
          // Pattern pose
          Eigen::Affine3d pattern_pose;
          double pose_error;
          ensenso_ptr_->estimateCalibrationPatternPose(pattern_pose, pose_error);
          tf::poseEigenToMsg(pattern_pose, res.pose);
          res.pose_error = pose_error;
        }
      return true;
    }

  bool mono_resetLink(ensenso::UpdateOffset::Request& req, ensenso::UpdateOffset::Response &res)
    {
      res.success = ensenso_ptr_->mono_resetCameraInfo(req.vector, req.write_to_device);
      return true;
    }
      
  
    bool mono_capturePatternCB(ensenso::CapturePattern::Request& req, ensenso::CapturePattern::Response &res)
    {
      if (sim) {
        res.success = true;
        res.pattern_count = 1;
        return true;
      }


      bool was_running = ensenso_ptr_->mono_isRunning();
      
      if (ensenso_ptr_->isRunning())
        ensenso_ptr_->stop();

      if (was_running)
        ensenso_ptr_->mono_stop();

      
      if (req.clear_buffer)
      {
        if (!ensenso_ptr_->clearCalibrationPatternBuffer ()) {
          ROS_ERROR("Couldn't clear calibration buffer");
          res.success = false;
          res.pattern_count = 0;
          return true;
        }
      }

      res.pattern_count = ensenso_ptr_->captureMonoCalibrationPattern();
      res.success = (res.pattern_count > 0);

      ROS_INFO_STREAM("PATTERN_COUNT "<<res.pattern_count);
      
      if (res.success  && res.pattern_count == 1)
      {
        // Pattern pose
        Eigen::Affine3d pattern_pose;
        double pose_error;
        ensenso_ptr_->mono_estimateCalibrationPatternPose(pattern_pose, pose_error);
        tf::poseEigenToMsg(pattern_pose, res.pose);
        res.pose_error = pose_error;
      }

      if (was_running)
        ensenso_ptr_->mono_start_up();

      
      return true;
    }

  
    bool checkCalibrationCB(ensenso::GetCalError::Request& req, ensenso::GetCalError::Response &res) {
      // Very important to stop the camera before performing the calibration
      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();

      ensenso_ptr_->getCalInfo();
      
      double error=0;
      res.success = ensenso_ptr_->checkCalibration(error);
      res.error=error;

      return true;
  }

  
    bool computeCalibrationCB(ensenso::ComputeCalibration::Request& req, ensenso::ComputeCalibration::Response &res)
    {

      if (sim) {
        res.success = true;
        res.result = req.seed;
        return true;
      }
      
      // Very important to stop the camera before performing the calibration
      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();
      std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > poses;
      for (size_t i = 0; i < req.robotposes.poses.size(); i++) {
        Eigen::Affine3d pose;
        tf::poseMsgToEigen(req.robotposes.poses[i], pose);
        poses.push_back(pose);
      }
      Eigen::Affine3d seed;
      tf::poseMsgToEigen(req.seed, seed);
      seed = seed.inverse();
      std::string result;
      double error;
      int iters;
      if (!ensenso_ptr_->computeCalibrationMatrix(poses, result, iters, error, "Moving", "Hand", seed))
        res.success = false;
      else {
        ROS_INFO("Calibration computation finished");
        // Populate the response
        res.success = true;
        
        
        double x,y,z,r,p,w;
        ensenso_ptr_->jsonTransformationToEulerAngles(result, x, y, z, r, p, w);
        x /= 1000;
        y /= 1000;
        z /= 1000;
        
        tf::Transform T;
        T.setOrigin(tf::Vector3(x,y,z));
        T.getBasis().setRPY(r,p,w);
        
        T=T.inverse();
        
        tf::poseTFToMsg(T, res.result);
        
        T.getBasis().getRPY(r,p,w);
        ROS_ERROR_STREAM("Euler answer:");
        ROS_INFO_STREAM("X,Y,Z: "<< T.getOrigin().x()<<" "<<T.getOrigin().y()<<" "<<T.getOrigin().z());
        ROS_INFO_STREAM("R,P,Y: "<<r<<" "<<p<<" "<<w);
        

        Eigen::Affine3d eigen_result;
        ensenso_ptr_->jsonTransformationToMatrix(result, eigen_result);
        eigen_result.translation () /= 1000.0;  // Convert translation to meters (Ensenso API returns milimeters)
        tf::poseEigenToTF(eigen_result, T);

        T=T.inverse();
        
        //tf::poseTFToMsg(T, res.result);
        
        T.getBasis().getRPY(r,p,w);
        ROS_ERROR_STREAM("Eigen answer:");
        ROS_INFO_STREAM("X,Y,Z: "<< T.getOrigin().x()<<" "<<T.getOrigin().y()<<" "<<T.getOrigin().z());
        ROS_INFO_STREAM("R,P,Y: "<<r<<" "<<p<<" "<<w);

        ROS_INFO_STREAM(res.result);
        
        res.reprojection_error = error;
        res.iterations = iters;
	//Ignore eeprom stuff with sterero to robot
        // if (req.store_to_eeprom)
        // {
        //   if (!ensenso_ptr_->clearEEPROMExtrinsicCalibration()) {
        //     ROS_WARN("Could not reset extrinsic calibration");
        //     res.success = false;
        //   }
        //   if (!ensenso_ptr_->storeEEPROMExtrinsicCalibration()) {
        //     ROS_WARN("Could not store new extrinsic calibration");
        //     res.success = false;
        //   }
        //   else
        //     ROS_INFO("Calibration stored into the EEPROM");
        // }
      }
      //      if (was_running)
      //        ensenso_ptr_->start();
      return true;
    }

    bool computeMonoCalibrationCB(ensenso::ComputeCalibration::Request& req, ensenso::ComputeCalibration::Response &res)
    {

      if (sim) {
        res.success = true;
        res.result = req.seed;
        return true;
      }

      // Very important to stop the camera before performing the calibration
      if (ensenso_ptr_->isRunning())
        ensenso_ptr_->stop();

      if (ensenso_ptr_->mono_isRunning())
        ensenso_ptr_->mono_stop();

      std::string result;
      if (!ensenso_ptr_->computeMonoCalibrationMatrix(result))
        res.success = false;
      else {
        ROS_INFO("Calibration computation finished");

        res.success = true;

        double x,y,z,r,p,w;
        ensenso_ptr_->jsonTransformationToEulerAngles(result, x,y,z,r,p,w);
        
        tf::Transform T;
        T.setOrigin(tf::Vector3(x/1000,y/1000,z/1000));
        T.getBasis().setRPY(r,p,w);
        
        T=T.inverse();
        
        tf::poseTFToMsg(T, res.result);
        
        T.getBasis().getRPY(r,p,w);
        ROS_INFO_STREAM("X,Y,Z: "<< T.getOrigin().x()<<" "<<T.getOrigin().y()<<" "<<T.getOrigin().z());
        ROS_INFO_STREAM("R,P,Y: "<<r<<" "<<p<<" "<<w);

        ROS_INFO_STREAM(res.result);
        
      }
      return true;
    }

  
    bool configureStreamingCB(ensenso::ConfigureStreaming::Request& req, ensenso::ConfigureStreaming::Response &res)
    {
      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();
      // Disconnect previous connection
      connection_.disconnect();
      connection2_.disconnect();
      // Connect new signals
      if (req.cloud)
      {
        boost::function<void(
          const boost::shared_ptr<PointCloudXYZ>&, 
          const boost::shared_ptr<PairOfImages>&)> f = boost::bind (&EnsensoNode::grabberCallback, this, _1, _2);
        connection_ = ensenso_ptr_->registerCallback(f);
      }
      if (req.images)
      {
        boost::function<void(
          const boost::shared_ptr<PairOfImages>&)> f = boost::bind (&EnsensoNode::grabberCallback, this, _1);
        connection2_ = ensenso_ptr_->registerCallback(f);
      }

      //      if (was_running)
      //        ensenso_ptr_->start();
      res.success = true;
      return true;
    }

  bool mono_configureStreaming()
    {
      bool was_running = ensenso_ptr_->mono_isRunning();
      if (was_running)
        ensenso_ptr_->mono_stop();
      // Disconnect previous connection
      mono_connection_.disconnect();
      // Connect new signals

      boost::function<void(const boost::shared_ptr<pcl::PCLImage>&,
                           const boost::shared_ptr<pcl::PCLImage>&)> f = boost::bind (&EnsensoNode::mono_grabberCallback, this, _1, _2);
      mono_connection_ = ensenso_ptr_->registerCallback(f);

      return true;
    }

  
    bool gridSpacingCB (ensenso::GridSpacing::Request& req, ensenso::GridSpacing::Response &res)
    {
      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();
      res.grid_spacing = ensenso_ptr_->getPatternGridSpacing();
      res.success = (res.grid_spacing > 0);
      //      if (was_running)
      //        ensenso_ptr_->start();
      return true;
    }
    
    bool initCalibrationCB(ensenso::InitCalibration::Request& req, ensenso::InitCalibration::Response &res)
    {
      if (sim) {
        res.used_grid_spacing = req.grid_spacing;
        res.success = true;
        return true;
      }
      
      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();

      // The grid_spacing value in the request has preference over the decode one

      pc_camera_configuration = false;
      if (!ensenso_ptr_->configureCapture (1, true, false, 1, 1, false, 4, false, true, true, 68, false, 80, "Software", true)) {
        res.success = false;
        return true;
      }
      
      
      double spacing;
      if (req.grid_spacing > 0)
        spacing = req.grid_spacing;
      else
        spacing = ensenso_ptr_->getPatternGridSpacing();
        
      if (spacing > 0)
      {
        res.success = ensenso_ptr_->initExtrinsicCalibration(spacing);
        res.used_grid_spacing = spacing;
      }
      else
        res.success = false;
      //      if (was_running)
        //        ensenso_ptr_->start();
      return true;
    }

    bool initMonoCalibrationCB(ensenso::InitCalibration::Request& req, ensenso::InitCalibration::Response &res)
    {

      if (sim) {
        res.used_grid_spacing = req.grid_spacing;
        res.success = true;
        return true;
      }

      if (ensenso_ptr_->isRunning())
        ensenso_ptr_->stop();

      
      bool was_running = ensenso_ptr_->mono_isRunning();
      if (was_running)
        ensenso_ptr_->mono_stop();


      // The grid_spacing value in the request has preference over the decode one

      pc_camera_configuration = false;
      if (!ensenso_ptr_->configureCapture (1, true, false, 1, 1, false, 4, false, true, true, 68, false, 80, "Software", true)) {
        res.success = false;
        return true;
      }

      mono_camera_configuration = false;
      if (!ensenso_ptr_->mono_configureCapture ()){ //(true, true, 1, 1, 4, 43, 80, "Software")
        res.success = false;
        return true;
      }

      
      double spacing;
      if (req.grid_spacing > 0)
        spacing = req.grid_spacing;
      else
        spacing = ensenso_ptr_->getPatternGridSpacing();
        
      if (spacing > 0)
      {
        res.success = ensenso_ptr_->initMonoCalibration(spacing);
        res.used_grid_spacing = spacing;
      }
      else
        res.success = false;

      if (was_running)
        ensenso_ptr_->mono_stop();
            
      return true;
    }

  
    bool startStreamingCB(ensenso::SetBool::Request& req, ensenso::SetBool::Response &res)
    {
      if (req.data) {
        if (ensenso_ptr_->isRunning()) {
          res.success=true;
          return true;
        }
        if (!ensenso_ptr_->configureCapture()) {
          res.success=false;
          return true;
        }
        pc_camera_configuration = true;
        res.success = ensenso_ptr_->start_up();
      }
      else {
        ensenso_ptr_->stop();
        res.success = true;
      }
      return true;
    }

    bool mono_startStreamingCB(ensenso::SetBool::Request& req, ensenso::SetBool::Response &res)
    {
      
      if (req.data) {
        if (ensenso_ptr_->mono_isRunning()) {
          res.success=true;
          return true;
        }
        if (!ensenso_ptr_->mono_configureCapture()) {
          res.success=false;
          return true;
        }
        mono_camera_configuration = true;
        res.success = ensenso_ptr_->mono_start_up();
      }
      else {
        ensenso_ptr_->mono_stop();
        res.success = true;
      }

      return true;
    }

  
    bool getSinglePCCB(ensenso::GetPC::Request& req, ensenso::GetPC::Response &res)
    {
      ros::Time  start = ros::Time::now();
      pcl::PointCloud<pcl::PointXYZ> pc;

      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();

      
      if (!pc_camera_configuration) {
        if (!ensenso_ptr_->configureCapture()) {
          res.success=false;
          return true;
        }
        pc_camera_configuration = true;
      }
      
      res.success = ensenso_ptr_->grabSingleCloud(pc);

      if (res.success) {
        pc.header.frame_id = camera_frame_id_;
        pcl::toROSMsg(pc, res.cloud);
        cloud_pub_.publish(res.cloud);
      }
      res.time = (ros::Time::now()-start).toSec();
      
      return true;
    }


  bool triggerSinglePCCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response &res)
    {

      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();

      
      if (!pc_camera_configuration) {
        if (!ensenso_ptr_->configureCapture()) {
          res.success=false;
          return true;
        }
        pc_camera_configuration = true;
      }
      
      res.success = ensenso_ptr_->triggerStereoImage();
      
      return true;
    }


    bool getTriggeredPCCB(ensenso::GetPC::Request& req, ensenso::GetPC::Response &res)
    {
      ros::Time  start = ros::Time::now();

      pcl::PointCloud<pcl::PointXYZ> pc;
      res.success = ensenso_ptr_->grabTriggeredPC(pc);

      if (res.success) {
        pc.header.frame_id = camera_frame_id_;
        pcl::toROSMsg(pc, res.cloud);
        cloud_pub_.publish(res.cloud);
      }
      res.time = (ros::Time::now()-start).toSec();
      
      return true;
    }

  
    bool getSingleMonoCB(ensenso::GetMono::Request& req, ensenso::GetMono::Response &res)
    {
      ros::Time  start = ros::Time::now();

      pcl::PCLImage rectimage;

      bool was_running = ensenso_ptr_->mono_isRunning();
      if (was_running)
        ensenso_ptr_->mono_stop();
      
      if (!mono_camera_configuration) {
        if (!ensenso_ptr_->mono_configureCapture()) {
          res.success=false;
          return true;
        }
        mono_camera_configuration = true;
      }
      
      res.success = ensenso_ptr_->grabSingleMono(rectimage);
      
      if (res.success) {
        
        res.image = *toImageMsg(rectimage);
        mono_rectified_pub_.publish(res.image);
      }
      res.time = (ros::Time::now()-start).toSec();
      return true;
    }

  

  void mono_grabberCallback( const boost::shared_ptr<pcl::PCLImage>& rawimage,
                             const boost::shared_ptr<pcl::PCLImage>& rectimage)
  {
      // Get cameras info
      sensor_msgs::CameraInfo info;
      ensenso_ptr_->mono_getCameraInfo(info);
      info.header.frame_id = mono_camera_frame_id_;

      // Images

      sensor_msgs::Image img = *toImageMsg(*rawimage);
      info.header.stamp = img.header.stamp;
      mono_raw_pub_.publish(img, info);
      if (!rectimage->data.empty())
        mono_rectified_pub_.publish(*toImageMsg(*rectimage));
      
    }

  
  void grabberCallback( const boost::shared_ptr<PairOfImages>& rawimages)
    {
      // Get cameras info
      sensor_msgs::CameraInfo linfo, rinfo;
      ensenso_ptr_->getCameraInfo("Left", linfo);
      ensenso_ptr_->getCameraInfo("Right", rinfo);
      linfo.header.frame_id = camera_frame_id_;
      rinfo.header.frame_id = camera_frame_id_;
      // Images
      sensor_msgs::Image img = *toImageMsg(rawimages->first);
      linfo.header.stamp = img.header.stamp;
      l_raw_pub_.publish(img, linfo);
      if (!rawimages->second.data.empty()) {
        img = *toImageMsg(rawimages->second);
        rinfo.header.stamp = img.header.stamp;
        r_raw_pub_.publish(img, rinfo);
      }
    }


    void grabberCallback( const boost::shared_ptr<PointCloudXYZ>& cloud,
                          const boost::shared_ptr<PairOfImages>& rectifiedimages)
    {
      // Get cameras info
      sensor_msgs::CameraInfo linfo, rinfo;
      ensenso_ptr_->getCameraInfo("Left", linfo);
      ensenso_ptr_->getCameraInfo("Right", rinfo);
      linfo.header.frame_id = camera_frame_id_;
      rinfo.header.frame_id = camera_frame_id_;
      // Images

      sensor_msgs::Image img = *toImageMsg(rectifiedimages->first);
      linfo.header.stamp = img.header.stamp;
      l_rectified_pub_.publish(img);
      
      img = *toImageMsg(rectifiedimages->second);
      rinfo.header.stamp = img.header.stamp;
      r_rectified_pub_.publish(img);
      // Camera_info
      linfo_pub_.publish(linfo);
      rinfo_pub_.publish(rinfo);
      // Point cloud
      cloud->header.frame_id = camera_frame_id_;
      sensor_msgs::PointCloud2 cloud_msg;
      pcl::toROSMsg(*cloud, cloud_msg);
      cloud_pub_.publish(cloud_msg);
    }

  
    sensor_msgs::ImagePtr toImageMsg(pcl::PCLImage pcl_image)
    {
      unsigned char *image_array = reinterpret_cast<unsigned char *>(&pcl_image.data[0]);
      int type(CV_8UC1);
      std::string encoding("mono8");
      if (pcl_image.encoding == "CV_8UC3")
      {
        type = CV_8UC3;
        encoding = "bgr8";
      }
      cv::Mat image_mat(pcl_image.height, pcl_image.width, type, image_array);
      cv::Mat new_image;
      std_msgs::Header header;
      header.frame_id = "";
      pcl_conversions::fromPCL(pcl_image.header.stamp,header.stamp);
      //      cv::flip(image_mat, new_image, -1);
      image_mat.copyTo(new_image);
      return cv_bridge::CvImage(header, encoding, new_image).toImageMsg();
    }
};


int main(int argc, char **argv)
{
  ros::init (argc, argv, "ensenso");
  EnsensoNode ensenso_node;
  ros::spin();
  ros::shutdown();
  return 0;
}
