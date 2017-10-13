// Notices Pose error: Stereo: 0.8 mm ; Mono to Stereo: 2.3mm


#include <pcl/pcl_config.h>
#include <pcl/exceptions.h>
#include <pcl/common/io.h>
#include <pcl/console/print.h>
#include <pcl/point_types.h>
#include <boost/format.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include "ensenso/ensenso_grabber.h"

void ensensoExceptionHandling (const NxLibException &ex,
                               std::string func_nam)
{
  PCL_ERROR ("%s: NxLib error %s (%d) occurred while accessing item %s.\n", func_nam.c_str (), ex.getErrorText ().c_str (), ex.getErrorCode (), ex.getItemPath ().c_str ());
  if (ex.getErrorCode () == NxLibExecutionFailed)
    {
      NxLibCommand cmd ("");
      PCL_WARN ("\n%s\n", cmd.result ().asJson (true, 4, false).c_str ());
    }
}


pcl::EnsensoGrabber::EnsensoGrabber () :
  device_open_ (false),
  mono_device_open_(false),
  tcp_open_ (false),
  running_ (false),
  mono_running_ (false)
{
  raw_images_signal_ = createSignal<sig_cb_ensenso_raw_images> ();
  point_cloud_images_signal_ = createSignal<sig_cb_ensenso_point_cloud_images> ();

  mono_images_signal_ = createSignal<sig_cb_mono_images> ();
  
  PCL_INFO ("Initialising nxLib\n");

  try
  {
    nxLibInitialize ();
    root_.reset (new NxLibItem);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "EnsensoGrabber");
    ROS_FATAL("Could not initialise NxLib.");  // If constructor fails; throw exception
  }
}

pcl::EnsensoGrabber::~EnsensoGrabber () throw ()
{
  try
  {
    stop ();
    mono_stop();
    root_.reset ();

    disconnect_all_slots<sig_cb_ensenso_raw_images> ();
    disconnect_all_slots<sig_cb_ensenso_point_cloud_images> ();
    disconnect_all_slots<sig_cb_mono_images> ();
    
    if (tcp_open_)
      closeTcpPort ();
    nxLibFinalize ();
  }
  catch (...)
  {
    // destructor never throws
  }
}

int pcl::EnsensoGrabber::enumDevices () const
{
  int camera_count = 0;

  try
  {
    NxLibItem cams = NxLibItem ("/Cameras/BySerialNo");
    camera_count = cams.count ();

    // Print information for all cameras in the tree
    PCL_INFO ("Number of connected cameras: %d\n", camera_count);
    PCL_INFO ("Serial No    Model   Status\n");

    for (int n = 0; n < cams.count (); ++n)
    {
      PCL_INFO ("%s   %s   %s\n", cams[n][itmSerialNumber].asString ().c_str (),
            cams[n][itmModelName].asString ().c_str (),
            cams[n][itmStatus].asString ().c_str ());
    }
    PCL_INFO ("\n");
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "enumDevices");
  }
  return (camera_count);
}

bool pcl::EnsensoGrabber::openDevice (std::string serial_no)
{
  if (device_open_) {
   ROS_ERROR("Cannot open multiple devices!");
   return false;
  }
  PCL_INFO ("Opening Ensenso stereo camera S/N: %s\n", serial_no.c_str());

  ros::Time start_time = ros::Time::now();
  bool isOpen = false;
  NxLibException exception("None",0);
  try
    {
      // Create a pointer referencing the camera's tree item, for easier access:
      camera_ = (*root_)[itmCameras][itmBySerialNo][serial_no];
        
      if (!camera_.exists () || camera_[itmType] != valStereo)
        {
          ROS_FATAL_STREAM("Stereo camera "<<serial_no<<" not found on system");
        }

      while (!camera_[itmStatus][itmAvailable].asBool()) {
        ROS_WARN_STREAM_THROTTLE(1,"Camera "<<serial_no<<" exists but is not available.  Trying again.");
      }
      
      do {
        NxLibCommand open (cmdOpen);
        open.parameters ()[itmCameras] = camera_[itmSerialNumber].asString ();
        open.execute ();
        serial_=serial_no;
        isOpen=camera_[itmStatus][itmOpen].asBool();
        if (!isOpen)
          ROS_WARN_STREAM_THROTTLE(1,"Having trouble opening camera "<<serial_no<<".  Trying again.");
      } while (!isOpen);
    }
  catch (NxLibException &ex)
    {
      exception = ex;
    }
  
  if (!isOpen) {
    ensensoExceptionHandling (exception, "openDevice");
    return (false);
  }

  ROS_INFO_STREAM("Camera "<<serial_no<<" ready for use");
  device_open_ = true;
  return (true);
}

bool pcl::EnsensoGrabber::mono_openDevice (std::string serial_no)
{
  if (mono_device_open_) {
   ROS_ERROR("Cannot open multiple devices!");
   return false;
  }
  PCL_INFO ("Opening Ueye Monocular camera S/N: %s\n", serial_no.c_str());

  ros::Time start_time = ros::Time::now();
  bool isOpen = false;
  NxLibException exception("None",0);
  try
    {
      // Create a pointer referencing the camera's tree item, for easier access:
      mono_camera_ = (*root_)[itmCameras][itmBySerialNo][serial_no];
        
      if (!mono_camera_.exists () || mono_camera_[itmType] != valMonocular)
        {
          ROS_FATAL_STREAM("Monocular camera "<<serial_no<<" not found on system");
        }

      while (!mono_camera_[itmStatus][itmAvailable].asBool()) {
        ROS_WARN_STREAM_THROTTLE(1,"Camera "<<serial_no<<" exists but is not available.  Trying again.");
      }
      
      do {
        NxLibCommand open (cmdOpen);
        open.parameters ()[itmCameras] = mono_camera_[itmSerialNumber].asString ();
        open.parameters ()[itmLoadCalibration] = true;
        open.execute ();
        mono_serial_=serial_no;
        isOpen=mono_camera_[itmStatus][itmOpen].asBool();
        if (!isOpen)
          ROS_WARN_STREAM_THROTTLE(1,"Having trouble opening camera "<<serial_no<<".  Trying again.");
      } while (!isOpen);
    }
  catch (NxLibException &ex)
    {
      exception = ex;
    }
  
  if (!isOpen) {
    ensensoExceptionHandling (exception, "mono_openDevice");
    return (false);
  }

  ROS_INFO_STREAM("Camera "<<serial_no<<" ready for use");
  mono_device_open_ = true;
  return (true);
}


bool pcl::EnsensoGrabber::closeDevices ()
{
  stop ();
  mono_stop();
  PCL_INFO ("Closing all nxLibrary cameras\n");

  try
  {
    NxLibCommand (cmdClose).execute ();
    device_open_ = false;
    mono_device_open_=false;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "closeDevices");
    return (false);
  }
  return (true);
}



bool pcl::EnsensoGrabber::start_up ()
{
  if (isRunning ())
    return true;

  if (!device_open_)
    return false;

  times_.clear();
  running_ = true;
  raw_initialized_=false;
  raw_thread_ = boost::thread (&pcl::EnsensoGrabber::processRaw, this);
  points_thread_ = boost::thread (&pcl::EnsensoGrabber::processPoints, this);
  return true;
}

bool pcl::EnsensoGrabber::mono_start_up ()
{
  if (mono_isRunning ())
    return true;

  if (!mono_device_open_)
    return false;

  mono_running_ = true;
  mono_thread_ = boost::thread (&pcl::EnsensoGrabber::processMono, this);
  return true;
}


void pcl::EnsensoGrabber::stop ()
{
  if (running_)
  {
    running_ = false;  // Stop processGrabbing () callback
    raw_thread_.join ();
    points_thread_.join ();
  }
}

void pcl::EnsensoGrabber::mono_stop ()
{
  if (mono_running_)
  {
    mono_running_ = false;  // Stop processGrabbing () callback
    mono_thread_.join ();
  }
}


bool pcl::EnsensoGrabber::isRunning () const
{
  return (running_);
}

bool pcl::EnsensoGrabber::mono_isRunning () const
{
  return (mono_running_);
}

bool pcl::EnsensoGrabber::isTcpPortOpen () const
{
  return (tcp_open_);
}

std::string pcl::EnsensoGrabber::getName () const
{
  return ("EnsensoGrabber");
}

bool pcl::EnsensoGrabber::configureCapture(const uint flexview,
                                           const bool auto_exposure,
                                           const bool auto_gain,
                                           const int bining,
                                           const float exposure,
                                           const bool front_light,
                                           const int gain,
                                           const bool gain_boost,
                                           const bool hardware_gamma,
                                           const bool hdr,
                                           const int pixel_clock,
                                           const bool projector,
                                           const int target_brightness,
                                           const std::string trigger_mode,
                                           const bool use_disparity_map_area_of_interest) const
{
  if (!device_open_)
    return (false);

  try
  {
    NxLibItem captureParams = camera_[itmParameters][itmCapture];
    captureParams[itmAutoExposure] = auto_exposure;
    captureParams[itmAutoGain] = auto_gain;
    captureParams[itmFlexView] = (int)flexview;
    captureParams[itmBinning] = bining;
    if (!auto_exposure)
      captureParams[itmExposure] = exposure;
    captureParams[itmFrontLight] = front_light;
    if (!auto_gain)
      captureParams[itmGain] = gain;
    captureParams[itmGainBoost] = gain_boost;
    captureParams[itmHardwareGamma] = hardware_gamma;
    captureParams[itmHdr] = hdr;
    captureParams[itmPixelClock] = pixel_clock;
    captureParams[itmProjector] = projector;
    captureParams[itmTargetBrightness] = target_brightness;
    captureParams[itmTriggerMode] = trigger_mode;
    captureParams[itmUseDisparityMapAreaOfInterest] = use_disparity_map_area_of_interest;

   
    int minDisp = -41;  //Max distance: 1.5m
    int numDisps = 96; //Min distance: 0.63m 

    NxLibItem stereoMatching = camera_[itmParameters][itmDisparityMap][itmStereoMatching];
 
    stereoMatching[itmMinimumDisparity] = minDisp; // set minimum disparity to desired value
    stereoMatching[itmNumberOfDisparities] = numDisps; // set number of disparity to desired value
    stereoMatching[itmShadowingThreshold] = 2; // I think
    stereoMatching[itmPadding] = false;

    NxLibItem postProcessing = camera_[itmParameters][itmDisparityMap][itmPostProcessing];

    postProcessing[itmUniquenessRatio]=0;

    postProcessing[itmMedianFilterRadius]=2; // Not sure about this
    
    postProcessing[itmSpeckleRemoval][itmComponentThreshold]=1;
    postProcessing[itmSpeckleRemoval][itmRegionSize]=1000;

    postProcessing[itmFilling][itmRegionSize]=0; //off

    //    NxLibCommand (cmdCapture).execute ();
    
    // NxLibCommand estimateDisaritySettings(cmdEstimateDisparitySettings);
    // estimateDisaritySettings.execute();

    // minDisp = estimateDisaritySettings.result()[serial_][itmMinimumDisparity].asInt();
    // numDisps = estimateDisaritySettings.result()[serial_][itmNumberOfDisparities].asInt();
 
    // ROS_INFO_STREAM("Ensenso firmware thinks good minimum/number of disparities are: "<<minDisp<<" & "<<numDisps);

    // stereoMatching[itmMinimumDisparity] = minDisp; // set minimum disparity to desired value
    // stereoMatching[itmNumberOfDisparities] = numDisps; // set number of disparity to desired value

    NxLibCommand capture(cmdCapture);
    capture.parameters()[itmCameras] = serial_;
    capture.execute ();

  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "configureCapture");
    return (false);
  }
  return (true);
}


bool pcl::EnsensoGrabber::mono_configureCapture(
                                           const bool auto_exposure,
                                           const bool auto_gain,
                                           const int bining,
                                           const float exposure,
                                           const int gain,
                                           const int pixel_clock,
                                           const int target_brightness,
                                           const std::string trigger_mode) const

{
  if (!mono_device_open_)
    return (false);

  try
  {
    NxLibItem captureParams = mono_camera_[itmParameters][itmCapture];
    captureParams[itmAutoExposure] = auto_exposure;
    captureParams[itmAutoGain] = auto_gain;
    captureParams[itmBinning] = bining;
    if (!auto_exposure)
      captureParams[itmExposure] = exposure;
    if (!auto_gain)
      captureParams[itmGain] = gain;
    captureParams[itmPixelClock] = pixel_clock;
    captureParams[itmTargetBrightness] = target_brightness;
    captureParams[itmTriggerMode] = trigger_mode;   
    
    NxLibCommand capture(cmdCapture);
    capture.parameters()[itmCameras] = mono_serial_;
    capture.parameters()[itmTimeout] = 3000;
    capture.execute ();


  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "mono_configureCapture");
    return false;
  }
  return (true);
}


bool pcl::EnsensoGrabber::grabSingleMono (pcl::PCLImage& image)
{
  if (!mono_device_open_)
    return (false);

  //  if (running_)
  //    return (false);

  
  for (uint i=0; i<3; i++) {
    try
      {
        
        NxLibCommand capture(cmdCapture);
        capture.parameters ()[itmCameras] = mono_serial_;
        capture.parameters()[itmTimeout] = 3000;
        capture.execute ();
        
        
        NxLibCommand rect(cmdRectifyImages);
        rect.parameters ()[itmCameras] = mono_serial_;
        rect.execute ();
        
        
        int width, height, channels, bpe;
        bool isFlt;
        
        double timestamp;
        mono_camera_[itmImages][itmRectified].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);          
        mono_camera_[itmImages][itmRectified].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
        image.header.stamp = getPCLStamp (timestamp);
        image.width = width;
        image.height = height;
        image.data.resize (width * height * sizeof(float));
        image.encoding = getOpenCVType (channels, bpe, isFlt);
        
        mono_camera_[itmImages][itmRectified].getBinaryData (image.data.data (), image.data.size (), 0, 0);
        
        return true;
      }
    catch (NxLibException &ex)
      {
        ROS_ERROR_STREAM("Ensenso: Grab Single Mono threw exception: "<<ex.getErrorText());
        ensensoExceptionHandling (ex, "grabSingleMono");
      }
  }
  return false;
}


bool pcl::EnsensoGrabber::grabSingleCloud (pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PCLImage& image)
{
  if (!device_open_)
    return (false);

  //  if (running_)
  //    return (false);

  try
  {
    NxLibCommand capture(cmdCapture);
    capture.parameters ()[itmCameras] = serial_;
    capture.execute ();
    // Stereo matching task
    NxLibCommand (cmdComputeDisparityMap).execute ();
    // Convert disparity map into XYZ data for each pixel
    NxLibCommand (cmdComputePointMap).execute ();
    // Get info about the computed point map and copy it into a std::vector
    double timestamp;
    std::vector<float> pointMap;
    camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);  // Get raw image timestamp

    int width, height, channels, bpe;
    bool isFlt;
        
    camera_[itmImages][itmDisparityMap].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
    image.header.stamp = getPCLStamp (timestamp);
    image.width = width;
    image.height = height;
    image.data.resize (width * height * sizeof(float));
    image.encoding = "CV_16SC1"; //getOpenCVType (channels, bpe, isFlt);
    camera_[itmImages][itmDisparityMap].getBinaryData (image.data.data (), image.data.size (), 0, 0);

    camera_[itmImages][itmPointMap].getBinaryDataInfo (&width, &height, 0, 0, 0, 0);
    camera_[itmImages][itmPointMap].getBinaryData (pointMap, 0);
    // Copy point cloud and convert in meters
    cloud.header.stamp = getPCLStamp (timestamp);
    cloud.resize (height * width);
    cloud.width = width;
    cloud.height = height;
    cloud.is_dense = false;
    // Copy data in point cloud (and convert milimeters in meters)
    for (size_t i = 0; i < pointMap.size (); i += 3)
    {
      cloud.points[i / 3].x = pointMap[i] / 1000.0;
      cloud.points[i / 3].y = pointMap[i + 1] / 1000.0;
      cloud.points[i / 3].z = pointMap[i + 2] / 1000.0;
    }
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "grabSingleCloud");
    return (false);
  }
}



bool pcl::EnsensoGrabber::triggerStereoImage ()
{
  if (!device_open_)
    return (false);

  //  if (running_)
  //    return (false);

  try
  {
    NxLibCommand capture(cmdCapture);
    capture.parameters ()[itmCameras] = serial_;
    capture.execute ();
  }
  catch (NxLibException &ex)
    {
      ensensoExceptionHandling (ex, "triggerStereoImage");
      return (false);
    }
  return true;
}


bool pcl::EnsensoGrabber::grabTriggeredPC (pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PCLImage& image)
{
  if (!device_open_)
    return (false);

  //  if (running_)
  //    return (false);

  try
  {
    // Stereo matching task
    NxLibCommand (cmdComputeDisparityMap).execute ();
    // Convert disparity map into XYZ data for each pixel
    NxLibCommand (cmdComputePointMap).execute ();
    // Get info about the computed point map and copy it into a std::vector
    double timestamp;
    std::vector<float> pointMap;

    int width, height, channels, bpe;
    bool isFlt;
          
          
    camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);  // Get raw image timestamp

    camera_[itmImages][itmDisparityMap].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
    image.header.stamp = getPCLStamp (timestamp);
    image.width = width;
    image.height = height;
    image.data.resize (width * height * sizeof(float));
    image.encoding = "CV_16SC1"; //getOpenCVType (channels, bpe, isFlt);
    camera_[itmImages][itmDisparityMap].getBinaryData (image.data.data (), image.data.size (), 0, 0);

    camera_[itmImages][itmPointMap].getBinaryDataInfo (&width, &height, 0, 0, 0, 0);
    camera_[itmImages][itmPointMap].getBinaryData (pointMap, 0);
    
    // Copy point cloud and convert in meters
    cloud.header.stamp = getPCLStamp (timestamp);
    cloud.resize (height * width);
    cloud.width = width;
    cloud.height = height;
    cloud.is_dense = false;
    // Copy data in point cloud (and convert milimeters in meters)
    for (size_t i = 0; i < pointMap.size (); i += 3)
    {
      cloud.points[i / 3].x = pointMap[i] / 1000.0;
      cloud.points[i / 3].y = pointMap[i + 1] / 1000.0;
      cloud.points[i / 3].z = pointMap[i + 2] / 1000.0;
    }
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "grabTriggeredPC");
    return (false);
  }
}




bool pcl::EnsensoGrabber::initExtrinsicCalibration (const double grid_spacing) const
{
  if (!device_open_)
    return (false);

  if (running_)
    return (false);

  try
  {
    if (!clearCalibrationPatternBuffer ())
      return (false);
    (*root_)[itmParameters][itmPattern][itmGridSpacing] = grid_spacing;
    // GridSize can't be changed, it's protected in the tree
    // With the speckle projector on it is nearly impossible to recognize the pattern
    // (the 3D calibration is based on stereo images, not on 3D depth map)
    
    // Most important parameters are: projector=off, front_light=on
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "initExtrinsicCalibration");
    return (false);
  }
  return (true);
}


bool pcl::EnsensoGrabber::initMonoCalibration (const double grid_spacing) const
{
  if (!device_open_ || !mono_device_open_)
    return (false);

  if (running_ || mono_running_)
    return (false);

  try
  {
    if (!clearCalibrationPatternBuffer ())
      return (false);
    (*root_)[itmParameters][itmPattern][itmGridSpacing] = grid_spacing;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "initExtrinsicCalibration");
    return (false);
  }
  return (true);
}


bool pcl::EnsensoGrabber::clearCalibrationPatternBuffer () const
{
  if (!device_open_)
    return (false);

  if (running_)
    return (false);
  try
  {
    NxLibCommand (cmdDiscardPatterns).execute ();
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "clearCalibrationPatternBuffer");
    return (false);
  }
  return (true);
}




int pcl::EnsensoGrabber::captureCalibrationPattern () const
{
  if (!device_open_)
    return (-1);

  if (running_)
    return (-1);

  try
  {
    NxLibCommand capture(cmdCapture);
    capture.parameters()[itmCameras] = serial_;
    capture.execute ();

    if (num_slots<sig_cb_ensenso_raw_images>()>0) {

      boost::shared_ptr<PairOfImages> rawimages (new PairOfImages);
      
      int width, height, channels, bpe;
      bool isFlt;
      
      double timestamp;
      camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);
      
      camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
      rawimages->first.header.stamp = rawimages->second.header.stamp = getPCLStamp (timestamp);
      rawimages->first.width = rawimages->second.width = width;
      rawimages->first.height = rawimages->second.height = height;
      rawimages->first.data.resize (width * height * sizeof(float));
      rawimages->second.data.resize (width * height * sizeof(float));
      rawimages->first.encoding = rawimages->second.encoding = getOpenCVType (channels, bpe, isFlt);
      camera_[itmImages][itmRaw][itmLeft].getBinaryData (rawimages->first.data.data (), rawimages->first.data.size (), 0, 0);
      camera_[itmImages][itmRaw][itmRight].getBinaryData (rawimages->second.data.data (), rawimages->second.data.size (), 0, 0);
      raw_images_signal_->operator () (rawimages);
    }    

  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "captureCalibrationPattern");
    return (-1);
  }

  try{
    NxLibCommand collect(cmdCollectPattern);
    collect.parameters()[itmCameras] = serial_;
    collect.parameters()[itmDecodeData] = false;
    collect.parameters()[itmRefinement] = valNone;
    collect.parameters()[itmReturnAllPattern] = true;
    collect.parameters()[itmFilter][itmCameras] = serial_;
    collect.parameters()[itmFilter][itmType] = valStatic;
    collect.parameters()[itmFilter][itmValue] = true;
    collect.execute();

    if (num_slots<sig_cb_ensenso_raw_images>()>0) {

      boost::shared_ptr<PairOfImages> rawimages (new PairOfImages);
      
      int width, height, channels, bpe;
      bool isFlt;
      
      double timestamp;
      camera_[itmImages][itmWithOverlay][itmLeft].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);
      
      camera_[itmImages][itmWithOverlay][itmLeft].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
      rawimages->first.header.stamp = rawimages->second.header.stamp = getPCLStamp (timestamp);
      rawimages->first.width = rawimages->second.width = width;
      rawimages->first.height = rawimages->second.height = height;
      rawimages->first.data.resize (width * height * sizeof(float));
      rawimages->second.data.resize (width * height * sizeof(float));
      rawimages->first.encoding = rawimages->second.encoding = getOpenCVType (channels, bpe, isFlt);
      camera_[itmImages][itmWithOverlay][itmLeft].getBinaryData (rawimages->first.data.data (), rawimages->first.data.size (), 0, 0);
      camera_[itmImages][itmWithOverlay][itmRight].getBinaryData (rawimages->second.data.data (), rawimages->second.data.size (), 0, 0);
      raw_images_signal_->operator () (rawimages);
    }    
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "captureCalibrationPattern");
    return (-1);
  }

  return ( (*root_)[itmParameters][itmPatternCount].asInt ());
}


int pcl::EnsensoGrabber::captureMonoCalibrationPattern () const
{
  if (!device_open_ || !mono_device_open_)
    return (-1);

  if (running_ || mono_running_)
    return (-1);

  std::string devices[2]={serial_,mono_serial_};
  try
  {
    NxLibCommand capture (cmdCapture);

    capture.parameters()[itmCameras][0] = devices[0];
    capture.parameters()[itmCameras][1] = devices[1];
    capture.parameters()[itmTimeout] = 3000;
    capture.execute();

    if (num_slots<sig_cb_ensenso_raw_images>()>0) {

      boost::shared_ptr<PairOfImages> rawimages (new PairOfImages);
      
      int width, height, channels, bpe;
      bool isFlt;
      
      double timestamp;
      camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);
      
      camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
      rawimages->first.header.stamp = getPCLStamp (timestamp);
      rawimages->first.width = width;
      rawimages->first.height = height;
      rawimages->first.data.resize (width * height * sizeof(float));
      rawimages->first.encoding = getOpenCVType (channels, bpe, isFlt);
      camera_[itmImages][itmRaw][itmLeft].getBinaryData (rawimages->first.data.data (), rawimages->first.data.size (), 0, 0);
      raw_images_signal_->operator () (rawimages);
    }

    if (num_slots<sig_cb_mono_images>()>0) {
          
      boost::shared_ptr<pcl::PCLImage> rawimage (new pcl::PCLImage);
      boost::shared_ptr<pcl::PCLImage> rectimage (new pcl::PCLImage);
      int width, height, channels, bpe;
      bool isFlt;
      
      double timestamp;
      mono_camera_[itmImages][itmRaw].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);
      
      mono_camera_[itmImages][itmRaw].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
      rawimage->header.stamp = getPCLStamp (timestamp);
      rawimage->width = width;
      rawimage->height = height;
      rawimage->data.resize (width * height * sizeof(float));
      rawimage->encoding = getOpenCVType (channels, bpe, isFlt);
      
      mono_camera_[itmImages][itmRaw].getBinaryData (rawimage->data.data (), rawimage->data.size (), 0, 0);
      
      mono_images_signal_->operator () (rawimage,rectimage);
    }


  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "captureMonoCalibrationPattern");
    return (-1);
  }

  
  try{
    NxLibCommand collect_pattern (cmdCollectPattern);
    collect_pattern.parameters()[itmCameras][0] = devices[0];
    collect_pattern.parameters()[itmCameras][1] = devices[1];
    collect_pattern.parameters()[itmDecodeData] = false;
    collect_pattern.parameters()[itmRefinement] = valNone;
    collect_pattern.parameters()[itmReturnAllPattern] = true;
    collect_pattern.parameters()[itmFilter][itmCameras][0] = devices[0];
    collect_pattern.parameters()[itmFilter][itmCameras][1] = devices[1];
    collect_pattern.parameters()[itmFilter][itmType] = valStatic;
    collect_pattern.parameters()[itmFilter][itmValue] = true;
    //collect_pattern.parameters ()[itmBuffer] = true);  // Store the pattern into the buffer
    collect_pattern.execute ();

    if (num_slots<sig_cb_ensenso_raw_images>()>0) {

      boost::shared_ptr<PairOfImages> rawimages (new PairOfImages);
      
      int width, height, channels, bpe;
      bool isFlt;
      
      double timestamp;
      camera_[itmImages][itmWithOverlay][itmLeft].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);
      
      camera_[itmImages][itmWithOverlay][itmLeft].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
      rawimages->first.header.stamp = getPCLStamp (timestamp);
      rawimages->first.width = width;
      rawimages->first.height = height;
      rawimages->first.data.resize (width * height * sizeof(float));
      rawimages->first.encoding = getOpenCVType (channels, bpe, isFlt);
      camera_[itmImages][itmWithOverlay][itmLeft].getBinaryData (rawimages->first.data.data (), rawimages->first.data.size (), 0, 0);
      raw_images_signal_->operator () (rawimages);
    }

    if (num_slots<sig_cb_mono_images>()>0) {
          
      boost::shared_ptr<pcl::PCLImage> rawimage (new pcl::PCLImage);
      boost::shared_ptr<pcl::PCLImage> rectimage (new pcl::PCLImage);
      int width, height, channels, bpe;
      bool isFlt;
      
      double timestamp;
      mono_camera_[itmImages][itmWithOverlay].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);
      
      mono_camera_[itmImages][itmWithOverlay].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
      rawimage->header.stamp = getPCLStamp (timestamp);
      rawimage->width = width;
      rawimage->height = height;
      rawimage->data.resize (width * height * sizeof(float));
      rawimage->encoding = getOpenCVType (channels, bpe, isFlt);
      
      mono_camera_[itmImages][itmWithOverlay].getBinaryData (rawimage->data.data (), rawimage->data.size (), 0, 0);
      
      mono_images_signal_->operator () (rawimage,rectimage);
    }
    
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "captureMonoCalibrationPattern");
    return (-1);
  }

  return ( (*root_)[itmParameters][itmPatternCount].asInt ());
}



bool pcl::EnsensoGrabber::mono_estimateCalibrationPatternPose (Eigen::Affine3d &pattern_pose, double& pose_error, const bool average) 
{
  if (!device_open_ || !mono_device_open_)
    return (false);

  if (running_)
    return (false);

  try
  {
    int num_items = (*root_)[itmParameters][itmPatternCount].asInt ();

 	NxLibCommand estimate(cmdEstimatePatternPose);

    estimate.parameters()[itmReprojectionErrorScale] = 100;
    estimate.parameters()[itmFilter][itmCameras][1] = mono_serial_;
    estimate.parameters()[itmFilter][itmCameras][0] = serial_;
    estimate.parameters()[itmFilter][itmType] = valStatic;
    estimate.parameters()[itmFilter][itmValue] = true;
    estimate.parameters()[itmAverage] = average;
    estimate.execute();
    
    std::string pose = estimate.result()[itmPatterns][num_items-1][itmPatternPose].asJson();
    double repError = estimate.result()[itmPatterns][num_items-1][itmReprojectionError].asDouble();
    double posError = estimate.result()[itmPatterns][num_items-1][itmPoseError].asDouble();
                

    
    //    double err;
    // if (num_items==1) {
    //   checkCalibration(err);
    //   estimate_pattern_pose.parameters ()[itmRecalibrate] = true);
    // }
    // else

    // if (num_items==1) 
    //   checkCalibration(err);

    //    NxLibItem tf = estimate_pattern_pose.result ()[itmPatterns][num_items-1][itmPatternPose];
    // Convert tf into a matrix
    if (!jsonTransformationToMatrix (pose, pattern_pose))
      return (false);
    pattern_pose.translation () /= 1000.0;  // Convert translation in meters (Ensenso API returns milimeters)
    pose_error = posError / 1000.0;
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "estimateCalibrationPatternPoses");
    return (false);
  }
}


bool pcl::EnsensoGrabber::estimateCalibrationPatternPose (Eigen::Affine3d &pattern_pose, double& pose_error, const bool average) 
{
  if (!device_open_)
    return (false);

  if (running_)
    return (false);

  try
  {

    NxLibCommand estimate (cmdEstimatePatternPose);
    
    estimate.parameters()[itmReprojectionErrorScale] = 100;
    estimate.parameters()[itmFilter][itmCameras] = serial_;
    estimate.parameters()[itmFilter][itmType] = valStatic;
    estimate.parameters()[itmFilter][itmValue] = true;
    estimate.parameters()[itmAverage] = average;
    estimate.execute();
    
    int num_items = (*root_)[itmParameters][itmPatternCount].asInt ();
    
    std::string pose = estimate.result()[itmPatterns][num_items-1][itmPatternPose].asJson();
    double repError = estimate.result()[itmPatterns][num_items-1][itmReprojectionError].asDouble();
    double posError = estimate.result()[itmPatterns][num_items-1][itmPoseError].asDouble();

    //    double err;
    // if (num_items==1) {
    //   checkCalibration(err);
    //   estimate_pattern_pose.parameters ()[itmRecalibrate] = true);
    // }
    // else
    // if (num_items==1) 
    //   checkCalibration(err);

    //    NxLibItem tf = estimate_pattern_pose.result ()[itmPatterns][num_items-1][itmPatternPose];
    // Convert tf into a matrix
    if (!jsonTransformationToMatrix (pose, pattern_pose))
      return (false);
    pattern_pose.translation () /= 1000.0;  // Convert translation in meters (Ensenso API returns milimeters)
    pose_error = posError /1000;
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "estimateCalibrationPatternPoses");
    return (false);
  }
}


bool pcl::EnsensoGrabber::computeCalibrationMatrix (const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > &robot_poses,
                          std::string &json,
                          int &iterations,
                          double &reprojection_error,
                          const std::string setup,
                          const std::string target,
                          const Eigen::Affine3d &guess_tf,
                          const bool pretty_format
                          ) 
{
  if ( (*root_)[itmVersion][itmMajor] < 2 && (*root_)[itmVersion][itmMinor] < 3)
    PCL_WARN ("EnsensoSDK 1.3.x fixes bugs into extrinsic calibration matrix optimization, please update your SDK!\n"
          "http://www.ensenso.de/support/sdk-download/\n");

  int num_items = (*root_)[itmParameters][itmPatternCount].asInt ();

  if (num_items != robot_poses.size()) {
    ROS_WARN("Number of robot poses not equal to collected images");
    return false;
  }
  
  NxLibCommand calibrate (cmdCalibrateHandEye);
  try
  {
    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > robot_poses_mm (robot_poses);
    std::vector<std::string> robot_poses_json;
    robot_poses_json.resize (robot_poses.size ());
    for (uint i = 0; i < robot_poses_json.size (); ++i)
    {
      robot_poses_mm[i].translation () *= 1000.0; // Convert meters in millimeters
      if (!matrixTransformationToJson (robot_poses_mm[i], robot_poses_json[i]))
        return (false);
    }
    // Set Hand-Eye calibration parameters
    if (boost::iequals (setup, "Fixed"))
      calibrate.parameters ()[itmSetup] = valFixed;
    else
      calibrate.parameters ()[itmSetup] = valMoving;
    calibrate.parameters ()[itmTarget] = target;

    // Set guess matrix
    if (guess_tf.matrix () != Eigen::Matrix4d::Identity ())
    {
      // Matrix > JSON > Angle axis
      if (!matrixTransformationToJson (guess_tf, json))
        return (false);
      NxLibItem tf ("/tmpTF");
      tf.setJson (json);

      // Rotation
      double theta = tf[itmRotation][itmAngle].asDouble ();  // Angle of rotation
      double x = tf[itmRotation][itmAxis][0].asDouble ();   // X component of Euler vector
      double y = tf[itmRotation][itmAxis][1].asDouble ();   // Y component of Euler vector
      double z = tf[itmRotation][itmAxis][2].asDouble ();   // Z component of Euler vector
      tf.erase(); // Delete tmpTF node
      
      calibrate.parameters ()[itmLink][itmRotation][itmAngle] = theta;
      calibrate.parameters ()[itmLink][itmRotation][itmAxis][0] = x;
      calibrate.parameters ()[itmLink][itmRotation][itmAxis][1] = y;
      calibrate.parameters ()[itmLink][itmRotation][itmAxis][2] = z;
      // Translation
      calibrate.parameters ()[itmLink][itmTranslation][0] = guess_tf.translation ()[0] * 1000.0;
      calibrate.parameters ()[itmLink][itmTranslation][1] = guess_tf.translation ()[1] * 1000.0;
      calibrate.parameters ()[itmLink][itmTranslation][2] = guess_tf.translation ()[2] * 1000.0;
    }

    // Feed all robot poses into the calibration command
    for (uint i = 0; i < robot_poses_json.size (); ++i)
    {
      // Very weird behavior here:
      // If you modify this loop, check that all the transformations are still here in the [itmExecute][itmParameters] node
      // because for an unknown reason sometimes the old transformations are erased in the tree ("null" in the tree)
      // Ensenso SDK 2.3.348: If not moved after guess calibration matrix, the vector is empty.
      calibrate.parameters ()[itmTransformations][i].setJson (robot_poses_json[i], false);
    }

    calibrate.execute ();  // Might take up to 120 sec (maximum allowed by Ensenso API)


    setExtrinsicCalibration(target);

    if (calibrate.successful())
    {
      json = calibrate.result()[itmLink].asJson (pretty_format);
      iterations = calibrate.result()[itmIterations].asInt();
      reprojection_error = calibrate.result()[itmReprojectionError].asDouble();
      ROS_INFO("computeCalibrationMatrix succeeded. Iterations: %d, Reprojection error: %.2f", iterations, reprojection_error);
      ROS_INFO_STREAM("Result: " << std::endl << json);
      return (true);
    }
    else
    {
      json.clear ();
      return (false);
    }
  }
  catch (NxLibException &ex)
  {
    try
    {
      ensensoExceptionHandling (ex, "computeCalibrationMatrix");
      int iters = calibrate.result()[itmIterations].asInt();
      double error = calibrate.result()[itmReprojectionError].asDouble();
      ROS_WARN("computeCalibrationMatrix Failed. Iterations: %d, Reprojection error: %.2f", iters, error);
      ROS_WARN_STREAM("Result: " << std::endl << calibrate.result()[itmLink].asJson(true));
    }
    catch (...) {
      ensensoExceptionHandling (ex, "computeCalibrationMatrix");
    } 
    return false;
  }
}



bool pcl::EnsensoGrabber::computeMonoCalibrationMatrix (
                          std::string &json,
                          const bool pretty_format
                          ) 
{

  int num_items = (*root_)[itmParameters][itmPatternCount].asInt ();
  
  NxLibCommand calibrate (cmdCalibrate);
  std::string devices[2]={serial_,mono_serial_};

  try
  {
    calibrate.parameters()[itmCameras][0] = devices[0];
    calibrate.parameters()[itmCameras][1] = devices[1];   
    calibrate.execute ();  

    if (calibrate.successful())
    {
      NxLibItem calibParams = mono_camera_[itmLink];
      json = calibParams.asJson (pretty_format);      
      ROS_INFO_STREAM("compute Mono Result: " << std::endl << json);

      NxLibCommand store (cmdStoreCalibration);
      store.parameters()[itmCameras] = mono_serial_;
      store.parameters()[itmLink] = true;
      // Assumes some Cal info
      store.parameters()[itmCalibration] = true;
      store.execute();
      
      return (true);
    }
    else
      return (false);
  }
  catch (NxLibException &ex)
    {
      try
        {
          return (false);
        }
      catch (...) {
        ensensoExceptionHandling (ex, "computeMonoCalibrationMatrix");
      } 
      
    }
}




bool pcl::EnsensoGrabber::storeEEPROMExtrinsicCalibration () const
{
  try
  {
    NxLibCommand store (cmdStoreCalibration);
    //    store.execute ();
    return (false);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "storeEEPROMExtrinsicCalibration");
    return (false);
  }
}

bool pcl::EnsensoGrabber::getCalInfo () 
{

  ROS_WARN_STREAM("Calibration info for camera: "<<serial_);

  ROS_INFO("Dynamic->Stereo->Left->Camera: ");
  for (uint i=0; i<3; i++) {
    for (uint j=0; j<3; j++)
      std::cerr<< camera_[itmCalibration][itmDynamic][itmStereo][itmLeft][itmCamera][j][i].asDouble()<<" ";
    std::cerr << "\n";
  }

  ROS_INFO("Dynamic->Stereo->Left->Rotation: ");
  for (uint i=0; i<3; i++) {
    for (uint j=0; j<3; j++)
      std::cerr<< camera_[itmCalibration][itmDynamic][itmStereo][itmLeft][itmRotation][j][i].asDouble()<<" ";
    std::cerr << "\n";
  }

    ROS_INFO("Dynamic->Stereo->Right->Camera: ");
  for (uint i=0; i<3; i++) {
    for (uint j=0; j<3; j++)
      std::cerr<< camera_[itmCalibration][itmDynamic][itmStereo][itmRight][itmCamera][j][i].asDouble()<<" ";
    std::cerr << "\n";
  }

  ROS_INFO("Dynamic->Stereo->Right->Rotation: ");
  for (uint i=0; i<3; i++) {
    for (uint j=0; j<3; j++)
      std::cerr<< camera_[itmCalibration][itmDynamic][itmStereo][itmRight][itmRotation][j][i].asDouble()<<" ";
    std::cerr << "\n";
  }
  
  ROS_INFO("Dynamic->Stereo->Reprojection: ");
  for (uint i=0; i<4; i++) {
    for (uint j=0; j<4; j++)
      std::cerr<< camera_[itmCalibration][itmDynamic][itmStereo][itmReprojection][j][i].asDouble()<<" ";
    std::cerr << "\n";
  }

  ROS_INFO("Dynamic->Stereo->Angle->Epipolar/Vergence: ");
  std::cerr<< camera_[itmCalibration][itmDynamic][itmStereo][itmAngle][itmEpipolar].asDouble()<<" ";
  std::cerr<< camera_[itmCalibration][itmDynamic][itmStereo][itmAngle][itmVergence].asDouble()<<"\n";


  ROS_INFO("Stereo->Left->Camera: ");
  for (uint i=0; i<3; i++) {
    for (uint j=0; j<3; j++)
      std::cerr<< camera_[itmCalibration][itmStereo][itmLeft][itmCamera][j][i].asDouble()<<" ";
    std::cerr << "\n";
  }

  ROS_INFO("Stereo->Left->Rotation: ");
  for (uint i=0; i<3; i++) {
    for (uint j=0; j<3; j++)
      std::cerr<< camera_[itmCalibration][itmStereo][itmLeft][itmRotation][j][i].asDouble()<<" ";
    std::cerr << "\n";
  }

    ROS_INFO("Stereo->Right->Camera: ");
  for (uint i=0; i<3; i++) {
    for (uint j=0; j<3; j++)
      std::cerr<< camera_[itmCalibration][itmStereo][itmRight][itmCamera][j][i].asDouble()<<" ";
    std::cerr << "\n";
  }

  ROS_INFO("Stereo->Right->Rotation: ");
  for (uint i=0; i<3; i++) {
    for (uint j=0; j<3; j++)
      std::cerr<< camera_[itmCalibration][itmStereo][itmRight][itmRotation][j][i].asDouble()<<" ";
    std::cerr << "\n";
  }
  
  ROS_INFO("Stereo->Reprojection: ");
  for (uint i=0; i<4; i++) {
    for (uint j=0; j<4; j++)
      std::cerr<< camera_[itmCalibration][itmStereo][itmReprojection][j][i].asDouble()<<" ";
    std::cerr << "\n";
  }

  ROS_INFO("Stereo->Angle->Epipolar/Vergence/OpticalAxis: ");
  std::cerr<< camera_[itmCalibration][itmStereo][itmAngle][itmEpipolar].asDouble()<<" ";
  std::cerr<< camera_[itmCalibration][itmStereo][itmAngle][itmVergence].asDouble()<<" ";
  std::cerr<< camera_[itmCalibration][itmStereo][itmAngle][itmOpticalAxis].asDouble()<<"\n";

  ROS_INFO("Stereo->Baseline: ");
  std::cerr<< camera_[itmCalibration][itmStereo][itmBaseline].asDouble()<<"\n";

  ROS_INFO("Monocular->Left->Camera: ");
  for (uint i=0; i<3; i++) {
    for (uint j=0; j<3; j++)
      std::cerr<< camera_[itmCalibration][itmMonocular][itmLeft][itmCamera][j][i].asDouble()<<" ";
    std::cerr << "\n";
  }

  ROS_INFO("Monocular->Left->Distortion: ");
  for (uint i=0; i<5; i++) 
    std::cerr<< camera_[itmCalibration][itmMonocular][itmLeft][itmDistortion][i].asDouble()<<" ";
  std::cerr<<"\n";

  ROS_INFO("Monocular->Right->Camera: ");
  for (uint i=0; i<3; i++) {
    for (uint j=0; j<3; j++)
      std::cerr<< camera_[itmCalibration][itmMonocular][itmRight][itmCamera][j][i].asDouble()<<" ";
    std::cerr << "\n";
  }

  ROS_INFO("Monocular->Right->Distortion: ");
  for (uint i=0; i<5; i++) 
    std::cerr<< camera_[itmCalibration][itmMonocular][itmRight][itmDistortion][i].asDouble()<<" ";
  std::cerr<<"\n";

  
}

bool pcl::EnsensoGrabber::checkCalibration (double& max_error) 
{
  int num_items = (*root_)[itmParameters][itmPatternCount].asInt ();

  // if (num_items < 1)
  //   return false;
  
  try
  {
    NxLibCommand check_calibration (cmdMeasureCalibration);      
    //    calibrate.execute ();  // Might take up to 120 sec (maximum allowed by Ensenso API)
    check_calibration.execute();

    max_error = 0;

    for (uint i=0; i<num_items; i++)
      max_error = std::max (max_error, check_calibration.result ()[itmPatterns][num_items-1][itmReprojectionError].asDouble());
    // Convert

    ROS_WARN_STREAM("Calibration error: "<<max_error);
    
  }
  catch (NxLibException &ex)
  {
    ROS_WARN("checkCalibration Failed");
    return (false);
  }
}




bool pcl::EnsensoGrabber::loadEEPROMExtrinsicCalibration () const
{
  try
  {
    NxLibCommand load (cmdLoadCalibration);
    load.parameters ()[itmCameras] = camera_[itmSerialNumber].asString ();
    load.execute ();
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "loadEEPROMExtrinsicCalibration");
    return (false);
  }
}


bool pcl::EnsensoGrabber::clearEEPROMExtrinsicCalibration ()
{
  try
  {
    setExtrinsicCalibration("");
    NxLibCommand store (cmdStoreCalibration);
    //    store.execute ();
    return (false);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "clearEEPROMExtrinsicCalibration");
    return (false);
  }
}

bool pcl::EnsensoGrabber::setExtrinsicCalibration (const double euler_angle,
                           Eigen::Vector3d &rotation_axis,
                           const Eigen::Vector3d &translation,
                           const std::string target)
{
  if (!device_open_)
    return (false);

  if (rotation_axis != Eigen::Vector3d (0, 0, 0))
    rotation_axis.normalize ();

  try
  {
    NxLibItem calibParams = camera_[itmLink];
    calibParams[itmTarget] = target;
    calibParams[itmRotation][itmAngle] = euler_angle;
    
    calibParams[itmRotation][itmAxis][0] = rotation_axis[0];
    calibParams[itmRotation][itmAxis][1] = rotation_axis[1];
    calibParams[itmRotation][itmAxis][2] = rotation_axis[2];
    
    calibParams[itmTranslation][0] = translation[0] * 1000.0;  // Convert in millimeters
    calibParams[itmTranslation][1] = translation[1] * 1000.0;
    calibParams[itmTranslation][2] = translation[2] * 1000.0;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setExtrinsicCalibration");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setExtrinsicCalibration (const std::string target)
{
  if (!device_open_)
    return (false);
  
  Eigen::Vector3d rotation (Eigen::Vector3d::Zero ());
  Eigen::Vector3d translation (Eigen::Vector3d::Zero ());
  return (setExtrinsicCalibration (0.0, rotation, translation, target));
}

bool pcl::EnsensoGrabber::setExtrinsicCalibration (const Eigen::Affine3d &transformation,
                           const std::string target)
{
  std::string json;
  if (!matrixTransformationToJson (transformation, json))
    return (false);
  
  double euler_angle;
  Eigen::Vector3d rotation_axis;
  Eigen::Vector3d translation;

  if (!jsonTransformationToAngleAxis (json, euler_angle, rotation_axis, translation))
    return (false);

  return (setExtrinsicCalibration (euler_angle, rotation_axis, translation, target));
}

float pcl::EnsensoGrabber::getFramesPerSecond () const
{
  boost::mutex::scoped_lock lock (fps_mutex_);
  if (times_.size () < 2)
    return (0.0);
  return ((times_.size () - 1) /
          (times_.back () - times_.front ()));
}

bool pcl::EnsensoGrabber::openTcpPort (const int port)
{
  try
  {
    nxLibOpenTcpPort (port);
    tcp_open_ = true;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "openTcpPort");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::closeTcpPort ()
{
  try
  {
    nxLibCloseTcpPort ();
    tcp_open_ = false;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "closeTcpPort");
    return (false);
  }
  return (true);
}

std::string pcl::EnsensoGrabber::getTreeAsJson (const bool pretty_format) const
{
  try
  {
    return (root_->asJson (pretty_format));
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "getTreeAsJson");
    return ("");
  }
}

std::string pcl::EnsensoGrabber::getResultAsJson (const bool pretty_format) const
{
  try
  {
    NxLibCommand cmd ("");
    return (cmd.result ().asJson (pretty_format));
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "getResultAsJson");
    return ("");
  }
}

double pcl::EnsensoGrabber::getPatternGridSpacing () const
{
  if (!device_open_)
    return (-1);
  if (running_)
    return (-1);

  double rc = -1;
  
  try
  {
    NxLibCommand capture(cmdCapture);
    capture.parameters()[itmCameras] = serial_;
    capture.execute ();

    if (num_slots<sig_cb_ensenso_raw_images>()>0) {

      boost::shared_ptr<PairOfImages> rawimages (new PairOfImages);
      
      int width, height, channels, bpe;
      bool isFlt;
      
      double timestamp;
      camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);
      
      camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
      rawimages->first.header.stamp = rawimages->second.header.stamp = getPCLStamp (timestamp);
      rawimages->first.width = rawimages->second.width = width;
      rawimages->first.height = rawimages->second.height = height;
      rawimages->first.data.resize (width * height * sizeof(float));
      rawimages->second.data.resize (width * height * sizeof(float));
      rawimages->first.encoding = rawimages->second.encoding = getOpenCVType (channels, bpe, isFlt);
      camera_[itmImages][itmRaw][itmLeft].getBinaryData (rawimages->first.data.data (), rawimages->first.data.size (), 0, 0);
      camera_[itmImages][itmRaw][itmRight].getBinaryData (rawimages->second.data.data (), rawimages->second.data.size (), 0, 0);
      raw_images_signal_->operator () (rawimages);
    }    

  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "getPatternGridSpacing");
    return (-1);
  }

  try {
    NxLibCommand collect_pattern (cmdCollectPattern);
    collect_pattern.parameters()[itmCameras] = serial_;
    collect_pattern.parameters ()[itmBuffer] = false;
    collect_pattern.parameters ()[itmDecodeData] = false;
    collect_pattern.execute ();
    rc = collect_pattern.result()[itmGridSpacing].asDouble();
    
    if (num_slots<sig_cb_ensenso_raw_images>()>0) {

      boost::shared_ptr<PairOfImages> rawimages (new PairOfImages);
      
      int width, height, channels, bpe;
      bool isFlt;
      
      double timestamp;
      camera_[itmImages][itmWithOverlay][itmLeft].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);
      
      camera_[itmImages][itmWithOverlay][itmLeft].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
      rawimages->first.header.stamp = rawimages->second.header.stamp = getPCLStamp (timestamp);
      rawimages->first.width = rawimages->second.width = width;
      rawimages->first.height = rawimages->second.height = height;
      rawimages->first.data.resize (width * height * sizeof(float));
      rawimages->second.data.resize (width * height * sizeof(float));
      rawimages->first.encoding = rawimages->second.encoding = getOpenCVType (channels, bpe, isFlt);
      camera_[itmImages][itmWithOverlay][itmLeft].getBinaryData (rawimages->first.data.data (), rawimages->first.data.size (), 0, 0);
      camera_[itmImages][itmWithOverlay][itmRight].getBinaryData (rawimages->second.data.data (), rawimages->second.data.size (), 0, 0);
      raw_images_signal_->operator () (rawimages);
    }    

  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "getPatternGridSpacing");
    return (-1);
  }
  
  return rc;
}

bool pcl::EnsensoGrabber::enableFrontLight(const bool enable) const
{
  try
  {
    camera_[itmParameters][itmCapture][itmFrontLight] = enable;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling(ex, "enableFrontLight");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::enableProjector(const bool enable) const
{
  try
  {
    camera_[itmParameters][itmCapture][itmProjector] = enable;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling(ex, "enableProjector");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::getCameraInfo(std::string cam, sensor_msgs::CameraInfo &cam_info) const
{
  try
  {

    double theta = 0;//camera_[itmLink][itmRotation][itmAngle].asDouble();
    double x = 0;//camera_[itmLink][itmRotation][itmAxis][0].asDouble();
    double y = 0;//camera_[itmLink][itmRotation][itmAxis][1].asDouble();
    double z = 0;//camera_[itmLink][itmRotation][itmAxis][2].asDouble();
    // Translation
    double trans_x = 0;//camera_[itmLink][itmTranslation][0].asDouble() /1000.0;
    double trans_y = 0;//camera_[itmLink][itmTranslation][1].asDouble() / 1000.0;
    double trans_z = 0;//camera_[itmLink][itmTranslation][2].asDouble() / 1000.0;

    // std::string json;
    // if (!angleAxisTransformationToJson(trans_x,trans_y,trans_z,x,y,z,theta,json))
    //   return false;

    // if (!jsonTransformationToEulerAngles(json, trans_x, trans_y, trans_z, x, y, z))
    //   return false;

    ROS_INFO_STREAM_THROTTLE(10,"Link offset from "<<serial_<<" to "<<camera_[itmLink][itmTarget].asString()<<": "<<trans_x<<" "<<trans_y<<" "<<trans_z<<" "<<x<<" "<<y<<" "<<z);

    
    cam_info.width = camera_[itmSensor][itmSize][0].asInt();
    cam_info.height = camera_[itmSensor][itmSize][1].asInt();
    cam_info.distortion_model = "plumb_bob";
    // Distorsion factors
    cam_info.D.resize(5);
    for(std::size_t i = 0; i < cam_info.D.size(); ++i)
      cam_info.D[i] = camera_[itmCalibration][itmMonocular][cam][itmDistortion][i].asDouble();
    // K and R matrices
    for(std::size_t i = 0; i < 3; ++i)
    {
      for(std::size_t j = 0; j < 3; ++j)
      {
        cam_info.K[3*i+j] = camera_[itmCalibration][itmMonocular][cam][itmCamera][j][i].asDouble();
        cam_info.R[3*i+j] = camera_[itmCalibration][itmStereo][cam][itmRotation][j][i].asDouble();
      }
    }
    cam_info.P[0] = camera_[itmCalibration][itmStereo][cam][itmCamera][0][0].asDouble();
    cam_info.P[1] = camera_[itmCalibration][itmStereo][cam][itmCamera][1][0].asDouble();
    cam_info.P[2] = camera_[itmCalibration][itmStereo][cam][itmCamera][2][0].asDouble();
    cam_info.P[3] = 0.0;
    cam_info.P[4] = camera_[itmCalibration][itmStereo][cam][itmCamera][0][1].asDouble();
    cam_info.P[5] = camera_[itmCalibration][itmStereo][cam][itmCamera][1][1].asDouble();
    cam_info.P[6] = camera_[itmCalibration][itmStereo][cam][itmCamera][2][1].asDouble();
    cam_info.P[7] = 0.0;
    cam_info.P[10] = 1.0;
    if (cam == "Right")
    {
      double B = camera_[itmCalibration][itmStereo][itmBaseline].asDouble() / 1000.0;
      double fx = cam_info.P[0];
      cam_info.P[3] = (-fx * B);
    }
    return true;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "getCameraInfo");
    return false;
  }
}


bool pcl::EnsensoGrabber::mono_resetCameraInfo(std::vector<double>& val, bool write_to_device) const
{
  
  try {

    std::string json;

    if (val.size() == 6) {
      if (!eulerAnglesTransformationToJson(val[0]*1000,val[1]*1000,val[2]*1000,val[3],val[4],val[5],json))
        return false;
    }
    else  
      if (!eulerAnglesTransformationToJson(0.0175 * 1000, 0.0325 * 1000, -0.01 * 1000,0,0,0,json))
        return false;

    double euler_angle;
    Eigen::Vector3d rotation_axis;
    Eigen::Vector3d translation;
    
    if (!jsonTransformationToAngleAxis (json, euler_angle, rotation_axis, translation))
      return (false);


    mono_camera_[itmLink][itmTarget] = serial_;

    mono_camera_[itmLink][itmRotation][itmAngle] = euler_angle;

    mono_camera_[itmLink][itmRotation][itmAxis][0] = rotation_axis[0];
    mono_camera_[itmLink][itmRotation][itmAxis][1] = rotation_axis[1];
    mono_camera_[itmLink][itmRotation][itmAxis][2] = rotation_axis[2];
    
    mono_camera_[itmLink][itmTranslation][0] = translation[0];  // Convert in millimeters
    mono_camera_[itmLink][itmTranslation][1] = translation[1];
    mono_camera_[itmLink][itmTranslation][2] = translation[2];

    ROS_WARN_STREAM("Reset "<<mono_serial_<<" to "<<serial_<<" transform."); 

    if (write_to_device) {
      NxLibCommand store (cmdStoreCalibration);
      store.parameters()[itmCameras] = mono_serial_;
      store.parameters()[itmLink] = true;
      // Assumes some Cal info
      store.parameters()[itmCalibration] = true;
      store.execute();
    }
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "mono_resetCameraInfo");
    return false;
  }
  return true;
}
  

bool pcl::EnsensoGrabber::mono_getCameraInfo(sensor_msgs::CameraInfo &cam_info) const
{
  try {
    
    double theta = mono_camera_[itmLink][itmRotation][itmAngle].asDouble();
    double x = mono_camera_[itmLink][itmRotation][itmAxis][0].asDouble();
    double y = mono_camera_[itmLink][itmRotation][itmAxis][1].asDouble();
    double z = mono_camera_[itmLink][itmRotation][itmAxis][2].asDouble();
    // Translation
    double trans_x = mono_camera_[itmLink][itmTranslation][0].asDouble() /1000.0;
    double trans_y = mono_camera_[itmLink][itmTranslation][1].asDouble() / 1000.0;
    double trans_z = mono_camera_[itmLink][itmTranslation][2].asDouble() / 1000.0;

    std::string json;
    if (!angleAxisTransformationToJson(trans_x,trans_y,trans_z,x,y,z,theta,json))
      return false;
    
    if (!jsonTransformationToEulerAngles(json, trans_x, trans_y, trans_z, x, y, z))
      return false;

    ROS_INFO_STREAM_THROTTLE(10,"Link offset from "<<mono_serial_<<" to "<<mono_camera_[itmLink][itmTarget].asString()<<": "<<trans_x<<" "<<trans_y<<" "<<trans_z<<" "<<x<<" "<<y<<" "<<z);
    
    cam_info.width = mono_camera_[itmSensor][itmSize][0].asInt();
    cam_info.height = mono_camera_[itmSensor][itmSize][1].asInt();
    cam_info.distortion_model = "plumb_bob";
    // Distorsion factors
    cam_info.D.resize(5);
    for(std::size_t i = 0; i < cam_info.D.size(); ++i)
      cam_info.D[i] = mono_camera_[itmCalibration][itmDistortion][i].asDouble();
    // K and R matrices
    for(std::size_t i = 0; i < 3; ++i)
    {
      for(std::size_t j = 0; j < 3; ++j)
        {
          cam_info.K[3*i+j] = mono_camera_[itmCalibration][itmCamera][j][i].asDouble();
      }
    }
    return true;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "mono_getCameraInfo");
    return false;
  }
}



bool pcl::EnsensoGrabber::jsonTransformationToEulerAngles (const std::string &json,
                               double &x,
                               double &y,
                               double &z,
                               double &r,
                               double &p,
                               double &w) const
{
  try
  {
    NxLibCommand convert (cmdConvertTransformation);
    convert.parameters ()[itmTransformation].setJson (json, false);
    convert.parameters ()[itmSplitRotation] = valXYZ;
    convert.execute ();
    
    NxLibItem tf = convert.result ()[itmTransformations];
    x = tf[0][itmTranslation][0].asDouble ();
    y = tf[0][itmTranslation][1].asDouble ();
    z = tf[0][itmTranslation][2].asDouble ();
    w = tf[0][itmRotation][itmAngle].asDouble ();  // yaW
    p = tf[1][itmRotation][itmAngle].asDouble ();  // Pitch
    r = tf[2][itmRotation][itmAngle].asDouble ();  // Roll
    return (true);
  }

  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "jsonTransformationToEulerAngles");
    return (false);
  }
}

bool pcl::EnsensoGrabber::jsonTransformationToAngleAxis (const std::string json,
                             double &alpha,
                             Eigen::Vector3d &axis,
                             Eigen::Vector3d &translation) const
{
  try
  {
    NxLibItem tf ("/tmpTF");
    tf.setJson(json);
    translation[0] = tf[itmTranslation][0].asDouble ();
    translation[1] = tf[itmTranslation][1].asDouble ();
    translation[2] = tf[itmTranslation][2].asDouble ();
    
    alpha = tf[itmRotation][itmAngle].asDouble ();  // Angle of rotation
    axis[0] = tf[itmRotation][itmAxis][0].asDouble ();  // X component of Euler vector
    axis[1] = tf[itmRotation][itmAxis][1].asDouble ();  // Y component of Euler vector
    axis[2] = tf[itmRotation][itmAxis][2].asDouble ();  // Z component of Euler vector
    tf.erase(); // Delete tmpTF node
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "jsonTransformationToAngleAxis");
    return (false);
  }
}

bool pcl::EnsensoGrabber::jsonTransformationToMatrix (const std::string transformation,
                            Eigen::Affine3d &matrix) const
{
  try
  {
    NxLibCommand convert_transformation (cmdConvertTransformation);
    convert_transformation.parameters ()[itmTransformation].setJson (transformation);
    convert_transformation.execute ();
    Eigen::Affine3d tmp (Eigen::Affine3d::Identity ());
    
    // Rotation
    tmp.linear ().col (0) = Eigen::Vector3d (convert_transformation.result ()[itmTransformation][0][0].asDouble (),
                         convert_transformation.result ()[itmTransformation][0][1].asDouble (),
                         convert_transformation.result ()[itmTransformation][0][2].asDouble ());

    tmp.linear ().col (1) = Eigen::Vector3d (convert_transformation.result ()[itmTransformation][1][0].asDouble (),
                         convert_transformation.result ()[itmTransformation][1][1].asDouble (),
                         convert_transformation.result ()[itmTransformation][1][2].asDouble ());

    tmp.linear ().col (2) = Eigen::Vector3d (convert_transformation.result ()[itmTransformation][2][0].asDouble (),
                         convert_transformation.result ()[itmTransformation][2][1].asDouble (),
                         convert_transformation.result ()[itmTransformation][2][2].asDouble ());

    // Translation
    tmp.translation () = Eigen::Vector3d (convert_transformation.result ()[itmTransformation][3][0].asDouble (),
                        convert_transformation.result ()[itmTransformation][3][1].asDouble (),
                        convert_transformation.result ()[itmTransformation][3][2].asDouble ());
    matrix = tmp;
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "jsonTransformationToMatrix");
    return (false);
  }
}

bool pcl::EnsensoGrabber::eulerAnglesTransformationToJson (const double x,
                               const double y,
                               const double z,
                               const double w,
                               const double p,
                               const double r,
                               std::string &json,
                               const bool pretty_format) const
{
  try
  {
    NxLibCommand chain (cmdChainTransformations);
    NxLibItem tf = chain.parameters ()[itmTransformations];
    
    if (!angleAxisTransformationToJson (x, y, z, 0, 0, 1, r, json))
      return (false);
    tf[0].setJson (json, false);  // Roll
    
    if (!angleAxisTransformationToJson (0, 0, 0, 0, 1, 0, p, json))
      return (false);
    tf[1].setJson (json, false);  // Pitch
    
    if (!angleAxisTransformationToJson (0, 0, 0, 1, 0, 0, w, json))
      return (false);
    tf[2].setJson (json, false);  // yaW
    
    chain.execute ();
    json = chain.result ()[itmTransformation].asJson (pretty_format);
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "eulerAnglesTransformationToJson");
    return (false);
  }
}

bool pcl::EnsensoGrabber::angleAxisTransformationToJson (const double x,
                             const double y,
                             const double z,
                             const double rx,
                             const double ry,
                             const double rz,
                             const double alpha,
                             std::string &json,
                             const bool pretty_format) const
{
  try
  {
    NxLibItem tf ("/tmpTF");
    tf[itmTranslation][0] = x;
    tf[itmTranslation][1] = y;
    tf[itmTranslation][2] = z;
    
    tf[itmRotation][itmAngle] = alpha;  // Angle of rotation
    tf[itmRotation][itmAxis][0] = rx;  // X component of Euler vector
    tf[itmRotation][itmAxis][1] = ry;  // Y component of Euler vector
    tf[itmRotation][itmAxis][2] = rz;  // Z component of Euler vector
    
    json = tf.asJson (pretty_format);
    tf.erase ();
    return (true);
  }

  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "angleAxisTransformationToJson");
    return (false);
  }
}

bool pcl::EnsensoGrabber::matrixTransformationToJson (const Eigen::Affine3d &matrix,
                            std::string &json,
                            const bool pretty_format) const
{
  try
  {
    NxLibCommand convert (cmdConvertTransformation);
    
    // Rotation
    convert.parameters ()[itmTransformation][0][0] = matrix.linear ().col (0)[0];
    convert.parameters ()[itmTransformation][0][1] = matrix.linear ().col (0)[1];
    convert.parameters ()[itmTransformation][0][2] = matrix.linear ().col (0)[2];
    convert.parameters ()[itmTransformation][0][3] = 0.0;
    
    convert.parameters ()[itmTransformation][1][0] = matrix.linear ().col (1)[0];
    convert.parameters ()[itmTransformation][1][1] = matrix.linear ().col (1)[1];
    convert.parameters ()[itmTransformation][1][2] = matrix.linear ().col (1)[2];
    convert.parameters ()[itmTransformation][1][3] = 0.0;
    
    convert.parameters ()[itmTransformation][2][0] = matrix.linear ().col (2)[0];
    convert.parameters ()[itmTransformation][2][1] = matrix.linear ().col (2)[1];
    convert.parameters ()[itmTransformation][2][2] = matrix.linear ().col (2)[2];
    convert.parameters ()[itmTransformation][2][3] = 0.0;
    
    // Translation
    convert.parameters ()[itmTransformation][3][0] = matrix.translation ()[0];
    convert.parameters ()[itmTransformation][3][1] = matrix.translation ()[1];
    convert.parameters ()[itmTransformation][3][2] = matrix.translation ()[2];
    convert.parameters ()[itmTransformation][3][3] = 1.0;
    
    convert.execute ();
    json = convert.result ()[itmTransformation].asJson (pretty_format);
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "matrixTransformationToJson");
    return (false);
  }
}

pcl::uint64_t pcl::EnsensoGrabber::getPCLStamp (const double ensenso_stamp)
{
#if defined _WIN32 || defined _WIN64
  return (ensenso_stamp * 1000000.0);
#else
  return ( (ensenso_stamp - 11644473600.0) * 1000000.0);
#endif
}

std::string pcl::EnsensoGrabber::getOpenCVType (const int channels,
                        const int bpe,
                        const bool isFlt)
{
  int bits = bpe * 8;
  char type = isFlt ? 'F' : (bpe > 3 ? 'S' : 'U');
  return (boost::str (boost::format ("CV_%i%cC%i") % bits % type % channels));
}

void pcl::EnsensoGrabber::processRaw ()
{
  NxLibCommand capture(cmdCapture, "raw");  
  ros::Rate loop_rate(3);
  
  bool continue_grabbing = running_;

  while (continue_grabbing)
  {
    try
    {

        capture.parameters()[itmCameras] = serial_;
        capture.execute();
        raw_initialized_=true;
        
        if (num_slots<sig_cb_ensenso_raw_images>()>0) {
          
          boost::shared_ptr<PairOfImages> rawimages (new PairOfImages);
          int width, height, channels, bpe;
          bool isFlt;
          
          double timestamp;
          camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);
          
          camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
          rawimages->first.header.stamp = rawimages->second.header.stamp = getPCLStamp (timestamp);
          rawimages->first.width = rawimages->second.width = width;
          rawimages->first.height = rawimages->second.height = height;
          rawimages->first.data.resize (width * height * sizeof(float));
          rawimages->second.data.resize (width * height * sizeof(float));
          rawimages->first.encoding = rawimages->second.encoding = getOpenCVType (channels, bpe, isFlt);
          camera_[itmImages][itmRaw][itmLeft].getBinaryData (rawimages->first.data.data (), rawimages->first.data.size (), 0, 0);
          camera_[itmImages][itmRaw][itmRight].getBinaryData (rawimages->second.data.data (), rawimages->second.data.size (), 0, 0);
          raw_images_signal_->operator () (rawimages);
        }
    }
    catch (NxLibException &ex)
      {
        ensensoExceptionHandling (ex, "processRaw");
      }
    loop_rate.sleep();
    continue_grabbing = running_;
  }
}



void pcl::EnsensoGrabber::processPoints ()
{

  NxLibCommand disparity(cmdComputeDisparityMap, "points");
  NxLibCommand points(cmdComputePointMap, "points");
  
  ros::Rate loop_rate(2);
  
  bool continue_grabbing = running_;
 
  while (continue_grabbing)
  {
    if (!raw_initialized_) {
      continue_grabbing = running_;
      continue;
    }
    
    try
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        boost::shared_ptr<PairOfImages> rectifiedimages (new PairOfImages);
        boost::shared_ptr<pcl::PCLImage> disparityImage (new pcl::PCLImage);
        
        int width, height, channels, bpe;
        bool isFlt;
        double timestamp;

        disparity.execute();
        points.execute();
        
        if (num_slots<sig_cb_ensenso_point_cloud_images>() > 0) {
          
          camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);  // Get raw image timestamp
          
          rectifiedimages->first.header.stamp = rectifiedimages->second.header.stamp = getPCLStamp (timestamp);
          // rectifiedimages
          camera_[itmImages][itmRectified][itmLeft].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
          
          rectifiedimages->first.width = rectifiedimages->second.width = width;
          rectifiedimages->first.height = rectifiedimages->second.height = height;
          rectifiedimages->first.data.resize (width * height * sizeof(float));
          rectifiedimages->second.data.resize (width * height * sizeof(float));
          rectifiedimages->first.encoding = rectifiedimages->second.encoding = getOpenCVType (channels, bpe, isFlt);
          
          camera_[itmImages][itmRectified][itmLeft].getBinaryData (rectifiedimages->first.data.data (), rectifiedimages->first.data.size (), 0, 0);
          camera_[itmImages][itmRectified][itmRight].getBinaryData (rectifiedimages->second.data.data (), rectifiedimages->second.data.size (), 0, 0);

          std::vector<float> pointMap;
          camera_[itmImages][itmPointMap].getBinaryDataInfo (&width, &height, 0, 0, 0, 0);
          camera_[itmImages][itmPointMap].getBinaryData (pointMap, 0);
          // Copy point cloud and convert in meters
          cloud->header.stamp = getPCLStamp (timestamp);
          cloud->points.resize (height * width);
          cloud->width = width;
          cloud->height = height;
          cloud->is_dense = false;
          // Copy data in point cloud (and convert milimeters in meters)
          for (size_t i = 0; i < pointMap.size (); i += 3)
          {
            cloud->points[i / 3].x = pointMap[i] / 1000.0;
            cloud->points[i / 3].y = pointMap[i + 1] / 1000.0;
            cloud->points[i / 3].z = pointMap[i + 2] / 1000.0;
          }

          camera_[itmImages][itmDisparityMap].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
          disparityImage->header.stamp = getPCLStamp (timestamp);
          disparityImage->width = width;
          disparityImage->height = height;
          disparityImage->data.resize (width * height * sizeof(float));
          disparityImage->encoding = "CV_16SC1"; //getOpenCVType (channels, bpe, isFlt);
          camera_[itmImages][itmDisparityMap].getBinaryData (disparityImage->data.data (), disparityImage->data.size (), 0, 0);
          
          point_cloud_images_signal_->operator () (cloud, rectifiedimages, disparityImage);
        }
        
        fps_mutex_.lock ();
        times_.push_back(ros::Time::now().toSec());
        if (times_.size() > 30)
          times_.pop_front();
        fps_mutex_.unlock ();
        
        
    }
    catch (NxLibException &ex)
      {
        ensensoExceptionHandling (ex, "processPoints");
      }

    loop_rate.sleep();
    
    continue_grabbing = running_;
  }
}


void pcl::EnsensoGrabber::processMono ()
{
  NxLibCommand capture(cmdCapture, "mono");
  NxLibCommand rect(cmdRectifyImages, "mono");
  
  ros::Rate loop_rate(3);
  
  bool continue_grabbing = mono_running_;

  while (continue_grabbing)
  {
    try
    {
        
        if (num_slots<sig_cb_mono_images>()>0) {

          capture.parameters()[itmCameras] = mono_serial_;
          capture.parameters()[itmTimeout] = 3000;
          capture.execute();        
          
          rect.parameters ()[itmCameras] = mono_serial_;
          rect.execute ();
          
          
          boost::shared_ptr<pcl::PCLImage> rawimage (new pcl::PCLImage);
          boost::shared_ptr<pcl::PCLImage> rectimage (new pcl::PCLImage);
          int width, height, channels, bpe;
          bool isFlt;
          
          double timestamp;
          mono_camera_[itmImages][itmRaw].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);
          
          mono_camera_[itmImages][itmRaw].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
          rawimage->header.stamp = getPCLStamp (timestamp);
          rawimage->width = width;
          rawimage->height = height;
          rawimage->data.resize (width * height * sizeof(float));
          rawimage->encoding = getOpenCVType (channels, bpe, isFlt);
          
          mono_camera_[itmImages][itmRaw].getBinaryData (rawimage->data.data (), rawimage->data.size (), 0, 0);

          mono_camera_[itmImages][itmRectified].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);
          
          mono_camera_[itmImages][itmRectified].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
          rectimage->header.stamp = getPCLStamp (timestamp);
          rectimage->width = width;
          rectimage->height = height;
          rectimage->data.resize (width * height * sizeof(float));
          rectimage->encoding = getOpenCVType (channels, bpe, isFlt);
          
          mono_camera_[itmImages][itmRectified].getBinaryData (rectimage->data.data (), rectimage->data.size (), 0, 0);

                    
          mono_images_signal_->operator () (rawimage,rectimage);
        }
    }
    catch (NxLibException &ex)
      {
        ensensoExceptionHandling (ex, "processMono");
      }
    loop_rate.sleep();
    continue_grabbing = mono_running_;
  }
}
