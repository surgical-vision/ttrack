#include "../../../headers/track/pwp3d/mono_pwp3d.hpp"
#include "../../../headers/utils/helpers.hpp"
#include<boost/filesystem.hpp>
using namespace ttrk;


/*** REMOVE THIS ***/


void MonoPWP3D::DrawModelOnFrame(const KalmanTracker &tracked_model, cv::Mat canvas) const {

  std::vector<SimplePoint<> > transformed_points = tracked_model.ModelPointsAtCurrentPose();
 
  for(auto point = transformed_points.begin(); point != transformed_points.end(); point++ ){

    cv::Vec2f projected = camera_->ProjectPoint(point->vertex_);
    
    for(auto neighbour_index = point->neighbours_.begin(); neighbour_index != point->neighbours_.end(); neighbour_index++){
      
      const SimplePoint<> &neighbour = transformed_points[*neighbour_index];
      cv::Vec2f projected_neighbour = camera_->ProjectPoint( neighbour.vertex_ );

      if(canvas.channels() == 3)
        line(canvas,cv::Point2f(projected),cv::Point2f(projected_neighbour),cv::Scalar(255,0,255),1,CV_AA);
      if(canvas.channels() == 1)
        line(canvas,cv::Point2f(projected),cv::Point2f(projected_neighbour),(unsigned char)255,1,CV_AA);
    }
  }

}


Pose MonoPWP3D::TrackTargetInFrame(KalmanTracker current_model, boost::shared_ptr<sv::Frame> frame){

  frame_ = frame;
  const int NUM_STEPS = 115;


  std::cerr << "starting pose is: "<< current_model.CurrentPose().rotation_ << " + " << cv::Point3f(current_model.CurrentPose().translation_) << std::endl;
  cv::Mat canvas = frame_->Mat().clone();
  DrawModelOnFrame(current_model,canvas);
  static int frame_count = 0;
  std::stringstream ss; ss << "frame_" << frame_count;
  boost::filesystem::create_directory(ss.str());
  cv::imwrite(ss.str()+"/step_init.png",canvas);
  frame_count++;
  

  cv::Vec3f initial_translation = current_model.CurrentPose().translation_;
  double min_energy = std::numeric_limits<double>::max();
  for(int step=0; step < NUM_STEPS; step++){

    cv::Mat sdf_image = ProjectShapeToSDF(current_model);
    cv::imwrite("sdf_image.png",sdf_image);
    //compute the normalization values n_f and n_b
    double norm_foreground,norm_background;
    ComputeNormalization(norm_foreground,norm_background,sdf_image);
    if(norm_foreground == 0) {
#ifdef DEBUG
      std::cerr << "The object is not in view!\n"; 
#endif
      return current_model.CurrentPose();
    }

    //compute the derivates of the sdf images
    cv::Mat dSDFdx, dSDFdy;
    cv::Sobel(sdf_image,dSDFdx,CV_32FC1,1,0,1);
    cv::Sobel(sdf_image,dSDFdy,CV_32FC1,0,1,1);

    //(x,y,z,w,r1,r2,r3)
    cv::Mat jacobian = cv::Mat::zeros(7,1,CV_64FC1);
    
    //cv::Mat ENERGY_IMAGE = cv::Mat::zeros(ROI_left_.size(),CV_32FC1);
    double energy = 0.0;
    
    for(int r=0;r<ROI_.rows;r++){
      for(int c=0;c<ROI_.cols;c++){
    
        //compute the energy value for this pixel - not used for pose jacobian, just for assessing minima/progress
        energy += GetEnergy(r,c,sdf_image.at<float>(r,c), norm_foreground, norm_background);

        //P_f - P_b / (H * P_f + (1 - H) * P_b)
        const double region_agreement = GetRegionAgreement(r, c, sdf_image.at<float>(r,c), norm_foreground, norm_background);

        //dH / dL
        const cv::Mat pose_derivatives = GetPoseDerivatives(r, c, sdf_image, dSDFdx.at<float>(r,c), dSDFdy.at<float>(r,c), current_model);
      
        //update the jacobian
        for(int i=0;i<pose_derivatives.rows;i++)
          jacobian.at<double>(i,0) += -1 * (region_agreement*pose_derivatives.at<double>(i,0));
        
      }
    }
    
    if(energy < min_energy) min_energy = energy;
    //else break;

    ScaleJacobian(jacobian,step);
    ApplyGradientDescentStep(jacobian,current_model.CurrentPose());
  
    cv::Mat canvas = frame_->Mat().clone();
    DrawModelOnFrame(current_model,canvas);
    std::stringstream ss_2; ss_2 << "step_" << step << ".png";
    cv::imwrite(ss.str()+"/"+ss_2.str(),canvas);


  }

  //update the velocity model... a bit crude
  cv::Vec3f translational_velocity = current_model.CurrentPose().translation_ - initial_translation;
  current_model.CurrentPose().translational_velocity_ = translational_velocity;

  return current_model.CurrentPose();
}


void MonoPWP3D::FindROI(const std::vector<cv::Vec2i> &convex_hull) {

  ROI() = frame_->Mat(); //UNTIL I DO THIS FUNCTION

  /*for(int r=0;r<frame_->rows();r++){
    for(int c=0;c<frame_->cols();c++){



    }
  }*/
  
}
