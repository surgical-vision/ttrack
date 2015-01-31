#include<boost/filesystem.hpp>

#include "../../../include/track/pwp3d/mono_pwp3d.hpp"
#include "../../../include/utils/helpers.hpp"

using namespace ttrk;

MonoPWP3D::MonoPWP3D(boost::shared_ptr<MonocularCamera> camera) : camera_(camera), PWP3D(camera->Width(), camera->Height()) {

  LoadShaders();

}


void MonoPWP3D::TrackTargetInFrame(boost::shared_ptr<Model> current_model, boost::shared_ptr<sv::Frame> frame){

  //frame_ = frame;
  //SetBlurringScaleFactor(frame_->GetImageROI().cols);
  //const int NUM_STEPS = 50;
  //cv::Vec3d initial_translation = current_model.CurrentPose().translation_;
  //
  ////iterate until convergence
  //for(int step=0,pixel_count=0; step < NUM_STEPS; ++step,pixel_count=0){

  //  //(x,y,z,w,r1,r2,r3)
  //  PoseDerivs image_pose_derivatives = PoseDerivs::Zeros();
  //  cv::Mat canvas, z_buffer, binary_image;
  //  GetRenderedModelAtPose(current_model, canvas, z_buffer, binary_image);
  //  cv::Mat sdf_image,front_intersection_image,back_intersection_image;
  //  ProcessSDFAndIntersectionImage(current_model, z_buffer, sdf_image, binary_image, front_intersection_image, back_intersection_image);

  //  //compute the derivates of the sdf images
  //  cv::Mat dSDFdx, dSDFdy;
  //  cv::Scharr(sdf_image,dSDFdx,CV_64FC1,1,0);
  //  cv::Scharr(sdf_image,dSDFdy,CV_64FC1,0,1);

  //  //do region based tracking
  //  for(int r=0;r<frame_->GetImageROI().rows;r+=3){
  //    for(int c=0;c<frame_->GetImageROI().cols;c+=3,++pixel_count){

  //      //speedup tests by checking if we need to evaluate the cost function in this region
  //      const double skip = Heaviside(sdf_image.at<double>(r,c), k_heaviside_width_ * blurring_scale_factor_);
  //      if( skip < 0.00001 || skip > 0.99999 ) continue;

  //      //P_f - P_b / (H * P_f + (1 - H) * P_b)
  //      const double region_agreement = GetRegionAgreement(r, c, sdf_image.at<double>(r,c));

  //      //dH / dL
  //      PoseDerivs per_pixel_pose_derivatives;
  //      GetPoseDerivatives(r, c, sdf_image, dSDFdx.at<double>(r,c), dSDFdy.at<double>(r,c), current_model, front_intersection_image, back_intersection_image, per_pixel_pose_derivatives);
  //      per_pixel_pose_derivatives.MultiplyByValue(-1 * region_agreement);

  //      image_pose_derivatives.AddValues(per_pixel_pose_derivatives);

  //    }
  //  }

  //  //do point based tracking
  //  std::vector<MatchedPair> pnp_pairs;
  //  cv::Mat point_save_image = frame_->GetImageROI().clone();
  //  register_points_.FindPointCorrespondencesWithPose(frame_,current_model.PtrToModel(),current_model.CurrentPose(),point_save_image);
  //  for(auto pnp=pnp_pairs.begin();pnp!=pnp_pairs.end();pnp++){
  //    register_points_.GetPointDerivative(pnp->learned_point,cv::Point2d(pnp->image_point[0],pnp->image_point[1]), current_model.CurrentPose(), image_pose_derivatives);
  //  } 

  //  //scale jacobian and update pose
  //  ApplyGradientDescentStep(image_pose_derivatives,current_model.CurrentPose(),step,pixel_count);
  //    
  //}
  //
  ////update the velocity model... a bit crude
  //cv::Vec3d translational_velocity = current_model.CurrentPose().translation_ - initial_translation;
  //current_model.CurrentPose().translational_velocity_ = translational_velocity;
  //
  //return current_model.CurrentPose();
}



//void MonoPWP3D::GetFastDOFDerivs(const Pose &pose, double *pose_derivs, double *intersection) {
//  pose.GetFastDOFDerivs(pose_derivs,intersection);
//}
