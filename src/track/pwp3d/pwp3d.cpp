#include "../../../headers/track/pwp3d/pwp3d.hpp"
#include "../../../headers/utils/helpers.hpp"
#include <boost/math/special_functions/fpclassify.hpp>
#include <ctime>
using namespace ttrk;

void PWP3D::ApplyGradientDescentStep(const cv::Mat &jacobian, Pose &pose, const int step, const int pixel_count){

  cv::Mat scaled_jacobian;
  jacobian.copyTo(scaled_jacobian);

  ScaleJacobian(scaled_jacobian,step,pixel_count);

  //update the translation
  cv::Vec3f translation = scaled_jacobian(cv::Range(0,3),cv::Range::all());
  pose.translation_ = pose.translation_ + translation;

  //update the rotation
  sv::Quaternion rotation( boost::math::quaternion<double>((float)scaled_jacobian.at<double>(3,0),(float)scaled_jacobian.at<double>(4,0),(float)scaled_jacobian.at<double>(5,0),(float)scaled_jacobian.at<double>(6,0)));

  pose.rotation_ = pose.rotation_ + rotation;
  pose.rotation_ = pose.rotation_.Normalize();
#ifdef SAVEDEBUG_2
  std::cerr << "Pose is now: " << pose << "\n";
#endif

}

void PWP3D::ScaleJacobian(cv::Mat &jacobian, const int step_number, const int pixel_count) const {

#ifdef SAVEDEBUG_2
  std::cerr << "Unscaled Jacobian = " << jacobian << "\n";
#endif

  double sum_jacobian = 0.0;
  for(int r=0;r<jacobian.rows;r++){
    sum_jacobian += std::abs(jacobian.at<double>(r,0));
  }

  static bool swap = true;
  
  const double SCALE_FACTOR =  (1.0/(pixel_count)) * 3;
    
  const double XY_scale = 0.2 * 0.005 * swap * SCALE_FACTOR;
  const double Z_scale = 0.5 * 0.005 * swap * SCALE_FACTOR;
  double R_SCALE = 10 * 0.00008 * swap * SCALE_FACTOR;
  
  jacobian.at<double>(0,0) *= XY_scale;//* 25;
  jacobian.at<double>(1,0) *= XY_scale ;
  jacobian.at<double>(2,0) *= Z_scale; //* camera_->Fx()/4;
  
  double largest = std::abs(jacobian.at<double>(0,0));
  if( largest < std::abs(jacobian.at<double>(1,0)) ) largest = std::abs(jacobian.at<double>(1,0));
  if( largest < std::abs(jacobian.at<double>(2,0)) ) largest = std::abs(jacobian.at<double>(2,0));

  jacobian.at<double>(0,0) = 0.5 * (jacobian.at<double>(0,0)*0.2)/largest;
  jacobian.at<double>(1,0) = 0.5 * (jacobian.at<double>(1,0)*0.2)/largest;
  jacobian.at<double>(2,0) = 0.5 * (jacobian.at<double>(2,0)*0.4)/largest;

  //jacobian.at<double>(5,0) *= 3;

  largest = std::abs(jacobian.at<double>(3,0));
  if( largest < std::abs(jacobian.at<double>(4,0)) ) largest = std::abs(jacobian.at<double>(4,0));
  if( largest < std::abs(jacobian.at<double>(5,0)) ) largest = std::abs(jacobian.at<double>(5,0));
  if( largest < std::abs(jacobian.at<double>(6,0)) ) largest = std::abs(jacobian.at<double>(6,0));

  for(int i=3;i<7;i++){
    jacobian.at<double>(i,0) *= 0.5 * (0.007 / largest);
  }

#ifdef SAVEDEBUG_1
  std::cerr << "Jacobian = " << jacobian << "\n";
#endif

  return;

  
}




double PWP3D::GetEnergy(const int r, const int c, const float sdf) const {

  const double pixel_probability = (double)frame_->GetClassificationMapROI().at<unsigned char>(r,c)/255.0;
  const double foreground_probability = pixel_probability;
  const double background_probability = (1-pixel_probability);
  
  double x = (Heaviside(sdf, k_heaviside_width_ * blurring_scale_factor_)*foreground_probability + (1-Heaviside(sdf, k_heaviside_width_ * blurring_scale_factor_))*background_probability);
  if ( x == std::numeric_limits<double>::infinity() )
    return 1000;  
  return x;

}

double PWP3D::GetRegionAgreement(const int r, const int c, const float sdf) {
  
  const double alpha = 1.0;//0.3; // DEBUGGING TEST !!!!
  const double beta = 1.0;//0.7; // DEBUGGING TEST !!!!

  const double pixel_probability = (double)frame_->GetClassificationMapROI().at<unsigned char>(r,c)/255.0;
  
  const double foreground_probability = pixel_probability;
  const double background_probability = (1.0 - pixel_probability);

  const double region_agreement = (foreground_probability - background_probability)/ (Heaviside(sdf, k_heaviside_width_ * blurring_scale_factor_)*foreground_probability + (1.0-Heaviside(sdf, k_heaviside_width_ * blurring_scale_factor_))*background_probability);
  if(region_agreement == std::numeric_limits<double>::infinity())
    return 0.0;
  return region_agreement;

}



bool PWP3D::GetTargetIntersections(const int r, const int c, cv::Vec3f &front_intersection, cv::Vec3f &back_intersection, const KalmanTracker &current_model, const cv::Mat &front_intersection_image, const cv::Mat &back_intersection_image) const {

  front_intersection = front_intersection_image.at<cv::Vec3f>(r,c);
  back_intersection = back_intersection_image.at<cv::Vec3f>(r,c);

  return  front_intersection != cv::Vec3f(0,0,0);

}


bool PWP3D::GetNearestIntersection(const int r, const int c, const cv::Mat &sdf, cv::Vec3f &front_intersection, cv::Vec3f &back_intersection, const KalmanTracker &current_model, const cv::Mat &front_intersection_image, const cv::Mat &back_intersection_image) const {

  static const int search_range= 20; //has to be larger than border for search to work!
  //if(sdf.at<float>(r,c) <  -10 || sdf.at<float>(r,c) >= 0) return false;
  if(r == 338 && c == 211)
    int asfjalsfj = 0;
  cv::Point2i min_point;
  const cv::Point2i start_pt(c,r);
  float min_dist = std::numeric_limits<float>::max();
  int max_r = r+search_range, min_r = r-search_range, max_c = c+search_range, min_c = c-search_range;
  if(max_r >= sdf.rows) max_r = sdf.rows-1;
  if(max_c >= sdf.cols) max_c = sdf.cols-1;
  if(min_r < 0) min_r = 0;
  if(min_c < 0) min_c = 0;

  for(int row=min_r;row<max_r;row++){
    for(int col=min_c;col<max_c;col++){
      
      if(sdf.at<float>(row,col) > 1.5 || sdf.at<float>(row,col) < 0.5) continue; //too far 'inside' or outside - we want teh tangent rays. need to aim for points 1 pixel inside to avoid slight inaccuracies reporting a miss.
      const cv::Point2i test_pt(col,row);
      const cv::Point2i dist = test_pt - start_pt;
      float dist_val = (float)sqrt((float)dist.x*dist.x + dist.y*dist.y);
      if(dist_val < min_dist){
        min_dist = dist_val;
        min_point = test_pt;
      }
    }
  }

  if(min_dist == std::numeric_limits<float>::max()) return false;

  front_intersection = front_intersection_image.at<cv::Vec3f>(min_point.y,min_point.x);
  back_intersection = back_intersection_image.at<cv::Vec3f>(min_point.y,min_point.x);

  return  front_intersection != cv::Vec3f(0,0,0);
}

cv::Mat PWP3D::GetPoseDerivatives(const int r, const int c, const cv::Mat &sdf, const float dSDFdx, const float dSDFdy, KalmanTracker &current_model, const cv::Mat &front_intersection_image, const cv::Mat &back_intersection_image){

  const int NUM_DERIVS = 7;
     
   //find the (x,y,z) coordinates of the front and back intersection between the ray from the current pixel and the target object. return zero vector for no intersection.
  cv::Vec3f front_intersection;
  cv::Vec3f back_intersection;
  bool intersects = GetTargetIntersections(r,c,front_intersection,back_intersection,current_model,front_intersection_image,back_intersection_image);
  
  //because the derivative only works for points which project to the target and we need it to be defined for points outside the contour, 'pretend' that a small region of these points actually hit the contour
  if(!intersects) {
    intersects = GetNearestIntersection(r,c,sdf,front_intersection,back_intersection,current_model,front_intersection_image,back_intersection_image);
    if(!intersects)
      return cv::Mat::zeros(NUM_DERIVS,1,CV_64FC1);
  }

  if(front_intersection == cv::Vec3f(0,0,0) || back_intersection == cv::Vec3f(0,0,0)) throw(std::runtime_error("Error, this is a value we should not get!\n"));


  //just in case...
  if(front_intersection[2] == 0) front_intersection[2] = 0.00000001;
  const double z_inv_sq_front = 1.0/(front_intersection[2]*front_intersection[2]);
  if(back_intersection[2] == 0) back_intersection[2] = 0.00000001;
  const double z_inv_sq_back = 1.0/(back_intersection[2]*back_intersection[2]);

  cv::Mat ret(NUM_DERIVS,1,CV_64FC1);
  for(int dof=0;dof<NUM_DERIVS;dof++){
    
    //compute the derivative for each dof
    const cv::Vec3f dof_derivatives_front = current_model.CurrentPose().GetDOFDerivatives(dof,front_intersection);
    const cv::Vec3f dof_derivatives_back = current_model.CurrentPose().GetDOFDerivatives(dof,back_intersection);

    const double dXdL = camera_->Fx() * (z_inv_sq_front*((front_intersection[2]*dof_derivatives_front[0]) - (front_intersection[0]*dof_derivatives_front[2]))) + camera_->Fx() * (z_inv_sq_back*((back_intersection[2]*dof_derivatives_back[0]) - (back_intersection[0]*dof_derivatives_back[2])));
    const double dYdL = camera_->Fy() * (z_inv_sq_front*((front_intersection[2]*dof_derivatives_front[1]) - (front_intersection[1]*dof_derivatives_front[2]))) + camera_->Fy() * (z_inv_sq_back*((back_intersection[2]*dof_derivatives_back[1]) - (back_intersection[1]*dof_derivatives_back[2])));
    ret.at<double>(dof,0) = DeltaFunction(sdf.at<float>(r,c), k_delta_function_std_ * blurring_scale_factor_ ) * ((dSDFdx * dXdL) + (dSDFdy * dYdL));
  
  }
  return ret;

}



void PWP3D::GetSDFAndIntersectionImage(KalmanTracker &current_model, cv::Mat &sdf_image, cv::Mat &front_intersection_image, cv::Mat &back_intersection_image) {

  //find all the pixels which project to intersection points on the model
  sdf_image = cv::Mat(frame_->GetImageROI().size(),CV_32FC1);
  front_intersection_image = cv::Mat::zeros(frame_->GetImageROI().size(),CV_32FC3);
  back_intersection_image = cv::Mat::zeros(frame_->GetImageROI().size(),CV_32FC3);

  //first draw the model at the current pose
  cv::Mat canvas = cv::Mat::zeros(frame_->GetImageROI().size(),CV_8UC1);
  std::vector<SimplePoint<> > transformed_points = current_model.ModelPointsAtCurrentPose();
  DrawModelOnFrame(transformed_points,canvas);
  cv::imwrite("canvas.png",canvas);


  //find the set of pixels which correspond to the drawn object
  std::vector<cv::Point2i> set_of_points;
  for(int r=0;r<sdf_image.rows;r++){
    for(int c=0;c<sdf_image.cols;c++){
      if(canvas.at<unsigned char>(r,c) == 255) set_of_points.push_back(cv::Point2i(c,r));
    }
  }

  //find it's convex hull and draw that
  std::vector<cv::Point2i> convex_hull;
  cv::convexHull(set_of_points,convex_hull);
  cv::Mat canvas_2 = cv::Mat::zeros(frame_->GetImageROI().size(),CV_8UC1);
  for(auto point=convex_hull.begin();point!=(convex_hull.end());){
    cv::Point a = *point;
    point++;
    if(point == convex_hull.end())break;
    cv::Point b = *point;
    cv::line(canvas_2,a,b,255,2);
  }
  cv::line(canvas_2,*convex_hull.begin(),*(convex_hull.end()-1),255,2);
  
  //color in the convex hull to make a mask
  std::vector<cv::Point2i> contour;
  cv::Vec2i center(0,0);
  for(int r=0;r<sdf_image.rows;r++){
    for(int c=0;c<sdf_image.cols;c++){
      if(canvas_2.at<unsigned char>(r,c) == 255) {
        contour.push_back(cv::Point2i(c,r));
        center += cv::Vec2i(c,r);
      }
    }
  }
  center[0] = center[0]/contour.size();
  center[1] = center[1]/contour.size();
  cv::floodFill(canvas_2,cv::Point(center),255);
  cv::Mat canvas_2_dilated;
  cv::dilate(canvas_2,canvas_2_dilated,cv::Mat(20,20,CV_8UC1));

  std::ofstream ofs("newpoints.xyz");

  cv::Mat pixels_intersect = cv::Mat::zeros(canvas_2_dilated.size(),CV_8UC1);
  for(int r=0;r<sdf_image.rows;r++){
    for(int c=0;c<sdf_image.cols;c++){
      if(canvas_2_dilated.at<unsigned char>(r,c) != 255) continue;
      cv::Vec3f ray = camera_->UnProjectPoint( cv::Point2i(c,r) );
      cv::Vec3f front_intersection(0,0,0),back_intersection(0,0,0);
      if (current_model.PtrToModel()->GetIntersection(ray, front_intersection , back_intersection ,current_model.CurrentPose())){
        ofs << front_intersection[0] << " " << front_intersection[1] << " " << front_intersection[2] << "\n";
        ofs << back_intersection[0] << " " << back_intersection[1] << " " << back_intersection[2] << "\n";
        front_intersection_image.at<cv::Vec3f>(r,c) = front_intersection;
        back_intersection_image.at<cv::Vec3f>(r,c) = back_intersection;
        pixels_intersect.at<unsigned char>(r,c) = 255;
      }else{
        front_intersection_image.at<cv::Vec3f>(r,c) = cv::Vec3f(0,0,0);
        back_intersection_image.at<cv::Vec3f>(r,c) = cv::Vec3f(0,0,0);  
      }

    }
  }



  //take this binary image and find the outer contour of it. then make a distance image from that contour.
  cv::Mat edge_image(pixels_intersect.size(),CV_8UC1);
  cv::Canny(pixels_intersect,edge_image,1,100);
  distanceTransform(~edge_image,sdf_image,CV_DIST_L2,CV_DIST_MASK_PRECISE);

  //flip the sign of the distance image for outside pixels
  for(int r=0;r<sdf_image.rows;r++){
    for(int c=0;c<sdf_image.cols;c++){
      if(pixels_intersect.at<unsigned char>(r,c) != 255)
        sdf_image.at<float>(r,c) *= -1;
    }
  }

#ifdef SAVEDEBUG_2
  std::srand(std::time(0));
  cv::Mat nearest_intersection_image = cv::Mat::zeros(canvas_2_dilated.size(),CV_8UC3);
  for(int r=0;r<sdf_image.rows;r++){
    for(int c=0;c<sdf_image.cols;c++){

      const double skip = Heaviside(sdf_image.at<float>(r,c), k_heaviside_width_ * blurring_scale_factor_);
      if( skip < 0.00001 || skip > 0.99999 ) continue;

      cv::Vec3f front_intersection;
      cv::Vec3f back_intersection;
      bool intersects = GetTargetIntersections(r,c,front_intersection,back_intersection,current_model,front_intersection_image,back_intersection_image);

      //because the derivative only works for points which project to the target and we need it to be defined for points outside the contour, 'pretend' that a small region of these points actually hit the contour
      if(!intersects) {
        intersects = GetNearestIntersection(r,c,sdf_image,front_intersection,back_intersection,current_model,front_intersection_image,back_intersection_image);
        if(intersects){
          float random_variable = (float)std::rand()/RAND_MAX; 
          if(random_variable > 0.95)
            cv::line(nearest_intersection_image,cv::Point2i(c,r),camera_->ProjectPointToPixel(front_intersection),cv::Scalar(255,0,24),1);
        }
      }
    }
  }

  cv::imwrite("debug/nearest_intersection.png",nearest_intersection_image);

#endif

}


bool PWP3D::ModelInFrame( const KalmanTracker &tracked_model, const cv::Mat &detect_image) const {

  const int rows = detect_image.rows;
  const int cols = detect_image.cols;
  
  size_t tp = 0;
  size_t fp = 0;
  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){
      cv::Vec3f ray = camera_->UnProjectPoint( cv::Point2i(c,r) );
      tp += tracked_model.PtrToModel()->GetIntersection(ray, cv::Vec3f() , cv::Vec3f() ,tracked_model.CurrentPose()) && detect_image.at<unsigned char>(r,c) >= 127;
      fp += tracked_model.PtrToModel()->GetIntersection(ray, cv::Vec3f() , cv::Vec3f() ,tracked_model.CurrentPose()) && detect_image.at<unsigned char>(r,c) < 127;
    }
  }
  
  return ((float)tp / (tp+fp) ) > 0.75;

}

cv::Point FindNearest(const int r, const int c, cv::Mat im){

  float distance = std::numeric_limits<float>::max();
  cv::Point best(-1,-1);

  //std::cout << "MAtching to " << cv::Point(c,r) << std::endl;

  for(int v=1;v<15;v++){

    for( int y = (r-v) ; y < (r+v) ; y++ ){
      for( int x = (c-v) ; x < c+(2*v)+1 ; x = x +(2*v) ){
        
        if(y < 0 || x < 0 || y > im.rows-1 || x > im.cols-1) continue;

        if(im.at<unsigned char>(y,x) == 255) {

          if(distance > l2_distance(cv::Vec2f(y,x),cv::Vec2f(r,c))){
            distance = l2_distance(cv::Vec2f(y,x),cv::Vec2f(r,c));
            best = cv::Point(x,y);
          }

        }
      }
    }

    for(int x=(c-v);x<(c+v);x++){
      for( int y = r-v; y<r+(2*v)+1; y = y+(2*v) ){

        if(y < 0 || x < 0 || y > im.rows-1 || x > im.cols-1) continue;

        if(im.at<unsigned char>(y,x) == 255) {

          if(distance > l2_distance(cv::Vec2f(y,x),cv::Vec2f(r,c))){
            distance = l2_distance(cv::Vec2f(y,x),cv::Vec2f(r,c));
            best = cv::Point(x,y);
          }

        }
      }
    }

    if(distance !=std::numeric_limits<float>::max()){
      //std::cerr << "Matched to " << best << "\n";
      return best;
    }

  }

  return cv::Point(-1,-1);

}

cv::Mat PWP3D::AlignObjectToEdges(KalmanTracker &current_model, const cv::Mat &frame, const cv::Mat &sdf_image, const cv::Mat &front_projection_image, cv::Mat &save_image) {

  return cv::Mat::zeros(7,1,CV_64FC1);

  cv::Mat blur;

  cv::resize(frame,blur,cv::Size(frame.cols/4,frame.rows/4));
  
  cv::Mat edges;
  cv::imwrite("debug/blurred.png",blur);
  cv::Canny(blur,edges,7,30);
  cv::resize(edges,blur,cv::Size(frame.cols,frame.rows));
  edges = blur;
  cv::imwrite("debug/edges.png",edges);
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP( edges, lines, 1, CV_PI/180, 60, 40, 10);
  cv::Mat im = cv::Mat::zeros(frame.size(),CV_8UC1);
  for( size_t i = 0; i < lines.size(); i++ ){
    cv::line( im, cv::Point(lines[i][0], lines[i][1]),
      cv::Point(lines[i][2], lines[i][3]), cv::Scalar(255), 1, 8 );
  }
  cv::imwrite("debug/lines.png",im);
  std::vector<cv::Point>object_points;
  std::vector<std::pair<cv::Point,cv::Point> >nearest_points;
  for(int r=0;r<sdf_image.rows;r+=2){
    for(int c=0;c<sdf_image.cols;c+=2){
      if(sdf_image.at<float>(r,c) < 1 && sdf_image.at<float>(r,c) > -1.0){
        object_points.push_back(cv::Point(c,r));
      }
    }
  }

  std::random_shuffle(object_points.begin(),object_points.end());
  for(auto point = object_points.begin(); point != object_points.end(); point++){

    cv::Point nearest = FindNearest(point->y,point->x,im);
    if(nearest == cv::Point(-1,-1) || front_projection_image.at<cv::Vec3f>(point->y,point->x) == cv::Vec3f(0,0,0))
      continue;

    nearest_points.push_back(std::make_pair<>(*point,nearest));

#ifdef SAVEDEBUG_2
    cv::circle(save_image,*point,2,cv::Scalar(255,0,0),1); //point on object
    cv::circle(save_image,nearest,2,cv::Scalar(0,0,255),1); //target in image
    cv::line(save_image,*point,nearest,cv::Scalar(13,189,12),1);
#endif

    if(nearest_points.size() > 200)
      break;

  }
  //static int count = 0;
  //std::stringstream ss; ss << "image" << count << ".png";
  //cv::imwrite(ss.str(),canvas);
  //count++;

  cv::Mat drawim = cv::Mat::zeros(im.size(),CV_8UC3);
  for(int r=0;r<sdf_image.rows;r+=2){
    for(int c=0;c<sdf_image.cols;c+=2){
      drawim.at<cv::Vec3b>(r,c) = cv::Vec3b(edges.at<unsigned char>(r,c),0,0);

    }
  }

  cv::Mat derivs = cv::Mat::zeros(7,1,CV_64FC1);

  for(auto matched_edge = nearest_points.begin(); matched_edge != nearest_points.end(); matched_edge++){

     cv::Vec3f point_on_object = front_projection_image.at<cv::Vec3f>(matched_edge->first.y,matched_edge->first.x);
      
     if(point_on_object == cv::Vec3f(0,0,0))throw(std::runtime_error("Err, should not happen!\n"));

     derivs += register_points_.GetPointDerivative(cv::Point3f(point_on_object),cv::Point2f(matched_edge->second),current_model.CurrentPose());

   }
   
   return derivs;//cv::Mat();
 }

 bool CheckEdgesForIntersection(const cv::Point &a, const cv::Point &b, const cv::Size &image_size, cv::Point &intersection){
   if(FindIntersection(a, b, cv::Point2i(0,0), cv::Point(0,image_size.height-1),intersection)){
     return true;
   }else if(FindIntersection(a, b, cv::Point2i(0,0), cv::Point(image_size.width-1,0) ,intersection)){
    return true;
  }else if(FindIntersection(a, b, cv::Point2i(0,image_size.height-1), cv::Point(image_size.width-1,image_size.height-1),intersection)){
    return true;
  }else if(FindIntersection(a, b, cv::Point2i(image_size.width-1,0), cv::Point(image_size.width-1,image_size.height-1),intersection)){
    return true;
  }
  return false;
}

void PWP3D::DrawModelOnFrame(const std::vector<SimplePoint<> > &transformed_points, cv::Mat canvas) {

  for(auto point = transformed_points.begin(); point != transformed_points.end(); point++ ){

    cv::Vec2f projected = camera_->ProjectPoint(point->vertex_);

    for(auto neighbour_index = point->neighbours_.begin(); neighbour_index != point->neighbours_.end(); neighbour_index++){

      const SimplePoint<> &neighbour = transformed_points[*neighbour_index];
      cv::Vec2f projected_neighbour = camera_->ProjectPoint( neighbour.vertex_ );

      if(!PointInImage(cv::Point2i(projected),canvas.size()) || !PointInImage(cv::Point2i(projected_neighbour),canvas.size())){
        cv::Point intersection;
        bool intersected = CheckEdgesForIntersection(cv::Point2i(projected),cv::Point2i(projected_neighbour),canvas.size(),intersection);
        if(intersected && !PointInImage(cv::Point2i(projected),canvas.size()) && PointInImage(cv::Point2i(projected_neighbour),canvas.size())){
          projected = cv::Vec2i(intersection);          
        } 
        else if(intersected && PointInImage(cv::Point2i(projected),canvas.size()) && !PointInImage(cv::Point2i(projected_neighbour),canvas.size())){
          projected_neighbour = cv::Vec2i(intersection);
        }
        else if(!PointInImage(cv::Point2i(projected),canvas.size()) && !PointInImage(cv::Point2i(projected_neighbour),canvas.size())){
          continue;
        }
      }

      if(canvas.channels() == 3)
        line(canvas,cv::Point2f(projected),cv::Point2f(projected_neighbour),cv::Scalar(255,123,25),1,CV_AA);
      if(canvas.channels() == 1)
        line(canvas,cv::Point2f(projected),cv::Point2f(projected_neighbour),(unsigned char)255,1,CV_AA);
    }
  }

}

bool PWP3D::HasGradientDescentConverged_UsingEnergy(std::vector<double> &energy_values) const {
#ifdef SAVEDEBUG_2
  std::cerr << "Energy for this step is " << energy_values.back() << "\n";
#endif
  const int NUM_VALUES_TO_USE = 7;
  if(energy_values.size() < NUM_VALUES_TO_USE ) return false;


  double energy_change = 0.0;
  for(auto value = energy_values.end()-(NUM_VALUES_TO_USE); value != energy_values.end()-1; value++ ){

    //std::cerr << "Energy change is: " << *(value+1) - *(value) << "\n";
    energy_change += *(value+1) - *(value);
    //std::cerr << "New energy is: " << energy_change << "\n";

  }

  energy_change /= NUM_VALUES_TO_USE - 1;
#ifdef SAVEDEBUG_2
  std::cerr << "Current ratio is: " <<  energy_change/energy_values.back() << "\n";
  std::cerr << "Target ratio for convergence is: " << 1.0/1000 << "\n";
#endif
  return !(energy_change/energy_values.back() > 1.0/1000);

}


