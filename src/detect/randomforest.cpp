#include "../../include/ttrack/detect/randomforest.hpp"

using namespace ttrk;

size_t RandomForest::PredictClass(const cv::Mat &pixel) const {

  return (size_t)forest_.predict(pixel);

}


float RandomForest::PredictProb(const cv::Mat &pixel, const size_t class_index) const {

  //sum of trees that match prediction class_index
  int sum=0;
  
  //iterate over the trees, incrementing the counter when a match is found
  for(int n=0;n<forest_.get_tree_count();n++){

    CvForestTree *tree = forest_.get_tree(n);
    sum += (tree->predict(pixel)->value == class_index); //returns target class?
    
  }

  //return fraction of matching trees
  return (float)sum/forest_.get_tree_count();

}


void RandomForest::Load(const std::string &url){
    
  forest_.load(url.c_str());  

}

void RandomForest::TrainClassifier(const cv::Mat &training_data, const cv::Mat &training_labels){

  cv::RandomTreeParams params;
  params.max_depth = 25;
  params.use_surrogates = false;
  params.term_crit = cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 5, 0.001);

  cv::Mat var_type(training_data.cols + 1, 1, CV_8U);
  var_type.setTo(cv::Scalar(CV_VAR_NUMERICAL)); // all inputs are numerical
  var_type.at<uchar>(training_data.cols, 0) = CV_VAR_CATEGORICAL;

  //forest_.train(training_data, CV_ROW_SAMPLE, training_labels, var_type, cv::Mat(), cv::Mat(), cv::Mat(), params);
  forest_.train(training_data, CV_ROW_SAMPLE, training_labels, cv::Mat() , cv::Mat(), cv::Mat(), cv::Mat(), params);

}


bool RandomForest::ClassifyFrame(boost::shared_ptr<sv::Frame> frame, const cv::Mat &sdf){

  if (frame == nullptr) return false;

  assert(frame->GetImageROI().type() == CV_8UC3);

  cv::Mat whole_frame = frame->GetImage();

  const int rows = whole_frame.rows;
  const int cols = whole_frame.cols;

  static size_t frame_count = 0;

  size_t pixel_count = 0;

  cv::Mat &f = frame->GetClassificationMap();
  float *frame_data = (float *)frame->GetClassificationMap().data;
  size_t classification_map_channels = frame->GetClassificationMap().channels();

  //cv::Mat saveframe = cv::Mat::zeros(frame->GetClassificationMap().size(), CV_8UC3);


  if (frame->GetClassificationMap().depth() != CV_32F){
    throw std::runtime_error("");
  }

  //cv::Mat hsv; 

  whole_frame.convertTo(whole_frame, CV_32F);
  whole_frame *= 1.0f / 255;
  float *whole_frame_data = (float *)whole_frame.data;

  //cv::cvtColor(whole_frame, hsv, CV_BGR2HSV);
  //float *hsv_data = (float *)hsv.data;

  cv::Mat gabor_image;
  GetGabor(whole_frame, gabor_image);
  float *gabor_image_data = (float *)gabor_image.data;
  cv::Mat CIE_lab;
  cv::cvtColor(whole_frame, CIE_lab, CV_BGR2Lab);
  float *CIE_lab_data = (float *)CIE_lab.data;

  memset(frame_data, 0, f.total()*f.channels()*sizeof(float));

  cv::Mat sample(4, 1, CV_32FC1);
  float *sample_d = (float *)sample.data;

  for (int r = 0; r < rows; r++){
    for (int c = 0; c < cols; c++){

      if (sdf.at<float>(r, c) < -40) continue;

      const int index = r*cols + c;

      //red
      //a
      //o1
      //gabor

      sample_d[0] = (float)(whole_frame_data[(index * 3) + 2]);
      //sample_d[1] = (float)(hsv_data[(index * 3) + 1]);
      sample_d[1] = (float)CIE_lab_data[(index * 3) + 1];
      sample_d[2] = 0.5f * ((float)whole_frame_data[(index * 3) + 2] - (float)whole_frame_data[(index * 3) + 1]);
      //sample_d[3] = (0.5f*(float)whole_frame_data[(index * 3)]) - (0.25f*((float)whole_frame_data[(index * 3) + 2] + (float)whole_frame_data[(index * 3) + 1]));
      sample_d[3] = gabor_image_data[index];
      PredictProb(sample, &frame_data[index*classification_map_channels]);

    }
  }

  return true;



}

void RandomForest::PredictProb(const cv::Mat &sample, float *probabilities){

  for (int n = 0; n<forest_.get_tree_count(); n++){

    CvForestTree *tree = forest_.get_tree(n);
    CvDTreeNode *thing = tree->predict(sample);

    if ((size_t)(tree->predict(sample)->value) > 0)
      probabilities[1] += 1.0f;
    else
      probabilities[0] += 1.0f;
  }

  for (int i = 0; i < 2; ++i) probabilities[i] = probabilities[i] / forest_.get_tree_count();

}

void RandomForest::TrainClassifier(boost::shared_ptr<cv::Mat> training_data, boost::shared_ptr<cv::Mat> labels, boost::shared_ptr<std::string> root_dir){

  // train
  const float priors[2] = {3.0,1.0};

  CvRTParams params(10, //max depth of trees
                    500, //minimum sample count at each leaf for a split
                    0.0, //minimum regression accuracy (ignored)
                    false, //use surrogates
                    10, //maximum number of categories to cluster - ignored in 2 class case
                    priors, //priors
                    true, //calculate the variable importance
                    0, //size of random subsets (0 = sqrt(N))
                    50, //max number of trees
                    0.01, //accuracy
                    CV_TERMCRIT_ITER | CV_TERMCRIT_EPS); //halting criteria


  //CvMat *var_type = cvCreateMat(training_data->cols+1,1,CV_8U);
  //cvSet(var_type,cvScalarAll(CV_VAR_ORDERED)); //data is ordered (can be compared)
  //cvSetReal1D(var_type,training_data->cols,CV_VAR_CATEGORICAL); //labels are categorical
  cv::Mat var_type = cv::Mat::zeros(training_data->cols + 1, 1, CV_8U);// (var_type, true);
  cvSet((CvMat *)(&var_type), cvScalarAll(CV_VAR_ORDERED));
  cvSetReal1D((CvMat *)(&var_type), training_data->cols, CV_VAR_CATEGORICAL); //labels are categorical

#ifdef DEBUG
  std::cout << "Training...";
  std::cout.flush();
#endif
  
  const int num = training_data->rows;
  for(int r=0;r<num;r++){

    
    uint32_t val = labels->at<uint32_t>(r,0) ;
    assert(val == 0 || val == 1);


  }

  forest_.train(*training_data,
                    CV_ROW_SAMPLE, //samples are in row form
                    *labels,
                    cv::Mat(),//variable index, used to mask certain features from the training
                    cv::Mat(),//sample index, used to mask certain samples entirely
                    var_type,//variable type (regression or classifiaction)
                    cv::Mat(),//missing data mask
                    params);
               
  
#ifdef DEBUG
  std::cout << " Done" << std::endl;
#endif
  
  std::string classifier_save_path = *root_dir + "/classifier/";

  boost::filesystem::create_directory(boost::filesystem::path(classifier_save_path));

  forest_.save( (classifier_save_path + "forest.xml").c_str());
  
  //cvReleaseMat(&var_type);

}

std::string RandomForest::NameAsString() const {
  return "random_forest";
}


void RandomForest::Save(const std::string &url) const {
  
  boost::filesystem::create_directory(boost::filesystem::path(url));

  forest_.save( (url + "/" + NameAsString()).c_str());

}
