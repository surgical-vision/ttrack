#include "../headers/randomforest.hpp"

using namespace ttrk;

size_t RandomForest::PredictClass(const cv::Vec3b &pixel) const {

  return forest_.predict(cv::Mat(pixel));

}


float RandomForest::PredictProb(const cv::Vec3b &pixel, const size_t class_index) const {

  //sum of trees that match prediction class_index
  int sum=0;
  
  //iterate over the trees, incrementing the counter when a match is found
  for(int n=0;n<forest_.get_tree_count();n++){

    CvForestTree *tree = forest_.get_tree(n);
    sum += (tree->predict(cv::Mat(pixel))->value == class_index); //returns target class?
    
  }

  //return fraction of matching trees
  return sum/forest_.get_tree_count();

}


void RandomForest::Load(const std::string &url){
    
  	forest_.load(url.c_str());  

}


void RandomForest::TrainClassifier(const cv::Mat &training_data, const cv::Mat &labels, const std::string &root_dir){

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


  CvMat *var_type = cvCreateMat(training_data.cols+1,1,CV_8U);
  cvSet(var_type,cvScalarAll(CV_VAR_ORDERED)); //data is ordered (can be compared)
  cvSetReal1D(var_type,training_data.cols,CV_VAR_CATEGORICAL); //labels are categorical
  cv::Mat var_type_(var_type,true);

#ifdef DEBUG
  std::cout << "Training...";
  std::cout.flush();
#endif

  forest_.train(training_data,
                    CV_ROW_SAMPLE, //samples are in row form
                    labels,
                    cv::Mat(),//variable index, used to mask certain features from the training
                    cv::Mat(),//sample index, used to mask certain samples entirely
                    var_type,//variable type (regression or classifiaction)
                    cv::Mat(),//missing data mask
                    params);
               
   
#ifdef DEBUG
  std::cout << " Done" << std::endl;
#endif
  
  std::string classifier_save_path = root_dir + "/classifier/";

  boost::filesystem::create_directory(boost::filesystem::path(classifier_save_path));

  forest_.save( (classifier_save_path + "forest.xml").c_str());
 
  cvReleaseMat(&var_type);

}



