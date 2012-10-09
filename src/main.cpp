#include "../headers/ttrack.hpp"
#include <iostream>

int main(int argc, char **argv){

  ttrk::TTrack &t = ttrk::TTrack::Instance();
  
  t.SetUpDirectoryTree("./data/data_11",true);

  try{
    //t.RunImages();
    t.Train();
    //t.TestDetector("./data/data_11/data/test_images/image_19.png","output.png");

  }catch(std::runtime_error &e){
    std::cerr << e.what() << "\n";
  }

  ttrk::TTrack::Destroy();

  return 0;

}
