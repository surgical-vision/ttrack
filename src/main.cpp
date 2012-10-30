#include "../headers/ttrack.hpp"

int main(int argc, char **argv){

  ttrk::TTrack &t = ttrk::TTrack::Instance();
  
  try{
    
    t.SetUp("./data/data_11",ttrk::RF,ttrk::X_VALIDATE);
    t.RunImages();
    //t.TestDetector("./data/data_11/data/test_images/image_19.png","output.png");

  }catch(std::runtime_error &e){
    std::cerr << e.what() << "\n";
    system("pause");
  }

  ttrk::TTrack::Destroy();
#ifdef _WIN32 | _WIN64
  _CrtDumpMemoryLeaks();
#endif
  return 0;

}
