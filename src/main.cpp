#include "../headers/ttrack.hpp"

int main(int argc, char **argv){

  ttrk::TTrack &t = ttrk::TTrack::Instance();
  
  try{
    
    //construct the helper classes and train the classifier
    t.SetUp("./data/data_11/",ttrk::RF,ttrk::SEPARATE);
    t.RunImages("data/images/");

  }catch(std::runtime_error &e){

    std::cerr << e.what() << "\n";
#if defined(_WIN32) || defined(_WIN64)
    system("pause");
#endif
  
  }
  
  ttrk::TTrack::Destroy();

#if defined(_WIN32) || defined(_WIN64)
  _CrtDumpMemoryLeaks();
#endif

  return 0;

}
