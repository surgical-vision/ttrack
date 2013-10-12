#include "../headers/ttrack.hpp"
#include <boost/program_options/option.hpp>
#if defined _WIN32 || _WIN64
  #include<direct.h>
#elif defined __linux__
#endif

int main(int argc, char **argv){

  ttrk::TTrack &t = ttrk::TTrack::Instance();

  try{
    
#if defined(_WIN32) || defined(_WIN64)
     _chdir("../");
#endif
     
     //construct the helper classes and train the classifier
     t.SetUp("./data/test_video/",ttrk::RF,ttrk::STEREO);
     //t.SetUp("./data/new_video/",ttrk::RF,ttrk::STEREO);
     //t.SetUp("./data/test_video/",ttrk::RF,ttrk::MONOCULAR);
     
     t.RunVideo("video.avi");
     //t.RunVideo("short2.avi");
     //t.RunVideo("left2.avi","right2.avi");


  }catch(std::runtime_error &e){

    std::cerr << e.what() << "\n";
#if defined(_WIN32) || defined(_WIN64)
    system("pause");
#endif
  
  }
  
  ttrk::TTrack::Destroy();

#if defined(_WIN32) || defined(_WIN64)
  _CrtDumpMemoryLeaks();
  system("pause");
#endif

  return 0;

}



