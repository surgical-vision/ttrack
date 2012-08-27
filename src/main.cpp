#include "../headers/ttrack.hpp"
#include <iostream>

int main(int argc, char **argv){
  
  ttrk::TTrack &t = ttrk::TTrack::Instance();
  
  std::string dataset("./data/data_2/");

  try{
    t.RunImages("./data/data_2/");
  }catch(std::runtime_error &e){
    std::cerr << e.what() << "\n";
  }
  ttrk::TTrack::Destroy();
  return 0;

}