#include "../headers/ttrack.hpp"

using namespace ttrk;

bool TTrack::constructed_ = false;
TTrack *TTrack::instance_ = 0;

TTrack::TTrack(){}

TTrack::TTrack(const TTrack &that){
  *this = that;
}

TTrack &TTrack::operator=(const TTrack &that){

  // check for self-assignment
  if(this == &that) return *this;
  tracker_ = that.tracker_;
  detector_ = that.detector_;
  current_frame_ = that.current_frame_;

  return *this;

}

TTrack &TTrack::Instance(){

  if(!constructed_){
    instance_ = new TTrack();
    constructed_ = true;
  }
  return *instance_;
}


void TTrack::RunVideo(const std::string in_videofile, const std::string out_videofile,
                      const std::string out_posefile){


}


void TTrack::RunImages(const std::string in_images, const std::string out_images,
                       const std::string out_posefile){


}

void TTrack::SaveFrame(const std::string url) const {


}

void ToggleVerbose(const bool toggle){



}

