#pragma once
#include <string>

namespace std_msgs
{
  struct Header{
//	  unsigned int seq;
	  std::string frame_id;
	  double stamp;

  };
}
