#include <cstdlib>

#define ROS_BREAK() std::abort();

#define ROS_INFO(...) {printf (__VA_ARGS__);printf("\n");}
#define ROS_INFO_STREAM(args) {std::cout<<args;}
#define ROS_WARN_STREAM(args) {std::cout<<args;}
#define ROS_DEBUG_STREAM(args) {std::cout<<args;}
#define ROS_WARN(...) {printf (__VA_ARGS__);printf("\n");}
#define ROS_DEBUG(...) {printf (__VA_ARGS__);printf("\n");}
#define ROS_INFO_STREAM(args) {std::cout<<args;}
