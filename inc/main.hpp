/**
* @file main.hpp
* @author vss2sn
* @brief Just includes includes. Needs to be refactored.
*/

#include <iostream>
#include <cstring>
#include <string>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fstream>
#include <thread>
#include <mutex>
#include <ctime>
#include <sstream>
#include <array>
#include <algorithm>
#include <iterator>
#include <memory>
#include <chrono>
#include <sys/syscall.h>

#include <iomanip>
#include <queue>
#include <list>
#include <map>

#include <cmath>
#include <thread>

#include<climits>
#include <unordered_map>

#ifndef BUILD_INDIVIDUAL // Only to make sure main is is included in documentation
#ifndef TEST             // Only to make sure main is is included in documentation
/**
* @brief Script main function. Generates start and end nodes as well as grid, then creates the algorithm objects and calls the main algorithm functions.
* @return 0
*/
int main();
#endif
#endif
