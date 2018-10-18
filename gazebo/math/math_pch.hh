/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/*
 * Do not include this file unless building precompiled headers.
 * This file contains all headers included, minus the ones maintained by OSRF.
 * To get a starting list for this file, I use the command:
 *    grep --include="*.hh" --include="*.cc" --no-filename -r "#include <" | sort -u
 */
#include <algorithm>
#include <assert.h>
#include <boost/algorithm/string.hpp>
#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/math/special_functions/round.hpp>
#include <boost/random.hpp>
#include <boost/shared_ptr.hpp>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <math.h>
#include <string>
#include <string.h>
#include <sys/types.h>
#include <vector>
