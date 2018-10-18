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
#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/function.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/unordered/unordered_set.hpp>
#include <deque>
#include <google/protobuf/message.h>
#include <iomanip>
#include <iostream>
#include <list>
#include <map>
#include <mutex>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/task.h>
#include <utility>
#include <vector>
