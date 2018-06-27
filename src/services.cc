/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * 
 * All rights reserved.
 * 
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <mapper/mapper_class.h>
#include <limits>
#include <vector>

namespace mapper {

// Update resolution of the map
bool MapperClass::UpdateResolution(mapper::SetFloat::Request &req,
                                     mapper::SetFloat::Response &res) {
    pthread_mutex_lock(&mutexes_.octomap);
        globals_.octomap.SetResolution(req.data);
    pthread_mutex_unlock(&mutexes_.octomap);

    res.success = true;
    return true;
}

// Update map memory time
bool MapperClass::UpdateMemoryTime(mapper::SetFloat::Request &req,
                                     mapper::SetFloat::Response &res) {
    pthread_mutex_lock(&mutexes_.octomap);
        globals_.octomap.SetMemory(req.data);
    pthread_mutex_unlock(&mutexes_.octomap);

    res.success = true;
    return true;
}

bool MapperClass::MapInflation(mapper::SetFloat::Request &req,
                                 mapper::SetFloat::Response &res) {
    pthread_mutex_lock(&mutexes_.octomap);
        globals_.octomap.SetMapInflation(req.data);
    pthread_mutex_unlock(&mutexes_.octomap);

    res.success = true;
    return true;
}

bool MapperClass::ResetMap(std_srvs::Trigger::Request &req,
                             std_srvs::Trigger::Response &res) {
    pthread_mutex_lock(&mutexes_.octomap);
        globals_.octomap.ResetMap();
    pthread_mutex_unlock(&mutexes_.octomap);

    res.success = true;
    res.message = "Map has been reset!";
    return true;
}

}  // namespace mapper
