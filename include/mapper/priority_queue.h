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

#ifndef MAPPER_PRIORITY_QUEUE_H_
#define MAPPER_PRIORITY_QUEUE_H_

#include <queue>
#include <utility>  // For pair<>
#include <functional>  // For greater<>
#include <vector>

namespace octoclass {

// Adapted from https://www.redblobgames.com/pathfinding/a-star/implementation.html#cpp-astar
template<typename T, typename priority_t>
struct PriorityQueue {
  // typedef std::pair<priority_t, T> PQElement;

  struct PQElement{
    priority_t priority;
    T item;
    bool operator<(const PQElement &other) const {
      return priority < other.priority;
    }
    bool operator>(const PQElement &other) const {
      return priority > other.priority;
    }
  };

  std::priority_queue<PQElement, std::vector<PQElement>,
                 std::greater<PQElement>> elements;

  inline bool empty() const { return elements.empty(); }

  inline void put(T item, priority_t priority) {
    // elements.emplace(priority, item);
    elements.push(PQElement{priority, item});
  }

  inline T get() {
    T best_item = elements.top().item;
    elements.pop();
    return best_item;
  }
};

}  // octoclass

#endif  // MAPPER_PRIORITY_QUEUE_H_
