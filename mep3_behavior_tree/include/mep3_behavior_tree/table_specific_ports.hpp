// Copyright 2022 Memristor Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MEP3_BEHAVIOR_TREE__TABLE_SPECIFIC_PORTS_HPP_
#define MEP3_BEHAVIOR_TREE__TABLE_SPECIFIC_PORTS_HPP_

#include <string>
#include <vector>

namespace mep3_behavior_tree
{

class InputPortNameFactory {
public:
  InputPortNameFactory() {
    this->predefined_tables = {};
  }

  void set_names(const std::vector<std::string> names) {
    this->predefined_tables = names;
  }

  const std::vector<std::string> get_names() {
    return this->predefined_tables;
  }

private:
  std::vector<std::string> predefined_tables;
};

// Globally shared singleton
InputPortNameFactory g_InputPortNameFactory;

}  // namespace mep3_behavior_tree

#endif  // MEP3_BEHAVIOR_TREE__TABLE_SPECIFIC_PORTS_HPP_
