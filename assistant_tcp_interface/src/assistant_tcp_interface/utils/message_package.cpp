/**
 * @file message_package.cpp
 * @author LMS ()
 * @brief
 * @version 0.1
 * @date 30-04-2020
 *
 * @copyright Copyright (c) 2020 (refactor and some additions)
 *
 * Initial copyright 2019 FZI Forschungszentrum Informatik
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

#include "assistant_tcp_interface/utils/message_package.h"

namespace assistant_tcp_interface {
namespace utils {

uint32_t MessagePackage::next_id_ = 0;

/**
 * Definition of all message fields that can be exchanged with PDL servers
 */
std::unordered_map<std::string, MessagePackage::_type_variants> MessagePackage::message_type_list_{
    {"timestamp", uint32_t()},
    {"image_data", vectorImageData()},
    {"image_size", uint32_t()},
    {"confirm_receive", uint32_t()},
};

void MessagePackage::setZero() {
  for (auto &name : descr_) {
    if (message_type_list_.find(name) != message_type_list_.end()) {
      _type_variants entry = message_type_list_[name];
      data_[name] = entry;
    }
  }
}

size_t MessagePackage::getCapacity() const {
  size_t size = 0;
  size += sizeof(id_);
  for (auto &item : descr_) {
    if (message_type_list_.find(item) != message_type_list_.end()) {
      size += boost::apply_visitor(SizeVisitor{}, message_type_list_[item]);
    } else {
      throw std::string("MessagePackage::getCapacity : Message description contains unkown message type. Please check "
                        "the message_type_list.");
    }
  }
  return size;
}

size_t MessagePackage::getSize() const {
  size_t size = 0;
  size += sizeof(id_);
  for (auto &name : descr_) {
    if (data_.find(name) != data_.end()) {
      size += boost::apply_visitor(SizeVisitor{}, data_.at(name));
    }
  }
  return size;
}

bool MessagePackage::parseWith(MessageParser &mp) {
  for (auto &item : descr_) {
    if (message_type_list_.find(item) != message_type_list_.end()) {
      _type_variants entry = message_type_list_[item];
      auto bound_visitor = std::bind(ParseVisitor(), std::placeholders::_1, mp);
      boost::apply_visitor(bound_visitor, entry);
      data_[item] = entry;
    } else {
      return false;
    }
  }
  return true;
}

size_t MessagePackage::serializePackage(uint8_t *buffer) {
  size_t size = 0;
  size += MessageSerializer::serialize(buffer + size, id_);
  for (auto &name : descr_) {
    if (data_.find(name) != data_.end()) {
      auto bound_visitor = std::bind(SerializeVisitor(), std::placeholders::_1, buffer + size);
      size += boost::apply_visitor(bound_visitor, data_[name]);
    }
  }

  return size;
}

std::string MessagePackage::toString() {
  std::stringstream ss;
  ss << "message id: " << id_ << std::endl;
  for (auto &name : descr_) {
    if (data_.find(name) != data_.end()) {
      ss << name << ": ";

      ss << boost::apply_visitor(StringVisitor{}, data_[name]) << std::endl;
    }
  }
  ss << "message size: " << getSize() << std::endl;
  ss << "message capacity: " << getCapacity() << std::endl;
  return ss.str();
}

} // namespace utils
} // namespace assistant_tcp_interface
