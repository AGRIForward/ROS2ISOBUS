/*
 *  This file is part of ROS2ISOBUS
 *
 *  Copyright 2026 Juha Backman / Natural Resources Institute Finland
 *
 *  ROS2ISOBUS is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  ROS2ISOBUS is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with ROS2ISOBUS.
 *  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AddressManager.hpp"

namespace ros2_isobus
{
AddressManager::AddressManager(const ByteArray8 & name, std::uint8_t preferred_address)
: isobusName_(name), address_(preferred_address)
{}

msg::IsobusFrame AddressManager::addressClaim() const
{
  // ISO 11783-5:2019, PGN 60928 Address Claim (Annex B; NAME/SA arbitration).
  msg::IsobusFrame imsg;
  imsg.pgn = 0xEEFF;
  imsg.page = 0;
  imsg.priority = 6;
  imsg.sa = address_;
  imsg.data = isobusName_;
  return imsg;
}

msg::IsobusFrame AddressManager::requestPGN(std::uint16_t PGN) const
{
  // ISO 11783-5:2019, PGN 59904 Request (requests PGN 60928 per clause 5.4).
  msg::IsobusFrame imsg;
  imsg.pgn = 0xEAFF;
  imsg.page = 0;
  imsg.priority = 3;
  imsg.sa = address_;
  imsg.data[0] = PGN & 0xFF;
  imsg.data[1] = (PGN >> 8) & 0xFF;
  imsg.data[2] = (PGN >> 16) & 0xFF;
  for (int i = 3; i < 8; ++i) {
    imsg.data[i] = 0xFF;
  }
  return imsg;
}

int AddressManager::compareNames(
  const ByteArray8 & contendingName, const ByteArray8 & comparedName) const
{
  for (int i = 7; i >= 0; --i) {
    if (contendingName[i] > comparedName[i]) {
      return 1;
    }
    if (contendingName[i] < comparedName[i]) {
      return -1;
    }
  }
  return 0;
}

std::uint8_t AddressManager::getMySA() const
{
  if (state_ == NameState::ADDRESS_CLAIMED) {
    return address_;
  }
  return 0;
}

std::uint8_t AddressManager::getSA(const ByteArray8 & name) const
{
  const auto it = addressbook_.find(name);
  if (it == addressbook_.end()) {
    return 0;
  }
  return it->second;
}

void AddressManager::handleMsg(const msg::IsobusFrame & imsg)
{
  bool addressbook_changed = false;
  // Address claim handling (PF=0xEE / PGN 60928 per ISO 11783-5:2019 Annex B).
  if (imsg.pf == 0xEE) {
    if (imsg.ps == address_) {
      if (compareNames(imsg.data, isobusName_)) {
        state_ = NameState::SEND_ADDRESS_CLAIM;
      } else {
        state_ = NameState::SELECT_ANOTHER_ADDRESS;
      }
    }

    auto iter = addressbook_.find(imsg.data);
    if (iter != addressbook_.end()) {
      if (iter->second != imsg.sa) {
        iter->second = imsg.sa;
        addressbook_changed = true;
        printWarn("ECU address has changed " + to_hex(iter->second) + "->" + to_hex(imsg.sa));
      }
    } else {
      addressbook_.emplace(imsg.data, imsg.sa);
      addressbook_changed = true;
      printInfo("Found new ECU address: " + to_hex(imsg.sa));
    }
  }

  // Request PGN handling (PF=0xEA / PGN 59904 per ISO 11783-5:2019 Annex B).
  if (imsg.pf == 0xEA) {
    if ((imsg.ps == address_) || (imsg.ps == 0xFF)) {
      const int requestedPGN = imsg.data[0] + (imsg.data[1] << 8) + (imsg.data[2] << 16);
      if ((requestedPGN == 60928) && (address_ != 255)) {
        state_ = NameState::SEND_ADDRESS_CLAIM;
      }
    }
  }

  if (addressbook_changed) {
    publishAddressBook(addressbook_);
  }
}

void AddressManager::run()
{
  // Wait until transport is ready to send (e.g. bus_tx has subscribers)
  if (!busReadyToSend()) {
    return;
  }
  // ISO 11783-5:2019, clause 5.4 Address Claim state machine.
  switch (state_) {
    case NameState::INIT:
      count_ = 3;
      sendFrame(addressClaim());
      state_ = NameState::SEND_ADDRESS_CLAIM_WAIT;
      break;

    case NameState::SEND_ADDRESS_CLAIM:
      sendFrame(addressClaim());
      state_ = prevState_;
      break;

    case NameState::SEND_ADDRESS_CLAIM_WAIT:
      if (--count_ <= 0) {
        state_ = NameState::BUILD_ADDRESSBOOK;
      }
      break;

    case NameState::BUILD_ADDRESSBOOK:
      sendFrame(requestPGN(static_cast<std::uint16_t>(0xEE00)));
      state_ = NameState::ADDRESS_CLAIMED;
      ensureSelfEntry();
      printInfo("Address claimed " + to_hex(address_));
      break;

    case NameState::ADDRESS_CLAIMED:
      break;

    case NameState::SELECT_ANOTHER_ADDRESS:
      address_ = static_cast<std::uint8_t>(((address_ - 127) % 111) + 128);
      count_ = 3;
      sendFrame(addressClaim());
      state_ = NameState::SEND_ADDRESS_CLAIM_WAIT;
      break;
  }

  if(prevState_ != state_) {
    emitStatus(state_, getMySA());
  }
  prevState_ = state_;
}

void AddressManager::ensureSelfEntry()
{
  const auto [iter, inserted] = addressbook_.emplace(isobusName_, address_);
  if (!inserted && iter->second == address_) {
    return;
  }
  iter->second = address_;
  publishAddressBook(addressbook_);
}

}  // namespace ros2_isobus
