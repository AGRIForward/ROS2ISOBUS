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

#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <string>

#include "ros2_isobus/msg/isobus_frame.hpp"

namespace ros2_isobus
{

using ByteArray8 = std::array<std::uint8_t, 8>;

ByteArray8 hex_to_byte_array(const std::string & hex);
std::string to_hex(std::uint32_t value);

/*
 *
 * AddressManager implements ISO 11783-5 (2019) address claim flow:
 *  - Sends PGN 60928 Address Claim with configured NAME/SA
 *  - Responds to PGN 59904 requests for PGN 60928
 *  - Maintains an address book of NAME -> SA pairs and publishes it latched
 *
 */
class AddressManager
{
public:
  // ISO 11783-5 Address Claim state machine
  enum class NameState
  {
    INIT,
    SEND_ADDRESS_CLAIM,
    SEND_ADDRESS_CLAIM_WAIT,
    BUILD_ADDRESSBOOK,
    ADDRESS_CLAIMED,
    SELECT_ANOTHER_ADDRESS
  };

  AddressManager(const ByteArray8 & name = ByteArray8{}, std::uint8_t preferred_address = 0);
  virtual ~AddressManager() = default;

  std::uint8_t getMySA() const;                      // Return our claimed SA (0xFE if not claimed).
  std::uint8_t getSA(const ByteArray8 & name) const; // Look up SA by NAME from the address book (0xFE if unknown).
  const std::map<ByteArray8, std::uint8_t> & address_book() const { return addressbook_; }

  NameState state() const { return state_; }

  void run();                       // State machine tick.
  void handleMsg(const msg::IsobusFrame & msg); // Process bus frames relevant to address claim.

private:
  msg::IsobusFrame addressClaim() const;                           // Build PGN 60928 Address Claim (ISO 11783-5:2019 Annex B).
  msg::IsobusFrame requestPGN(std::uint16_t PGN) const;            // Build PGN 59904 Request (ISO 11783-5:2019 Annex B).
  int compareNames(const ByteArray8 & contendingName, const ByteArray8 & comparedName) const; // NAME arbitration (ISO 11783-5:2019 clause 4.2).

  // Transport hooks
  virtual void sendFrame(const msg::IsobusFrame &frame) = 0;
  virtual void emitStatus(NameState state, std::uint8_t sa) = 0;
  virtual void publishAddressBook(const std::map<ByteArray8, std::uint8_t> & book) = 0;
  virtual void printInfo(const std::string & msg) = 0;
  virtual void printWarn(const std::string & msg) = 0;
  virtual bool busReadyToSend() const = 0;

protected:
  void ensureSelfEntry();                                          // Keep our NAME/SA in the address book.

  ByteArray8 isobusName_{};
  std::uint8_t address_ = 0;

  std::map<ByteArray8, std::uint8_t> addressbook_;

  NameState state_ = NameState::INIT;
  NameState prevState_ = NameState::INIT;
  int count_ = 0;
};


}  // namespace ros2_isobus
