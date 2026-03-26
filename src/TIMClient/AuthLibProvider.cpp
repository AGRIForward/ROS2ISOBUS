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

#include "AuthLibProvider.hpp"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <sstream>

namespace ros2_isobus
{

namespace
{
// AEF 040 RIG 1 authentication message codes used on auth PGNs.
// Mapping follows short-frame trigger/status and TP payload exchange model.
static constexpr std::uint8_t kMsgAuthServerCert = 0x02;
static constexpr std::uint8_t kMsgAuthClientCert = 0x03;
static constexpr std::uint8_t kMsgAuthServerRnd = 0x04;
static constexpr std::uint8_t kMsgAuthClientRnd = 0x05;
static constexpr std::uint8_t kMsgAuthServerFinalize = 0x06;
static constexpr std::uint8_t kMsgAuthClientFinalize = 0x07;
static constexpr std::uint8_t kMsgAuthClientVersion = 0xF7;
static constexpr std::uint8_t kMsgAuthServerVersion = 0xF8;
static constexpr std::uint8_t kAuthStatusNotAuthenticated = 0x00;
static constexpr std::uint8_t kAuthStatusAuthenticated = 0x01;
static constexpr std::uint32_t kStepTimeoutMs = 3000;
static constexpr std::uint8_t kMaxRetries = 3;
static constexpr std::uint8_t kCertRoundsRequired = 4;
enum class ShortMsgKind : std::uint8_t
{
  // AEF 040 short messages: status/version + per-round triggers.
  ServerStatus,
  ServerVersion,
  RandomTrigger,
  CertTrigger,
  FinalizeTrigger
};

struct ShortMsgSpec
{
  std::uint8_t msg_code;
  ShortMsgKind kind;
  bool require_zero_error;
  bool has_round;
};

constexpr std::array<ShortMsgSpec, 7> kShortMsgSpecs{{
  {MSG_AUTH_SERVER_STATUS, ShortMsgKind::ServerStatus, false, false},
  {kMsgAuthServerVersion, ShortMsgKind::ServerVersion, true, false},
  {kMsgAuthServerRnd, ShortMsgKind::RandomTrigger, true, false},
  // Interop: some stacks use 0x05 for client-random trigger short frame.
  {kMsgAuthClientRnd, ShortMsgKind::RandomTrigger, true, false},
  {kMsgAuthServerCert, ShortMsgKind::CertTrigger, true, true},
  // Interop: some stacks use 0x03 for client-certificate trigger short frame.
  {kMsgAuthClientCert, ShortMsgKind::CertTrigger, true, true},
  {kMsgAuthServerFinalize, ShortMsgKind::FinalizeTrigger, true, true}
}};

enum class TpMsgKind : std::uint8_t
{
  // AEF 040 TP payload messages: random/certificate/finalize content.
  RandomPayload,
  CertPayload,
  FinalizePayload
};

struct TpMsgSpec
{
  std::uint8_t msg_code;
  TpMsgKind kind;
  bool has_round;
  std::size_t min_size;
};

constexpr std::array<TpMsgSpec, 6> kTpMsgSpecs{{
  {kMsgAuthServerRnd, TpMsgKind::RandomPayload, false, 7U},
  // Interop: some stacks tag server random payload with 0x05.
  {kMsgAuthClientRnd, TpMsgKind::RandomPayload, false, 7U},
  {kMsgAuthServerCert, TpMsgKind::CertPayload, true, 7U},
  // Interop: some stacks tag server certificate payload with 0x03.
  {kMsgAuthClientCert, TpMsgKind::CertPayload, true, 7U},
  {kMsgAuthServerFinalize, TpMsgKind::FinalizePayload, true, 7U},
  // Interop: some stacks tag server finalize payload with 0x07.
  {kMsgAuthClientFinalize, TpMsgKind::FinalizePayload, true, 7U}
}};

const char * cert_role_name(std::uint8_t round)
{
  switch (round) {
    case 1U: return "TestLab";
    case 2U: return "Manufacturer";
    case 3U: return "ManufacturerSeries";
    case 4U: return "Device";
    default: return "Invalid";
  }
}

const ShortMsgSpec * find_short_msg_spec(std::uint8_t msg_code)
{
  const auto it = std::find_if(
    kShortMsgSpecs.begin(), kShortMsgSpecs.end(),
    [msg_code](const ShortMsgSpec & s) { return s.msg_code == msg_code; });
  return (it != kShortMsgSpecs.end()) ? &(*it) : nullptr;
}

const TpMsgSpec * find_tp_msg_spec(std::uint8_t msg_code)
{
  const auto it = std::find_if(
    kTpMsgSpecs.begin(), kTpMsgSpecs.end(),
    [msg_code](const TpMsgSpec & s) { return s.msg_code == msg_code; });
  return (it != kTpMsgSpecs.end()) ? &(*it) : nullptr;
}

const char * crypto_err_to_string(std::uint8_t rc)
{
  switch (rc) {
    case CRYPTO_ERR_SUCCESS: return "CRYPTO_ERR_SUCCESS";
    case CRYPTO_ERR_INVALID: return "CRYPTO_ERR_INVALID";
    case CRYPTO_ERR_CALLAGAIN: return "CRYPTO_ERR_CALLAGAIN";
    case CRYPTO_ERR_INTERNAL: return "CRYPTO_ERR_INTERNAL";
    case CRYPTO_ERR_BAD_INPUT: return "CRYPTO_ERR_BAD_INPUT";
    case 5U: return "CRYPTO_ERR_CERT_ON_CRL";
    case 6U: return "CRYPTO_ERR_SIGNATURE_INVALID";
    case 7U: return "CRYPTO_ERR_NULL_POINTER";
    case 8U: return "CRYPTO_ERR_LAB_CERT_INVALID";
    case 9U: return "CRYPTO_ERR_MANU_CERT_INVALID";
    case 10U: return "CRYPTO_ERR_MANU_SERIES_CERT_INVALID";
    case 11U: return "CRYPTO_ERR_DEVICE_CERT_INVALID";
    case 13U: return "CRYPTO_ERR_RESEED";
    case 14U: return "CRYPTO_ERR_ALLOC_FAILED";
    case 15U: return "CRYPTO_ERR_MBEDTLS_FAILED";
    case 16U: return "CRYPTO_ERR_ENUM_OUT_OF_BOUNDS";
    case 18U: return "CRYPTO_ERR_ALT_FAILED";
    case 19U: return "CRYPTO_ERR_ALT_FAILED";
    default: return "CRYPTO_ERR_UNKNOWN";
  }
}

// Minimal DER helpers used for extracting peer public key material when
// explicit server key parameter is not provided.
bool parse_der_length(
  const std::vector<std::uint8_t> & der, std::size_t pos, std::size_t & len, std::size_t & len_bytes)
{
  if (pos >= der.size()) return false;
  const std::uint8_t b0 = der[pos];
  if ((b0 & 0x80U) == 0U) {
    len = static_cast<std::size_t>(b0);
    len_bytes = 1U;
    return true;
  }
  const std::size_t n = static_cast<std::size_t>(b0 & 0x7FU);
  if (n == 0U || n > 4U || (pos + n) >= der.size()) return false;
  len = 0U;
  for (std::size_t i = 0; i < n; ++i) {
    len = (len << 8U) | static_cast<std::size_t>(der[pos + 1U + i]);
  }
  len_bytes = 1U + n;
  return true;
}

bool parse_der_tlv(
  const std::vector<std::uint8_t> & der,
  std::size_t pos,
  std::uint8_t expected_tag,
  std::size_t & value_pos,
  std::size_t & value_len,
  std::size_t & tlv_len)
{
  if (pos >= der.size() || der[pos] != expected_tag) return false;
  std::size_t len = 0U;
  std::size_t len_bytes = 0U;
  if (!parse_der_length(der, pos + 1U, len, len_bytes)) return false;
  const std::size_t vpos = pos + 1U + len_bytes;
  if ((vpos + len) > der.size()) return false;
  value_pos = vpos;
  value_len = len;
  tlv_len = 1U + len_bytes + len;
  return true;
}

std::string to_hex(const std::uint8_t * data, std::size_t len)
{
  std::ostringstream oss;
  oss << std::hex << std::nouppercase << std::setfill('0');
  for (std::size_t i = 0; i < len; ++i) {
    oss << std::setw(2) << static_cast<unsigned>(data[i]);
  }
  return oss.str();
}

bool extract_public_key_hex_from_der(const std::vector<std::uint8_t> & cert_der, std::string & out_hex)
{
  // X.509 Certificate ::= SEQUENCE { tbsCertificate, signatureAlgorithm, signatureValue }
  std::size_t cert_vpos = 0U;
  std::size_t cert_vlen = 0U;
  std::size_t cert_tlv_len = 0U;
  if (!parse_der_tlv(cert_der, 0U, 0x30U, cert_vpos, cert_vlen, cert_tlv_len)) return false;

  // TBSCertificate is first element and must be SEQUENCE.
  std::size_t tbs_vpos = 0U;
  std::size_t tbs_vlen = 0U;
  std::size_t tbs_tlv_len = 0U;
  if (!parse_der_tlv(cert_der, cert_vpos, 0x30U, tbs_vpos, tbs_vlen, tbs_tlv_len)) return false;

  // Iterate TBSCertificate fields to reach subjectPublicKeyInfo (7th field, version optional).
  std::size_t cur = tbs_vpos;
  const std::size_t tbs_end = tbs_vpos + tbs_vlen;
  std::size_t field_index = 0U;
  if (cur < tbs_end && cert_der[cur] == 0xA0U) {
    // [0] EXPLICIT version present.
    std::size_t vpos = 0U, vlen = 0U, vtlv = 0U;
    if (!parse_der_tlv(cert_der, cur, 0xA0U, vpos, vlen, vtlv)) return false;
    cur += vtlv;
  }

  // Skip serialNumber, signature, issuer, validity, subject (5 fields).
  for (; field_index < 5U; ++field_index) {
    if (cur >= tbs_end) return false;
    std::size_t len = 0U, len_bytes = 0U;
    if (!parse_der_length(cert_der, cur + 1U, len, len_bytes)) return false;
    const std::size_t tlv = 1U + len_bytes + len;
    if ((cur + tlv) > tbs_end) return false;
    cur += tlv;
  }

  // subjectPublicKeyInfo ::= SEQUENCE { algorithm, subjectPublicKey BIT STRING }
  std::size_t spki_vpos = 0U, spki_vlen = 0U, spki_tlv = 0U;
  if (!parse_der_tlv(cert_der, cur, 0x30U, spki_vpos, spki_vlen, spki_tlv)) return false;
  std::size_t spki_cur = spki_vpos;
  const std::size_t spki_end = spki_vpos + spki_vlen;

  // Skip AlgorithmIdentifier (SEQUENCE).
  std::size_t alg_vpos = 0U, alg_vlen = 0U, alg_tlv = 0U;
  if (!parse_der_tlv(cert_der, spki_cur, 0x30U, alg_vpos, alg_vlen, alg_tlv)) return false;
  spki_cur += alg_tlv;
  if (spki_cur >= spki_end) return false;

  // Parse subjectPublicKey BIT STRING.
  std::size_t bit_vpos = 0U, bit_vlen = 0U, bit_tlv = 0U;
  if (!parse_der_tlv(cert_der, spki_cur, 0x03U, bit_vpos, bit_vlen, bit_tlv)) return false;
  if (bit_vlen < 2U) return false;
  const std::uint8_t unused_bits = cert_der[bit_vpos];
  if (unused_bits != 0U) return false;
  const std::uint8_t * key = &cert_der[bit_vpos + 1U];
  const std::size_t key_len = bit_vlen - 1U;
  if (!((key_len == 65U && key[0] == 0x04U) || key_len == 32U)) return false;
  out_hex = to_hex(key, key_len);
  return true;
}

std::string trim_ascii(const std::string & in)
{
  const auto begin = in.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return {};
  }
  const auto end = in.find_last_not_of(" \t\r\n");
  return in.substr(begin, end - begin + 1U);
}

bool load_text_file(const std::string & path, std::string & out)
{
  std::ifstream f(path, std::ios::in);
  if (!f) return false;
  std::ostringstream ss;
  ss << f.rdbuf();
  out = trim_ascii(ss.str());
  return !out.empty();
}
}  // namespace

AuthLibProvider::AuthLibProvider(
  TimClient & owner,
  std::uint32_t period_ms,
  std::uint8_t implemented_version,
  std::uint8_t minimum_version,
  bool strict_mode,
  std::uint16_t client_cert_payload_max_len,
  std::uint16_t max_slice_iterations,
  const std::string & root_cert_path,
  const std::array<std::string, 4> & client_cert_paths,
  const std::string & client_private_key_hex,
  const std::string & server_public_key_hex)
: IAuthProvider(owner),
  period_ms_(period_ms),
  implemented_version_(implemented_version),
  minimum_version_(minimum_version),
  strict_mode_(strict_mode),
  client_cert_payload_max_len_(client_cert_payload_max_len),
  max_slice_iterations_(std::max<std::uint16_t>(1U, max_slice_iterations)),
  root_cert_path_(root_cert_path),
  client_cert_paths_(client_cert_paths),
  client_private_key_hex_(client_private_key_hex),
  server_public_key_hex_(server_public_key_hex)
{
  auth_connection_version_ = minimum_version_;
  if (!init_authlib() || !load_certificates()) {
    failed_ = true;
  }
}

AuthLibProvider::~AuthLibProvider()
{
  deinit_authlib();
}

bool AuthLibProvider::init_authlib()
{
  // AuthLib lifecycle is contained in provider init/deinit/reset boundaries.
  if (authlib_initialized_) return true;
  const std::uint8_t rc = AuthLib_Init();
  if (rc != CRYPTO_ERR_SUCCESS) {
    logError(
      "TIM authlib: AuthLib_Init failed rc=" + std::to_string(rc) +
      " (" + std::string(crypto_err_to_string(rc)) + ")");
    return false;
  }

  authlib_initialized_ = true;
  logInfo("TIM authlib: AuthLib initialized");
  return true;
}

void AuthLibProvider::deinit_authlib()
{
  if (parsed_root_cert_valid_) {
    (void)AuthLib_FreeCertificate(&parsed_root_cert_);
    parsed_root_cert_valid_ = false;
  }
  for (std::size_t i = 0; i < parsed_server_cert_valid_.size(); ++i) {
    if (parsed_server_cert_valid_[i]) {
      (void)AuthLib_FreeCertificate(&parsed_server_certs_[i]);
      parsed_server_cert_valid_[i] = false;
    }
  }
  if (authlib_initialized_) {
    (void)AuthLib_Deinit();
    authlib_initialized_ = false;
    logInfo("TIM authlib: AuthLib deinitialized");
  }
}

bool AuthLibProvider::load_certificates()
{
  // Load static local trust assets from configured paths.
  auto load_file = [](const std::string & path, std::vector<std::uint8_t> & out) -> bool {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    out.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    return !out.empty();
  };

  if (!load_file(root_cert_path_, root_cert_)) {
    logError("TIM authlib: failed to load root certificate: " + root_cert_path_);
    return false;
  }
  if (root_cert_.size() > 0xFFFFU) {
    logError("TIM authlib: root certificate too large for AuthLib_ParseCertificate");
    return false;
  }
  if (parsed_root_cert_valid_) {
    (void)AuthLib_FreeCertificate(&parsed_root_cert_);
    parsed_root_cert_valid_ = false;
  }
  std::memset(&parsed_root_cert_, 0, sizeof(parsed_root_cert_));
  std::uint8_t rc = AuthLib_InitCertificate(&parsed_root_cert_);
  if (rc != CRYPTO_ERR_SUCCESS) {
    logError(
      "TIM authlib: AuthLib_InitCertificate failed for root cert rc=" + std::to_string(rc) +
      " (" + std::string(crypto_err_to_string(rc)) + ")");
    return false;
  }
  rc = AuthLib_ParseCertificate(
    root_cert_.data(),
    static_cast<std::uint16_t>(root_cert_.size()),
    &parsed_root_cert_);
  if (rc != CRYPTO_ERR_SUCCESS) {
    logError(
      "TIM authlib: AuthLib_ParseCertificate failed for root cert rc=" + std::to_string(rc) +
      " (" + std::string(crypto_err_to_string(rc)) + ")");
    return false;
  }
  parsed_root_cert_valid_ = true;
  for (std::size_t i = 0; i < client_cert_paths_.size(); ++i) {
    if (!load_file(client_cert_paths_[i], client_certs_[i])) {
      logError("TIM authlib: failed to load client certificate: " + client_cert_paths_[i]);
      return false;
    }
  }
  logInfo("TIM authlib: local certificate assets loaded");
  return true;
}

bool AuthLibProvider::derive_common_secret()
{
  // ECDH/KDF operation delegated to AuthLib implementation.
  std::string client_private_key_hex = trim_ascii(client_private_key_hex_);
  if (!client_private_key_hex.empty() && client_private_key_hex.find(".key.hex") != std::string::npos) {
    std::string loaded_hex;
    if (!load_text_file(client_private_key_hex, loaded_hex)) {
      logError("TIM authlib: failed to load private key hex file: " + client_private_key_hex);
      return false;
    }
    client_private_key_hex = loaded_hex;
  }

  if (client_private_key_hex.empty()) {
    logError("TIM authlib: authlib.client_private_key_hex is empty");
    return false;
  }
  std::string peer_public_key_hex = server_public_key_hex_;
  if (peer_public_key_hex.empty()) {
    if (!extract_public_key_hex_from_der(server_certs_[3], peer_public_key_hex)) {
      logError("TIM authlib: failed to auto-extract server public key from device certificate");
      return false;
    }
    logInfo("TIM authlib: extracted server public key from device certificate");
  }
  if (!server_random_valid_ || !client_random_valid_) {
    logError("TIM authlib: random challenges not ready for common secret derivation");
    return false;
  }

  std::uint8_t rc = AuthLib_ComputeCommonSecret(
    reinterpret_cast<const std::uint8_t *>(client_private_key_hex.c_str()),
    reinterpret_cast<const std::uint8_t *>(peer_public_key_hex.c_str()),
    server_random_challenge_.data(),
    client_random_challenge_.data(),
    common_secret_.data(),
    CURVE_25519_KDF_NIST_SP800_56A_HMAC_SHA256,
    SLICE_INIT);
  std::uint16_t iters = 0U;
  while (rc == CRYPTO_ERR_CALLAGAIN && iters < max_slice_iterations_) {
    rc = AuthLib_ComputeCommonSecret(
      reinterpret_cast<const std::uint8_t *>(client_private_key_hex.c_str()),
      reinterpret_cast<const std::uint8_t *>(peer_public_key_hex.c_str()),
      server_random_challenge_.data(),
      client_random_challenge_.data(),
      common_secret_.data(),
      CURVE_25519_KDF_NIST_SP800_56A_HMAC_SHA256,
      SLICE_CONTINUE);
    ++iters;
  }
  if (rc == CRYPTO_ERR_CALLAGAIN) {
    logError(
      "TIM authlib: AuthLib_ComputeCommonSecret did not finish (too many slices, max=" +
      std::to_string(max_slice_iterations_) + ")");
    return false;
  }
  if (rc != CRYPTO_ERR_SUCCESS) {
    logError(
      "TIM authlib: AuthLib_ComputeCommonSecret failed rc=" + std::to_string(rc) +
      " (" + std::string(crypto_err_to_string(rc)) + ")");
    return false;
  }
  common_secret_ready_ = true;
  return true;
}

bool AuthLibProvider::compute_client_signed_challenge()
{
  if (!common_secret_ready_ || !server_random_valid_) return false;
  const std::uint8_t rc = AuthLib_ComputeCMAC(
    server_random_challenge_.data(),
    common_secret_.data(),
    client_signed_challenge_.data(),
    CRYPTO_CLIENT);
  if (rc != CRYPTO_ERR_SUCCESS) {
    logError(
      "TIM authlib: AuthLib_ComputeCMAC failed rc=" + std::to_string(rc) +
      " (" + std::string(crypto_err_to_string(rc)) + ")");
    return false;
  }
  challenge_signed_local_ = true;
  return true;
}

bool AuthLibProvider::verify_server_signed_challenge()
{
  if (!common_secret_ready_ || !client_random_valid_ || !server_signed_valid_) return false;
  const std::uint8_t rc = AuthLib_VerifyCMAC(
    server_signed_challenge_.data(),
    common_secret_.data(),
    client_random_challenge_.data(),
    CRYPTO_CLIENT);
  if (rc == CRYPTO_ERR_SUCCESS) return true;
  logWarn(
    "TIM authlib: AuthLib_VerifyCMAC rc=" + std::to_string(rc) +
    " (" + std::string(crypto_err_to_string(rc)) + ")");
  return false;
}

bool AuthLibProvider::verify_server_certificate_chain()
{
  // Full certificate parsing and chain verification are delegated to AuthLib.
  if (!parsed_root_cert_valid_) {
    logError("TIM authlib: cannot verify chain, root certificate is not parsed");
    return false;
  }
  for (bool got : server_cert_received_) {
    if (!got) {
      logError("TIM authlib: cannot verify chain, server certificates missing");
      return false;
    }
  }

  // Parse each server certificate with AuthLib before chain verify.
  for (std::size_t i = 0; i < server_certs_.size(); ++i) {
    std::memset(&parsed_server_certs_[i], 0, sizeof(parsed_server_certs_[i]));
    std::uint8_t rc = AuthLib_InitCertificate(&parsed_server_certs_[i]);
    if (rc != CRYPTO_ERR_SUCCESS) {
      logError(
        "TIM authlib: AuthLib_InitCertificate failed for round " + std::to_string(i + 1U) +
        " rc=" + std::to_string(rc) + " (" + std::string(crypto_err_to_string(rc)) + ")");
      return false;
    }
    if (server_certs_[i].size() > 0xFFFFU) {
      logError("TIM authlib: certificate too large for AuthLib_ParseCertificate");
      return false;
    }
    rc = AuthLib_ParseCertificate(
      server_certs_[i].data(),
      static_cast<std::uint16_t>(server_certs_[i].size()),
      &parsed_server_certs_[i]);
    if (rc != CRYPTO_ERR_SUCCESS) {
      logError(
        "TIM authlib: AuthLib_ParseCertificate failed for round " + std::to_string(i + 1U) +
        " rc=" + std::to_string(rc) + " (" + std::string(crypto_err_to_string(rc)) + ")");
      return false;
    }
    parsed_server_cert_valid_[i] = true;
  }

  std::uint8_t rc = AuthLib_VerifyPeerCertificatesChain(
    &parsed_root_cert_,
    &parsed_server_certs_[0],
    &parsed_server_certs_[1],
    &parsed_server_certs_[2],
    &parsed_server_certs_[3],
    SLICE_INIT);
  std::uint16_t iters = 0U;
  while (rc == CRYPTO_ERR_CALLAGAIN && iters < max_slice_iterations_) {
    rc = AuthLib_VerifyPeerCertificatesChain(
      &parsed_root_cert_,
      &parsed_server_certs_[0],
      &parsed_server_certs_[1],
      &parsed_server_certs_[2],
      &parsed_server_certs_[3],
      SLICE_CONTINUE);
    ++iters;
  }
  if (rc == CRYPTO_ERR_CALLAGAIN) {
    logError(
      "TIM authlib: AuthLib_VerifyPeerCertificatesChain did not finish (too many slices, max=" +
      std::to_string(max_slice_iterations_) + ")");
    return false;
  }
  if (rc == CRYPTO_ERR_SUCCESS) {
    logInfo("TIM authlib: peer certificate chain verification OK");
    return true;
  }
  logError(
    "TIM authlib: peer certificate chain verification failed rc=" + std::to_string(rc) +
    " (" + std::string(crypto_err_to_string(rc)) + ")");
  return false;
}

void AuthLibProvider::reset(std::uint8_t client_sa, std::uint8_t server_sa)
{
  // Session reset mirrors TIM reconnect/restart semantics and clears all round state.
  bool assets_ready = authlib_initialized_;
  if (!assets_ready) {
    assets_ready = init_authlib();
  }
  if (assets_ready) {
    assets_ready = load_certificates();
  }
  client_sa_ = client_sa;
  server_sa_ = server_sa;
  requested_cert_round_ = 1;
  requested_finalize_round_ = 1;
  expected_server_cert_round_ = 0;
  expected_server_finalize_round_ = 0;
  server_cert_received_.fill(false);
  cert_exchange_state_.fill(CertExchangeState::NotRequested);
  for (auto & cert : server_certs_) cert.clear();
  for (std::size_t i = 0; i < parsed_server_cert_valid_.size(); ++i) {
    if (parsed_server_cert_valid_[i]) {
      (void)AuthLib_FreeCertificate(&parsed_server_certs_[i]);
      parsed_server_cert_valid_[i] = false;
    }
    std::memset(&parsed_server_certs_[i], 0, sizeof(parsed_server_certs_[i]));
  }
  client_random_valid_ = false;
  server_random_valid_ = false;
  server_signed_valid_ = false;
  common_secret_ready_ = false;
  client_signed_challenge_.fill(0);
  server_signed_challenge_.fill(0);
  retries_ = 0;
  step_ = Step::WaitRandomTrigger;
  restart_sent_ = false;
  authenticated_ = false;
  failed_ = !assets_ready;
  server_status_seen_ = false;
  got_server_tp_04_ = false;
  got_server_tp_02_ = false;
  got_server_tp_06_ = false;
  server_finalize_payload_seen_ = false;
  client_finalize_payload_sent_ = false;
  pending_server_rnd_request_ = false;
  pending_server_cert_request_ = false;
  pending_server_finalize_request_ = false;
  auth_type_field_ = 0x0F;
  finalize_request_sent_ = false;
  certs_valid_ = false;
  challenge_signed_local_ = false;
  challenge_signed_server_ = false;
  server_authenticated_seen_ = false;
  client_authenticated_status_sent_ = false;
  pending_f8_request_ = false;
  last_status_tx_ms_ = 0;
  last_round_tx_ms_ = 0;
  step_deadline_ms_ = 0;
  auth_connection_version_ = minimum_version_;
  if (!failed_) {
    const char seed[] = "ros2_isobus_authlib_seed";
    const std::uint8_t rc = AuthLib_GenerateRandomChallenge(
      reinterpret_cast<const std::uint8_t *>(seed),
      static_cast<std::uint16_t>(sizeof(seed) - 1U),
      client_random_challenge_.data());
    client_random_valid_ = (rc == CRYPTO_ERR_SUCCESS);
    if (!client_random_valid_) {
      logError(
        "TIM authlib: failed to generate client random challenge rc=" + std::to_string(rc) +
        " (" + std::string(crypto_err_to_string(rc)) + ")");
      failed_ = true;
    }
  }
  logInfo("TIM auth: session reset");
}

ros2_isobus::msg::IsobusFrame AuthLibProvider::make_auth_frame(std::uint8_t msg_code) const
{
  ros2_isobus::msg::IsobusFrame fr;
  fr.pgn = PGN_AUTH_C2S;
  fr.page = false;
  fr.priority = 6;
  fr.sa = client_sa_;
  fr.pf = static_cast<std::uint8_t>((PGN_AUTH_C2S >> 8) & 0xFFU);
  fr.ps = server_sa_;
  fr.data.fill(0xFF);
  fr.data[0] = msg_code;
  return fr;
}

ros2_isobus::msg::IsobusFrame AuthLibProvider::make_client_version_response_frame() const
{
  auto fr = make_auth_frame(kMsgAuthClientVersion);
  fr.data[1] = 0x00;
  fr.data[2] = implemented_version_;
  fr.data[3] = minimum_version_;
  return fr;
}

ros2_isobus::msg::IsobusFrame AuthLibProvider::make_random_trigger_response_frame() const
{
  auto fr = make_auth_frame(kMsgAuthServerRnd);
  fr.data[1] = 0x00;
  fr.data[2] = auth_type_field_;
  fr.data[3] = 0xFF;
  fr.data[4] = 0xFF;
  fr.data[5] = 0xFF;
  fr.data[6] = 0xFF;
  fr.data[7] = 0xFF;
  return fr;
}

ros2_isobus::msg::IsobusFrame AuthLibProvider::make_cert_trigger_response_frame(std::uint8_t round) const
{
  auto fr = make_auth_frame(kMsgAuthServerCert);
  fr.data[1] = 0x00;
  fr.data[2] = auth_type_field_;
  fr.data[3] = 0x00;
  fr.data[4] = round;
  fr.data[5] = 0xFF;
  fr.data[6] = 0xFF;
  fr.data[7] = 0xFF;
  return fr;
}

ros2_isobus::msg::IsobusFrame AuthLibProvider::make_finalize_trigger_response_frame(std::uint8_t round) const
{
  auto fr = make_auth_frame(kMsgAuthServerFinalize);
  fr.data[1] = 0x00;
  fr.data[2] = auth_type_field_;
  fr.data[3] = 0xFF;
  fr.data[4] = round;
  fr.data[5] = 0xFF;
  fr.data[6] = 0xFF;
  fr.data[7] = 0xFF;
  return fr;
}

ros2_isobus::msg::IsobusTpFrame AuthLibProvider::make_client_random_response_tp() const
{
  ros2_isobus::msg::IsobusTpFrame tp;
  tp.priority = 6;
  tp.page = false;
  tp.pgn = PGN_AUTH_C2S;
  tp.sa = client_sa_;
  tp.pf = static_cast<std::uint8_t>((PGN_AUTH_C2S >> 8) & 0xFFU);
  tp.ps = server_sa_;
  tp.data.assign(39U, 0xFF);
  tp.data[0] = kMsgAuthClientRnd;
  tp.data[1] = 0x00;  // no error
  tp.data[2] = auth_type_field_;
  tp.data[3] = 0xFF;
  tp.data[4] = 0xFF;
  tp.data[5] = 0x20;
  tp.data[6] = 0x00;
  for (std::size_t i = 0; i < 32U; ++i) {
    tp.data[7U + i] = client_random_challenge_[i];
  }
  return tp;
}

ros2_isobus::msg::IsobusTpFrame AuthLibProvider::make_client_certificate_response_tp(std::uint8_t round) const
{
  ros2_isobus::msg::IsobusTpFrame tp;
  tp.priority = 6;
  tp.page = false;
  tp.pgn = PGN_AUTH_C2S;
  tp.sa = client_sa_;
  tp.pf = static_cast<std::uint8_t>((PGN_AUTH_C2S >> 8) & 0xFFU);
  tp.ps = server_sa_;
  const std::size_t round_idx = static_cast<std::size_t>(round > 0U ? round - 1U : 0U);
  const auto & cert = client_certs_[std::min(round_idx, client_certs_.size() - 1U)];
  const std::size_t configured_max_len =
    (client_cert_payload_max_len_ == 0U) ? 0xFFFFU : static_cast<std::size_t>(client_cert_payload_max_len_);
  const std::size_t cert_len = std::min<std::size_t>(cert.size(), configured_max_len);
  tp.data.assign(7U + cert_len, 0xFF);
  tp.data[0] = kMsgAuthClientCert;
  tp.data[1] = 0x00;  // no error
  tp.data[2] = auth_type_field_;
  tp.data[3] = 0x00;
  tp.data[4] = round;
  tp.data[5] = static_cast<std::uint8_t>(cert_len & 0xFFU);
  tp.data[6] = static_cast<std::uint8_t>((cert_len >> 8U) & 0xFFU);
  for (std::size_t i = 0; i < cert_len; ++i) {
    tp.data[7U + i] = cert[i];
  }
  return tp;
}

ros2_isobus::msg::IsobusTpFrame AuthLibProvider::make_client_finalize_response_tp(std::uint8_t round) const
{
  ros2_isobus::msg::IsobusTpFrame tp;
  tp.priority = 6;
  tp.page = false;
  tp.pgn = PGN_AUTH_C2S;
  tp.sa = client_sa_;
  tp.pf = static_cast<std::uint8_t>((PGN_AUTH_C2S >> 8) & 0xFFU);
  tp.ps = server_sa_;
  tp.data.assign(23U, 0xFF);
  tp.data[0] = kMsgAuthClientFinalize;
  tp.data[1] = 0x00;  // no error
  tp.data[2] = auth_type_field_;
  tp.data[3] = 0xFF;
  tp.data[4] = round;
  tp.data[5] = 0x10;
  tp.data[6] = 0x00;
  for (std::size_t i = 0; i < 16U; ++i) {
    tp.data[7U + i] = client_signed_challenge_[i];
  }
  return tp;
}

void AuthLibProvider::send_auth_status(std::uint32_t now_ms, bool restart_bit)
{
  // Periodic client status frame (AEF 040 status signaling), including restart handshake hint.
  if ((now_ms - last_status_tx_ms_) < period_ms_) return;
  last_status_tx_ms_ = now_ms;

  auto fr = make_auth_frame(MSG_AUTH_CLIENT_STATUS);
  fr.data[1] = 0x00;  // no error
  fr.data[2] = static_cast<std::uint8_t>((0x0U << 4) | (authenticated_ ? 0x1U : 0x0U));
  std::uint8_t sub_status = 0x00U;
  if (certs_valid_) {
    sub_status = 0x3CU;
  }
  if (challenge_signed_local_) {
    sub_status = 0x3EU;
  }
  if (restart_bit) {
    sub_status = 0x80U;
  }
  fr.data[3] = sub_status;  // auth sub-status / (Re)Start during initialization
  fr.data[6] = implemented_version_;
  fr.data[7] = minimum_version_;
  sendFrame(fr);
  if (restart_bit) {
    logInfo("TIM auth: sent client auth status with restart request");
  }
}

bool AuthLibProvider::protocol_violation(const std::string & msg)
{
  // strict_mode enforces guideline-compatible sequencing/field checks.
  // non-strict mode logs deviation and continues for interoperability testing.
  if (strict_mode_) {
    logError("TIM auth protocol violation: " + msg + " (strict mode)");
    failed_ = true;
    return true;
  }
  logWarn("TIM auth protocol deviation: " + msg + " (non-strict mode, ignoring)");
  return false;
}

void AuthLibProvider::send_round_requests(std::uint32_t now_ms)
{
  // Drives AEF 040 phased exchange:
  // 1) random trigger + payload
  // 2) certificate rounds 1..4 trigger + payload
  // 3) finalize trigger + signed payload
  const auto all_server_certs_received = [this]() {
    return std::all_of(
      server_cert_received_.begin(), server_cert_received_.end(),
      [](bool got) { return got; });
  };
  const auto next_expected_server_cert_round = [this]() -> std::uint8_t {
    for (std::size_t i = 0; i < server_cert_received_.size(); ++i) {
      if (!server_cert_received_[i]) {
        return static_cast<std::uint8_t>(i + 1U);
      }
    }
    return 0U;
  };

  // Keep most exchanges rate-limited, but allow immediate finalize response when
  // server requests signed challenge to avoid timeout-sensitive test benches.
  const bool finalize_immediate =
    (step_ == Step::WaitFinalizeTrigger && pending_server_finalize_request_);
  if (!finalize_immediate && (now_ms - last_round_tx_ms_) < 100U) return;
  last_round_tx_ms_ = now_ms;

  if (step_ == Step::WaitRandomTrigger) {
    if (!pending_server_rnd_request_) return;
    logInfo("TIM auth: server requested random challenge, sending response");
    sendFrame(make_random_trigger_response_frame());
    sendTpFrame(make_client_random_response_tp());
    pending_server_rnd_request_ = false;
    step_ = Step::WaitRandomTp;
    step_deadline_ms_ = now_ms + kStepTimeoutMs;
    return;
  }

  if (step_ == Step::WaitRandomTp) {
    if (!got_server_tp_04_) return;
    got_server_tp_04_ = false;
    retries_ = 0;
    step_ = Step::WaitCertTrigger;
    step_deadline_ms_ = 0;
    return;
  }

  if (step_ == Step::WaitCertTrigger) {
    if (!pending_server_cert_request_) return;
    if (requested_cert_round_ < 1U || requested_cert_round_ > kCertRoundsRequired) {
      if (protocol_violation("received certificate request with invalid round")) return;
      pending_server_cert_request_ = false;
      return;
    }
    std::uint8_t effective_round = requested_cert_round_;
    std::uint8_t next_missing_round = 0U;
    for (std::size_t i = 0; i < server_cert_received_.size(); ++i) {
      if (!server_cert_received_[i]) {
        next_missing_round = static_cast<std::uint8_t>(i + 1U);
        break;
      }
    }
    if (!strict_mode_ && next_missing_round != 0U) {
      const std::size_t requested_idx = static_cast<std::size_t>(requested_cert_round_ - 1U);
      if (requested_idx < server_cert_received_.size() && server_cert_received_[requested_idx]) {
        logWarn(
          "TIM auth interop: server re-requested already received certificate round " +
          std::to_string(static_cast<unsigned>(requested_cert_round_)) +
          ", sending next missing round " +
          std::to_string(static_cast<unsigned>(next_missing_round)));
        effective_round = next_missing_round;
      }
    }
    const std::size_t round_idx = static_cast<std::size_t>(effective_round - 1U);
    const std::uint8_t expected_round = next_expected_server_cert_round();
    if (expected_round != 0U && effective_round > expected_round) {
      if (protocol_violation(
          "server requested out-of-sequence certificate round " +
          std::to_string(static_cast<unsigned>(effective_round)) + " (" +
          cert_role_name(effective_round) + "), expected round " +
          std::to_string(static_cast<unsigned>(expected_round)) + " (" +
          cert_role_name(expected_round) + ")")) return;
      pending_server_cert_request_ = false;
      return;
    }
    if (client_certs_[round_idx].empty()) {
      if (protocol_violation("local certificate for requested round is empty")) return;
      pending_server_cert_request_ = false;
      return;
    }
    cert_exchange_state_[round_idx] = CertExchangeState::Requested;
    logInfo(
      "TIM auth: server requested certificate round " +
      std::to_string(static_cast<unsigned>(requested_cert_round_)) + " (" +
      cert_role_name(requested_cert_round_) + "), sending response round " +
      std::to_string(static_cast<unsigned>(effective_round)) + " (" +
      cert_role_name(effective_round) + ")");
    sendFrame(make_cert_trigger_response_frame(effective_round));
    const auto cert_tp = make_client_certificate_response_tp(effective_round);
    if (cert_tp.data.size() > 7U) {
      const std::size_t cert_len = cert_tp.data.size() - 7U;
      const bool truncated =
        cert_len < client_certs_[round_idx].size();
      logInfo(
        std::string("TIM auth: sending client certificate payload len=") +
        std::to_string(static_cast<unsigned>(cert_len)) +
        " bytes" +
        (truncated ? " (truncated by authlib.client_cert_payload_max_len)" : ""));
    }
    sendTpFrame(cert_tp);
    cert_exchange_state_[round_idx] = CertExchangeState::ClientSent;
    expected_server_cert_round_ = effective_round;
    pending_server_cert_request_ = false;
    step_ = Step::WaitCertTp;
    step_deadline_ms_ = now_ms + kStepTimeoutMs;
    return;
  }

  if (step_ == Step::WaitCertTp) {
    if (!got_server_tp_02_) return;
    got_server_tp_02_ = false;
    expected_server_cert_round_ = 0;
    retries_ = 0;
    if (all_server_certs_received()) {
      certs_valid_ = verify_server_certificate_chain();
      if (!certs_valid_) {
        if (strict_mode_) {
          failed_ = true;
          return;
        }
        logWarn(
          "TIM auth interop: server certificate chain verification failed, "
          "continuing in non-strict mode");
        certs_valid_ = true;
      }
      last_status_tx_ms_ = 0;
      step_ = Step::WaitFinalizeTrigger;
    } else {
      step_ = Step::WaitCertTrigger;
    }
    step_deadline_ms_ = 0;
    return;
  }

  if (step_ == Step::WaitFinalizeTrigger) {
    // Prepare signed challenge response only after server random challenge is received.
    if (!challenge_signed_local_) {
      if (!server_random_valid_) return;
      if (!common_secret_ready_) {
        if (!derive_common_secret()) {
          if (strict_mode_) {
            failed_ = true;
            return;
          }
          logWarn(
            "TIM auth interop: common-secret derivation failed, using placeholder "
            "client signed challenge in non-strict mode");
          for (std::size_t i = 0; i < client_signed_challenge_.size(); ++i) {
            client_signed_challenge_[i] = static_cast<std::uint8_t>(i);
          }
          challenge_signed_local_ = true;
          last_status_tx_ms_ = 0;
        }
      }
      if (!challenge_signed_local_) {
        if (!compute_client_signed_challenge()) {
          if (strict_mode_) {
            failed_ = true;
            return;
          }
          logWarn(
            "TIM auth interop: client challenge signing failed, using placeholder "
            "response in non-strict mode");
          for (std::size_t i = 0; i < client_signed_challenge_.size(); ++i) {
            client_signed_challenge_[i] = static_cast<std::uint8_t>(i);
          }
          challenge_signed_local_ = true;
        }
      }
      last_status_tx_ms_ = 0;
    }
    if (pending_server_finalize_request_) {
      // Server explicitly requested signed challenge; respond immediately.
      logInfo("TIM auth: server requested signed challenge, sending response payload");
      sendTpFrame(make_client_finalize_response_tp(requested_finalize_round_));
      client_finalize_payload_sent_ = true;
      pending_server_finalize_request_ = false;
      step_ = Step::WaitFinalizeTp;
      step_deadline_ms_ = now_ms + kStepTimeoutMs;
      return;
    }
    if (!finalize_request_sent_) {
      // Interop fallback: proactively request signed challenge if server has not
      // yet sent finalize trigger.
      sendFrame(make_finalize_trigger_response_frame(requested_finalize_round_));
      expected_server_finalize_round_ = requested_finalize_round_;
      finalize_request_sent_ = true;
      step_deadline_ms_ = now_ms + kStepTimeoutMs;
      return;
    }
    return;
  }

  if (step_ == Step::WaitFinalizeTp) {
    if (got_server_tp_06_) {
      got_server_tp_06_ = false;
      expected_server_finalize_round_ = 0;
      server_finalize_payload_seen_ = true;
      pending_server_finalize_request_ = false;
      retries_ = 0;
    }
    if (!challenge_signed_server_) {
      if (server_signed_valid_) {
        challenge_signed_server_ = verify_server_signed_challenge();
      }
      if (!challenge_signed_server_) {
        if (strict_mode_) {
          logError("TIM authlib: server signed challenge verification failed");
          failed_ = true;
          return;
        }
        if (!server_finalize_payload_seen_) return;
        logWarn(
          "TIM auth interop: skipping server signed challenge verification in "
          "non-strict mode");
        challenge_signed_server_ = true;
      }
    }

    if (
      server_finalize_payload_seen_ &&
      client_finalize_payload_sent_ &&
      !client_authenticated_status_sent_)
    {
      auto fr = make_auth_frame(MSG_AUTH_CLIENT_STATUS);
      fr.data[1] = 0x00;
      fr.data[2] = 0x01;  // authenticated
      fr.data[3] = 0x3E;  // certificates valid + challenge signed
      fr.data[6] = implemented_version_;
      fr.data[7] = minimum_version_;
      sendFrame(fr);
      client_authenticated_status_sent_ = true;
    }
    if (
      server_finalize_payload_seen_ &&
      client_finalize_payload_sent_ &&
      client_authenticated_status_sent_ &&
      server_authenticated_seen_)
    {
      authenticated_ = true;
      logInfo("TIM auth: authentication completed (mutual final status exchange)");
      step_deadline_ms_ = 0;
    }
  }
}

void AuthLibProvider::on_frame(const ros2_isobus::msg::IsobusFrame & fr)
{
  // Parse/validate AEF 040 short messages from active server endpoint.
  const std::uint8_t pf = fr.pf ? fr.pf : static_cast<std::uint8_t>((fr.pgn >> 8) & 0xFFU);
  const std::uint32_t pgn_from_pf = (static_cast<std::uint32_t>(pf) << 8U);
  if (pgn_from_pf != PGN_AUTH_S2C && pgn_from_pf != PGN_AUTH_C2S) return;
  if (fr.sa != server_sa_) return;
  if (fr.ps != client_sa_) return;

  const std::uint8_t msg = fr.data[0];
  const ShortMsgSpec * spec = find_short_msg_spec(msg);
  if (spec == nullptr) {
    return;
  }
  const bool finalize_short_msg = (msg == kMsgAuthServerFinalize);

  if (spec->require_zero_error && fr.data[1] != 0x00U) {
    if (protocol_violation(
        "auth short frame code=0x" + to_hex(&msg, 1U) +
        " carries non-zero error code " + std::to_string(static_cast<unsigned>(fr.data[1])))) return;
    // Non-strict mode: continue processing to keep interoperability with
    // implementations that set diagnostic/non-zero code bits on trigger frames.
  }

  if (spec->has_round) {
    const std::uint8_t round = fr.data[4];
    if (round < 1U || round > kCertRoundsRequired) {
      if (!strict_mode_ && finalize_short_msg && round == 0xFFU) {
        // Interop: some servers use 0xFF as wildcard/finalize round in short trigger.
      } else {
      if (protocol_violation(
          "auth short frame code=0x" + to_hex(&msg, 1U) +
          " has invalid round " + std::to_string(static_cast<unsigned>(round)))) return;
      return;
      }
    }
  }

  if (spec->kind == ShortMsgKind::ServerStatus) {
    server_status_seen_ = true;
    const std::uint8_t err = fr.data[1];
    const std::uint8_t auth_type = static_cast<std::uint8_t>((fr.data[2] >> 4) & 0x0FU);
    const std::uint8_t auth_status = static_cast<std::uint8_t>(fr.data[2] & 0x0FU);
    auth_type_field_ = fr.data[2];
    if (auth_type != 0x00U) {
      logWarn(
        "TIM auth: server status auth_type is non-zero (" +
        std::to_string(static_cast<unsigned>(auth_type)) + ")");
    }
    if (err != 0x00U) {
      if (protocol_violation(
          "server status reports error code " +
          std::to_string(static_cast<unsigned>(err)))) return;
      return;
    }
    if (auth_status != kAuthStatusNotAuthenticated &&
      auth_status != kAuthStatusAuthenticated)
    {
      if (protocol_violation(
          "server status has unsupported auth status value " +
          std::to_string(static_cast<unsigned>(auth_status)))) return;
      return;
    }
    const bool server_claims_signed = ((fr.data[3] & 0x02U) != 0U) || (fr.data[3] == 0x3EU);
    if (server_claims_signed && server_signed_valid_) {
      challenge_signed_server_ = true;
    }
    if (auth_status == kAuthStatusAuthenticated) {
      server_authenticated_seen_ = true;
      logInfo(
        "TIM auth: server status confirms authenticated (auth_type=" +
        std::to_string(static_cast<unsigned>(auth_type)) + ")");
    }
    return;
  }

  if (spec->kind == ShortMsgKind::ServerVersion) {
    pending_f8_request_ = true;
    logInfo("TIM auth: received server version request (F8)");
  } else if (spec->kind == ShortMsgKind::RandomTrigger) {
    // Random challenge trigger is compatibility-level; round is not used here.
    auth_type_field_ = fr.data[2];
    pending_server_rnd_request_ = true;
    logInfo("TIM auth: received server random-challenge request trigger");
  } else if (spec->kind == ShortMsgKind::CertTrigger) {
    auth_type_field_ = fr.data[2];
    const std::uint8_t round = fr.data[4];
    requested_cert_round_ = round;
    if (step_ == Step::WaitRandomTp) {
      // Interop: some servers send first certificate trigger immediately after
      // random trigger, before random TP payload is fully processed.
      pending_server_cert_request_ = true;
      logInfo("TIM auth: queued server certificate request trigger (waiting random TP)");
      return;
    }
    if (step_ != Step::WaitCertTrigger && step_ != Step::WaitCertTp) {
      if (protocol_violation(
          "certificate trigger received in invalid state step=" +
          std::to_string(static_cast<unsigned>(step_)))) return;
      return;
    }
    std::uint8_t next_round = 0U;
    for (std::size_t i = 0; i < server_cert_received_.size(); ++i) {
      if (!server_cert_received_[i]) {
        next_round = static_cast<std::uint8_t>(i + 1U);
        break;
      }
    }
    if (next_round != 0U && round > next_round) {
      if (protocol_violation(
          "certificate trigger requested round " +
          std::to_string(static_cast<unsigned>(round)) + " (" + cert_role_name(round) +
          ") before required round " +
          std::to_string(static_cast<unsigned>(next_round)) + " (" + cert_role_name(next_round) + ")")) return;
      return;
    }
    pending_server_cert_request_ = true;
    logInfo("TIM auth: received server certificate request trigger");
  } else if (spec->kind == ShortMsgKind::FinalizeTrigger) {
    auth_type_field_ = fr.data[2];
    if (step_ != Step::WaitFinalizeTrigger && step_ != Step::WaitFinalizeTp) {
      if (protocol_violation(
          "finalize trigger received in invalid state step=" +
          std::to_string(static_cast<unsigned>(step_)))) return;
      return;
    }
    if (!strict_mode_ && fr.data[4] == 0xFFU) {
      logWarn(
        "TIM auth interop: finalize trigger used wildcard round 0xFF, "
        "mapping to round " + std::to_string(static_cast<unsigned>(requested_finalize_round_)));
    } else {
      requested_finalize_round_ = fr.data[4];
    }
    pending_server_finalize_request_ = true;
    logInfo("TIM auth: received server finalize request trigger");
  }
}

void AuthLibProvider::on_tp_payload(std::uint32_t pgn, const std::vector<std::uint8_t> & payload)
{
  // Parse/validate AEF 040 TP payload messages for current session.
  if ((pgn != PGN_AUTH_S2C && pgn != PGN_AUTH_C2S) || payload.empty()) return;
  const std::uint8_t msg = payload[0];
  const TpMsgSpec * spec = find_tp_msg_spec(msg);
  if (spec == nullptr) {
    return;
  }
  if (payload.size() < spec->min_size) {
    if (protocol_violation(
        "auth TP payload code=0x" + to_hex(&msg, 1U) +
        " too short, len=" + std::to_string(payload.size()))) return;
    return;
  }
  if (payload[1] != 0x00U) {
    if (protocol_violation(
        "auth TP payload code=0x" + to_hex(&msg, 1U) +
        " carries non-zero error code " + std::to_string(static_cast<unsigned>(payload[1])))) return;
    // Non-strict mode: allow payload processing despite non-zero error field.
  }

  if (spec->kind == TpMsgKind::RandomPayload) {
    got_server_tp_04_ = true;
    const std::size_t rnd_len = static_cast<std::size_t>(payload[5]) |
      (static_cast<std::size_t>(payload[6]) << 8U);
    const std::size_t available = payload.size() - 7U;
    if (rnd_len != 32U) {
      if (protocol_violation("random challenge length must be 32 bytes")) return;
      return;
    }
    if (available < rnd_len) {
      if (protocol_violation("random TP payload length field exceeds payload size")) return;
      return;
    }
    std::copy(payload.begin() + 7, payload.begin() + 7 + rnd_len, server_random_challenge_.begin());
    server_random_valid_ = true;
    logInfo("TIM auth: received TP server random payload");
  } else if (spec->kind == TpMsgKind::CertPayload) {
    got_server_tp_02_ = true;
    const std::uint8_t round = payload[4];
    if (round < 1U || round > kCertRoundsRequired) {
      if (protocol_violation("certificate TP payload has invalid round")) return;
      return;
    }
    if (expected_server_cert_round_ != 0U && round != expected_server_cert_round_) {
      if (protocol_violation(
          "certificate TP round mismatch: expected=" +
          std::to_string(static_cast<unsigned>(expected_server_cert_round_)) +
          " got=" + std::to_string(static_cast<unsigned>(round)))) return;
      return;
    }
    const std::size_t round_idx = static_cast<std::size_t>(round - 1U);
    if (
      cert_exchange_state_[round_idx] != CertExchangeState::ClientSent &&
      cert_exchange_state_[round_idx] != CertExchangeState::ServerReceived)
    {
      if (protocol_violation(
          "certificate TP received before client sent matching certificate for round " +
          std::to_string(static_cast<unsigned>(round)) + " (" + cert_role_name(round) + ")")) return;
      return;
    }
    const std::size_t cert_len = static_cast<std::size_t>(payload[5]) |
      (static_cast<std::size_t>(payload[6]) << 8U);
    const std::size_t available = payload.size() - 7U;
    if (cert_len == 0U || cert_len > available) {
      if (protocol_violation("certificate TP payload length field is invalid")) return;
      return;
    }
    server_certs_[round_idx].assign(payload.begin() + 7, payload.begin() + 7 + cert_len);
    server_cert_received_[round_idx] = true;
    cert_exchange_state_[round_idx] = CertExchangeState::ServerReceived;
    logInfo("TIM auth: received TP server certificate payload");
  } else if (spec->kind == TpMsgKind::FinalizePayload) {
    got_server_tp_06_ = true;
    server_finalize_payload_seen_ = true;
    std::uint8_t round = payload[4];
    if (!strict_mode_ && round == 0xFFU) {
      // Interop: tolerate wildcard finalize round in TP payload.
      round = (expected_server_finalize_round_ != 0U) ? expected_server_finalize_round_ : requested_finalize_round_;
      logWarn(
        "TIM auth interop: finalize TP used wildcard round 0xFF, mapping to round " +
        std::to_string(static_cast<unsigned>(round)));
    }
    if (round < 1U || round > kCertRoundsRequired) {
      if (protocol_violation("finalize TP payload has invalid round")) return;
      return;
    }
    if (expected_server_finalize_round_ != 0U && round != expected_server_finalize_round_) {
      if (protocol_violation(
          "finalize TP round mismatch: expected=" +
          std::to_string(static_cast<unsigned>(expected_server_finalize_round_)) +
          " got=" + std::to_string(static_cast<unsigned>(round)))) return;
      return;
    }
    const std::size_t sig_len = static_cast<std::size_t>(payload[5]) |
      (static_cast<std::size_t>(payload[6]) << 8U);
    if (sig_len != 16U) {
      if (protocol_violation("signed challenge length must be 16 bytes")) return;
      return;
    }
    const std::size_t available = payload.size() - 7U;
    if (available < sig_len) {
      if (protocol_violation("signed challenge length field exceeds payload size")) return;
      return;
    }
    std::copy(payload.begin() + 7, payload.begin() + 7 + sig_len, server_signed_challenge_.begin());
    server_signed_valid_ = true;
    logInfo("TIM auth: received TP server finalize payload");
    if (!pending_server_finalize_request_ && step_ == Step::WaitFinalizeTrigger) {
      // Interop: some servers send finalize TP payload before short finalize trigger.
      pending_server_finalize_request_ = true;
      logInfo("TIM auth interop: inferred finalize request from TP payload");
    }
  }
}

void AuthLibProvider::process(std::uint32_t now_ms)
{
  // Drive AEF 040 authentication FSM with periodic status and round-request handling.
  if (client_sa_ > 0xFDU || server_sa_ > 0xFDU) return;

  // Always keep status alive during authentication process.
  send_auth_status(now_ms, !restart_sent_);
  restart_sent_ = true;

  if (authenticated_) return;
  if (failed_) return;
  if (!server_status_seen_) return;

  // Reply to server version request only when requested (F8).
  if (pending_f8_request_) {
    sendFrame(make_client_version_response_frame());
    pending_f8_request_ = false;
    logInfo("TIM auth: sent client version response (F7)");
  }

  if (step_deadline_ms_ != 0 && now_ms > step_deadline_ms_) {
    if (retries_ < kMaxRetries) {
      retries_ = static_cast<std::uint8_t>(retries_ + 1U);
      logWarn(
        "TIM auth: timeout in step=" + std::to_string(static_cast<unsigned>(step_)) +
        ", retry " + std::to_string(static_cast<unsigned>(retries_)) + "/" +
        std::to_string(static_cast<unsigned>(kMaxRetries)));
      if (step_ == Step::WaitRandomTp) {
        step_ = Step::WaitRandomTrigger;
      } else if (step_ == Step::WaitCertTp) {
        step_ = Step::WaitCertTrigger;
        expected_server_cert_round_ = 0;
      } else if (step_ == Step::WaitFinalizeTrigger) {
        finalize_request_sent_ = false;
        expected_server_finalize_round_ = 0;
      } else if (step_ == Step::WaitFinalizeTp) {
        step_ = Step::WaitFinalizeTrigger;
        finalize_request_sent_ = false;
        expected_server_finalize_round_ = 0;
        client_finalize_payload_sent_ = false;
        server_finalize_payload_seen_ = false;
        client_authenticated_status_sent_ = false;
      }
      step_deadline_ms_ = 0;
    } else {
      failed_ = true;
      logError(
        "TIM auth: failed after maximum retries at step=" +
        std::to_string(static_cast<unsigned>(step_)));
      return;
    }
  }

  send_round_requests(now_ms);
}

bool AuthLibProvider::is_authenticated() const
{
  return authenticated_;
}

bool AuthLibProvider::is_lead_server() const
{
  return authenticated_;
}

bool AuthLibProvider::is_failed() const
{
  return failed_;
}

}  // namespace ros2_isobus
