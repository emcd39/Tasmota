
/// @file
/// @brief Support for Kelon 168-bit IR protocol only. (KELON168)
/// Removed legacy 48-bit KELON protocol support.

#include "ir_Kelon.h"
#include "IRrecv.h"
#include "IRsend.h"
#include "IRutils.h"
#include "IRtext.h"

// Retain only KELON168 protocol code.

// Constants
const uint32_t kKelon168FooterSpace = 8000;
const uint16_t kKelon168Section1Size = 6;
const uint16_t kKelon168Section2Size = 8;
const uint16_t kKelon168Section3Size = 7;
const uint16_t kKelonFreq = 38000;

#if SEND_KELON168
void IRsend::sendKelon168(const uint8_t data[], const uint16_t nbytes, const uint16_t repeat) {
  assert(kKelon168StateLength == kKelon168Section1Size + kKelon168Section2Size + kKelon168Section3Size);
  if (nbytes < kKelon168StateLength) return;

  for (uint16_t r = 0; r <= repeat; r++) {
    sendGeneric(kKelonHdrMark, kKelonHdrSpace, kKelonBitMark, kKelonOneSpace,
                kKelonBitMark, kKelonZeroSpace, kKelonBitMark, kKelon168FooterSpace,
                data, kKelon168Section1Size, kKelonFreq, false, 0, kDutyDefault);
    sendGeneric(0, 0, kKelonBitMark, kKelonOneSpace, kKelonBitMark, kKelonZeroSpace,
                kKelonBitMark, kKelon168FooterSpace,
                data + kKelon168Section1Size, kKelon168Section2Size, kKelonFreq, false, 0, kDutyDefault);
    sendGeneric(0, 0, kKelonBitMark, kKelonOneSpace, kKelonBitMark, kKelonZeroSpace,
                kKelonBitMark, kKelonGap,
                data + kKelon168Section1Size + kKelon168Section2Size, kKelon168Section3Size,
                kKelonFreq, false, 0, kDutyDefault);
  }
}
#endif  // SEND_KELON168

#if DECODE_KELON168
bool IRrecv::decodeKelon168(decode_results *results, uint16_t offset,
                            const uint16_t nbits, const bool strict) {
  if (strict && nbits != kKelon168Bits) return false;
  if (results->rawlen <= 2 * nbits + kHeader + kFooter * 2 - 1 + offset)
    return false;

  uint16_t used = 0;
  used = matchGeneric(results->rawbuf + offset, results->state,
                      results->rawlen - offset, kKelon168Section1Size * 8,
                      kKelonHdrMark, kKelonHdrSpace,
                      kKelonBitMark, kKelonOneSpace,
                      kKelonBitMark, kKelonZeroSpace,
                      kKelonBitMark, kKelon168FooterSpace,
                      false, _tolerance, 0, false);
  if (!used) return false;
  offset += used;

  used = matchGeneric(results->rawbuf + offset, results->state + kKelon168Section1Size,
                      results->rawlen - offset, kKelon168Section2Size * 8,
                      0, 0, kKelonBitMark, kKelonOneSpace,
                      kKelonBitMark, kKelonZeroSpace,
                      kKelonBitMark, kKelon168FooterSpace,
                      false, _tolerance, 0, false);
  if (!used) return false;
  offset += used;

  used = matchGeneric(results->rawbuf + offset,
                      results->state + kKelon168Section1Size + kKelon168Section2Size,
                      results->rawlen - offset,
                      nbits - (kKelon168Section1Size + kKelon168Section2Size) * 8,
                      0, 0, kKelonBitMark, kKelonOneSpace,
                      kKelonBitMark, kKelonZeroSpace,
                      kKelonBitMark, kKelonGap,
                      true, _tolerance, 0, false);
  if (!used) return false;

  results->decode_type = decode_type_t::KELON168;
  results->bits = nbits;
  return true;
}
#endif
