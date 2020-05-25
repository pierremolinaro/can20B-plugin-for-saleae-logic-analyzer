#include "CANMolinaroSimulationDataGenerator.h"
#include "CANMolinaroAnalyzerSettings.h"

//--------------------------------------------------------------------------------------------------

#include <AnalyzerHelpers.h>

//--------------------------------------------------------------------------------------------------
//  CAN FRAME GENERATOR
//--------------------------------------------------------------------------------------------------

typedef enum {standard, extended} FrameFormat ;

//--------------------------------------------------------------------------------------------------

typedef enum {data, remote} FrameType ;

//--------------------------------------------------------------------------------------------------

typedef enum {ACK_SLOT_DOMINANT, ACK_SLOT_RECESSIVE} AckSlot ;

//--------------------------------------------------------------------------------------------------

static const uint32_t CAN_FRAME_MAX_LENGTH = 160 ;

//--------------------------------------------------------------------------------------------------

class CANFrameBitsGenerator {
  public : CANFrameBitsGenerator (const uint32_t inIdentifier,
                                  const FrameFormat inFrameFormat,
                                  const uint8_t inDataLength,
                                  const uint8_t inData [8],
                                  const FrameType inFrameType,
                                  const AckSlot inAckSlot) ;

//--- Public methods
  public : inline uint32_t frameLength (void) const { return mFrameLength ; }
  public : bool bitAtIndex (const uint32_t inIndex) const ;

//--- Private methods (used during frame generation)
  private: void enterBitAppendStuff (const bool inBit) ;

  private: void enterBitNoStuff (const bool inBit) ;

//--- Private properties
  private: uint32_t mBits [5] ;
  private: uint8_t mFrameLength ;

//--- CRC computation
  private : uint32_t mConsecutiveBitCount = 1 ;
  private : bool mLastBitValue = true ;
  private : uint16_t mCRCAccumulator = 0 ;
} ;

//--------------------------------------------------------------------------------------------------

CANFrameBitsGenerator::CANFrameBitsGenerator (const uint32_t inIdentifier,
                                              const FrameFormat inFrameFormat,
                                              const uint8_t inDataLength,
                                              const uint8_t inData [8],
                                              const FrameType inFrameType,
                                              const AckSlot inAckSlot) :
mBits (),
mFrameLength (0) {
  for (uint32_t i=0 ; i<5 ; i++) {
    mBits [i] = UINT32_MAX ;
  }
  const uint8_t dataLength = (inDataLength > 15) ? 15 : inDataLength ;
//--- Generate frame
  enterBitAppendStuff (false) ; // SOF
  switch (inFrameFormat) {
  case FrameFormat::extended :
    for (uint8_t idx = 28 ; idx >= 18 ; idx--) { // Identifier
      const bool bit = (inIdentifier & (1 << idx)) != 0 ;
      enterBitAppendStuff (bit) ;
    }
    enterBitAppendStuff (true) ; // SRR
    enterBitAppendStuff (true) ; // IDE
    for (int idx = 17 ; idx >= 0 ; idx--) { // Identifier
      const bool bit = (inIdentifier & (1 << idx)) != 0 ;
      enterBitAppendStuff (bit) ;
    }
    break ;
  case FrameFormat::standard :
    for (int idx = 10 ; idx >= 0 ; idx--) { // Identifier
      const bool bit = (inIdentifier & (1 << idx)) != 0 ;
      enterBitAppendStuff (bit) ;
    }
    break ;
  }
  enterBitAppendStuff (inFrameType == FrameType::remote) ; // RTR
  enterBitAppendStuff (false) ; // RESERVED 1
  enterBitAppendStuff (false) ; // RESERVED 0
  enterBitAppendStuff ((dataLength & 8) != 0) ; // DLC 3
  enterBitAppendStuff ((dataLength & 4) != 0) ; // DLC 2
  enterBitAppendStuff ((dataLength & 2) != 0) ; // DLC 1
  enterBitAppendStuff ((dataLength & 1) != 0) ; // DLC 0
//--- Enter DATA
  if (inFrameType == FrameType::data) {
    const uint8_t maxLength = (dataLength > 8) ? 8 : dataLength ;
    for (uint8_t dataIdx = 0 ; dataIdx < maxLength ; dataIdx ++) {
      for (int bitIdx = 7 ; bitIdx >= 0 ; bitIdx--) {
        enterBitAppendStuff ((inData [dataIdx] & (1 << bitIdx)) != 0) ;
      }
    }
  }
//--- Enter CRC SEQUENCE
  const uint16_t frameCRC = mCRCAccumulator ;
  for (int idx = 14 ; idx >= 0 ; idx--) {
    const bool bit = (frameCRC & (1 << idx)) != 0 ;
    enterBitAppendStuff (bit) ;
  }
//--- Enter ACK, EOF, INTERMISSION
  enterBitNoStuff (true) ; // CRC DEL
  switch (inAckSlot) {
  case AckSlot::ACK_SLOT_DOMINANT :
    enterBitNoStuff (false) ;
    break ;
  case AckSlot::ACK_SLOT_RECESSIVE :
    enterBitNoStuff (true) ;
    break ;
  }
//--- ACK DEL, EOF (7), INTERMISSION (3), all RECESSIVE
  mFrameLength += 11 ;
}

//--------------------------------------------------------------------------------------------------

void CANFrameBitsGenerator::enterBitAppendStuff (const bool inBit) {
//--- Compute CRC
  const bool bit14 = (mCRCAccumulator & (1 << 14)) != 0 ;
  const bool crc_nxt = inBit ^ bit14 ;
  mCRCAccumulator <<= 1 ;
  mCRCAccumulator &= 0x7FFF ;
  if (crc_nxt) {
    mCRCAccumulator ^= 0x4599 ;
  }
//--- Emit bit
  if (!inBit) {
    const uint32_t idx = mFrameLength / 32 ;
    const uint32_t offset = mFrameLength % 32 ;
    mBits [idx] &= ~ (1U << offset) ;
  }
  mFrameLength ++ ;
//--- Add a stuff bit ?
  if (mLastBitValue == inBit) {
    mConsecutiveBitCount += 1 ;
    if (mConsecutiveBitCount == 5) {
      mLastBitValue ^= true ;
      if (!mLastBitValue) {
        const uint32_t idx = mFrameLength / 32 ;
        const uint32_t offset = mFrameLength % 32 ;
        mBits [idx] &= ~ (1U << offset) ;
      }
      mFrameLength ++ ;
      mConsecutiveBitCount = 1 ;
    }
  }else{
    mLastBitValue = inBit ;
    mConsecutiveBitCount = 1 ;
  }
}

//--------------------------------------------------------------------------------------------------

void CANFrameBitsGenerator::enterBitNoStuff (const bool inBit) {
//--- Emit bit
  if (!inBit) {
    const uint32_t idx = mFrameLength / 32 ;
    const uint32_t offset = mFrameLength % 32 ;
    mBits [idx] &= ~ (1U << offset) ;
  }
  mFrameLength ++ ;
}

//--------------------------------------------------------------------------------------------------

bool CANFrameBitsGenerator::bitAtIndex (const uint32_t inIndex) const {
  bool result = true ; // RECESSIF
  if (inIndex < mFrameLength) {
    const uint32_t idx = inIndex / 32 ;
    const uint32_t offset = inIndex % 32 ;
    result = (mBits [idx] & (1U << offset)) != 0 ;
  }
  return result ;
}

//--------------------------------------------------------------------------------------------------
//  CANMolinaroSimulationDataGenerator
//--------------------------------------------------------------------------------------------------

CANMolinaroSimulationDataGenerator::CANMolinaroSimulationDataGenerator() {
}

//--------------------------------------------------------------------------------------------------

CANMolinaroSimulationDataGenerator::~CANMolinaroSimulationDataGenerator () {
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroSimulationDataGenerator::Initialize(const U32 simulation_sample_rate,
                                                    CANMolinaroAnalyzerSettings * settings) {
  mSimulationSampleRateHz = simulation_sample_rate;
  mSettings = settings;

  mSerialSimulationData.SetChannel (mSettings->mInputChannel);
  mSerialSimulationData.SetSampleRate (simulation_sample_rate) ;
  mSerialSimulationData.SetInitialBitState (BIT_HIGH) ;
}

//--------------------------------------------------------------------------------------------------

U32 CANMolinaroSimulationDataGenerator::GenerateSimulationData (const U64 largest_sample_requested,
                                                                const U32 sample_rate,
                                        SimulationChannelDescriptor** simulation_channel) {
  const U64 adjusted_largest_sample_requested = AnalyzerHelpers::AdjustSimulationTargetSample (
    largest_sample_requested,
    sample_rate,
    mSimulationSampleRateHz
  );

  while (mSerialSimulationData.GetCurrentSampleNumber() < adjusted_largest_sample_requested) {
    CreateCANFrame () ;
  }

  *simulation_channel = &mSerialSimulationData;
  return 1;
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroSimulationDataGenerator::CreateCANFrame () {
  const U32 samples_per_bit = mSimulationSampleRateHz / mSettings->mBitRate;
  const bool inverted = mSettings->inverted () ;
  const SimulatorGeneratedFrameType frameTypes = mSettings->generatedFrameType () ;
//--- Generate random Frame
  bool extended = false ;
  bool remoteFrame = false ;
  switch (frameTypes) {
  case GENERATE_ALL_FRAME_TYPES :
    extended = (random () & 1) != 0 ;
    remoteFrame = (random () & 1) != 0 ;
    break ;
  case GENERATE_ONLY_STANDARD_DATA :
    break ;
  case GENERATE_ONLY_EXTENDED_DATA :
    extended = true ;
    break ;
  case GENERATE_ONLY_STANDARD_REMOTE :
    remoteFrame = true ;
    break ;
  case GENERATE_ONLY_EXTENDED_REMOTE :
    extended = true ;
    remoteFrame = true ;
    break ;
  }
  uint8_t data [8] ;
  const FrameFormat format = extended ? FrameFormat::extended : FrameFormat::standard ;
  const FrameType type = remoteFrame ? FrameType::remote : FrameType::data ;
  const uint32_t identifier = uint32_t (random ()) & (extended ? 0x1FFFFFFF : 0x7FF) ;
  const uint8_t dataLength = uint8_t (random ()) % 9 ;
  if (! remoteFrame) {
    for (uint32_t i=0 ; i<dataLength ; i++) {
      data [i] = uint8_t (random ()) ;
    }
  }
  AckSlot ack = AckSlot::ACK_SLOT_DOMINANT ;
  switch (mSettings->generatedAckSlot ()) {
  case GENERATE_ACK_DOMINANT :
    break ;
  case GENERATE_ACK_RECESSIVE :
    ack = AckSlot::ACK_SLOT_RECESSIVE ;
    break ;
  GENERATE_ACK_RANDOMLY :
    ack = ((random () & 1) != 0) ? AckSlot::ACK_SLOT_DOMINANT : AckSlot::ACK_SLOT_RECESSIVE ;
    break ;
  }
  const CANFrameBitsGenerator frame (identifier, format, dataLength, data, type, ack) ;
//--- Let's move forward for 11 recessive bits
  mSerialSimulationData.TransitionIfNeeded (inverted ? BIT_LOW : BIT_HIGH) ;  // Edge for IDLE
  mSerialSimulationData.Advance (samples_per_bit * 11) ;
  mSerialSimulationData.TransitionIfNeeded (inverted ? BIT_HIGH : BIT_LOW) ;  // Edge for SOF bit
//--- Now, send frame
  for (U32 i=0 ; i < frame.frameLength () ; i++) {
    const bool bit = frame.bitAtIndex (i) ^ inverted ;
    mSerialSimulationData.TransitionIfNeeded (bit ? BIT_HIGH : BIT_LOW) ;
    mSerialSimulationData.Advance (samples_per_bit) ;
  }
  mSerialSimulationData.TransitionIfNeeded (inverted ? BIT_LOW : BIT_HIGH) ; //we need to end recessive
}

//--------------------------------------------------------------------------------------------------
