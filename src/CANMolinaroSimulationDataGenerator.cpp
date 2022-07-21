#include "CANMolinaroSimulationDataGenerator.h"
#include "CANMolinaroAnalyzerSettings.h"

//--------------------------------------------------------------------------------------------------

#include <AnalyzerHelpers.h>

//--------------------------------------------------------------------------------------------------
//  CAN FRAME GENERATOR
//--------------------------------------------------------------------------------------------------

typedef enum {standardFrame, extendedFrame} FrameFormat ;

//--------------------------------------------------------------------------------------------------

typedef enum {dataFrame, remoteFrame} FrameType ;

//--------------------------------------------------------------------------------------------------

typedef enum {ACK_SLOT_DOMINANT, ACK_SLOT_RECESSIVE} AckSlot ;

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
  private : uint32_t mConsecutiveBitCount ;
  private : bool mLastBitValue ;
  private : uint16_t mCRCAccumulator ;
} ;

//--------------------------------------------------------------------------------------------------

CANFrameBitsGenerator::CANFrameBitsGenerator (const uint32_t inIdentifier,
                                              const FrameFormat inFrameFormat,
                                              const uint8_t inDataLength,
                                              const uint8_t inData [8],
                                              const FrameType inFrameType,
                                              const AckSlot inAckSlot) :
mBits (),
mFrameLength (0),
mConsecutiveBitCount (1),
mLastBitValue (true),
mCRCAccumulator (0) {
  for (uint32_t i=0 ; i<5 ; i++) {
    mBits [i] = UINT32_MAX ;
  }
  const uint8_t dataLength = (inDataLength > 15) ? 15 : inDataLength ;
//--- Generate frame
  enterBitAppendStuff (false) ; // SOF
  switch (inFrameFormat) {
  case extendedFrame :
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
  case standardFrame :
    for (int idx = 10 ; idx >= 0 ; idx--) { // Identifier
      const bool bit = (inIdentifier & (1 << idx)) != 0 ;
      enterBitAppendStuff (bit) ;
    }
    break ;
  }
  enterBitAppendStuff (inFrameType == remoteFrame) ; // RTR
  enterBitAppendStuff (false) ; // RESERVED 1
  enterBitAppendStuff (false) ; // RESERVED 0
  enterBitAppendStuff ((dataLength & 8) != 0) ; // DLC 3
  enterBitAppendStuff ((dataLength & 4) != 0) ; // DLC 2
  enterBitAppendStuff ((dataLength & 2) != 0) ; // DLC 1
  enterBitAppendStuff ((dataLength & 1) != 0) ; // DLC 0
//--- Enter DATA
  if (inFrameType == dataFrame) {
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
  case ACK_SLOT_DOMINANT :
    enterBitNoStuff (false) ;
    break ;
  case ACK_SLOT_RECESSIVE :
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

CANMolinaroSimulationDataGenerator::CANMolinaroSimulationDataGenerator (void) :
mSettings (nullptr),
mSimulationSampleRateHz (0),
mSeed (0),
mSerialSimulationData (new SimulationChannelDescriptor ()) {
}

//--------------------------------------------------------------------------------------------------

CANMolinaroSimulationDataGenerator::~CANMolinaroSimulationDataGenerator (void) {
  delete mSerialSimulationData ;
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroSimulationDataGenerator::Initialize (const U32 simulation_sample_rate,
                                                     CANMolinaroAnalyzerSettings * settings) {
  mSimulationSampleRateHz = simulation_sample_rate;
  mSettings = settings;

  mSerialSimulationData->SetChannel (mSettings->mInputChannel);
  mSerialSimulationData->SetSampleRate (simulation_sample_rate) ;
  mSerialSimulationData->SetInitialBitState (BIT_HIGH) ;
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

//--- Random Seed
  mSeed = mSettings->simulatorRandomSeed () ;

//--- Let's move forward for 11 recessive bits
  const U32 samplesPerBit = mSimulationSampleRateHz / mSettings->mBitRate;
  const bool inverted = mSettings->inverted () ;
  mSerialSimulationData->TransitionIfNeeded (inverted ? BIT_LOW : BIT_HIGH) ;  // Edge for IDLE
  mSerialSimulationData->Advance (samplesPerBit * 11) ;
  mSerialSimulationData->TransitionIfNeeded (inverted ? BIT_HIGH : BIT_LOW) ;  // Edge for SOF bit

  while (mSerialSimulationData->GetCurrentSampleNumber() < adjusted_largest_sample_requested) {
    createCANFrame (samplesPerBit, inverted) ;
  }
  mSerialSimulationData->TransitionIfNeeded (inverted ? BIT_LOW : BIT_HIGH) ; //we need to end recessive

  *simulation_channel = mSerialSimulationData ;
  return 1 ;
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroSimulationDataGenerator::createCANFrame (const U32 inSamplesPerBit,
                                                         const bool inInverted) {
  const U32 frameTypes = mSettings->generatedFrameType () ;
  const U32 simulatorFrameValidity = mSettings->generatedFrameValidity () ;
//--- Generate random Frame
  bool extended = false ;
  bool remote = false ;
  switch (frameTypes) {
  case GENERATE_ONLY_STANDARD_DATA :
    break ;
  case GENERATE_ONLY_EXTENDED_DATA :
    extended = true ;
    break ;
  case GENERATE_ONLY_STANDARD_REMOTE :
    remote = true ;
    break ;
  case GENERATE_ONLY_EXTENDED_REMOTE :
    extended = true ;
    remote = true ;
    break ;
  default : // GENERATE_ALL_FRAME_TYPES
    extended = (pseudoRandomValue () & 1) != 0 ;
    remote = (pseudoRandomValue () & 1) != 0 ;
    break ;
  }
//--- ACK slot
  AckSlot ack = ACK_SLOT_DOMINANT ;
  switch (mSettings->generatedAckSlot ()) {
  case GENERATE_ACK_DOMINANT :
    break ;
  case GENERATE_ACK_RECESSIVE :
    ack = ACK_SLOT_RECESSIVE ;
    break ;
  case GENERATE_ACK_RANDOMLY :
    ack = ((pseudoRandomValue () & 1) != 0) ? ACK_SLOT_DOMINANT : ACK_SLOT_RECESSIVE ;
    break ;
  }
//--- Generate frame
  uint8_t data [8] ;
  const FrameFormat format = extended ? extendedFrame : standardFrame ;
  const FrameType type = remote ? remoteFrame : dataFrame ;
  const uint32_t identifier = uint32_t (pseudoRandomValue ()) & (extended ? 0x1FFFFFFF : 0x7FF) ;
  const uint8_t dataLength = uint8_t (pseudoRandomValue ()) % 9 ;
  if (! remoteFrame) {
    for (uint32_t i=0 ; i<dataLength ; i++) {
      data [i] = uint8_t (pseudoRandomValue ()) ;
    }
  }
  const CANFrameBitsGenerator frame (identifier, format, dataLength, data, type, ack) ;
//--- Generated bit error index
  U32 generatedErrorBitIndex = U32 (pseudoRandomValue ()) % frame.frameLength () ;
  if (simulatorFrameValidity == GENERATE_VALID_FRAMES) {
    generatedErrorBitIndex = 255 ;  // Means no generated error
  }
//--- Now, send frame
  for (U32 i=0 ; i < frame.frameLength () ; i++) {
    const bool generateBitError = (i == generatedErrorBitIndex) ;
    const bool bit = frame.bitAtIndex (i) ^ inInverted ^ generateBitError ;
    mSerialSimulationData->TransitionIfNeeded (bit ? BIT_HIGH : BIT_LOW) ;
    mSerialSimulationData->Advance (inSamplesPerBit) ;
  }
//  mSerialSimulationData->TransitionIfNeeded (inInverted ? BIT_LOW : BIT_HIGH) ; //we need to end recessive
}

//--------------------------------------------------------------------------------------------------
