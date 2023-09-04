#include "CANMolinaroAnalyzer.h"
#include "CANMolinaroAnalyzerSettings.h"

#include <AnalyzerChannelData.h>

#include <string>
#include <sstream>

//----------------------------------------------------------------------------------------
//   CANMolinaroAnalyzer
//----------------------------------------------------------------------------------------

CANMolinaroAnalyzer::CANMolinaroAnalyzer () :
Analyzer2 (),
mSettings (new CANMolinaroAnalyzerSettings ()),
mSimulationInitilized (false) {
  SetAnalyzerSettings (mSettings.get()) ;
  UseFrameV2 () ;
}

//----------------------------------------------------------------------------------------

CANMolinaroAnalyzer::~CANMolinaroAnalyzer () {
  KillThread () ;
}

//----------------------------------------------------------------------------------------

U32 CANMolinaroAnalyzer::bitRate (void) const {
  return mSettings->mBitRate ;
}

//----------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::SetupResults () {
  mResults.reset (new CANMolinaroAnalyzerResults (this, mSettings.get())) ;
  SetAnalyzerResults (mResults.get()) ;
  mResults->AddChannelBubblesWillAppearOn (mSettings->mInputChannel) ;
}


//----------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::WorkerThread (void) {
  const bool inverted = mSettings->inverted () ;
  mSampleRateHz = GetSampleRate () ;
  AnalyzerChannelData * serial = GetAnalyzerChannelData (mSettings->mInputChannel) ;
//--- Sample settings
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
//--- Synchronize to recessive level
  if (serial->GetBitState () == (inverted ? BIT_HIGH : BIT_LOW)) {
    serial->AdvanceToNextEdge () ;
  }
//--- Initial bit state
  mFrameFieldEngineState = IDLE ;
  mPreviousBit = (serial->GetBitState () == BIT_HIGH) ^ inverted ;
  mUnstuffingActive = false ;
  while (1) {
    const bool currentBitValue = (serial->GetBitState () == BIT_HIGH) ^ inverted ;
    const U64 start = serial->GetSampleNumber () ;
    const U64 nextEdge = serial->GetSampleOfNextEdge () ;
    const U64 bitCount = (nextEdge - start + samplesPerBit / 2) / samplesPerBit ;
    for (U64 i=0 ; i<bitCount ; i++) {
      enterBit (currentBitValue, start + i * samplesPerBit + samplesPerBit / 2) ;
    }
    mResults->CommitResults () ;
    serial->AdvanceToNextEdge () ;
  }
}

//----------------------------------------------------------------------------------------

bool CANMolinaroAnalyzer::NeedsRerun () {
  return false;
}

//----------------------------------------------------------------------------------------

U32 CANMolinaroAnalyzer::GenerateSimulationData (U64 minimum_sample_index,
                                                 U32 device_sample_rate,
                                                 SimulationChannelDescriptor** simulation_channels ) {
  if (mSimulationInitilized == false) {
    mSimulationDataGenerator.Initialize( GetSimulationSampleRate(), mSettings.get() );
    mSimulationInitilized = true;
  }
  return mSimulationDataGenerator.GenerateSimulationData (minimum_sample_index,
                                                          device_sample_rate,
                                                          simulation_channels) ;
}

//----------------------------------------------------------------------------------------

U32 CANMolinaroAnalyzer::GetMinimumSampleRateHz () {
  return mSettings->mBitRate * 5 ;
}

//----------------------------------------------------------------------------------------

const char* CANMolinaroAnalyzer::GetAnalyzerName () const {
  return "CAN 2.0B (Molinaro)";
}

//----------------------------------------------------------------------------------------

const char* GetAnalyzerName (void) {
  return "CAN 2.0B (Molinaro)";
}

//----------------------------------------------------------------------------------------

Analyzer* CreateAnalyzer (void) {
  return new CANMolinaroAnalyzer () ;
}

//----------------------------------------------------------------------------------------

void DestroyAnalyzer (Analyzer* analyzer) {
  delete analyzer;
}

//----------------------------------------------------------------------------------------
//  CAN FRAME DECODER
//----------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::enterBit (const bool inBitValue,
                                    const U64 inSampleNumber) {
  if (!mUnstuffingActive) {
    decodeFrameBit (inBitValue, inSampleNumber) ;
  }else if ((mConsecutiveBitCountOfSamePolarity == 5) && (inBitValue != mPreviousBit)) {
   // Stuff bit - discarded
    addMark (inSampleNumber, AnalyzerResults::X);
    mConsecutiveBitCountOfSamePolarity = 1 ;
    mPreviousBit = inBitValue ;
    mStuffBitCount += 1 ;
  }else if ((mConsecutiveBitCountOfSamePolarity == 5) && (mPreviousBit == inBitValue)) { // Stuff Error
    addMark (inSampleNumber, AnalyzerResults::ErrorX);
    const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
    enterInErrorMode (inSampleNumber + samplesPerBit / 2) ;
    mConsecutiveBitCountOfSamePolarity += 1 ;
  }else if (mPreviousBit == inBitValue) {
    mConsecutiveBitCountOfSamePolarity += 1 ;
    decodeFrameBit (inBitValue, inSampleNumber) ;
  }else{
    mConsecutiveBitCountOfSamePolarity = 1 ;
    mPreviousBit = inBitValue ;
    decodeFrameBit (inBitValue, inSampleNumber) ;
  }
}

//----------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::decodeFrameBit (const bool inBitValue,
                                          const U64 inSampleNumber) {
  switch (mFrameFieldEngineState) {
  case IDLE :
    handle_IDLE_state (inBitValue, inSampleNumber) ;
    break ;
  case IDENTIFIER :
    handle_IDENTIFIER_state (inBitValue, inSampleNumber) ;
    break ;
  case EXTENDED_IDF :
    handle_EXTENDED_IDF_state (inBitValue, inSampleNumber) ;
    break ;
  case CONTROL :
    handle_CONTROL_state (inBitValue, inSampleNumber) ;
    break ;
  case DATA :
    handle_DATA_state (inBitValue, inSampleNumber) ;
    break ;
  case CRC15 :
    handle_CRC15_state (inBitValue, inSampleNumber) ;
    break ;
  case CRC_DEL :
    handle_CRCDEL_state (inBitValue, inSampleNumber) ;
    break ;
  case ACK :
    handle_ACK_state (inBitValue, inSampleNumber) ;
    break ;
  case END_OF_FRAME :
    handle_ENDOFFRAME_state (inBitValue, inSampleNumber) ;
    break ;
  case INTERMISSION :
    handle_INTERMISSION_state (inBitValue, inSampleNumber) ;
    break ;
  case DECODER_ERROR :
    handle_DECODER_ERROR_state (inBitValue, inSampleNumber) ;
    break ;
  }
}

//----------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::handle_IDLE_state (const bool inBitValue,
                                             const U64 inSampleNumber) {
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
  if (inBitValue) {
    addMark (inSampleNumber, AnalyzerResults::Stop) ;
  }else{ // SOF
    mUnstuffingActive = true ;
    mCRC15Accumulator = 0 ;
    mConsecutiveBitCountOfSamePolarity = 1 ;
    mPreviousBit = false ;
    enterBitInCRC15 (inBitValue) ;
    addMark (inSampleNumber, AnalyzerResults::Start) ;
    mFieldBitIndex = 0 ;
    mIdentifier = 0 ;
    mFrameFieldEngineState = IDENTIFIER ;
    mStartOfFieldSampleNumber = inSampleNumber + samplesPerBit / 2 ;
    mStartOfFrameSampleNumber = inSampleNumber ;
    mStuffBitCount = 0 ;
  }
}

//----------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::handle_IDENTIFIER_state (const bool inBitValue,
                                                   const U64 inSampleNumber) {
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
  enterBitInCRC15 (inBitValue) ;
  mFieldBitIndex ++ ;
  if (mFieldBitIndex <= 11) { // Standard identifier
    addMark (inSampleNumber, AnalyzerResults::Dot);
    mIdentifier <<= 1 ;
    mIdentifier |= inBitValue ;
  }else if (mFieldBitIndex == 12) { // RTR bit
    addMark (inSampleNumber, inBitValue ? AnalyzerResults::UpArrow : AnalyzerResults::DownArrow) ;
    mFrameType = inBitValue ? remoteFrame : dataFrame  ;
  }else{ // IDE
    addMark (inSampleNumber, AnalyzerResults::Dot);
    if (inBitValue) {
      mFrameFieldEngineState = EXTENDED_IDF ;
      mFieldBitIndex = 0 ;
    }else{
      addBubble (STANDARD_IDENTIFIER_FIELD_RESULT,
                 mIdentifier,
                 mFrameType == dataFrame, // 0 -> remote, 1 -> data
                 inSampleNumber - samplesPerBit / 2) ;
      mDataCodeLength = 0 ;
      mFrameFieldEngineState = CONTROL ;
      mFieldBitIndex = 1 ;
    }
  }
}

//----------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::handle_EXTENDED_IDF_state (const bool inBitValue,
                                                     const U64 inSampleNumber) {
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
  enterBitInCRC15 (inBitValue) ;
  mFieldBitIndex ++ ;
  if (mFieldBitIndex <= 18) { // Extended identifier
    addMark (inSampleNumber, AnalyzerResults::Dot);
    mIdentifier <<= 1 ;
    mIdentifier |= inBitValue ;
  }else if (mFieldBitIndex == 19) { // RTR bit
    addMark (inSampleNumber, inBitValue ? AnalyzerResults::UpArrow : AnalyzerResults::DownArrow) ;
    mFrameType = inBitValue ? remoteFrame : dataFrame  ;
  }else{ // R1: should be dominant
    addMark (inSampleNumber, inBitValue ? AnalyzerResults::ErrorX : AnalyzerResults::Zero) ;
    addBubble (EXTENDED_IDENTIFIER_FIELD_RESULT,
               mIdentifier,
               mFrameType == dataFrame, // 0 -> remote, 1 -> data
               inSampleNumber - samplesPerBit / 2) ;
    if (inBitValue) {
      enterInErrorMode (inSampleNumber + samplesPerBit / 2) ;
    }else{
      mFrameFieldEngineState = CONTROL ;
      mFieldBitIndex = 1 ;
      mDataCodeLength = 0 ;
    }
  }
}

//----------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::handle_CONTROL_state (const bool inBitValue,
                                                const U64 inSampleNumber) {
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
  enterBitInCRC15 (inBitValue) ;
  mFieldBitIndex ++ ;
  if (mFieldBitIndex == 2) { // R0
    addMark (inSampleNumber, inBitValue ? AnalyzerResults::ErrorX : AnalyzerResults::Zero) ;
    if (inBitValue) {
      enterInErrorMode (inSampleNumber + samplesPerBit / 2) ;
    }
  }else{
    addMark (inSampleNumber, AnalyzerResults::Dot);
    mDataCodeLength <<= 1 ;
    mDataCodeLength |= inBitValue ;
    if (mFieldBitIndex == 6) {
      addBubble (CONTROL_FIELD_RESULT, mDataCodeLength, 0, inSampleNumber + samplesPerBit / 2) ;
      mFieldBitIndex = 0 ;
      if (mDataCodeLength > 8) {
        mDataCodeLength = 8 ;
      }
      mCRC15 = mCRC15Accumulator ;
      mFrameFieldEngineState = ((mDataCodeLength == 0) || (mFrameType == remoteFrame))
        ? CRC15
        : DATA
      ;
    }
  }
}

//----------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::handle_DATA_state (const bool inBitValue,
                                             const U64 inSampleNumber) {
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
  enterBitInCRC15 (inBitValue) ;
  addMark (inSampleNumber, AnalyzerResults::Dot);
  mData [mFieldBitIndex / 8] <<= 1 ;
  mData [mFieldBitIndex / 8] |= inBitValue ;
  mFieldBitIndex ++ ;
  if ((mFieldBitIndex % 8) == 0) {
    const U32 dataIndex = (mFieldBitIndex - 1) / 8 ;
    addBubble (DATA_FIELD_RESULT, mData [dataIndex], dataIndex, inSampleNumber + samplesPerBit / 2) ;
  }
  if (mFieldBitIndex == (8 * mDataCodeLength)) {
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = CRC15 ;
    mCRC15 = mCRC15Accumulator ;
  }
}

//----------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::handle_CRC15_state (const bool inBitValue,
                                              const U64 inSampleNumber) {
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
  enterBitInCRC15 (inBitValue) ;
  addMark (inSampleNumber, AnalyzerResults::Dot);
  mFieldBitIndex ++ ;
  if (mFieldBitIndex == 15) {
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = CRC_DEL ;
    addBubble (CRC_FIELD_RESULT, mCRC15, mCRC15Accumulator, inSampleNumber + samplesPerBit / 2) ;
    if (mCRC15Accumulator != 0) {
      mFrameFieldEngineState = DECODER_ERROR ;
    }
  }
}

//----------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::handle_CRCDEL_state (const bool inBitValue,
                                               const U64 inSampleNumber) {
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
  mUnstuffingActive = false ;
  if (inBitValue) {
    addMark (inSampleNumber, AnalyzerResults::One) ;
  }else{
    enterInErrorMode (inSampleNumber) ;
  }
  mStartOfFieldSampleNumber = inSampleNumber + samplesPerBit / 2 ;
  mFrameFieldEngineState = ACK ;
}

//----------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::handle_ACK_state (const bool inBitValue,
                                            const U64 inSampleNumber) {
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
  U8 u8Acked = 0;
  mFieldBitIndex ++ ;
  if (mFieldBitIndex == 1) { // ACK SLOT
    addMark (inSampleNumber, inBitValue ? AnalyzerResults::ErrorSquare : AnalyzerResults::DownArrow);
    u8Acked = inBitValue;
  }else{ // ACK DELIMITER
    addBubble (ACK_FIELD_RESULT, u8Acked, 0, inSampleNumber + samplesPerBit / 2) ;
    mFrameFieldEngineState = END_OF_FRAME ;
    if (inBitValue) {
      addMark (inSampleNumber, AnalyzerResults::One) ;
    }else{
      addMark (inSampleNumber, AnalyzerResults::ErrorDot) ;
      enterInErrorMode (inSampleNumber) ;
    }
    mFieldBitIndex = 0 ;
  }
}

//----------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::handle_ENDOFFRAME_state (const bool inBitValue,
                                                   const U64 inSampleNumber) {
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
  if (inBitValue) {
    addMark (inSampleNumber, AnalyzerResults::One) ;
  }else{
    enterInErrorMode (inSampleNumber) ;
  }
  mFieldBitIndex ++ ;
  if (mFieldBitIndex == 7) {
    addBubble (EOF_FIELD_RESULT, 0, 0, inSampleNumber + samplesPerBit / 2) ;
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = INTERMISSION ;
  }
}

//----------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::handle_INTERMISSION_state (const bool inBitValue,
                                                     const U64 inSampleNumber) {
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
  if (inBitValue) {
    addMark (inSampleNumber, AnalyzerResults::One) ;
  }else{
    enterInErrorMode (inSampleNumber) ;
  }
  mFieldBitIndex ++ ;
  if (mFieldBitIndex == 3) {
    const U64 frameSampleCount = inSampleNumber - mStartOfFrameSampleNumber ;
    addBubble (INTERMISSION_FIELD_RESULT,
               frameSampleCount,
               mStuffBitCount,
               inSampleNumber + samplesPerBit / 2) ;
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = IDLE ;
  }
}

//----------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::handle_DECODER_ERROR_state (const bool inBitValue,
                                                      const U64 inSampleNumber) {
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
  mUnstuffingActive = false ;
  addMark (inSampleNumber, AnalyzerResults::ErrorDot);
  if (mPreviousBit != inBitValue) {
    mConsecutiveBitCountOfSamePolarity = 1 ;
    mPreviousBit = inBitValue ;
  }else if (inBitValue) {
    mConsecutiveBitCountOfSamePolarity += 1 ;
    if (mConsecutiveBitCountOfSamePolarity == 11) {
      addBubble (CAN_ERROR_RESULT, 0, 0, inSampleNumber + samplesPerBit / 2) ;
      mFrameFieldEngineState = IDLE ;
    }
  }
}

//----------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::enterBitInCRC15 (const bool inBitValue) {
  const bool bit14 = (mCRC15Accumulator & (1 << 14)) != 0 ;
  const bool crc_nxt = inBitValue ^ bit14 ;
  mCRC15Accumulator <<= 1 ;
  mCRC15Accumulator &= 0x7FFF ;
  if (crc_nxt) {
    mCRC15Accumulator ^= 0x4599 ;
  }
}

//----------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::addMark (const U64 inSampleNumber,
                                   const AnalyzerResults::MarkerType inMarker) {
  mResults->AddMarker (inSampleNumber, inMarker, mSettings->mInputChannel);
}

//----------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::addBubble (const U8 inBubbleType,
                                     const U64 inData1,
                                     const U64 inData2,
                                     const U64 inEndSampleNumber) {
  Frame frame ;
  frame.mType = inBubbleType ;
  frame.mFlags = 0 ;
  frame.mStartingSampleInclusive = mStartOfFieldSampleNumber ;
  frame.mEndingSampleInclusive = inEndSampleNumber ;
  frame.mData1 = inData1 ;
  frame.mData2 = inData2 ;
  mResults->AddFrame (frame) ;

  FrameV2 frameV2 ;
  switch (inBubbleType) {
  case STANDARD_IDENTIFIER_FIELD_RESULT :
    { const U8 idf [2] = { U8 (inData1 >> 8), U8 (inData1) } ;
      frameV2.AddByteArray ("Value", idf, 2) ;
      mResults->AddFrameV2 (frameV2, "Std Idf", mStartOfFieldSampleNumber, inEndSampleNumber) ;
    }
    break ;
  case EXTENDED_IDENTIFIER_FIELD_RESULT :
    { const U8 idf [4] = {
        U8 (inData1 >> 24), U8 (inData1 >> 16), U8 (inData1 >> 8), U8 (inData1)
      } ;
      frameV2.AddByteArray ("Value", idf, 4) ;
      mResults->AddFrameV2 (frameV2, "Ext Idf", mStartOfFieldSampleNumber, inEndSampleNumber) ;
    }
    break ;
  case CONTROL_FIELD_RESULT :
    frameV2.AddByte ("Value", inData1) ;
    mResults->AddFrameV2 (frameV2, "Ctrl", mStartOfFieldSampleNumber, inEndSampleNumber) ;
    break ;
  case DATA_FIELD_RESULT :
    { frameV2.AddByte ("Value", inData1) ;
      std::stringstream str ;
      str << "D" << inData2 ;
      mResults->AddFrameV2 (frameV2, str.str ().c_str (), mStartOfFieldSampleNumber, inEndSampleNumber) ;
    }
    break ;
  case CRC_FIELD_RESULT :
    { const U8 crc [2] = { U8 (inData1 >> 8), U8 (inData1) } ;
      frameV2.AddByteArray ("Value", crc, 2) ;
      mResults->AddFrameV2 (frameV2, "CRC", mStartOfFieldSampleNumber, inEndSampleNumber) ;
    }
    break ;
  case ACK_FIELD_RESULT :
	frameV2.AddByte("Value", inData1);
    mResults->AddFrameV2 (frameV2, "ACK", mStartOfFieldSampleNumber, inEndSampleNumber) ;
    break ;
  case EOF_FIELD_RESULT :
    mResults->AddFrameV2 (frameV2, "EOF", mStartOfFieldSampleNumber, inEndSampleNumber) ;
    break ;
  case INTERMISSION_FIELD_RESULT :
    { const U64 frameSampleCount = inData1 ;
      const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
      const U64 length = (frameSampleCount + samplesPerBit / 2) / samplesPerBit ;
      const U64 stuffBitCount = inData2 ;
      const U64 durationMicroSeconds = frameSampleCount * 1000000 / mSampleRateHz ;
      std::stringstream str ;
      str << length << " bits, "
          << durationMicroSeconds << "µs, "
          << stuffBitCount << " stuff bit" << ((inData2 > 1) ? "s" : "") ;
      frameV2.AddString ("Value", str.str ().c_str ()) ;
      mResults->AddFrameV2 (frameV2, "IFS", mStartOfFieldSampleNumber, inEndSampleNumber) ;
    }
    break ;
  case CAN_ERROR_RESULT :
    mResults->AddFrameV2 (frameV2, "Error", mStartOfFieldSampleNumber, inEndSampleNumber) ;
    break ;
  default:
    mResults->AddFrameV2 (frameV2, "?", mStartOfFieldSampleNumber, inEndSampleNumber) ;
    break ;
  }

  mResults->CommitResults () ;
  ReportProgress (frame.mEndingSampleInclusive) ;
//--- Prepare for next bubble
  mStartOfFieldSampleNumber = inEndSampleNumber ;
}

//----------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::enterInErrorMode (const U64 inSampleNumber) {
  mStartOfFieldSampleNumber = inSampleNumber ;
  mFrameFieldEngineState = DECODER_ERROR ;
  mUnstuffingActive = false ;
}

//----------------------------------------------------------------------------------------
