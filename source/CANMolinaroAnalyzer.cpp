#include "CANMolinaroAnalyzer.h"
#include "CANMolinaroAnalyzerSettings.h"

#include <AnalyzerChannelData.h>

//--------------------------------------------------------------------------------------------------
//   CANMolinaroAnalyzer
//--------------------------------------------------------------------------------------------------

CANMolinaroAnalyzer::CANMolinaroAnalyzer () :
Analyzer2 (),
mSettings (new CANMolinaroAnalyzerSettings ()),
mSimulationInitilized (false) {
  SetAnalyzerSettings (mSettings.get()) ;
}

//--------------------------------------------------------------------------------------------------

CANMolinaroAnalyzer::~CANMolinaroAnalyzer () {
  KillThread () ;
}

//--------------------------------------------------------------------------------------------------

U32 CANMolinaroAnalyzer::bitRate (void) const {
  return mSettings->mBitRate ;
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::SetupResults () {
  mResults.reset (new CANMolinaroAnalyzerResults (this, mSettings.get())) ;
  SetAnalyzerResults (mResults.get()) ;
  mResults->AddChannelBubblesWillAppearOn (mSettings->mInputChannel) ;
}


//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::WorkerThread () {
  const bool inverted = mSettings->inverted () ;
  mSampleRateHz = GetSampleRate () ;
  mSerial = GetAnalyzerChannelData (mSettings->mInputChannel) ;
//--- Sample settings
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
//--- Synchronize to recessive level
  if (mSerial->GetBitState() == (inverted ? BIT_HIGH : BIT_LOW)) {
    mSerial->AdvanceToNextEdge () ;
  }
  while (1) {
  //--- Synchronize on falling edge: this SOF bit
    mFrameFieldEngineState = IDLE ;
    mUnstuffingActive = false ;
  //--- Loop util the end of the frame (11 consecutive high bits)
    bool currentBitState = true ;
    do{
      mSerial->AdvanceToNextEdge () ;
      currentBitState ^= true ;
      const U64 start = mSerial->GetSampleNumber () ;
      const U64 nextEdge = mSerial->GetSampleOfNextEdge () ;
      const U64 bitCount = (nextEdge - start + samplesPerBit / 2) / samplesPerBit ;
      for (U64 i=0 ; i<bitCount ; i++) {
        enterBit (currentBitState, start + samplesPerBit / 2 + i * samplesPerBit) ;
      }
    }while (mFrameFieldEngineState != IDLE) ;
  //---
    mResults->CommitResults () ;
  }
}

//--------------------------------------------------------------------------------------------------

bool CANMolinaroAnalyzer::NeedsRerun () {
  return false;
}

//--------------------------------------------------------------------------------------------------

U32 CANMolinaroAnalyzer::GenerateSimulationData (U64 minimum_sample_index,
                                                 U32 device_sample_rate,
                                                 SimulationChannelDescriptor** simulation_channels ) {
  if( mSimulationInitilized == false ) {
    mSimulationDataGenerator.Initialize( GetSimulationSampleRate(), mSettings.get() );
    mSimulationInitilized = true;
  }
  return mSimulationDataGenerator.GenerateSimulationData (minimum_sample_index,
                                                          device_sample_rate,
                                                          simulation_channels) ;
}

//--------------------------------------------------------------------------------------------------

U32 CANMolinaroAnalyzer::GetMinimumSampleRateHz () {
  return mSettings->mBitRate * 5 ;
}

//--------------------------------------------------------------------------------------------------

const char* CANMolinaroAnalyzer::GetAnalyzerName () const {
  return "CAN 2.0B (Molinaro)";
}

//--------------------------------------------------------------------------------------------------

const char* GetAnalyzerName (void) {
  return "CAN 2.0B (Molinaro)";
}

//--------------------------------------------------------------------------------------------------

Analyzer* CreateAnalyzer (void) {
  return new CANMolinaroAnalyzer () ;
}

//--------------------------------------------------------------------------------------------------

void DestroyAnalyzer (Analyzer* analyzer) {
  delete analyzer;
}

//--------------------------------------------------------------------------------------------------
//  CAN FRAME DECODER
//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::enterBit (const bool inBit, const U64 inSampleNumber) {
  if (!mUnstuffingActive) {
    decodeFrameBit (inBit, inSampleNumber) ;
  }else if ((mConsecutiveBitCountOfSamePolarity == 5) && (inBit != mPreviousBit)) {
   // Stuff bit - discarded
    addMark (inSampleNumber, AnalyzerResults::X);
    mConsecutiveBitCountOfSamePolarity = 1 ;
    mPreviousBit = inBit ;
    mStuffBitCount += 1 ;
  }else if ((mConsecutiveBitCountOfSamePolarity == 5) && (mPreviousBit == inBit)) { // Stuff Error
    addMark (inSampleNumber, AnalyzerResults::ErrorX);
    const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
    enterInErrorMode (inSampleNumber + samplesPerBit / 2) ;
    mConsecutiveBitCountOfSamePolarity += 1 ;
  }else if (mPreviousBit == inBit) {
    mConsecutiveBitCountOfSamePolarity += 1 ;
    decodeFrameBit (inBit, inSampleNumber) ;
  }else{
    mConsecutiveBitCountOfSamePolarity = 1 ;
    mPreviousBit = inBit ;
    decodeFrameBit (inBit, inSampleNumber) ;
  }
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::decodeFrameBit (const bool inBit, const U64 inSampleNumber) {
  switch (mFrameFieldEngineState) {
  case IDLE :
    handle_IDLE_state (inBit, inSampleNumber) ;
    break ;
  case IDENTIFIER :
    handle_IDENTIFIER_state (inBit, inSampleNumber) ;
    break ;
  case EXTENDED_IDF :
    handle_EXTENDED_IDF_state (inBit, inSampleNumber) ;
    break ;
  case CONTROL :
    handle_CONTROL_state (inBit, inSampleNumber) ;
    break ;
  case DATA :
    handle_DATA_state (inBit, inSampleNumber) ;
    break ;
  case CRC15 :
    handle_CRC15_state (inBit, inSampleNumber) ;
    break ;
  case CRC_DEL :
    handle_CRCDEL_state (inBit, inSampleNumber) ;
    break ;
  case ACK :
    handle_ACK_state (inBit, inSampleNumber) ;
    break ;
  case END_OF_FRAME :
    handle_ENDOFFRAME_state (inBit, inSampleNumber) ;
    break ;
  case INTERMISSION :
    handle_INTERMISSION_state (inBit, inSampleNumber) ;
    break ;
  case DECODER_ERROR :
    handle_DECODER_ERROR_state (inBit, inSampleNumber) ;
    break ;
  }
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::handle_IDLE_state (const bool inBit, const U64 inSampleNumber) {
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
  if (!inBit) {
    mUnstuffingActive = true ;
    mCRC15Accumulator = 0 ;
    mConsecutiveBitCountOfSamePolarity = 1 ;
    mPreviousBit = false ;
    enterBitInCRC15 (inBit) ;
    addMark (inSampleNumber, AnalyzerResults::Start) ;
    mFieldBitIndex = 0 ;
    mIdentifier = 0 ;
    mFrameFieldEngineState = IDENTIFIER ;
    mStartOfFieldSampleNumber = inSampleNumber + samplesPerBit / 2 ;
    mStartOfFrameSampleNumber = inSampleNumber ;
    mStuffBitCount = 0 ;
  }
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::handle_IDENTIFIER_state (const bool inBit, const U64 inSampleNumber) {
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
  enterBitInCRC15 (inBit) ;
  mFieldBitIndex ++ ;
  if (mFieldBitIndex <= 11) { // Standard identifier
    addMark (inSampleNumber, AnalyzerResults::Dot);
    mIdentifier <<= 1 ;
    mIdentifier |= inBit ;
  }else if (mFieldBitIndex == 12) { // RTR bit
    addMark (inSampleNumber, inBit ? AnalyzerResults::UpArrow : AnalyzerResults::DownArrow) ;
    mFrameType = inBit ? remoteFrame : dataFrame  ;
  }else{ // IDE
    addMark (inSampleNumber, AnalyzerResults::Dot);
    if (inBit) {
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

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::handle_EXTENDED_IDF_state (const bool inBit, const U64 inSampleNumber) {
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
  enterBitInCRC15 (inBit) ;
  mFieldBitIndex ++ ;
  if (mFieldBitIndex <= 18) { // Extended identifier
    addMark (inSampleNumber, AnalyzerResults::Dot);
    mIdentifier <<= 1 ;
    mIdentifier |= inBit ;
  }else if (mFieldBitIndex == 19) { // RTR bit
    addMark (inSampleNumber, inBit ? AnalyzerResults::UpArrow : AnalyzerResults::DownArrow) ;
    mFrameType = inBit ? remoteFrame : dataFrame  ;
  }else{ // R1: should be dominant
    addMark (inSampleNumber, inBit ? AnalyzerResults::ErrorX : AnalyzerResults::Zero) ;
    addBubble (EXTENDED_IDENTIFIER_FIELD_RESULT,
               mIdentifier,
               mFrameType == dataFrame, // 0 -> remote, 1 -> data
               inSampleNumber - samplesPerBit / 2) ;
    if (inBit) {
      enterInErrorMode (inSampleNumber + samplesPerBit / 2) ;
    }else{
      mFrameFieldEngineState = CONTROL ;
      mFieldBitIndex = 1 ;
      mDataCodeLength = 0 ;
    }
  }
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::handle_CONTROL_state (const bool inBit, const U64 inSampleNumber) {
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
  enterBitInCRC15 (inBit) ;
  mFieldBitIndex ++ ;
  if (mFieldBitIndex == 2) { // R0
    addMark (inSampleNumber, inBit ? AnalyzerResults::ErrorX : AnalyzerResults::Zero) ;
    if (inBit) {
      enterInErrorMode (inSampleNumber + samplesPerBit / 2) ;
    }
  }else{
    addMark (inSampleNumber, AnalyzerResults::Dot);
    mDataCodeLength <<= 1 ;
    mDataCodeLength |= inBit ;
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

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::handle_DATA_state (const bool inBit, const U64 inSampleNumber) {
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
  enterBitInCRC15 (inBit) ;
  addMark (inSampleNumber, AnalyzerResults::Dot);
  mData [mFieldBitIndex / 8] <<= 1 ;
  mData [mFieldBitIndex / 8] |= inBit ;
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

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::handle_CRC15_state (const bool inBit, const U64 inSampleNumber) {
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
  enterBitInCRC15 (inBit) ;
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

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::handle_CRCDEL_state (const bool inBit, const U64 inSampleNumber) {
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
  mUnstuffingActive = false ;
  if (inBit) {
    addMark (inSampleNumber, AnalyzerResults::One) ;
  }else{
    enterInErrorMode (inSampleNumber) ;
  }
  mStartOfFieldSampleNumber = inSampleNumber + samplesPerBit / 2 ;
  mFrameFieldEngineState = ACK ;
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::handle_ACK_state (const bool inBit, const U64 inSampleNumber) {
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
  mFieldBitIndex ++ ;
  if (mFieldBitIndex == 1) { // ACK SLOT
    addMark (inSampleNumber, inBit ? AnalyzerResults::ErrorSquare : AnalyzerResults::Square);
  }else{ // ACK DELIMITER
    addBubble (ACK_FIELD_RESULT, 0, 0, inSampleNumber + samplesPerBit / 2) ;
    if (inBit) {
      addMark (inSampleNumber, AnalyzerResults::One) ;
    }else{
      enterInErrorMode (inSampleNumber) ;
    }
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = END_OF_FRAME ;
  }
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::handle_ENDOFFRAME_state (const bool inBit, const U64 inSampleNumber) {
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
  if (inBit) {
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

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::handle_INTERMISSION_state (const bool inBit, const U64 inSampleNumber) {
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
  if (inBit) {
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

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::handle_DECODER_ERROR_state (const bool inBit, const U64 inSampleNumber) {
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
  mUnstuffingActive = false ;
  addMark (inSampleNumber, AnalyzerResults::ErrorDot);
  if (mPreviousBit != inBit) {
    mConsecutiveBitCountOfSamePolarity = 1 ;
    mPreviousBit = inBit ;
  }else if (inBit) {
    mConsecutiveBitCountOfSamePolarity += 1 ;
    if (mConsecutiveBitCountOfSamePolarity == 11) {
      addBubble (CAN_ERROR_RESULT, 0, 0, inSampleNumber + samplesPerBit / 2) ;
      mFrameFieldEngineState = IDLE ;
    }
  }
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::enterBitInCRC15 (const bool inBit) {
  const bool bit14 = (mCRC15Accumulator & (1 << 14)) != 0 ;
  const bool crc_nxt = inBit ^ bit14 ;
  mCRC15Accumulator <<= 1 ;
  mCRC15Accumulator &= 0x7FFF ;
  if (crc_nxt) {
    mCRC15Accumulator ^= 0x4599 ;
  }
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::addMark (const U64 inSampleNumber,
                                   const AnalyzerResults::MarkerType inMarker) {
  mResults->AddMarker (inSampleNumber, inMarker, mSettings->mInputChannel);
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::addBubble (const U8 inBubbleType,
                                     const U64 inData1,
                                     const U64 inData2,
                                     const U64 inEndSampleNumber) {
  Frame frame ;
  frame.mType = inBubbleType ;
  frame.mFlags = 0 ;
  frame.mData1 = inData1 ;
  frame.mData2 = inData2 ;
  frame.mStartingSampleInclusive = mStartOfFieldSampleNumber ;
  frame.mEndingSampleInclusive = inEndSampleNumber ;
  mResults->AddFrame (frame) ;
  mResults->CommitResults () ;
  ReportProgress (frame.mEndingSampleInclusive) ;
//--- Prepare for next bubble
  mStartOfFieldSampleNumber = inEndSampleNumber ;
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::enterInErrorMode (const U64 inSampleNumber) {
  mStartOfFieldSampleNumber = inSampleNumber ;
  mFrameFieldEngineState = DECODER_ERROR ;
  mUnstuffingActive = false ;
}

//--------------------------------------------------------------------------------------------------

