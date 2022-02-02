#include "CANMolinaroAnalyzer.h"
#include "CANMolinaroAnalyzerSettings.h"

#include <AnalyzerChannelData.h>

//--------------------------------------------------------------------------------------------------
//   CANFDMolinaroAnalyzer
//--------------------------------------------------------------------------------------------------

CANMolinaroAnalyzer::CANMolinaroAnalyzer () :
Analyzer2(),
mSettings (new CANMolinaroAnalyzerSettings ()),
mSimulationInitilized (false) {
  SetAnalyzerSettings( mSettings.get() );
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
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
  switch (mFrameFieldEngineState) {
  case IDLE :
    if (!inBit) {
      mUnstuffingActive = true ;
      mCRCAccumulator = 0 ;
      mConsecutiveBitCountOfSamePolarity = 1 ;
      mPreviousBit = false ;
      enterBitInCRC (inBit) ;
      addMark (inSampleNumber, AnalyzerResults::Start) ;
      mFieldBitIndex = 0 ;
      mIdentifier = 0 ;
      mFrameFieldEngineState = IDENTIFIER ;
      mStartOfFieldSampleNumber = inSampleNumber + samplesPerBit / 2 ;
      mStartOfFrameSampleNumber = inSampleNumber ;
      mStuffBitCount = 0 ;
    }
    break ;
  case IDENTIFIER :
    enterBitInCRC (inBit) ;
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
      mFieldBitIndex = 0 ;
      if (inBit) {
        mFrameFieldEngineState = EXTENDED_IDF ;
      }else{
        addBubble (STANDARD_IDENTIFIER_FIELD_RESULT,
                   mIdentifier,
                   mFrameType == dataFrame, // 0 -> remote, 1 -> data
                   inSampleNumber + samplesPerBit / 2) ;
        mDataLength = 0 ;
        mFrameFieldEngineState = CONTROL ;
      }
    }
    break ;
  case EXTENDED_IDF :
    enterBitInCRC (inBit) ;
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
                 inSampleNumber + samplesPerBit / 2) ;
      mDataLength = 0 ;
      mFieldBitIndex = 0 ;
      if (inBit) {
        enterInErrorMode (inSampleNumber + samplesPerBit / 2) ;
      }else{
        mFrameFieldEngineState = CONTROL ;
      }
    }
    break ;
  case CONTROL :
    enterBitInCRC (inBit) ;
    mFieldBitIndex ++ ;
    if (mFieldBitIndex == 1) { // R0
      addMark (inSampleNumber, inBit ? AnalyzerResults::ErrorX : AnalyzerResults::Zero) ;
      if (inBit) {
        enterInErrorMode (inSampleNumber + samplesPerBit / 2) ;
      }
    }else{
      addMark (inSampleNumber, AnalyzerResults::Dot);
      mDataLength <<= 1 ;
      mDataLength |= inBit ;
      if (mFieldBitIndex == 5) {
        addBubble (CONTROL_FIELD_RESULT, mDataLength, 0, inSampleNumber + samplesPerBit / 2) ;
        mFieldBitIndex = 0 ;
        if (mDataLength > 8) {
          mDataLength = 8 ;
        }
        mCRC = mCRCAccumulator ;
        mFrameFieldEngineState = ((mDataLength == 0) || (mFrameType == remoteFrame))
          ? CRC
          : DATA
        ;
      }
    }
    break ;
  case DATA :
    enterBitInCRC (inBit) ;
    addMark (inSampleNumber, AnalyzerResults::Dot);
    mData [mFieldBitIndex / 8] <<= 1 ;
    mData [mFieldBitIndex / 8] |= inBit ;
    mFieldBitIndex ++ ;
    if ((mFieldBitIndex % 8) == 0) {
      const U32 dataIndex = (mFieldBitIndex - 1) / 8 ;
      addBubble (DATA_FIELD_RESULT, mData [dataIndex], dataIndex, inSampleNumber + samplesPerBit / 2) ;
    }
    if (mFieldBitIndex == (8 * mDataLength)) {
      mFieldBitIndex = 0 ;
      mFrameFieldEngineState = CRC ;
      mCRC = mCRCAccumulator ;
    }
    break ;
  case CRC :
    enterBitInCRC (inBit) ;
    addMark (inSampleNumber, AnalyzerResults::Dot);
    mFieldBitIndex ++ ;
    if (mFieldBitIndex == 15) {
      mFieldBitIndex = 0 ;
      mFrameFieldEngineState = CRC_DEL ;
      addBubble (CRC_FIELD_RESULT, mCRC, mCRCAccumulator, inSampleNumber + samplesPerBit / 2) ;
      if (mCRCAccumulator != 0) {
        mFrameFieldEngineState = DECODER_ERROR ;
      }
    }
    break ;
  case CRC_DEL :
    mUnstuffingActive = false ;
    if (inBit) {
      addMark (inSampleNumber, AnalyzerResults::One) ;
    }else{
      enterInErrorMode (inSampleNumber) ;
    }
    mStartOfFieldSampleNumber = inSampleNumber + samplesPerBit / 2 ;
    mFrameFieldEngineState = ACK ;
    break ;
  case ACK :
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
    break ;
  case END_OF_FRAME :
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
    break ;
  case INTERMISSION :
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
    break ;
  case DECODER_ERROR :
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
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::enterBitInCRC (const bool inBit) {
  const bool bit14 = (mCRCAccumulator & (1 << 14)) != 0 ;
  const bool crc_nxt = inBit ^ bit14 ;
  mCRCAccumulator <<= 1 ;
  mCRCAccumulator &= 0x7FFF ;
  if (crc_nxt) {
    mCRCAccumulator ^= 0x4599 ;
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


