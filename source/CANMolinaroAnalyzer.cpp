#include "CANMolinaroAnalyzer.h"
#include "CANMolinaroAnalyzerSettings.h"
#include <AnalyzerChannelData.h>

//--------------------------------------------------------------------------------------------------
//   CANMolinaroAnalyzer
//--------------------------------------------------------------------------------------------------

CANMolinaroAnalyzer::CANMolinaroAnalyzer () :
Analyzer2(),
mSettings (new CANMolinaroAnalyzerSettings ()),
mSimulationInitilized (false) {
  SetAnalyzerSettings( mSettings.get() );
}

//--------------------------------------------------------------------------------------------------

CANMolinaroAnalyzer::~CANMolinaroAnalyzer () {
  KillThread();
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzer::SetupResults () {
  mResults.reset( new CANMolinaroAnalyzerResults( this, mSettings.get() ) );
  SetAnalyzerResults( mResults.get() );
  mResults->AddChannelBubblesWillAppearOn( mSettings->mInputChannel );
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
    mFrameFieldEngineState = FrameFieldEngineState::IDLE ;
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
    }while (mFrameFieldEngineState != FrameFieldEngineState::IDLE) ;
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
  return mSimulationDataGenerator.GenerateSimulationData( minimum_sample_index, device_sample_rate, simulation_channels );
}

//--------------------------------------------------------------------------------------------------

U32 CANMolinaroAnalyzer::GetMinimumSampleRateHz() {
  return mSettings->mBitRate * 5;
}

//--------------------------------------------------------------------------------------------------

const char* CANMolinaroAnalyzer::GetAnalyzerName () const {
  return "CAN 2.0B (Molinaro)";
}

//--------------------------------------------------------------------------------------------------

const char* GetAnalyzerName () {
  return "CAN 2.0B (Molinaro)";
}

//--------------------------------------------------------------------------------------------------

Analyzer* CreateAnalyzer () {
  return new CANMolinaroAnalyzer();
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
    mResults->AddMarker (inSampleNumber, AnalyzerResults::X, mSettings->mInputChannel);
    mConsecutiveBitCountOfSamePolarity = 1 ;
    mPreviousBit = inBit ;
  }else if ((mConsecutiveBitCountOfSamePolarity == 5) && (mPreviousBit == inBit)) { // Stuff Error
    mResults->AddMarker (inSampleNumber, AnalyzerResults::ErrorX, mSettings->mInputChannel);
    mFrameFieldEngineState = FrameFieldEngineState::STUFF_ERROR ;
    mConsecutiveBitCountOfSamePolarity += 1 ;
    mStartOfFieldSampleNumber = inSampleNumber + mSampleRateHz / mSettings->mBitRate / 2 ;
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
  case FrameFieldEngineState::IDLE :
    if (!inBit) {
      mUnstuffingActive = true ;
      mCRCAccumulator = 0 ;
      mConsecutiveBitCountOfSamePolarity = 1 ;
      mPreviousBit = false ;
      enterBitInCRC (inBit) ;
      mResults->AddMarker (inSampleNumber, AnalyzerResults::Start, mSettings->mInputChannel);
      mFieldBitIndex = 0 ;
      mIdentifier = 0 ;
      mFrameFieldEngineState = FrameFieldEngineState::IDENTIFIER ;
      mStartOfFieldSampleNumber = inSampleNumber + samplesPerBit / 2 ;
    }
    break ;
  case FrameFieldEngineState::IDENTIFIER :
    enterBitInCRC (inBit) ;
    mFieldBitIndex ++ ;
    if (mFieldBitIndex <= 11) { // Standard identifier
      mResults->AddMarker (inSampleNumber, AnalyzerResults::Dot, mSettings->mInputChannel);
      mIdentifier <<= 1 ;
      mIdentifier |= inBit ;
    }else if (mFieldBitIndex == 12) { // RTR bit
      mResults->AddMarker (inSampleNumber,
                           inBit ? AnalyzerResults::UpArrow : AnalyzerResults::DownArrow,
                           mSettings->mInputChannel) ;
      mFrameType = inBit ? FrameType::remote : FrameType::data  ;
    }else{ // IDE
      mResults->AddMarker (inSampleNumber, AnalyzerResults::Dot, mSettings->mInputChannel);
      mFieldBitIndex = 0 ;
      if (inBit) {
        mFrameFieldEngineState = FrameFieldEngineState::EXTENDED_IDF ;
      }else{
        Frame frame ;
        frame.mType = STANDARD_IDENTIFIER_FIELD_RESULT ;
        frame.mData1 = mIdentifier ;
        frame.mData2 = mFrameType == FrameType::data ; // 0 -> remote, 1 -> data
        frame.mFlags = 0 ;
        frame.mStartingSampleInclusive = mStartOfFieldSampleNumber ;
        frame.mEndingSampleInclusive = inSampleNumber + samplesPerBit / 2 ;
        mResults->AddFrame (frame) ;
        ReportProgress (frame.mEndingSampleInclusive) ;
        mDataLength = 0 ;
        mFrameFieldEngineState = FrameFieldEngineState::CONTROL ;
        mStartOfFieldSampleNumber = inSampleNumber + samplesPerBit / 2 ;
      }
    }
    break ;
  case FrameFieldEngineState::EXTENDED_IDF :
    enterBitInCRC (inBit) ;
    mFieldBitIndex ++ ;
    if (mFieldBitIndex <= 18) { // Extended identifier
      mResults->AddMarker (inSampleNumber, AnalyzerResults::Dot, mSettings->mInputChannel);
      mIdentifier <<= 1 ;
      mIdentifier |= inBit ;
    }else if (mFieldBitIndex == 19) { // RTR bit
      mResults->AddMarker (inSampleNumber,
                           inBit ? AnalyzerResults::UpArrow : AnalyzerResults::DownArrow,
                           mSettings->mInputChannel) ;
      mFrameType = inBit ? FrameType::remote : FrameType::data  ;
    }else{ // R1: should be dominant
      mResults->AddMarker (inSampleNumber,
                           inBit ? AnalyzerResults::ErrorX : AnalyzerResults::Zero,
                           mSettings->mInputChannel);
      Frame frame ;
      frame.mType = EXTENDED_IDENTIFIER_FIELD_RESULT ;
      frame.mData1 = mIdentifier ;
      frame.mData2 = mFrameType == FrameType::data  ; // 0 -> remote, 1 -> data
      frame.mFlags = 0 ;
      frame.mStartingSampleInclusive = mStartOfFieldSampleNumber ;
      frame.mEndingSampleInclusive = inSampleNumber + samplesPerBit / 2 ;
      mResults->AddFrame (frame) ;
      ReportProgress (frame.mEndingSampleInclusive) ;
      mStartOfFieldSampleNumber = inSampleNumber + samplesPerBit / 2 ;
      mDataLength = 0 ;
      mFieldBitIndex = 0 ;
      mFrameFieldEngineState = FrameFieldEngineState::CONTROL ;
    }
    break ;
  case FrameFieldEngineState::CONTROL :
    enterBitInCRC (inBit) ;
    mFieldBitIndex ++ ;
    if (mFieldBitIndex == 1) { // R0
      mResults->AddMarker (inSampleNumber,
                           inBit ? AnalyzerResults::ErrorX : AnalyzerResults::Zero,
                           mSettings->mInputChannel);
    }else{
      mResults->AddMarker (inSampleNumber, AnalyzerResults::Dot, mSettings->mInputChannel);
      mDataLength <<= 1 ;
      mDataLength |= inBit ;
      if (mFieldBitIndex == 5) {
        Frame frame ;
        frame.mType = CONTROL_FIELD_RESULT ;
        frame.mData1 = mDataLength ;
        frame.mFlags = 0 ;
        frame.mStartingSampleInclusive = mStartOfFieldSampleNumber ;
        frame.mEndingSampleInclusive = inSampleNumber + samplesPerBit / 2 ;
        mResults->AddFrame (frame) ;
        ReportProgress (frame.mEndingSampleInclusive) ;
        mStartOfFieldSampleNumber = inSampleNumber + samplesPerBit / 2 ;
        mFieldBitIndex = 0 ;
        if (mDataLength > 8) {
          mDataLength = 8 ;
        }
        mCRC = mCRCAccumulator ;
        mFrameFieldEngineState = ((mDataLength == 0) || (mFrameType == FrameType::remote))
          ? FrameFieldEngineState::CRC
          : FrameFieldEngineState::DATA
        ;
      }
    }
    break ;
  case FrameFieldEngineState::DATA :
    enterBitInCRC (inBit) ;
    mResults->AddMarker (inSampleNumber, AnalyzerResults::Dot, mSettings->mInputChannel);
    mData [mFieldBitIndex / 8] <<= 1 ;
    mData [mFieldBitIndex / 8] |= inBit ;
    mFieldBitIndex ++ ;
    if ((mFieldBitIndex % 8) == 0) {
      const U32 dataIndex = (mFieldBitIndex - 1) / 8 ;
      Frame frame ;
      frame.mType = DATA_FIELD_RESULT ;
      frame.mData1 = mData [dataIndex] ;
      frame.mData2 = dataIndex ;
      frame.mFlags = 0 ;
      frame.mStartingSampleInclusive = mStartOfFieldSampleNumber ;
      frame.mEndingSampleInclusive = inSampleNumber + samplesPerBit / 2 ;
      mResults->AddFrame (frame) ;
      mStartOfFieldSampleNumber = inSampleNumber + samplesPerBit / 2 ;
      ReportProgress (frame.mEndingSampleInclusive) ;
    }
    if (mFieldBitIndex == (8 * mDataLength)) {
      mFieldBitIndex = 0 ;
      mFrameFieldEngineState = FrameFieldEngineState::CRC ;
      mCRC = mCRCAccumulator ;
    }
    break ;
  case FrameFieldEngineState::CRC :
    enterBitInCRC (inBit) ;
    mResults->AddMarker (inSampleNumber, AnalyzerResults::Dot, mSettings->mInputChannel);
    mFieldBitIndex ++ ;
    if (mFieldBitIndex == 15) {
      mFieldBitIndex = 0 ;
      mFrameFieldEngineState = FrameFieldEngineState::CRCDEL ;
      Frame frame ;
      frame.mType = CRC_FIELD_RESULT ;
      frame.mData1 = mCRC ;
      frame.mData2 = mCRCAccumulator ;
      frame.mFlags = 0 ;
      frame.mStartingSampleInclusive = mStartOfFieldSampleNumber ;
      frame.mEndingSampleInclusive = inSampleNumber + samplesPerBit / 2 ;
      mResults->AddFrame (frame) ;
      ReportProgress (frame.mEndingSampleInclusive) ;
    }
    break ;
  case FrameFieldEngineState::CRCDEL :
    mUnstuffingActive = false ;
    mResults->AddMarker (inSampleNumber,
                         inBit ? AnalyzerResults::One : AnalyzerResults::ErrorX,
                         mSettings->mInputChannel);
    mStartOfFieldSampleNumber = inSampleNumber + samplesPerBit / 2 ;
    mFrameFieldEngineState = FrameFieldEngineState::ACK ;
    break ;
  case FrameFieldEngineState::ACK :
    mFieldBitIndex ++ ;
    if (mFieldBitIndex == 1) { // ACK SLOT
      mResults->AddMarker (inSampleNumber,
                           inBit ? AnalyzerResults::ErrorX : AnalyzerResults::Dot,
                           mSettings->mInputChannel);
    }else{ // ACK DELIMITER
      mResults->AddMarker (inSampleNumber,
                           inBit ? AnalyzerResults::One : AnalyzerResults::ErrorX,
                           mSettings->mInputChannel);
      Frame frame ;
      frame.mType = ACK_FIELD_RESULT ;
      frame.mFlags = 0 ;
      frame.mStartingSampleInclusive = mStartOfFieldSampleNumber ;
      frame.mEndingSampleInclusive = inSampleNumber + samplesPerBit / 2 ;
      mResults->AddFrame (frame) ;
      mStartOfFieldSampleNumber = inSampleNumber + samplesPerBit / 2 ;
      ReportProgress (frame.mEndingSampleInclusive) ;
      mFieldBitIndex = 0 ;
      mFrameFieldEngineState = FrameFieldEngineState::ENDOFFRAME ;
    }
    break ;
  case FrameFieldEngineState::ENDOFFRAME :
    mConsecutiveBitCountOfSamePolarity = 0 ; // For disabling bit stuffing
    mResults->AddMarker (inSampleNumber,
                         inBit ? AnalyzerResults::One : AnalyzerResults::ErrorX,
                         mSettings->mInputChannel);
    mFieldBitIndex ++ ;
    if (mFieldBitIndex == 7) {
      Frame frame ;
      frame.mType = EOF_FIELD_RESULT ;
      frame.mFlags = 0 ;
      frame.mStartingSampleInclusive = mStartOfFieldSampleNumber ;
      frame.mEndingSampleInclusive = inSampleNumber + samplesPerBit / 2 ;
      mResults->AddFrame (frame) ;
      mStartOfFieldSampleNumber = inSampleNumber + samplesPerBit / 2 ;
      ReportProgress (frame.mEndingSampleInclusive) ;
      mFieldBitIndex = 0 ;
      mFrameFieldEngineState = FrameFieldEngineState::INTERMISSION ;
    }
    break ;
  case FrameFieldEngineState::INTERMISSION :
    mResults->AddMarker (inSampleNumber,
                         inBit ? AnalyzerResults::One : AnalyzerResults::ErrorX,
                         mSettings->mInputChannel);
    mFieldBitIndex ++ ;
    if (mFieldBitIndex == 3) {
      Frame frame ;
      frame.mType = INTERMISSION_FIELD_RESULT ;
      frame.mFlags = 0 ;
      frame.mStartingSampleInclusive = mStartOfFieldSampleNumber ;
      frame.mEndingSampleInclusive = inSampleNumber + samplesPerBit / 2 ;
      mResults->AddFrame (frame) ;
      mStartOfFieldSampleNumber = inSampleNumber + samplesPerBit / 2 ;
      ReportProgress (frame.mEndingSampleInclusive) ;
      mFieldBitIndex = 0 ;
      mFrameFieldEngineState = FrameFieldEngineState::IDLE ;
    }
    break ;
  case FrameFieldEngineState::STUFF_ERROR :
    mUnstuffingActive = false ;
    mResults->AddMarker (inSampleNumber, AnalyzerResults::ErrorDot, mSettings->mInputChannel);
    if (mPreviousBit != inBit) {
      mConsecutiveBitCountOfSamePolarity = 1 ;
      mPreviousBit = inBit ;
    }else if (inBit) {
      mConsecutiveBitCountOfSamePolarity += 1 ;
      if (mConsecutiveBitCountOfSamePolarity == 11) {
        Frame frame ;
        frame.mType = CAN_ERROR_RESULT ;
        frame.mFlags = 0 ;
        frame.mStartingSampleInclusive = mStartOfFieldSampleNumber ;
        frame.mEndingSampleInclusive = inSampleNumber + samplesPerBit / 2 ;
        mResults->AddFrame (frame) ;
        ReportProgress (frame.mEndingSampleInclusive) ;
        mFrameFieldEngineState = FrameFieldEngineState::IDLE ;
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

