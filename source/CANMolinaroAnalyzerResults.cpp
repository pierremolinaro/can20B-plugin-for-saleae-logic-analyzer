#include "CANMolinaroAnalyzerResults.h"
#include <AnalyzerHelpers.h>
#include "CANMolinaroAnalyzer.h"
#include "CANMolinaroAnalyzerSettings.h"
#include <iostream>
#include <fstream>
#include <sstream>

//--------------------------------------------------------------------------------------------------

CANMolinaroAnalyzerResults::CANMolinaroAnalyzerResults (CANMolinaroAnalyzer* analyzer,
                                                        CANMolinaroAnalyzerSettings* settings) :
AnalyzerResults (),
mSettings (settings),
mAnalyzer (analyzer) {
}

//--------------------------------------------------------------------------------------------------

CANMolinaroAnalyzerResults::~CANMolinaroAnalyzerResults (void) {
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzerResults::GenerateText (const Frame & inFrame,
                                               const DisplayBase inDisplayBase,
                                               const bool inBubbleText,
                                               std::stringstream & ioText) {
  char numberString [128] = "" ;
  switch (inFrame.mType) {
  case STANDARD_IDENTIFIER_FIELD_RESULT :
    AnalyzerHelpers::GetNumberString (inFrame.mData1, inDisplayBase, 12, numberString, 128);
    ioText << ((inFrame.mData2 == 0) ? "Std Remote idf: " : "Std Data idf: ") ;
    ioText << numberString ;
    break ;
  case EXTENDED_IDENTIFIER_FIELD_RESULT :
    AnalyzerHelpers::GetNumberString (inFrame.mData1, inDisplayBase, 32, numberString, 128);
    ioText << ((inFrame.mData2 == 0) ? "Extended Remote idf: " : "Extended Data idf: ") ;
    ioText << numberString ;
    break ;
  case CONTROL_FIELD_RESULT :
    if (inBubbleText) {
      ioText << "Ctrl: " << inFrame.mData1 ;
    }
    break ;
  case DATA_FIELD_RESULT :
    AnalyzerHelpers::GetNumberString (inFrame.mData1, inDisplayBase, 8, numberString, 128);
    if (!inBubbleText) {
      ioText << "  " ;
    }
    ioText << "Data " << inFrame.mData2 << ": " << numberString ;
    break ;
  case CRC_FIELD_RESULT : // Data1: CRC, Data2: is 0 if CRC ok
    AnalyzerHelpers::GetNumberString (inFrame.mData1, inDisplayBase, 16, numberString, 128);
    if (inFrame.mData2 != 0) {
      if (!inBubbleText) {
        ioText << "  " ;
      }
      ioText << "CRC: " << numberString << " (error)" ;
    }else if (inBubbleText) {
      ioText << "CRC: " << numberString ;
    }
    break ;
  case ACK_FIELD_RESULT :
    if (inBubbleText) {
      ioText << "ACK" ;
    }
    break ;
  case EOF_FIELD_RESULT :
    if (inBubbleText) {
      ioText << "EOF" ;
    }
    break ;
  case INTERMISSION_FIELD_RESULT :
    if (inBubbleText) {
      ioText << "IFS" ;
    }else{
      const U64 frameSampleCount = inFrame.mData1 ;
      const U32 sampleRateHz = mAnalyzer->sampleRateHz () ;
      const U32 bitRate = mAnalyzer->bitRate () ;
      const U32 samplesPerBit = sampleRateHz / bitRate ;
      ioText << "  Length: " << ((frameSampleCount + samplesPerBit / 2) / samplesPerBit) << " bits ("
             << (frameSampleCount * 1000000 / sampleRateHz) << " µs), "
             << inFrame.mData2 << " stuff bit"
             << ((inFrame.mData2 > 1) ? "s" : "") ;
    }
    break ;
  default :
    ioText << "Error" ;
    break ;
  }
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzerResults::GenerateBubbleText (const U64 inFrameIndex,
                                                     Channel & channel,
                                                     const DisplayBase inDisplayBase) {
  const Frame frame = GetFrame (inFrameIndex) ;
  std::stringstream text ;
  GenerateText (frame, inDisplayBase, true, text) ;
  ClearResultStrings () ;
  AddResultString (text.str().c_str ()) ;
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzerResults::GenerateFrameTabularText (const U64 inFrameIndex,
                                                           const DisplayBase inDisplayBase) {
  #ifdef SUPPORTS_PROTOCOL_SEARCH
    const Frame frame = GetFrame (inFrameIndex) ;
    std::stringstream text ;
    GenerateText (frame, inDisplayBase, false, text) ;
    ClearTabularText () ;
    if (text.str().length () > 0) {
      AddTabularText (text.str().c_str ()) ;
    }
  #endif
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzerResults::GenerateExportFile( const char* file, DisplayBase display_base, U32 export_type_user_id )
{
  std::ofstream file_stream( file, std::ios::out );

  U64 trigger_sample = mAnalyzer->GetTriggerSample();
  U32 sample_rate = mAnalyzer->GetSampleRate();

  file_stream << "Time [s],Value" << std::endl;

  U64 num_frames = GetNumFrames();
  for( U32 i=0; i < num_frames; i++ )
  {
    Frame frame = GetFrame( i );

    char time_str[128];
    AnalyzerHelpers::GetTimeString( frame.mStartingSampleInclusive, trigger_sample, sample_rate, time_str, 128 );

    char number_str[128];
    AnalyzerHelpers::GetNumberString( frame.mData1, display_base, 8, number_str, 128 );

    file_stream << time_str << "," << number_str << std::endl;

    if( UpdateExportProgressAndCheckForCancel( i, num_frames ) == true )
    {
      file_stream.close();
      return;
    }
  }

  file_stream.close();
}

//--------------------------------------------------------------------------------------------------


void CANMolinaroAnalyzerResults::GeneratePacketTabularText( U64 packet_id, DisplayBase display_base )
{
  //not supported

}

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzerResults::GenerateTransactionTabularText( U64 transaction_id, DisplayBase display_base )
{
  //not supported
}

//--------------------------------------------------------------------------------------------------
