#ifndef CANMOLINARO_ANALYZER_RESULTS
#define CANMOLINARO_ANALYZER_RESULTS

//----------------------------------------------------------------------------------------

#include <AnalyzerResults.h>

//----------------------------------------------------------------------------------------

enum CanFrameType {
  STANDARD_IDENTIFIER_FIELD_RESULT,
  EXTENDED_IDENTIFIER_FIELD_RESULT,
  CONTROL_FIELD_RESULT,
  DATA_FIELD_RESULT,
  CRC_FIELD_RESULT,
  ACK_FIELD_RESULT,
  EOF_FIELD_RESULT,
  INTERMISSION_FIELD_RESULT,
  CAN_ERROR_RESULT
} ;

//----------------------------------------------------------------------------------------

class CANMolinaroAnalyzer;
class CANMolinaroAnalyzerSettings;

//----------------------------------------------------------------------------------------

class CANMolinaroAnalyzerResults : public AnalyzerResults {
public:
  CANMolinaroAnalyzerResults( CANMolinaroAnalyzer* analyzer, CANMolinaroAnalyzerSettings* settings );
  virtual ~CANMolinaroAnalyzerResults();

  virtual void GenerateBubbleText( U64 frame_index, Channel& channel, DisplayBase display_base );
  virtual void GenerateExportFile( const char* file, DisplayBase display_base, U32 export_type_user_id );

  virtual void GenerateFrameTabularText(U64 frame_index, DisplayBase display_base );
  virtual void GeneratePacketTabularText( U64 packet_id, DisplayBase display_base );
  virtual void GenerateTransactionTabularText( U64 transaction_id, DisplayBase display_base );

protected: //functions
  void GenerateText (const Frame & inFrame,
                     const DisplayBase inDisplayBase,
                     const bool inBubbleText,
                     std::stringstream & ioText) ;

protected:  //vars
  CANMolinaroAnalyzerSettings* mSettings;
  CANMolinaroAnalyzer* mAnalyzer;
};

//----------------------------------------------------------------------------------------

#endif //CANMOLINARO_ANALYZER_RESULTS
