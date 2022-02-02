#ifndef CANMOLINARO_ANALYZER_H
#define CANMOLINARO_ANALYZER_H

//--------------------------------------------------------------------------------------------------

#include <Analyzer.h>
#include "CANMolinaroAnalyzerResults.h"
#include "CANMolinaroSimulationDataGenerator.h"

//--------------------------------------------------------------------------------------------------

class CANMolinaroAnalyzerSettings;

//--------------------------------------------------------------------------------------------------

class ANALYZER_EXPORT CANMolinaroAnalyzer : public Analyzer2 {

  public: CANMolinaroAnalyzer();

  public: virtual ~CANMolinaroAnalyzer();

  public: virtual void SetupResults();

  public: virtual void WorkerThread();

  public: virtual U32 GenerateSimulationData (U64 newest_sample_requested,
                                              U32 sample_rate,
                                              SimulationChannelDescriptor** simulation_channels);
  public: virtual U32 GetMinimumSampleRateHz();

  public: virtual const char* GetAnalyzerName() const;

  public: virtual bool NeedsRerun();

  private: void enterBit (const bool inBit, const U64 inSampleNumber) ;
  private: void decodeFrameBit (const bool inBit, const U64 inSampleNumber) ;


//--- Protected properties
  protected: std::auto_ptr< CANMolinaroAnalyzerSettings > mSettings;
  protected: std::auto_ptr< CANMolinaroAnalyzerResults > mResults;
  protected: AnalyzerChannelData* mSerial;

  protected: CANMolinaroSimulationDataGenerator mSimulationDataGenerator;
  protected: bool mSimulationInitilized;

  //Serial analysis vars:
  protected: U32 mSampleRateHz;
  protected: U32 mStartOfStopBitOffset;
  protected: U32 mEndOfStopBitOffset;

  public: inline U32 sampleRateHz (void) const { return mSampleRateHz ;  }
  public: U32 bitRate (void) const ;

//---------------- CAN decoder properties
//--- CAN protocol
  private: typedef enum  {
    IDLE, IDENTIFIER, EXTENDED_IDF, CONTROL, DATA, CRC, CRC_DEL, ACK,
    END_OF_FRAME, INTERMISSION, DECODER_ERROR
  } FrameFieldEngineState ;

  private: FrameFieldEngineState mFrameFieldEngineState ;
  private: int mFieldBitIndex ;
  private: int mConsecutiveBitCountOfSamePolarity ;
  private: bool mPreviousBit ;
  private: bool mUnstuffingActive ;

  private: U64 mStartOfFrameSampleNumber ;
  private: U64 mStuffBitCount ;
  private: U64 mStartOfFieldSampleNumber ;
  private: U16 mCRCAccumulator ;

//--- Received frame
  private: typedef enum {dataFrame, remoteFrame} FrameType ;
  private: uint32_t mIdentifier ;
  private: FrameType mFrameType ; // data, remote
  private: int mDataLength ;
  private: uint8_t mData [8] ;
  private: uint16_t mCRC ;

//---------------- CAN decoder methods
  private: void enterBitInCRC (const bool inBit) ;
  private: void addMark (const U64 inSampleNumber, const AnalyzerResults::MarkerType inMarker) ;
  private: void addBubble (const U8 inBubbleType, const U64 inData1, const U64 inData2, const U64 inEndSampleNumber) ;
  private: void enterInErrorMode (const U64 inSampleNumber) ;
} ;

//--------------------------------------------------------------------------------------------------

extern "C" ANALYZER_EXPORT const char* __cdecl GetAnalyzerName();
extern "C" ANALYZER_EXPORT Analyzer* __cdecl CreateAnalyzer( );
extern "C" ANALYZER_EXPORT void __cdecl DestroyAnalyzer( Analyzer* analyzer );

//--------------------------------------------------------------------------------------------------

#endif //CANMOLINARO_ANALYZER_H
