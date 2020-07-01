#ifndef CANMOLINARO_ANALYZER_SETTINGS
#define CANMOLINARO_ANALYZER_SETTINGS

//--------------------------------------------------------------------------------------------------

#include <AnalyzerSettings.h>
#include <AnalyzerTypes.h>

//--------------------------------------------------------------------------------------------------

typedef enum {
  GENERATE_ACK_DOMINANT,
  GENERATE_ACK_RECESSIVE,
  GENERATE_ACK_RANDOMLY
} SimulatorGeneratedAckSlot ;

//--------------------------------------------------------------------------------------------------

typedef enum {
  GENERATE_ALL_FRAME_TYPES,
  GENERATE_ONLY_STANDARD_DATA,
  GENERATE_ONLY_EXTENDED_DATA,
  GENERATE_ONLY_STANDARD_REMOTE,
  GENERATE_ONLY_EXTENDED_REMOTE
} SimulatorGeneratedFrameType ;

//--------------------------------------------------------------------------------------------------

class CANMolinaroAnalyzerSettings : public AnalyzerSettings {

  public: CANMolinaroAnalyzerSettings (void);
  public: virtual ~CANMolinaroAnalyzerSettings();

  public: virtual bool SetSettingsFromInterfaces();
  public: void UpdateInterfacesFromSettings();
  public: virtual void LoadSettings( const char* settings );
  public: virtual const char* SaveSettings();


  public: Channel mInputChannel;
  public: U32 mBitRate;

  public: bool inverted (void) const { return mInverted ; }

  public: SimulatorGeneratedAckSlot generatedAckSlot (void) const {
    return mSimulatorGeneratedAckSlot ;
  }

  public: SimulatorGeneratedFrameType generatedFrameType (void) const {
   return mSimulatorGeneratedFrameType ;
  }

  protected: std::auto_ptr< AnalyzerSettingInterfaceChannel >  mInputChannelInterface;
  protected: std::auto_ptr< AnalyzerSettingInterfaceInteger >  mBitRateInterface;
  protected: std::auto_ptr< AnalyzerSettingInterfaceNumberList > mCanChannelInvertedInterface ;
  protected: std::auto_ptr< AnalyzerSettingInterfaceNumberList > mSimulatorAckGenerationInterface ;
  protected: std::auto_ptr< AnalyzerSettingInterfaceNumberList > mSimulatorFrameTypeGenerationInterface ;

  protected: SimulatorGeneratedAckSlot mSimulatorGeneratedAckSlot ;
  protected: SimulatorGeneratedFrameType mSimulatorGeneratedFrameType ;
  protected: bool mInverted ;
};

//--------------------------------------------------------------------------------------------------

#endif //CANMOLINARO_ANALYZER_SETTINGS
