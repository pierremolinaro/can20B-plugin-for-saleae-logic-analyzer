#ifndef CANMOLINARO_ANALYZER_SETTINGS
#define CANMOLINARO_ANALYZER_SETTINGS

//--------------------------------------------------------------------------------------------------

#include <AnalyzerSettings.h>
#include <AnalyzerTypes.h>

//--------------------------------------------------------------------------------------------------

static const U32 GENERATE_ACK_DOMINANT = 0 ;
static const U32 GENERATE_ACK_RECESSIVE = 1 ;
static const U32 GENERATE_ACK_RANDOMLY = 2 ;

//--------------------------------------------------------------------------------------------------

static const U32 GENERATE_ALL_FRAME_TYPES = 0 ;
static const U32 GENERATE_ONLY_STANDARD_DATA = 1 ;
static const U32 GENERATE_ONLY_EXTENDED_DATA = 2 ;
static const U32 GENERATE_ONLY_STANDARD_REMOTE = 3 ;
static const U32 GENERATE_ONLY_EXTENDED_REMOTE = 4 ;

//--------------------------------------------------------------------------------------------------

static const U32 GENERATE_VALID_FRAMES = 0 ;
static const U32 GENERATE_ONE_RANDOM_ERROR_BIT = 1 ;

//--------------------------------------------------------------------------------------------------

class CANMolinaroAnalyzerSettings : public AnalyzerSettings {

  public: CANMolinaroAnalyzerSettings (void) ;
  public: virtual ~CANMolinaroAnalyzerSettings (void) ;

  public: virtual bool SetSettingsFromInterfaces (void) ;
  public: virtual void LoadSettings (const char* settings) ;
  public: virtual const char* SaveSettings (void) ;


  public: Channel mInputChannel;
  public: U32 mBitRate;
  public: U32 bitRate (void) const { return mBitRate ; }

  public: bool inverted (void) const { return mInverted ; }

  public: U32 generatedAckSlot (void) const {
    return mSimulatorGeneratedAckSlot ;
  }

  public: U32 generatedFrameType (void) const {
   return mSimulatorGeneratedFrameType ;
  }

  public: U32 generatedFrameValidity (void) const {
   return mGeneratedFrameValidity ;
  }

  public: U32 simulatorRandomSeed (void) const {
   return mSimulatorRandomSeed ;
  }


  protected: std::unique_ptr < AnalyzerSettingInterfaceChannel >  mInputChannelInterface;
  protected: std::unique_ptr < AnalyzerSettingInterfaceInteger >  mBitRateInterface;
  protected: std::unique_ptr < AnalyzerSettingInterfaceNumberList > mCanChannelInvertedInterface ;
  protected: std::unique_ptr < AnalyzerSettingInterfaceNumberList > mSimulatorAckGenerationInterface ;
  protected: std::unique_ptr < AnalyzerSettingInterfaceNumberList > mSimulatorFrameTypeGenerationInterface ;
  protected: std::unique_ptr < AnalyzerSettingInterfaceNumberList > mSimulatorFrameValidityInterface ;
  protected: std::unique_ptr < AnalyzerSettingInterfaceInteger > mSimulatorRandomSeedInterface ;

  protected: U32 mSimulatorGeneratedAckSlot ;
  protected: U32 mSimulatorGeneratedFrameType ;
  protected: U32 mGeneratedFrameValidity ;
  protected: U32 mSimulatorRandomSeed ;
  protected: bool mInverted ;
} ;

//--------------------------------------------------------------------------------------------------

#endif //CANMOLINARO_ANALYZER_SETTINGS
