#include "CANMolinaroAnalyzerSettings.h"
#include <AnalyzerHelpers.h>

//----------------------------------------------------------------------------------------

CANMolinaroAnalyzerSettings::CANMolinaroAnalyzerSettings (void) :
mInputChannel (UNDEFINED_CHANNEL),
mBitRate (125 * 1000),
mInputChannelInterface (),
mBitRateInterface (),
mCanChannelInvertedInterface (),
mSimulatorAckGenerationInterface (),
mSimulatorFrameTypeGenerationInterface (),
mSimulatorFrameValidityInterface (),
mSimulatorRandomSeedInterface (),
mSimulatorGeneratedAckSlot (GENERATE_ACK_DOMINANT),
mSimulatorGeneratedFrameType (GENERATE_ALL_FRAME_TYPES),
mGeneratedFrameValidity (GENERATE_VALID_FRAMES),
mSimulatorRandomSeed (0),
mInverted (false) {
//--- Input Channel interface
  mInputChannelInterface.reset (new AnalyzerSettingInterfaceChannel ());
  mInputChannelInterface->SetTitleAndTooltip ("Serial", "CAN 2.0B");
  mInputChannelInterface->SetChannel (mInputChannel);

//--- Bit rate interface
  mBitRateInterface.reset (new AnalyzerSettingInterfaceInteger ()) ;
  mBitRateInterface->SetTitleAndTooltip ("CAN Bit Rate (bit/s)",
                                         "Specify the CAN bit rate in bits per second." );
  mBitRateInterface->SetMax (1 * 1000 * 1000) ;
  mBitRateInterface->SetMin (1) ;
  mBitRateInterface->SetInteger (mBitRate) ;

//--- Add Channel level inversion
  mCanChannelInvertedInterface.reset (new AnalyzerSettingInterfaceNumberList ( )) ;
  mCanChannelInvertedInterface->SetTitleAndTooltip ("Dominant Logic Level", "" );
  mCanChannelInvertedInterface->AddNumber (0.0,
                                           "Low",
                                           "Low is the usual dominant level") ;
  mCanChannelInvertedInterface->AddNumber (1.0,
                                           "High",
                                           "High is the inverted dominant level") ;
  mCanChannelInvertedInterface->SetNumber (0.0) ;

//--- Simulator random Seed
  mSimulatorRandomSeedInterface.reset (new AnalyzerSettingInterfaceInteger ()) ;
  mSimulatorRandomSeedInterface->SetTitleAndTooltip ("Simulator Random Seed", "") ;
  mSimulatorRandomSeedInterface->SetMax (1 * 1000 * 1000) ;
  mSimulatorRandomSeedInterface->SetMin (0) ;
  mSimulatorRandomSeedInterface->SetInteger (mSimulatorRandomSeed) ;

//--- Simulator ACK level
  mSimulatorAckGenerationInterface.reset (new AnalyzerSettingInterfaceNumberList ()) ;
  mSimulatorAckGenerationInterface->SetTitleAndTooltip ("Simulator ACK SLOT generated level", "");
  mSimulatorAckGenerationInterface->AddNumber (0.0,
                                               "Dominant",
                                               "Dominant is the valid level for ACK SLOT") ;
  mSimulatorAckGenerationInterface->AddNumber (1.0,
                                               "Recessive",
                                               "Recessive is the invalid level for ACK SLOT") ;
  mSimulatorAckGenerationInterface->AddNumber (2.0,
                                               "Random",
                               "The simulator generates dominant or recessive level randomly") ;
  mSimulatorAckGenerationInterface->SetNumber (0.0) ;

//--- Simulator Generated frames format
  mSimulatorFrameTypeGenerationInterface.reset (new AnalyzerSettingInterfaceNumberList ()) ;
  mSimulatorFrameTypeGenerationInterface->SetTitleAndTooltip ("Simulator Generated Frames Format", "");
  mSimulatorFrameTypeGenerationInterface->AddNumber (0.0, "All Types", "") ;
  mSimulatorFrameTypeGenerationInterface->AddNumber (1.0, "Only Standard Data Frames", "") ;
  mSimulatorFrameTypeGenerationInterface->AddNumber (2.0, "Only Extended Data Frames", "") ;
  mSimulatorFrameTypeGenerationInterface->AddNumber (3.0, "Only Standard Remote Frames", "") ;
  mSimulatorFrameTypeGenerationInterface->AddNumber (4.0, "Only Extended Remote Frames", "") ;
  mSimulatorFrameTypeGenerationInterface->SetNumber (0.0) ;

//--- Simulator Generated frames validity
  mSimulatorFrameValidityInterface.reset (new AnalyzerSettingInterfaceNumberList ()) ;
  mSimulatorFrameValidityInterface->SetTitleAndTooltip ("Simulator Generated Frames Validity", "");
  mSimulatorFrameValidityInterface->AddNumber (0.0, "Generate Valid Frames", "") ;
  mSimulatorFrameValidityInterface->AddNumber (1.0, "Randomly toggle one bit", "") ;
  mSimulatorFrameValidityInterface->SetNumber (0.0) ;

//--- Install interfaces
  AddInterface (mInputChannelInterface.get ()) ;
  AddInterface (mBitRateInterface.get ());
  AddInterface (mCanChannelInvertedInterface.get ());
  AddInterface (mSimulatorRandomSeedInterface.get ());
  AddInterface (mSimulatorFrameTypeGenerationInterface.get ());
  AddInterface (mSimulatorAckGenerationInterface.get ());
  AddInterface (mSimulatorFrameValidityInterface.get ());

//   AddExportOption( 0, "Export as text/csv file" );
//   AddExportExtension( 0, "text", "txt" );
//   AddExportExtension( 0, "csv", "csv" );

  ClearChannels () ;
  AddChannel (mInputChannel, "Serial", false) ;
}

//----------------------------------------------------------------------------------------

CANMolinaroAnalyzerSettings::~CANMolinaroAnalyzerSettings (void) {
}

//----------------------------------------------------------------------------------------

bool CANMolinaroAnalyzerSettings::SetSettingsFromInterfaces (void) {
  mInputChannel = mInputChannelInterface->GetChannel () ;
  mBitRate = mBitRateInterface->GetInteger () ;
  mInverted = U32 (mCanChannelInvertedInterface->GetNumber ()) != 0 ;
  mSimulatorRandomSeed = mSimulatorRandomSeedInterface->GetInteger () ;
  mSimulatorGeneratedAckSlot = U32 (mSimulatorAckGenerationInterface->GetNumber ()) ;
  mSimulatorGeneratedFrameType = U32 (mSimulatorFrameTypeGenerationInterface->GetNumber ()) ;
  mGeneratedFrameValidity = U32 (mSimulatorFrameValidityInterface->GetNumber ()) ;

  ClearChannels () ;
  AddChannel (mInputChannel, "CAN", true) ;

  return true ;
}

//----------------------------------------------------------------------------------------

const char * CANMolinaroAnalyzerSettings::SaveSettings (void) {
  SimpleArchive text_archive;

  text_archive << mInputChannel ;
  text_archive << mBitRate ;
  text_archive << mInverted ;
  text_archive << mSimulatorRandomSeed ;
  text_archive << mSimulatorGeneratedAckSlot ;
  text_archive << mSimulatorGeneratedFrameType ;
  text_archive << mGeneratedFrameValidity ;

  return SetReturnString (text_archive.GetString ()) ;
}

//----------------------------------------------------------------------------------------

void CANMolinaroAnalyzerSettings::LoadSettings (const char* settings) {
  SimpleArchive text_archive;
  text_archive.SetString (settings) ;

  text_archive >> mInputChannel;
  text_archive >> mBitRate;
  text_archive >> mInverted;
  text_archive >> mSimulatorRandomSeed ;
  text_archive >> mSimulatorGeneratedAckSlot ;
  text_archive >> mSimulatorGeneratedFrameType ;
  text_archive >> mGeneratedFrameValidity ;

 // ClearChannels();
  AddChannel (mInputChannel, "CAN 2.0B (Molinaro)", true) ;

//--- Update interface
  mInputChannelInterface->SetChannel (mInputChannel) ;
  mBitRateInterface->SetInteger (mBitRate) ;
  mSimulatorRandomSeedInterface->SetInteger (mSimulatorRandomSeed) ;
  mCanChannelInvertedInterface->SetNumber (double (mInverted)) ;
  mSimulatorAckGenerationInterface->SetNumber (mSimulatorGeneratedAckSlot) ;
  mSimulatorFrameTypeGenerationInterface->SetNumber (mSimulatorGeneratedFrameType) ;
  mSimulatorFrameValidityInterface->SetNumber (mGeneratedFrameValidity) ;
}

//----------------------------------------------------------------------------------------
