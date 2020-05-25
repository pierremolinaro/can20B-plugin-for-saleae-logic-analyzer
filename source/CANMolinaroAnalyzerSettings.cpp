#include "CANMolinaroAnalyzerSettings.h"
#include <AnalyzerHelpers.h>

//--------------------------------------------------------------------------------------------------

CANMolinaroAnalyzerSettings::CANMolinaroAnalyzerSettings() :
mInputChannel (UNDEFINED_CHANNEL),
mBitRate (125 * 1000) {
	mInputChannelInterface.reset (new AnalyzerSettingInterfaceChannel ());
	mInputChannelInterface->SetTitleAndTooltip ("Serial", "Standard Molinaro's CAN");
	mInputChannelInterface->SetChannel (mInputChannel);

	mBitRateInterface.reset (new AnalyzerSettingInterfaceInteger ()) ;
	mBitRateInterface->SetTitleAndTooltip ("CAN Bit Rate (bit/s)",
                                         "Specify the CAN bit rate in bits per second." );
	mBitRateInterface->SetMax (25 * 1000 * 1000) ;
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

//--- Simulator Generated frames
  mSimulatorFrameTypeGenerationInterface.reset (new AnalyzerSettingInterfaceNumberList ()) ;
	mSimulatorFrameTypeGenerationInterface->SetTitleAndTooltip ("Simulator Generated Frames", "");
  mSimulatorFrameTypeGenerationInterface->AddNumber (0.0, "All Types", "") ;
  mSimulatorFrameTypeGenerationInterface->AddNumber (1.0, "Only Standard Data Frames", "") ;
  mSimulatorFrameTypeGenerationInterface->AddNumber (2.0, "Only Extended Data Frames", "") ;
  mSimulatorFrameTypeGenerationInterface->AddNumber (3.0, "Only Standard Remote Frames", "") ;
  mSimulatorFrameTypeGenerationInterface->AddNumber (4.0, "Only Extended Remote Frames", "") ;
  mSimulatorFrameTypeGenerationInterface->SetNumber (0.0) ;

//--- Install interfaces
	AddInterface (mInputChannelInterface.get ()) ;
	AddInterface (mBitRateInterface.get ());
	AddInterface (mCanChannelInvertedInterface.get ());
	AddInterface (mSimulatorAckGenerationInterface.get ());
	AddInterface (mSimulatorFrameTypeGenerationInterface.get ());

	AddExportOption( 0, "Export as text/csv file" );
	AddExportExtension( 0, "text", "txt" );
	AddExportExtension( 0, "csv", "csv" );

	ClearChannels ();
	AddChannel (mInputChannel, "Serial", false) ;
}

//--------------------------------------------------------------------------------------------------

CANMolinaroAnalyzerSettings::~CANMolinaroAnalyzerSettings(){
}

//--------------------------------------------------------------------------------------------------

bool CANMolinaroAnalyzerSettings::SetSettingsFromInterfaces () {
	mInputChannel = mInputChannelInterface->GetChannel();
	mBitRate = mBitRateInterface->GetInteger();
  mInverted = U32 (mCanChannelInvertedInterface->GetNumber ()) != 0 ;
  mSimulatorGeneratedAckSlot
    = SimulatorGeneratedAckSlot (mSimulatorAckGenerationInterface->GetNumber ()) ;
  mSimulatorGeneratedFrameType
    = SimulatorGeneratedFrameType (mSimulatorFrameTypeGenerationInterface->GetNumber ()) ;
	ClearChannels();
	AddChannel (mInputChannel, "CAN", true) ;

	return true;
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzerSettings::UpdateInterfacesFromSettings() {
	mInputChannelInterface->SetChannel (mInputChannel) ;
	mBitRateInterface->SetInteger (mBitRate) ;
	mCanChannelInvertedInterface->SetNumber (double (mInverted)) ;
  mSimulatorAckGenerationInterface->SetNumber (mSimulatorGeneratedAckSlot) ;
  mSimulatorFrameTypeGenerationInterface->SetNumber (mSimulatorGeneratedFrameType) ;

}

//--------------------------------------------------------------------------------------------------

void CANMolinaroAnalyzerSettings::LoadSettings (const char* settings) {
	SimpleArchive text_archive;
	text_archive.SetString (settings) ;

	text_archive >> mInputChannel;
	text_archive >> mBitRate;
	text_archive >> mInverted;
  U32 value ;
	text_archive >> value ;
  mSimulatorGeneratedAckSlot = SimulatorGeneratedAckSlot (value) ;
	text_archive >> value ;
  mSimulatorGeneratedFrameType = SimulatorGeneratedFrameType (value) ;

	ClearChannels();
	AddChannel( mInputChannel, "CAN 2.0B (Molinaro)", true );

	UpdateInterfacesFromSettings();
}

//--------------------------------------------------------------------------------------------------

const char* CANMolinaroAnalyzerSettings::SaveSettings () {
	SimpleArchive text_archive;

	text_archive << mInputChannel;
	text_archive << mBitRate;
	text_archive << mInverted;
	text_archive << U32 (mSimulatorGeneratedAckSlot) ;
	text_archive << U32 (mSimulatorGeneratedFrameType) ;

	return SetReturnString (text_archive.GetString ()) ;
}

//--------------------------------------------------------------------------------------------------
