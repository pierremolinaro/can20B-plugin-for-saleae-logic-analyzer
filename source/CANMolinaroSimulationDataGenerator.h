#ifndef CANMOLINARO_SIMULATION_DATA_GENERATOR
#define CANMOLINARO_SIMULATION_DATA_GENERATOR

//--------------------------------------------------------------------------------------------------

#include <SimulationChannelDescriptor.h>
#include <string>

//--------------------------------------------------------------------------------------------------

class CANMolinaroAnalyzerSettings;

//--------------------------------------------------------------------------------------------------

class CANMolinaroSimulationDataGenerator {

public: CANMolinaroSimulationDataGenerator (void) ;
public:	~CANMolinaroSimulationDataGenerator (void) ;

public: void Initialize (U32 simulation_sample_rate,
                         CANMolinaroAnalyzerSettings * settings) ;

public: U32 GenerateSimulationData (U64 newest_sample_requested,
                                    U32 sample_rate,
                                    SimulationChannelDescriptor ** simulation_channel) ;

protected: CANMolinaroAnalyzerSettings* mSettings ;
protected: U32 mSimulationSampleRateHz ;

protected: void CreateCANFrame ();

protected: SimulationChannelDescriptor mSerialSimulationData;

} ;

//--------------------------------------------------------------------------------------------------

#endif //CANMOLINARO_SIMULATION_DATA_GENERATOR
