#ifndef CANMOLINARO_SIMULATION_DATA_GENERATOR
#define CANMOLINARO_SIMULATION_DATA_GENERATOR

//----------------------------------------------------------------------------------------

#include <SimulationChannelDescriptor.h>
#include <string>

//----------------------------------------------------------------------------------------

class CANMolinaroAnalyzerSettings ;

//----------------------------------------------------------------------------------------

class CANMolinaroSimulationDataGenerator {

  public: CANMolinaroSimulationDataGenerator (void) ;
  public:  ~CANMolinaroSimulationDataGenerator (void) ;

  public: void Initialize (U32 simulation_sample_rate,
                           CANMolinaroAnalyzerSettings * settings) ;

  public: U32 GenerateSimulationData (U64 newest_sample_requested,
                                      U32 sample_rate,
                                      SimulationChannelDescriptor ** simulation_channel) ;

  protected: CANMolinaroAnalyzerSettings * mSettings ;
  protected: U32 mSimulationSampleRateHz ;

//---------------- Pseudo Random Generator
//https://stackoverflow.com/questions/15500621/c-c-algorithm-to-produce-same-pseudo-random-number-sequences-from-same-seed-on
  protected: U32 mSeed ;
  protected: U32 pseudoRandomValue (void) {
    mSeed = 8253729U * mSeed + 2396403U ;
    return mSeed ;
  }

  protected: void createCANFrame (const U32 inSamplesPerBit, const bool inInverted) ;

  protected: SimulationChannelDescriptor * mSerialSimulationData ;
} ;

//----------------------------------------------------------------------------------------

#endif //CANMOLINARO_SIMULATION_DATA_GENERATOR
