#include "gdpGPSTraceClient.h"

extern "C" {
    #include "ns3/utmups.h"
}

namespace ns3
{
  GDPGPSTraceClient::GDPGPSTraceClient()
  {
    m_gps_trace_client=NULL;
    m_id="(null)";
  }

  GDPGPSTraceClient::GDPGPSTraceClient(Ptr<GPSTraceClient> gps_tc,std::string id)
  {
    m_gps_trace_client=gps_tc;
    m_id=id;
  }

  GDP::satmap<GDP::CEM_mandatory_data_t>
  GDPGPSTraceClient::getCEMMandatoryData()
  {
    GDP::satmap<GDP::CEM_mandatory_data_t> cemMandatoryDataMap;

    cemMandatoryDataMap[satsigID(0,0)].satellitePRN=SatellitePRN_unavailable;
    cemMandatoryDataMap[satsigID(0,0)].constellationID=ConstellationID_unavailable;

    return cemMandatoryDataMap;
  }

  GDP::satmap<Pseudorange_t>
  GDPGPSTraceClient::getPseudoRange()
  {
    GDP::satmap<Pseudorange_t> pseudoRangeMap;

    // For each element of the satmap...
    intmax_t pseudorange00=Pseudorange_unavailable;
    Pseudorange_t pseudorangeI00;
    memset(&pseudorangeI00, 0, sizeof(INTEGER_t));
    asn_imax2INTEGER(&pseudorangeI00, pseudorange00);
    pseudoRangeMap[satsigID(0,0)]=pseudorangeI00;

    return pseudoRangeMap;
  }

  GDP::satmap<GPSDataUncertainty_t>
  GDPGPSTraceClient::getPseudoRangeUncertainty()
  {
    GDP::satmap<GPSDataUncertainty_t> pseudoRangeUncertaintyMap;

    pseudoRangeUncertaintyMap[satsigID(0,0)]=GPSDataUncertainty_unavailable;

    return pseudoRangeUncertaintyMap;
  }

  GDP::satmap<DifferentialPseudorange_t>
  GDPGPSTraceClient::getDifferentialPseudoRange()
  {
    GDP::satmap<DifferentialPseudorange_t> differentialPseudoRangeMap;

    differentialPseudoRangeMap[satsigID(0,0)]=DifferentialPseudorange_unavailable;

    return differentialPseudoRangeMap;
  }

  GDP::satmap<OptionalContainer_t *>
  GDPGPSTraceClient::getFullPrecisionOptionalContainer()
  {
    GDP::satmap<OptionalContainer_t *> optionalContainerMap;
    OptionalContainer_t *optionalContainerPtr = (OptionalContainer_t *) calloc(1,sizeof(OptionalContainer_t));

    // This should then be repeated, in a loop, for each satellite and signal
    if(optionalContainerPtr)
    {
      optionalContainerPtr->doppler=Doppler_unavailable;
      optionalContainerPtr->dopplerUncertainty=GPSDataUncertainty_unavailable;

      intmax_t carrierPhase=CarrierPhase_unavailable;
      Pseudorange_t carrierPhaseI;
      memset(&carrierPhaseI, 0, sizeof(INTEGER_t));
      asn_imax2INTEGER(&carrierPhaseI, carrierPhase);
      optionalContainerPtr->carrierPhase=carrierPhaseI;

      optionalContainerPtr->carrierPhaseUncertainty=GPSDataUncertainty_unavailable;
    }

    // If optionalContainerPtr is NULL, NULL will be assigned to the current map entry
    optionalContainerMap[satsigID(0,0)]=optionalContainerPtr;

    return optionalContainerMap;
  }

  GDP::satmap<DifferentialOptionalContainer_t *>
  GDPGPSTraceClient::getDifferentialOptionalContainer()
  {
    GDP::satmap<DifferentialOptionalContainer_t *> differentialOptionalContainerMap;
    DifferentialOptionalContainer_t *differentialOptionalContainerPtr = (DifferentialOptionalContainer_t *) calloc(1,sizeof(DifferentialOptionalContainer_t));

    // This should then be repeated, in a loop, for each satellite and signal
    if(differentialOptionalContainerPtr)
    {
      differentialOptionalContainerPtr->differentialDoppler=DifferentialDoppler_unavailable;
      differentialOptionalContainerPtr->differentialDopplerUncertainty=GPSDataUncertainty_unavailable;
      differentialOptionalContainerPtr->differentialCarrierPhase=DifferentialCarrierPhase_unavailable;
      differentialOptionalContainerPtr->differentialCarrierPhaseUncertainty=GPSDataUncertainty_unavailable;
    }

    // If optionalContainerPtr is NULL, NULL will be assigned to the current map entry
    differentialOptionalContainerMap[satsigID(0,0)]=differentialOptionalContainerPtr;

    return differentialOptionalContainerMap;
  }
}
