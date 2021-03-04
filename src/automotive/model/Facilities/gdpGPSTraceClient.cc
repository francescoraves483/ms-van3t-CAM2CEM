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
    long long pseudorange00=Pseudorange_unavailable;
    Pseudorange_t pseudorangeI00;
    memset(&pseudorangeI00, 0, sizeof(INTEGER_t));
    asn_long2INTEGER(&pseudorangeI00, pseudorange00);
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

  GDP::satmap<OptionalContainer_t> *
  GDPGPSTraceClient::getFullPrecisionOptionalContainer()
  {
    return NULL;
  }

  GDP::satmap<DifferentialOptionalContainer_t> *
  GDPGPSTraceClient::getDifferentialOptionalContainer()
  {
    return NULL;
  }
}
