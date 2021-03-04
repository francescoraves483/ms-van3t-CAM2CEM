#ifndef GDPGPSTRACECLIENT_H
#define GDPGPSTRACECLIENT_H

#include "gdp.h"
#include "ns3/gps-tc.h"

namespace ns3 {
  class GDPGPSTraceClient : public GDP
  {
  public:
    GDPGPSTraceClient(Ptr<GPSTraceClient>,std::string);
    GDPGPSTraceClient();

    // Implementation of the GDP virtual functions
    // A "satmap" is returned, i.e. a C++ map containing the required information for each
    // satellite and for each satellite signal. The key of the satmap is a tuple of integers,
    // used to identify each satellite and signal of each satellite
    GDP::satmap<CEM_mandatory_data_t> getCEMMandatoryData();
    GDP::satmap<Pseudorange_t> getPseudoRange();
    GDP::satmap<GPSDataUncertainty_t> getPseudoRangeUncertainty();
    GDP::satmap<DifferentialPseudorange_t> getDifferentialPseudoRange();

    // Implementation of the GDP virtual functions returning a pointer (Optional Containers)
    GDP::satmap<OptionalContainer_t> *getFullPrecisionOptionalContainer();
    GDP::satmap<DifferentialOptionalContainer_t> *getDifferentialOptionalContainer();

    void gdpFree(void* optional_field)
    {
      if(optional_field!=NULL)
        {
          free(optional_field);
        }
    }

    private:
      std::string m_id;
      Ptr<GPSTraceClient> m_gps_trace_client;
  };
}

#endif // VDPGPSTRACECLIENT_H
