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
    GDP::satmap<CEM_mandatory_data_t> getCEMMandatoryData();
    GDP::satmap<Pseudorange_t> getPseudoRange();
    GDP::satmap<GPSDataUncertainty_t> getPseudoRangeUncertainty();
    GDP::satmap<DifferentialPseudorange_t> getDifferentialPseudoRange();

    // Implementation of the GDP virtual functions returning a pointer (Optional Containers)
    GDP::satmap<OptionalContainer_t *> getFullPrecisionOptionalContainer();
    GDP::satmap<DifferentialOptionalContainer_t *> getDifferentialOptionalContainer();
    GDP::satmap<UncertaintyContainer_t *> getFullPrecisionUncertaintyContainer();

    void gdpFree(void* optional_field)
    {
      if(optional_field!=NULL)
        {
          free(optional_field);
        }
    }

    void gdpINTEGERFree(INTEGER_t *integer_field)
    {
      if(integer_field!=NULL && integer_field->buf!=NULL)
      {
        free(integer_field->buf);
      }
    }

    private:
      std::string m_id;
      Ptr<GPSTraceClient> m_gps_trace_client;
  };
}

#endif // VDPGPSTRACECLIENT_H
