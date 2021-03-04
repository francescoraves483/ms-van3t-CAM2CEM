#ifndef GDP_H
#define GDP_H

#include "ns3/CEM.h"
#include "asn_utils.h"
#include <float.h>

// GPS Data Provider class
namespace ns3
{
  class GDP
  {
    public:
      typedef std::tuple<int, int> satsigID;

      template <typename T> using satmap = std::map<satsigID, T>;

      // Data structure containing the data mandatory for any kind of CEM frame
      typedef struct CEM_mandatory_data {
        ConstellationID_t constellationID;
        SatellitePRN_t satellitePRN;
      } CEM_mandatory_data_t;

      // Mandatory data for any kind of CEM frame
      virtual satmap<CEM_mandatory_data_t> getCEMMandatoryData() = 0;

      // Mandatory data for Full Precision Interframes
      virtual satmap<Pseudorange_t> getPseudoRange() = 0;
      virtual satmap<GPSDataUncertainty_t> getPseudoRangeUncertainty() = 0;

      // Mandatory data for Differential Microframes
      virtual satmap<DifferentialPseudorange_t> getDifferentialPseudoRange() = 0;

      // Optional data for Full Precision Interframes and Differential Microframes
      // These methods must return 'NULL' when the datum should not be inserted inside the CEM messages
      virtual satmap<OptionalContainer_t> *getFullPrecisionOptionalContainer() = 0;
      virtual satmap<DifferentialOptionalContainer_t> *getDifferentialOptionalContainer() = 0;

      // Function freeing the memory allocated for any GDP optional data
      // Basically, it should check if the void* pointer is not NULL and then free() the corresponding memory
      virtual void gdpFree(void*) = 0;
  };
}
#endif // GDP_H
