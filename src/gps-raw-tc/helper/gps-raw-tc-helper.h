/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef GPS_RAW_TC_HELPER_H
#define GPS_RAW_TC_HELPER_H

#include <map>
#include "ns3/gps-raw-tc.h"

#define GPS_TC_MAP_ITERATOR(mapname,itername) for(std::map<std::string,GPSTraceClient*>::iterator itername=mapname.begin(); itername!=mapname.end(); ++itername)
#define GPS_TC_IT_OBJECT(itername) itername->second

namespace ns3 {

class GPSRawTraceClientHelper
{
    public:
      GPSRawTraceClientHelper();

      std::map<std::string,GPSRawTraceClientHelper*> createRawTraceClientsFromCSV(std::string filepath);

      // Set or unset a verbose mode
      // Default: false (verbose mode not activated)
      void setVerbose(bool verbose) {
        m_verbose=verbose;
      }

      void setHeaderPresence(bool header_presence) {
        m_header = header_presence;
      }

      void setOneTraceAllVehicles(bool single_trace) {
        m_singletrace = single_trace;
      }

    private:
      bool m_verbose;
      bool m_header;
      bool m_singletrace;
      int m_increasingVehID;
};

}

#endif /* GPS_RAW_TC_HELPER_H */

