#include "PRRSupervisor.h"
#include <sstream>
#include <cfloat>

#define DEG_2_RAD(val) ((val)*M_PI/180.0)

namespace {
  double PRRSupervisor_haversineDist(double lat_a, double lon_a, double lat_b, double lon_b) {
      // 12742000 is the mean Earth radius (6371 km) * 2 * 1000 (to convert from km to m)
      return 12742000.0*asin(sqrt(sin(DEG_2_RAD(lat_b-lat_a)/2)*sin(DEG_2_RAD(lat_b-lat_a)/2)+cos(DEG_2_RAD(lat_a))*cos(DEG_2_RAD(lat_b))*sin(DEG_2_RAD(lon_b-lon_a)/2)*sin(DEG_2_RAD(lon_b-lon_a)/2)));
  }
}

namespace ns3 {
  NS_LOG_COMPONENT_DEFINE("PRRSupervisor");

  TypeId
  PRRSupervisor::GetTypeId ()
  {
    static TypeId tid = TypeId("ns3::PRRSupervisor")
        .SetParent <Object>()
        .AddConstructor <PRRSupervisor>();
    return tid;
  }

  PRRSupervisor::~PRRSupervisor ()
  {
    NS_LOG_FUNCTION(this);

    std::list<EventId>::iterator eventit = eventList.begin();
    while(eventit != eventList.end())
    {
      if(eventit->IsExpired() == false)
      {
        Simulator::Cancel(*eventit);
      }
      eventit = eventList.erase(eventit);
    }
  }

  std::string
  PRRSupervisor::bufToString(uint8_t *buf, uint32_t bufsize)
  {
    std::stringstream bufss;

    bufss << std::hex << std::setfill('0');

    for(size_t i=0;i<bufsize;++i)
    {
      bufss << std::setw(2) << static_cast<unsigned>(buf[i]);
    }

    return bufss.str();
  }

  void
  PRRSupervisor::signalSentPacket(std::string buf,double lat,double lon)
  {
    EventId computePRR_id;

    if(m_traci_ptr == nullptr)
    {
      NS_FATAL_ERROR("Fatal error: TraCI client not set in PRR Supervisor.");
    }

    std::vector<std::string> ids = m_traci_ptr->TraCIAPI::vehicle.getIDList ();

    for(std::vector<std::string>::iterator it=ids.begin();it!=ids.end();++it)
    {
      uint64_t stationID = std::stol(it->substr (3));
      libsumo::TraCIPosition pos = m_traci_ptr->TraCIAPI::vehicle.getPosition (*it);
      pos=m_traci_ptr->TraCIAPI::simulation.convertXYtoLonLat (pos.x,pos.y);

      if(PRRSupervisor_haversineDist(lat,lon,pos.y,pos.x)<=m_baseline_m)
      {
          m_packetbuff_map[buf].vehList.push_back(stationID);
      }
    }

    computePRR_id = Simulator::Schedule(Seconds (3), &PRRSupervisor::computePRR, this, buf);
    eventList.push_back (computePRR_id);

    m_latency_map[buf] = Simulator::Now ().GetNanoSeconds ();
  }

  void
  PRRSupervisor::signalReceivedPacket(std::string buf, uint64_t vehicleID)
  {
    baselineVehicleData_t currBaselineData;
    double curr_latency_ms = DBL_MAX;

    if(m_traci_ptr == nullptr)
    {
      NS_FATAL_ERROR("Fatal error: TraCI client not set in PRR Supervisor.");
    }

    if(m_packetbuff_map.count(buf)>0)
    {
      currBaselineData = m_packetbuff_map[buf];

      if(std::find(currBaselineData.vehList.begin(), currBaselineData.vehList.end(), vehicleID) != currBaselineData.vehList.end())
      {
        (m_packetbuff_map[buf].x)++;
      }
    }

    // Compute latency in ms
    if(m_latency_map.count(buf)>0)
    {
        curr_latency_ms = static_cast<double>(Simulator::Now ().GetNanoSeconds () - m_latency_map[buf])/1000000.0;
        m_count_latency++;

        m_avg_latency_ms += (curr_latency_ms-m_avg_latency_ms)/m_count_latency;
    }
  }

  void
  PRRSupervisor::computePRR(std::string buf)
  {
    double PRR = 0.0;

    if(m_packetbuff_map[buf].vehList.size()>1)
    {
      PRR = (double) m_packetbuff_map[buf].x/(double) (m_packetbuff_map[buf].vehList.size()-1.0);
      m_count++;
      m_avg_PRR += (PRR-m_avg_PRR)/m_count;

      m_packetbuff_map.erase(buf);
    }

    // Some time has passed -> remove the packet from the latency map too
    m_latency_map.erase(buf);
  }
}
