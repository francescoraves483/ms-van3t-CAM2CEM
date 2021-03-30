/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 * Created by:
 *  Giuseppe Avino, Politecnico di Torino (giuseppe.avino@polito.it)
 *  Marco Malinverno, Politecnico di Torino (marco.malinverno1@gmail.com)
 *  Francesco Raviglione, Politecnico di Torino (francescorav.es483@gmail.com)
*/

#include "gps-raw-tc.h"

extern "C" {
  #include "ns3/utmups_math.h"
}

namespace ns3
{
  NS_LOG_COMPONENT_DEFINE("GPSRawTraceClient");

  GPSRawTraceClient::GPSRawTraceClient(std::string vehID)
  {
      //ctor
      m_lastvehicledataidx=0;
      m_startTrace=nullptr;
      m_endTrace=nullptr;
      m_vehNode=NULL;
      m_vehID=vehID;
  }

  GPSRawTraceClient::~GPSRawTraceClient()
  {
      m_lastvehicledataidx=0;
      m_startTrace=nullptr;
      m_endTrace=nullptr;
  }

  std::string
  GPSRawTraceClient::getID()
  {
    return m_vehID;
  }

  void
  GPSRawTraceClient::setNewIFrame(uint64_t cemTstamp,iframe_data_t data)
  {
      vehiclesdata[vehiclesdata.size()-1].type = FULL_PRECISION_I_FRAME;
      vehiclesdata[vehiclesdata.size()-1].cemTstamp = cemTstamp;
      vehiclesdata[vehiclesdata.size()-1].iframe_data = data;
  }

  void
  GPSRawTraceClient::setNewDFrame(uint64_t cemTstamp,dframe_data_t data)
  {
      vehiclesdata[vehiclesdata.size()-1].type = DIFFERENTIAL_D_FRAME;
      vehiclesdata[vehiclesdata.size()-1].cemTstamp = cemTstamp;
      vehiclesdata[vehiclesdata.size()-1].dframe_data = data;
  }

  GPSRawTraceClient::raw_positioning_data_t
  GPSRawTraceClient::getLastFrameData()
  {
      return vehiclesdata[m_lastvehicledataidx];
  }

  void
  GPSRawTraceClient::sortRawdata()
  {
      std::sort(vehiclesdata.begin(), vehiclesdata.end());
  }

  void
  GPSRawTraceClient::printDebugRawData()
  {
      std::cout << "Number of raw data element (I+D frames): " << vehiclesdata.size() << std::endl;
      for (unsigned int i=0; i<vehiclesdata.size(); i++) {
          if(vehiclesdata[i].type == FULL_PRECISION_I_FRAME) {
              std::cout << "Full-precision interframe @ " << std::setprecision(12) << vehiclesdata[i].cemTstamp << " - number of satellites/signals (looking at pseudorange): " << vehiclesdata[i].iframe_data.pseudorange.size () << std::endl;
          } else if(vehiclesdata[i].type == DIFFERENTIAL_D_FRAME) {
              std::cout << "Differential frame @ " << std::setprecision(12) << vehiclesdata[i].cemTstamp << " - number of satellites/signals (looking at differential pseudorange): " << vehiclesdata[i].dframe_data.differential_pseudorange.size () << std::endl;
          } else {
              std::cerr << "Error: unknown frame @ " << std::setprecision(12) << vehiclesdata[i].cemTstamp << std::endl;
          }
      }
  }

  void
  GPSRawTraceClient::GPSRawTraceClientSetup(std::function<Ptr<Node>()> start_fcn,std::function<void(Ptr<Node>)> end_fcn)
  {
      m_startTrace=start_fcn;
      m_endTrace=end_fcn;
  }

  void
  GPSRawTraceClient::playTrace(Time const &delay)
  {
    Simulator::Schedule(delay, &GPSRawTraceClient::CreateNode, this);
  }

  void
  GPSRawTraceClient::CreateNode()
  {
      m_vehNode=m_startTrace();

      // First position update
      m_lastvehicledataidx=0;

      UpdateRawData();
  }

  // "vehiclesdata" is expected to be ordered by utc_time
  // The helper should and must take care of that by calling sortVehiclesdata() at least once!
  void
  GPSRawTraceClient::UpdateRawData(void)
  {
    if(m_vehNode==NULL)
    {
      NS_FATAL_ERROR("NULL vehicle node pointer passed to GPSRawTraceClient::UpdateRawData (vehicle/object ID: "<<m_vehID<<".");
    }

    m_lastvehicledataidx++;

    if(m_lastvehicledataidx+1==vehiclesdata.size())
      {
        m_endTrace(m_vehNode);
        m_vehNode=NULL;

        NS_LOG_INFO("Raw Trace terminated for vehicle/object with ID: "<<m_vehID);

        return;
      }

    m_event_updaterawdata=Simulator::Schedule(MicroSeconds (vehiclesdata[m_lastvehicledataidx+1].cemTstamp-vehiclesdata[m_lastvehicledataidx].cemTstamp), &GPSRawTraceClient::UpdateRawData, this);
  }

  void
  GPSRawTraceClient::StopUpdates(void)
  {
      m_event_updaterawdata.Cancel ();
  }

}

