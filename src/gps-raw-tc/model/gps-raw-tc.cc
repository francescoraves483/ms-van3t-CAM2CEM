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
      m_endTrace=nullptr;
      m_vehNode=nullptr;
      m_vehID=vehID;
      m_frame_callback=nullptr;
  }

  GPSRawTraceClient::~GPSRawTraceClient()
  {
      m_lastvehicledataidx=0;
      m_endTrace=nullptr;
      m_frame_callback=nullptr;
      //vehiclesdata.clear();
      //vehiclesdata.shrink_to_fit();
  }

  std::string
  GPSRawTraceClient::getID()
  {
    return m_vehID;
  }

  void
  GPSRawTraceClient::setNewIFrame(double cemTstamp,double abstimestamp,GDP::satmap<iframe_data_t> data)
  {
      GPSRawTraceClient::raw_positioning_data_t curr_rawdata;

      curr_rawdata.type = FULL_PRECISION_I_FRAME;
      curr_rawdata.cemTstamp = cemTstamp;
      curr_rawdata.iframe_data = data;
      curr_rawdata.absTstamp = abstimestamp;

      vehiclesdata.push_back (curr_rawdata);
  }

  void
  GPSRawTraceClient::setNewDFrame(double cemTstamp,double abstimestamp,GDP::satmap<dframe_data_t> data)
  {
      GPSRawTraceClient::raw_positioning_data_t curr_rawdata;

      curr_rawdata.type = DIFFERENTIAL_D_FRAME;
      curr_rawdata.cemTstamp = cemTstamp;
      curr_rawdata.dframe_data = data;
      curr_rawdata.absTstamp = abstimestamp;

      vehiclesdata.push_back (curr_rawdata);
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
              std::cout << "Full-precision interframe @ " << std::setprecision(12) << vehiclesdata[i].cemTstamp << " - number of satellites/signals: " << vehiclesdata[i].iframe_data.size () << std::endl;
          } else if(vehiclesdata[i].type == DIFFERENTIAL_D_FRAME) {
              std::cout << "Differential frame @ " << std::setprecision(12) << vehiclesdata[i].cemTstamp << " - number of satellites/signals: " << vehiclesdata[i].dframe_data.size () << std::endl;
          } else {
              std::cerr << "Error: unknown frame @ " << std::setprecision(12) << vehiclesdata[i].cemTstamp << std::endl;
          }
      }
  }

  void
  GPSRawTraceClient::GPSRawTraceClientSetup(Ptr<Node> veh_node,std::function<void(Ptr<Node>)> end_fcn)
  {
      m_vehNode = veh_node;
      m_endTrace=end_fcn;
  }

  void
  GPSRawTraceClient::playTrace(Time const &delay)
  {
    if(m_frame_callback == nullptr)
    {
        NS_FATAL_ERROR ("Error: attempted to start a GPS Raw Data Trace Client without specifying any frame callback");
      }

    Simulator::Schedule(delay, &GPSRawTraceClient::CreateNode, this);
  }

  void
  GPSRawTraceClient::CreateNode()
  {
      // Do not start "playing" any trace if no node has been associated with this GPS Raw Trace Client instance
      if(m_vehNode == nullptr)
      {
          return;
      }

      // First position update
      m_lastvehicledataidx=0;

      UpdateRawData();
  }

  // "vehiclesdata" is expected to be ordered by utc_time
  // The helper should and must take care of that by calling sortVehiclesdata() at least once!
  void
  GPSRawTraceClient::UpdateRawData(void)
  {
    if(m_vehNode==nullptr)
    {
      NS_FATAL_ERROR("NULL vehicle node pointer passed to GPSRawTraceClient::UpdateRawData (vehicle/object ID: "<<m_vehID<<".");
    }

    m_frame_callback(getLastFrameData());

    m_lastvehicledataidx++;

    if(m_lastvehicledataidx+1==vehiclesdata.size())
      {
        if(m_vehNode != nullptr && m_endTrace != nullptr)
        {
          m_endTrace(m_vehNode);
          m_vehNode=nullptr;
        }

        std::cout << "Raw Trace terminated for vehicle/object with ID: "<<m_vehID << std::endl;

        return;
      }

    m_event_updaterawdata=Simulator::Schedule(MilliSeconds ((vehiclesdata[m_lastvehicledataidx+1].absTstamp-vehiclesdata[m_lastvehicledataidx].absTstamp)*100), &GPSRawTraceClient::UpdateRawData, this);
  }

  void
  GPSRawTraceClient::StopUpdates(void)
  {
      m_event_updaterawdata.Cancel ();
  }

}

