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
#include "simpleCAMSender-gps-tc-cem.h"

#include "ns3/CAM.h"
#include "ns3/DENM.h"
#include "ns3/CEM.h"
#include "ns3/vdpGPSTraceClient.h"
#include "ns3/vdpTraci.h"
#include "ns3/socket.h"
#include "ns3/network-module.h"
#include "ns3/gn-utils.h"

#include "ns3/cv2x_lte-net-device.h"
#include "ns3/cv2x_lte-ue-phy.h"
#include "ns3/net-device.h"
#include "ns3/cv2x_lte-ue-net-device.h"

namespace ns3
{
  NS_LOG_COMPONENT_DEFINE("simpleCAMSenderCEM");

  NS_OBJECT_ENSURE_REGISTERED(simpleCAMSenderCEM);

  TypeId
  simpleCAMSenderCEM::GetTypeId (void)
  {
    static TypeId tid =
        TypeId ("ns3::simpleCAMSenderCEM")
        .SetParent<Application> ()
        .SetGroupName ("Applications")
        .AddConstructor<simpleCAMSenderCEM> ()
        .AddAttribute ("RealTime",
            "To compute properly timestamps",
            BooleanValue(false),
            MakeBooleanAccessor (&simpleCAMSenderCEM::m_real_time),
            MakeBooleanChecker ())
        .AddAttribute ("TraCIClient",
            "TraCI client",
            PointerValue (0),
            MakePointerAccessor (&simpleCAMSenderCEM::m_traci_client),
            MakePointerChecker<TraciClient> ())
        .AddAttribute ("DisseminationDelay",
            "Delay in seconds after which the CAM and CEM dissemination should start",
            DoubleValue (0.0),
            MakeDoubleAccessor (&simpleCAMSenderCEM::m_dissemination_delay_seconds),
            MakeDoubleChecker<double> ())
        .AddAttribute ("IpAddr",
            "IpAddr",
            Ipv4AddressValue ("10.0.0.1"),
            MakeIpv4AddressAccessor (&simpleCAMSenderCEM::m_ipAddress),
            MakeIpv4AddressChecker ())
        .AddAttribute ("SendCEM",
            "To enable CEM dissemination",
            BooleanValue(true),
            MakeBooleanAccessor (&simpleCAMSenderCEM::m_send_cem),
            MakeBooleanChecker ())
        .AddAttribute ("TerminateAt",
            "Instant in time at which the dissemination shall be terminated (-1 = disabled)",
            DoubleValue (-1.0),
            MakeDoubleAccessor (&simpleCAMSenderCEM::m_terminate_at),
            MakeDoubleChecker<double> ())
        .AddAttribute ("Model",
            "Physical and MAC layer communication model",
            StringValue ("cv2x"),
            MakeStringAccessor (&simpleCAMSenderCEM::m_model),
            MakeStringChecker ());
        return tid;
  }

  simpleCAMSenderCEM::simpleCAMSenderCEM ()
  {
    NS_LOG_FUNCTION(this);
    m_traci_client = nullptr;
    m_gps_raw_tc_client = nullptr;
    m_app_stopped = false;

    m_cam_sent = 0;
    m_cem_sent = 0;

    m_rx_cem = 0;
    m_rx_cam = 0;

    m_csv_stream_ptr = nullptr;

    m_dissemination_delay_seconds = 0;
    m_send_cem=true;

    m_terminate_at = -1; // -1 means "do not terminate the dissemination unless the simulation has terminated"
    m_terminate_at_triggered = false; // This flag becomes "true" when a "terminate_at" event has been triggered and the dissemination of CAMs and CEMs has already been terminated
  }

  simpleCAMSenderCEM::~simpleCAMSenderCEM ()
  {
    NS_LOG_FUNCTION(this);
  }

  void
  simpleCAMSenderCEM::DoDispose (void)
  {
    NS_LOG_FUNCTION(this);
    Application::DoDispose ();
  }

  void
  simpleCAMSenderCEM::StartApplication (void)
  {

    /*
     * This application connects with the gps-tc module and simply generates CAM.
     * In this case, the position of the vehicle will arrive from a GPS trace placed
     * in src/gps-tc/examples/GPS-Traces-Sample, NOT FROM SUMO!
     */

    NS_LOG_FUNCTION(this);

    // Ensure that the mobility client has been set
    if(m_traci_client==nullptr)
    {
        NS_FATAL_ERROR("No mobility client specified in simpleCAMSender");
    }
    m_id = m_traci_client->GetVehicleId (this->GetNode ());
    std::cout << "veh_id:" << m_id << std::endl;

    // Create new BTP and GeoNet objects and set them in DENBasicService and CABasicService
    m_btp = CreateObject <btp>();
    m_geoNet = CreateObject <GeoNet>();
    m_btp->setGeoNet(m_geoNet);
    m_denService.setBTP(m_btp);
    m_caService.setBTP(m_btp);
    m_ceService.setBTP(m_btp);

    /* Create the socket for TX and RX */
    TypeId tid;
    if(m_model=="80211p")
      tid = TypeId::LookupByName ("ns3::PacketSocketFactory");
    else if(m_model=="cv2x")
      tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    else
      NS_FATAL_ERROR ("No communication model set - check simulation script - valid models: '80211p' or 'cv2x'");

    // TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    m_socket = Socket::CreateSocket (GetNode (), tid);

    if(m_model=="80211p")
    {
        /* Bind the socket to local address */
        PacketSocketAddress local = getGNAddress(GetNode ()->GetDevice (0)->GetIfIndex (),
                                                GetNode ()->GetDevice (0)->GetAddress () );
        if (m_socket->Bind (local) == -1)
        {
          NS_FATAL_ERROR ("Failed to bind client socket for BTP + GeoNetworking (802.11p)");
        }
        // Set the socketAddress for broadcast
        PacketSocketAddress remote = getGNAddress(GetNode ()->GetDevice (0)->GetIfIndex (),
                                                GetNode ()->GetDevice (0)->GetBroadcast () );
        m_socket->Connect (remote);
    }
    else if (m_model=="cv2x")
    {
      /* TX socket for CAMs and CEMs */
      /* Bind the socket to local address */
      if (m_socket->Bind (InetSocketAddress (Ipv4Address::GetAny (), 19)) == -1)
      {
        NS_FATAL_ERROR ("Failed to bind client socket for C-V2X");
      }
      m_socket->Connect (InetSocketAddress(m_ipAddress,19));
    }

    /* Set sockets, callback and station properties in DENBasicService */
    m_denService.setSocketTx (m_socket);
    m_denService.setSocketRx (m_socket);

    m_caService.setStationProperties (std::stol(m_id.substr (3)), StationType_passengerCar);
    m_denService.setStationProperties (std::stol(m_id.substr (3)), StationType_passengerCar);
    m_ceService.setStationProperties (std::stol(m_id.substr (3)), StationType_passengerCar);

    m_denService.addDENRxCallback (std::bind(&simpleCAMSenderCEM::receiveDENM,this,std::placeholders::_1,std::placeholders::_2));
    m_denService.setRealTime (m_real_time);

    /* Set sockets, callback, station properties in CABasicService */
    m_caService.setSocketTx (m_socket);
    m_caService.setSocketRx (m_socket);
    m_caService.addCARxCallback (std::bind(&simpleCAMSenderCEM::receiveCAM,this,std::placeholders::_1,std::placeholders::_2));
    m_caService.setRealTime (m_real_time);

    /* Set sockets, callback, station properties in CEBasicService */
    m_ceService.setRealTime (m_real_time);
    m_ceService.setSocketRx (m_socket);
    m_ceService.setSocketTx (m_socket);
    m_ceService.addCERxCallback (std::bind(&simpleCAMSenderCEM::receiveCEM,this,std::placeholders::_1,std::placeholders::_2));

    VDP* traci_vdp = new VDPTraCI(m_traci_client,m_id);
    m_caService.setVDP(traci_vdp);
    m_denService.setVDP(traci_vdp);

    // Set the raw GNSS Data Trace client in the CE Basic Service object
    m_ceService.setGPSRawTraceClient(m_gps_raw_tc_client);

    /* Schedule CAM dissemination */
    std::srand(std::hash<std::string>()(m_id)+Simulator::Now().GetNanoSeconds ());
    double desync = ((double)std::rand()/RAND_MAX);

    m_caService.startCamDissemination(m_dissemination_delay_seconds+desync);

    desync = ((double)std::rand()/RAND_MAX);

    /* Schedule CEM dissemination (which will also start, in turn, the GPS Raw Trace Client Data dissemination */
    if (m_send_cem)
    {
        m_ceService.startCemDissemination (m_dissemination_delay_seconds+(desync));
    }


    if(m_terminate_at > 0)
    {
        // Schedule() scheduled AFTER some time, not AT some time, thus to terminate at a specific time, we need to use "Seconds (m_terminate_at) - Simulator::Now"
        m_event_terminate_at = Simulator::Schedule (Seconds (m_terminate_at) - Simulator::Now(), &simpleCAMSenderCEM::stopDissemination, this);
    }
  }

  void
  simpleCAMSenderCEM::stopDissemination()
  {
    if(m_terminate_at_triggered == false)
    {
        m_gps_raw_tc_client->StopUpdates ();

        m_cam_sent = m_caService.terminateDissemination (m_cam_bytes_tx,m_cam_bytes_rx);
        m_cem_sent = m_ceService.terminateDissemination (m_cem_bytes_tx,m_cem_bytes_rx);

        m_terminate_at_triggered = true;
    }

    std::cout<<"Dissemination for vehicle "<<m_id<<" terminated."<<std::endl;
  }

  void
  simpleCAMSenderCEM::StopApplication ()
  {
    NS_LOG_FUNCTION(this);

    if(m_app_stopped == false)
    {
      if(m_terminate_at_triggered == false)
      {
          m_gps_raw_tc_client->StopUpdates ();

          m_cam_sent = m_caService.terminateDissemination (m_cam_bytes_tx,m_cam_bytes_rx);
          m_cem_sent = m_ceService.terminateDissemination (m_cem_bytes_tx,m_cem_bytes_rx);

          m_terminate_at_triggered = true; // Set this to true just to be sure to never call terminateDissemination() twice
      }
      else
      {
         m_cam_bytes_rx = m_caService.getBytesRx();
         m_cem_bytes_rx = m_ceService.getBytesRx();
      }

      std::cout << "Vehicle " << m_id
                << " has sent " << m_cam_sent
                << " CAMs and " << m_cem_sent
                << " CEMs" << std::endl;

      std::cout << "Vehicle " << m_id
                << " has received " << m_rx_cem << " CEMs and " << m_rx_cam << " CAMs." << std::endl;

      std::cout << "Vehicle " << m_id
                << " has sent " << m_cam_bytes_tx
                << " bytes [CAM] and " << m_cem_bytes_tx
                << " bytes [CEM]" << std::endl;

      std::cout << "Vehicle " << m_id
                << " has received " << m_cam_bytes_rx << " bytes [CAMs] and " << m_cem_bytes_rx << " bytes [CEMs]." << std::endl;

      if(m_csv_stream_ptr!=nullptr)
      {
          (*m_csv_stream_ptr) << m_id << "," << m_rx_cam << "," << m_cam_sent << "," << m_cam_bytes_tx << "," << m_cam_bytes_rx << ","
                              << m_rx_cem << "," << m_cem_sent << "," << m_cem_bytes_tx << "," << m_cem_bytes_rx << std::endl;
      }

      if(m_terminate_at > 0)
      {
         Simulator::Remove (m_event_terminate_at);
      }
    }

    m_app_stopped = true;
  }

  void
  simpleCAMSenderCEM::StopApplicationNow ()
  {
    NS_LOG_FUNCTION(this);
    StopApplication ();
  }

  void
  simpleCAMSenderCEM::receiveCAM (CAM_t *cam, Address from)
  {
    /* Implement CAM strategy here */
    std::cout <<"VehicleID: " << m_id
            <<" | Rx CAM from "<<cam->header.stationID
            <<" | Remote vehicle position: ("<<(double)cam->cam.camParameters.basicContainer.referencePosition.latitude/DOT_ONE_MICRO<<","
            <<(double)cam->cam.camParameters.basicContainer.referencePosition.longitude/DOT_ONE_MICRO<<")"<<std::endl;

    m_rx_cam++;

    // Free the received CAM data structure
    ASN_STRUCT_FREE(asn_DEF_CAM,cam);
  }

  void
  simpleCAMSenderCEM::receiveCEM (CEM_t *cem, Address from)
  {
    std::cout << "N applications:" << GetNode()->GetNApplications () << std::endl;

    /* Implement CEM strategy here */
    std::cout << "[ " << m_id << "] Received a new CEM from " << cem->header.stationID << ". Type: ";

    if(cem->cem.present == CoopEnhancement_PR_fps)
    {
        std::cout << "I frame" << std::endl;
    }
    else if(cem->cem.present == CoopEnhancement_PR_dmf)
    {
        std::cout << "D frame" << std::endl;
    }
    else
    {
        std::cout << "unknown (error occurred)" << std::endl;
    }

    m_rx_cem++;

    // Free the received CEM data structure
    ASN_STRUCT_FREE(asn_DEF_CEM,cem);
  }

  void
  simpleCAMSenderCEM::receiveDENM (denData denm, Address from)
  {
    /* This is just a sample dummy receiveDENM function. The user can customize it to parse the content of a DENM when it is received. */
    (void) denm; // Contains the data received from the DENM
    (void) from; // Contains the address from which the DENM has been received
    std::cout<<"Received a new DENM."<<std::endl;
  }

}



