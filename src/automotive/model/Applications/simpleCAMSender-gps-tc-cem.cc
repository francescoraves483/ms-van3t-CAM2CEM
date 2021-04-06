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
        .AddAttribute ("GPSClient",
            "GPS-TC client",
            PointerValue (0),
            MakePointerAccessor (&simpleCAMSenderCEM::m_gps_tc_client),
            MakePointerChecker<GPSTraceClient> ())
        .AddAttribute ("GPSRawClient",
            "Raw GNSS Data Trace Client (GPS-Raw-TC)",
            PointerValue (0),
            MakePointerAccessor (&simpleCAMSenderCEM::m_gps_raw_tc_client),
            MakePointerChecker<GPSRawTraceClient> ());
        return tid;
  }

  simpleCAMSenderCEM::simpleCAMSenderCEM ()
  {
    NS_LOG_FUNCTION(this);
    m_gps_tc_client = nullptr;
    m_gps_raw_tc_client = nullptr;
    m_app_stopped = false;

    m_cam_sent = 0;
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
    if(m_gps_tc_client==nullptr)
    {
        NS_FATAL_ERROR("No mobility client specified in simpleCAMSender");
    }
    m_id = m_gps_tc_client->getID();

    // Create new BTP and GeoNet objects and set them in DENBasicService and CABasicService
    m_btp = CreateObject <btp>();
    m_geoNet = CreateObject <GeoNet>();
    m_btp->setGeoNet(m_geoNet);
    m_denService.setBTP(m_btp);
    m_caService.setBTP(m_btp);

    /* Create the socket for TX and RX */
    TypeId tid = TypeId::LookupByName ("ns3::PacketSocketFactory");
    m_socket = Socket::CreateSocket (GetNode (), tid);

    /* TX socket for CAMs and CEMs */
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

    /* Set sockets, callback and station properties in DENBasicService */
    m_denService.setSocketTx (m_socket);
    m_denService.setSocketRx (m_socket);

    m_caService.setStationProperties (std::stol(m_id), StationType_passengerCar);
    m_denService.setStationProperties (std::stol(m_id), StationType_passengerCar);
    m_ceService.setStationProperties (std::stol(m_id), StationType_passengerCar);

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

    VDP* gpstc_vdp = new VDPGPSTraceClient(m_gps_tc_client,m_id);
    m_caService.setVDP(gpstc_vdp);
    m_denService.setVDP(gpstc_vdp);

    // Set the raw GNSS Data Trace client in the CE Basic Service object
    m_ceService.setGPSRawTraceClient(m_gps_raw_tc_client);

    /* Schedule CAM dissemination */
    std::srand(Simulator::Now().GetNanoSeconds ());
    double desync = ((double)std::rand()/RAND_MAX);
    m_caService.startCamDissemination(desync);

    /* Schedule CEM dissemination (which will also start, in turn, the GPS Raw Trace Client Data dissemination */
    m_ceService.startCemDissemination ();
  }

  void
  simpleCAMSenderCEM::StopApplication ()
  {
    NS_LOG_FUNCTION(this);

    if(m_app_stopped == false)
    {
      uint64_t cam_sent, cem_sent;

      m_gps_tc_client->StopUpdates();
      m_gps_raw_tc_client->StopUpdates ();

      cam_sent = m_caService.terminateDissemination ();
      cem_sent = m_ceService.terminateDissemination ();

      std::cout << "Vehicle " << m_id
                << " has sent " << cam_sent
                << " CAMs and " << cem_sent
                << " CEMs" << std::endl;
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

    // Free the received CAM data structure
    ASN_STRUCT_FREE(asn_DEF_CAM,cam);
  }

  void
  simpleCAMSenderCEM::receiveCEM (CEM_t *cem, Address from)
  {
    /* Implement CEM strategy here */
    std::cout << "Received a new CEM. Type: ";

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

    // Free the received CEM data structure
    ASN_STRUCT_FREE(asn_DEF_CAM,cem);
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



