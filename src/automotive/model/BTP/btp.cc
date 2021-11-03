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

 * Edited by Carlos Risma, Politecnico di Torino
 * (carlosrisma@gmail.com)
*/
#include "btp.h"


namespace ns3
{
  NS_LOG_COMPONENT_DEFINE("btp");

  TypeId
  btp::GetTypeId ()
  {
    static TypeId tid = TypeId("ns3::btp")
        .SetParent <Object>()
        .AddConstructor <btp>();
    return tid;
  }

  btp::~btp ()
  {
    NS_LOG_FUNCTION(this);
  }

  btp::btp()
  {
    m_geonet = NULL;
  }

  void
  btp::cleanup ()
  {
    // Cleanup the GeoNet object
    m_geonet->cleanup();
  }

  void
  btp::setSocketRx (Ptr<Socket> socket_rx)
  {
    socket_rx->SetRecvCallback (MakeCallback(&GeoNet::receiveGN, m_geonet));
    m_geonet->addRxCallback(std::bind(&btp::receiveBTP,this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3));
  }
  
/*  void
  btp::setSocketRxPkt (Ptr<Socket> socket_rx)
  {
    socket_rx->SetRecvCallback (MakeCallback(&GeoNet::receiveGN, m_geonet));
    m_geonet->addRxCallbackPkt(std::bind(static_cast<void(btp::*)(GNDataIndication_t,Address,Ptr<Packet>)>(&btp::receiveBTP),this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4));
  }*/

  int
  btp::sendBTP(BTPDataRequest_t dataRequest)
  {
    GNDataConfirm_t dataConfirm;
    GNDataRequest_t GnDataRequest = {};
    btpHeader header;
    int numbytes;
    header.SetDestinationPort (dataRequest.destPort);

    if(dataRequest.BTPType==BTP_A) // BTP-A
    {
      header.SetSourcePort (dataRequest.sourcePort);
      GnDataRequest.upperProtocol = BTP_A;
    }
    else    //  BTP-B
    {
      header.SetDestinationPortInfo (dataRequest.destPInfo);
      GnDataRequest.upperProtocol = BTP_B;
    }
    dataRequest.data->AddHeader (header);

    //Filling the GN-dataRequest
    GnDataRequest.GNType = dataRequest.GNType;
    GnDataRequest.GnAddress = dataRequest.GnAddress;
    GnDataRequest.GNCommProfile = dataRequest.GNCommProfile;
    GnDataRequest.GNRepInt = dataRequest.GNRepInt;
    GnDataRequest.GNMaxRepTime = dataRequest.GNMaxRepInt;
    GnDataRequest.GNMaxLife = dataRequest.GNMaxLife;
    GnDataRequest.GNMaxHL = dataRequest.GNMaxHL;
    GnDataRequest.GNTraClass = dataRequest.GNTraClass;
    GnDataRequest.data = dataRequest.data;
    GnDataRequest.lenght = dataRequest.lenght + 4;

    dataConfirm = m_geonet->sendGN(GnDataRequest,numbytes);
    if(dataConfirm != ACCEPTED)
    {
      NS_LOG_ERROR("GeoNet can't send packet. Error code: " << dataConfirm);
    }

    return numbytes;
  }

  void
  btp::receiveBTP(GNDataIndication_t dataIndication,Address address,uint32_t originalPacketSize)
  {
    btpHeader header;
    BTPDataIndication_t btpDataIndication = {};
    dataIndication.data->RemoveHeader (header, 4);

    btpDataIndication.BTPType = dataIndication.upperProtocol;
    btpDataIndication.destPort = header.GetDestinationPort ();
    if(btpDataIndication.BTPType == BTP_A)
    {
      btpDataIndication.sourcePort = header.GetSourcePort ();
      btpDataIndication.destPInfo = 0;
    }
    else //BTP-B
    {
      btpDataIndication.destPInfo = header.GetDestinationPortInfo ();
      btpDataIndication.sourcePort = 0;
    }
    btpDataIndication.GnAddress = dataIndication.GnAddressDest;
    btpDataIndication.GNTraClass = dataIndication.GNTraClass;
    btpDataIndication.GNRemPLife = dataIndication.GNRemainingLife;
    btpDataIndication.GNPositionV = dataIndication.SourcePV;
    btpDataIndication.data = dataIndication.data;
    btpDataIndication.lenght = dataIndication.data->GetSize ();

    if(btpDataIndication.destPort == CA_PORT)
      m_cam_ReceiveCallback(btpDataIndication,address,originalPacketSize);
    else if(btpDataIndication.destPort == DEN_PORT)
      m_denm_ReceiveCallback(btpDataIndication,address,originalPacketSize);
    else if(btpDataIndication.destPort == CE_PORT)
      m_cem_ReceiveCallback(btpDataIndication,address,originalPacketSize);
    else
      NS_LOG_ERROR("BTP : Unknown port");
  }

/*  void
  btp::receiveBTP(GNDataIndication_t dataIndication,Address address,Ptr<Packet> pkt)
  {
    btpHeader header;
    BTPDataIndication_t btpDataIndication = {};
    dataIndication.data->RemoveHeader (header, 4);

    btpDataIndication.BTPType = dataIndication.upperProtocol;
    btpDataIndication.destPort = header.GetDestinationPort ();
    if(btpDataIndication.BTPType == BTP_A)
    {
      btpDataIndication.sourcePort = header.GetSourcePort ();
      btpDataIndication.destPInfo = 0;
    }
    else //BTP-B
    {
      btpDataIndication.destPInfo = header.GetDestinationPortInfo ();
      btpDataIndication.sourcePort = 0;
    }
    btpDataIndication.GnAddress = dataIndication.GnAddressDest;
    btpDataIndication.GNTraClass = dataIndication.GNTraClass;
    btpDataIndication.GNRemPLife = dataIndication.GNRemainingLife;
    btpDataIndication.GNPositionV = dataIndication.SourcePV;
    btpDataIndication.data = dataIndication.data;
    btpDataIndication.lenght = dataIndication.data->GetSize ();

    if(btpDataIndication.destPort == CA_PORT)
      m_cam_ReceiveCallbackPkt(btpDataIndication,address,pkt);
    else if(btpDataIndication.destPort == DEN_PORT)
      m_denm_ReceiveCallback(btpDataIndication,address);
    else
      NS_LOG_ERROR("BTP : Unknown port");
  }*/
}
