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
 *  Marco Malinverno, Politecnico di Torino (marco.malinverno1@gmail.com)
 *  Francesco Raviglione, Politecnico di Torino (francescorav.es483@gmail.com)
 *  Giuseppe Avino, Politecnico di Torino (giuseppe.avino@polito.it)
*/

#include "ceBasicService.h"
#include "ns3/ItsPduHeader.h"

namespace ns3
{
  NS_LOG_COMPONENT_DEFINE("CEBasicService");

  CEBasicService::CEBasicService()
  {
    m_station_id = ULONG_MAX;
    m_socket_tx=NULL;
    m_btp = NULL;
    m_real_time=false;

    m_cem_sent=0;

    m_dissemination_started=false;

    m_fullPrecisionID = 0;
    m_differentialID = 0;
  }

  CEBasicService::CEBasicService(unsigned long fixed_stationid,long fixed_stationtype,bool real_time)
  {
    CEBasicService();
    m_station_id = (StationID_t) fixed_stationid;
    m_stationtype = (StationType_t) fixed_stationtype;
//    m_vdp=vdp;
//    m_gdp=gdp;
    m_real_time=real_time;
  }

  CEBasicService::CEBasicService(unsigned long fixed_stationid,long fixed_stationtype,bool real_time,Ptr<Socket> socket_tx)
  {
    CEBasicService(fixed_stationid,fixed_stationtype,real_time);

    m_socket_tx=socket_tx;
  }

  void
  CEBasicService::setStationProperties(unsigned long fixed_stationid,long fixed_stationtype)
  {
    m_station_id=fixed_stationid;
    m_stationtype=fixed_stationtype;
    m_btp->setStationProperties(fixed_stationid,fixed_stationtype);
  }

  void
  CEBasicService::setFixedPositionRSU(double latitude_deg, double longitude_deg)
  {
    m_btp->setFixedPositionRSU(latitude_deg,longitude_deg);
  }

  void
  CEBasicService::setStationID(unsigned long fixed_stationid)
  {
    m_station_id=fixed_stationid;
    m_btp->setStationID(fixed_stationid);
  }

  void
  CEBasicService::setStationType(long fixed_stationtype)
  {
    m_stationtype=fixed_stationtype;
    m_btp->setStationType(fixed_stationtype);
  }

  void
  CEBasicService::setSocketRx (Ptr<Socket> socket_rx)
  {
    m_btp->setSocketRx(socket_rx);
    m_btp->addCEMRxCallback (std::bind(&CEBasicService::receiveCem,this,std::placeholders::_1,std::placeholders::_2));
  }

  void
  CEBasicService::startCemDissemination()
  {
    if(m_dissemination_started == true)
    {
      NS_FATAL_ERROR("Error: attempting to start an already started CEM dissemination");
    }

    m_gps_raw_trace_client->playTrace (Seconds(0));
    m_dissemination_started = true;
  }

  void
  CEBasicService::startCemDissemination(double desync_s)
  {
    if(m_dissemination_started == true)
    {
      NS_FATAL_ERROR("Error: attempting to start an already started CEM dissemination");
    }

    m_gps_raw_trace_client->playTrace (Seconds(desync_s));
    m_dissemination_started = true;
  }

  void
  CEBasicService::frameCallback(GPSRawTraceClient::raw_positioning_data_t raw_frame_data)
  {
    generateAndEncodeCem(raw_frame_data);
  }

  void
  CEBasicService::receiveCem (BTPDataIndication_t dataIndication, Address from)
  {
    Ptr<Packet> packet;
    CEM_t *decoded_cem;

    packet = dataIndication.data;
    uint8_t *buffer; //= new uint8_t[packet->GetSize ()];
    buffer=(uint8_t *)malloc((packet->GetSize ())*sizeof(uint8_t));
    packet->CopyData (buffer, packet->GetSize ());

    /* Try to check if the received packet is really a CEM */
    if (buffer[1]!=FIX_CEMID)
      {
        NS_LOG_ERROR("Warning: received a message which has messageID '"<<buffer[1]<<"' but '200' was expected.");
        free(buffer);
        return;
      }

    /** Decoding **/
    void *decoded_=NULL;
    asn_dec_rval_t decode_result;

    //do {
      decode_result = asn_decode(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_CEM, &decoded_, buffer, packet->GetSize ());
    //} while(decode_result.code==RC_WMORE);

    free(buffer);

    if(decode_result.code!=RC_OK || decoded_==NULL) {
        NS_LOG_ERROR("Warning: unable to decode a received CAM.");
        if(decoded_) free(decoded_);
        return;
      }

    decoded_cem = (CEM_t *) decoded_;

    m_CEReceiveCallback(decoded_cem,from);
  }

  CEBasicService_error_t
  CEBasicService::generateAndEncodeCem(GPSRawTraceClient::raw_positioning_data_t rawdata)
  {
    CEM_t *cem;
    CEBasicService_error_t errval=CEM_NO_ERROR;

    Ptr<Packet> packet;

    BTPDataRequest_t dataRequest = {};

    asn_encode_to_new_buffer_result_t encode_result={.buffer=NULL};

    cem=(CEM_t*) calloc(1, sizeof(CEM_t));
    if(cem==NULL)
    {
      return CEM_ALLOC_ERROR;
    }

    /* Fill the header */
    cem->header.messageID = messageID_cem;
    cem->header.protocolVersion = protocolVersion_currentVersion;
    cem->header.stationID = m_station_id;

    /* Construct CAM and pass it to the lower layers (now UDP, in the future BTP and GeoNetworking, then UDP) */
    // CONVERSIONS ARE STILL MISSING
    if(rawdata.type == GPSRawTraceClient::FULL_PRECISION_I_FRAME)
    {
        // Full precision ID
        cem->cem.choice.fps.fullPrecisionID = m_fullPrecisionID;

        m_fullPrecisionID = (m_fullPrecisionID + 1) % (m_fullPrecisionIDMax + 1);
        m_differentialID = 0;

        // CEM timestamp
        INTEGER_t cemTstamp_I;
        memset(&cemTstamp_I, 0, sizeof(INTEGER_t));
        asn_imax2INTEGER(&cemTstamp_I, rawdata.cemTstamp);
        cem->cem.choice.fps.cemTstamp = cemTstamp_I;

        for (auto const& x : rawdata.iframe_data)
        {
            SatelliteSignalInfo_t *ssi = reinterpret_cast<SatelliteSignalInfo_t *>(calloc(1, sizeof(SatelliteSignalInfo_t *)));

            // Fill in the satellite signal info data for this satellite and signal ...

            // Mandatory Container
            ssi->mandatoryContainer.signalID = x.first.second;
            ssi->mandatoryContainer.satellitePRN = x.first.first;

            INTEGER_t pseudorange_I;
            memset(&pseudorange_I, 0, sizeof(INTEGER_t));
            asn_imax2INTEGER(&pseudorange_I, x.second.pseudorange);
            ssi->mandatoryContainer.pseudorange = pseudorange_I;

            // Optional Container
            ssi->optionalContainer = (OptionalContainer_t *)calloc(1,sizeof(OptionalContainer_t));
            ssi->optionalContainer->doppler = (Doppler_t) x.second.doppler;

            INTEGER_t carrierphase_I;
            memset(&carrierphase_I, 0, sizeof(INTEGER_t));
            asn_imax2INTEGER(&carrierphase_I, x.second.carrierphase);
            ssi->optionalContainer->carrierPhase = carrierphase_I;

            ssi->optionalContainer->signalStrength = (SignalStrength_t) x.second.signalstrength;

            // Uncertainty Container
            // This container is inserted only if at least one uncertainty value is available
            if(x.second.doppler_uncertainty != GPSDataUncertainty_unavailable ||
               x.second.pseudorange_uncertainty != GPSDataUncertainty_unavailable ||
               x.second.carrierphase_uncertainty != GPSDataUncertainty_unavailable) {

                ssi->uncertaintyContainer = (UncertaintyContainer_t *)calloc(1,sizeof(UncertaintyContainer_t));

                ssi->uncertaintyContainer->dopplerUncertainty = (GPSDataUncertainty_t) x.second.doppler_uncertainty;
                ssi->uncertaintyContainer->pseudorangeUncertainty = (GPSDataUncertainty_t) x.second.pseudorange_uncertainty;
                ssi->uncertaintyContainer->carrierPhaseUncertainty = (GPSDataUncertainty_t) x.second.carrierphase_uncertainty;
            }

            ASN_SEQUENCE_ADD(&cem->cem.choice.fps.satelliteSignalInfo, ssi);
        }

    }
    else if(rawdata.type == GPSRawTraceClient::DIFFERENTIAL_D_FRAME)
    {
        // Full precision ID (i.e. ID of the corresponding "I" frame)
        cem->cem.choice.dmf.fullPrecisionID = m_fullPrecisionID;

        // Differential ID
        cem->cem.choice.dmf.differentialID = m_differentialID;

        m_differentialID = (m_differentialID + 1) % (m_differentialIDMax + 1);

        // CEM timestamp
        INTEGER_t cemTstamp_I;
        memset(&cemTstamp_I, 0, sizeof(INTEGER_t));
        asn_imax2INTEGER(&cemTstamp_I, rawdata.cemTstamp);
        cem->cem.choice.dmf.cemTstamp = cemTstamp_I;

        for (auto const& x : rawdata.dframe_data)
        {
            DifferentialSatelliteSignalInfo *dssi = reinterpret_cast<DifferentialSatelliteSignalInfo *>(calloc(1, sizeof(DifferentialSatelliteSignalInfo *)));

            // Differential Mandatory Container
            dssi->differentialMandatoryContainer.differentialPseudorange = (DifferentialPseudorange_t) x.second.differential_pseudorange;

            // Differential Optional Container
            dssi->differentialOptionalContainer = (DifferentialOptionalContainer_t *)calloc(1,sizeof(DifferentialOptionalContainer_t));
            dssi->differentialOptionalContainer->differentialDoppler = (DifferentialDoppler_t) x.second.differential_doppler;
            dssi->differentialOptionalContainer->differentialCarrierPhase = (DifferentialCarrierPhase_t) x.second.differential_carrierphase;

            ASN_SEQUENCE_ADD(&cem->cem.choice.dmf.differentialSatelliteSignalInfo, dssi);
        }
    }

    /** Encoding **/
    char errbuff[ERRORBUFF_LEN];
    size_t errlen=sizeof(errbuff);

    if(asn_check_constraints(&asn_DEF_CEM,(CEM_t *)cem,errbuff,&errlen) == -1) {
        NS_LOG_ERROR("Unable to validate the ASN.1 contraints for the current CEM."<<std::endl);
        NS_LOG_ERROR("Details: " << errbuff << std::endl);
        errval=CEM_ASN1_UPER_ENC_ERROR;
        goto error;
    }

    encode_result = asn_encode_to_new_buffer(NULL,ATS_UNALIGNED_BASIC_PER,&asn_DEF_CEM, cem);
    if (encode_result.result.encoded==-1)
      {
        errval=CEM_ASN1_UPER_ENC_ERROR;
        if(encode_result.buffer) free(encode_result.buffer);
        goto error;
      }

    packet = Create<Packet> ((uint8_t*) encode_result.buffer, encode_result.result.encoded);

    dataRequest.BTPType = BTP_B; //!< BTP-B
    dataRequest.destPort = CE_PORT;
    dataRequest.destPInfo = 0;
    dataRequest.GNType = TSB;
    dataRequest.GNCommProfile = UNSPECIFIED;
    dataRequest.GNRepInt =0;
    dataRequest.GNMaxRepInt=0;
    dataRequest.GNMaxLife = 1;
    dataRequest.GNMaxHL = 1;
    dataRequest.GNTraClass = 0x02; // Store carry foward: no - Channel offload: no - Traffic Class ID: 2
    dataRequest.lenght = packet->GetSize ();
    dataRequest.data = packet;
    m_btp->sendBTP(dataRequest);

    m_cem_sent++;

    error:
    if(encode_result.buffer) free(encode_result.buffer);

    // Free all the previously allocated memory
    // This is IMPORTANT! Do not forget to perform this operation...

    if(cem) free(cem);

    return errval;
  }

  uint64_t
  CEBasicService::terminateDissemination()
  {
    Simulator::Remove(m_event_cemDisseminationStart);
    return m_cem_sent;
  }
}
