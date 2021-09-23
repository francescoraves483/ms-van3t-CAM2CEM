#ifndef CEBASICSERVICE_H
#define CEBASICSERVICE_H

#include "ns3/socket.h"
#include "ns3/core-module.h"
#include "ns3/vdp.h"
#include "ns3/gdp.h"
#include "ns3/asn_utils.h"
#include "ns3/btp.h"
#include "ns3/btpHeader.h"
#include "ns3/gps-raw-tc.h"

namespace ns3
{
  typedef enum {
    CEM_NO_ERROR=0,
    CEM_WRONG_INTERVAL=1,
    CEM_ALLOC_ERROR=2,
    CEM_NO_RSU_CONTAINER=3,
    CEM_ASN1_UPER_ENC_ERROR=4,
    CEM_CANNOT_SEND=5
  } CEBasicService_error_t;

  class CEBasicService
  {
  public:
    CEBasicService();
    CEBasicService(unsigned long fixed_stationid, long fixed_stationtype, bool real_time);
    CEBasicService(unsigned long fixed_stationid, long fixed_stationtype, bool real_time, Ptr<Socket> socket_tx);

    // void setGDP(GDP *gdp) {m_gdp=gdp;}

    void setStationProperties(unsigned long fixed_stationid,long fixed_stationtype);
    void setFixedPositionRSU(double latitude_deg, double longitude_deg);
    void setStationID(unsigned long fixed_stationid);
    void setStationType(long fixed_stationtype);
    void setSocketTx(Ptr<Socket> socket_tx) {m_btp->setSocketTx (socket_tx);}
    void setSocketRx(Ptr<Socket> socket_rx);
    // void setVDP(VDP* vdp) {m_vdp=vdp;}
    void setGPSRawTraceClient(std::shared_ptr<GPSRawTraceClient> gps_raw_trace_client);
    void setBTP(Ptr<btp> btp){m_btp=btp;}

    void receiveCem(BTPDataIndication_t dataIndication, Address from, uint32_t originalPacketSize);
    void addCERxCallback(std::function<void(CEM_t *, Address)> rx_callback) {m_CEReceiveCallback=rx_callback;}
    void setRealTime(bool real_time){m_real_time=real_time;}

    uint64_t getBytesRx(){return m_bytes_received;}

    void startCemDissemination();
    void startCemDissemination(double desync_s);

    uint64_t terminateDissemination();
    uint64_t terminateDissemination(uint64_t &bytes_tx,uint64_t &bytes_rx);

  private:
    const int m_fullPrecisionIDMax = 65535;
    const int m_differentialIDMax = 8;
    const double cemTimestampMultiplier = 1e10;
    const long pseudoRangeMultiplier = 1e2;
    const long carrierPhaseMultiplier = 1e3;
    const long differentialPseudoRangeMultiplier = 1e2;
    const long signalStrenghtUpperLim = 200;
    const long signalStrenghtLowerLim = 0;
    const unsigned long long pseudoRangeUpperLim = 1800000000;
    const unsigned long long pseudoRangeLowerLim = 2900000000;
    const unsigned long long carrierPhaseUpperLim = 70000000000;
    const unsigned long long carrierPhaseLowerLim = 160000000000;
    const long long differentialPseudoRangeUpperLim = -100000;
    const long long differentialPseudoRangeLowerLim = 100000;

    void frameCallback(GPSRawTraceClient::raw_positioning_data_t raw_frame_data);
    void freeCEM(CEM_t *cem);

    CEBasicService_error_t generateAndEncodeCem(GPSRawTraceClient::raw_positioning_data_t rawdata);

    std::function<void(CEM_t *, Address)> m_CEReceiveCallback;

    Ptr<btp> m_btp;

    bool m_real_time;
    // VDP* m_vdp; // Vehicle Data Provider (GN)
    // GDP* m_gdp; // GNSS Data Provider (CEM)
    bool m_dissemination_started;

    std::shared_ptr<GPSRawTraceClient> m_gps_raw_trace_client;

    Ptr<Socket> m_socket_tx; // Socket TX

    StationID_t m_station_id;
    StationType_t m_stationtype;

    // Statistic: number of CEMs successfully sent since the CE Basic Service has been started
    // The CE Basic Service can count up to 18446744073709551615 (UINT64_MAX) CEMs
    uint64_t m_cem_sent;

    uint64_t m_bytes_sent;
    uint64_t m_bytes_received;

    // ns-3 event IDs used to properly stop the simulation with terminateDissemination()
    EventId m_event_cemDisseminationStart;

    int m_fullPrecisionID;
    int m_differentialID;

    std::queue<void *> m_ptr_queue;
  };
}

#endif // CEBASICSERVICE_H
