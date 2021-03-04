#ifndef CEBASICSERVICE_H
#define CEBASICSERVICE_H

#include "ns3/socket.h"
#include "ns3/core-module.h"
#include "ns3/vdp.h"
#include "ns3/gdp.h"
#include "ns3/asn_utils.h"
#include "ns3/btp.h"
#include "ns3/btpHeader.h"

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
    CEBasicService(unsigned long fixed_stationid,long fixed_stationtype,VDP* vdp,GDP *gdp,bool real_time);
    CEBasicService(unsigned long fixed_stationid,long fixed_stationtype,VDP* vdp,GDP *gdp,bool real_time,Ptr<Socket> socket_tx);

    void setGDP(GDP *gdp) {m_gdp=gdp;}

    void setStationProperties(unsigned long fixed_stationid,long fixed_stationtype);
    void setFixedPositionRSU(double latitude_deg, double longitude_deg);
    void setStationID(unsigned long fixed_stationid);
    void setStationType(long fixed_stationtype);
    void setSocketTx(Ptr<Socket> socket_tx) {m_btp->setSocketTx (socket_tx);}
    void setSocketRx(Ptr<Socket> socket_rx);
    void setVDP(VDP* vdp) {m_vdp=vdp;}
    void setBTP(Ptr<btp> btp){m_btp=btp;}

    void receiveCem(BTPDataIndication_t dataIndication, Address from);
    void addCERxCallback(std::function<void(CEM_t *, Address)> rx_callback) {m_CEReceiveCallback=rx_callback;}
    void setRealTime(bool real_time){m_real_time=real_time;}

    void startCemDissemination();
    void startCemDissemination(double desync_s);

    uint64_t terminateDissemination();

  private:
    const int m_T_FixedGenCemHz=10; // CEM generation frequency ([Hz])
    double m_T_FixedGenCemMs; // Computed by CE Basic Service from m_T_FixedGenCemHz

    void initDissemination();
    CEBasicService_error_t generateAndEncodeCem();

    std::function<void(CEM_t *, Address)> m_CEReceiveCallback;

    Ptr<btp> m_btp;

    bool m_real_time;
    VDP* m_vdp; // Vehicle Data Provider (GN)
    GDP* m_gdp; // GNSS Data Provider (CEM)

    Ptr<Socket> m_socket_tx; // Socket TX

    StationID_t m_station_id;
    StationType_t m_stationtype;

    // Statistic: number of CEMs successfully sent since the CE Basic Service has been started
    // The CE Basic Service can count up to 18446744073709551615 (UINT64_MAX) CEMs
    uint64_t m_cem_sent;

    // ns-3 event IDs used to properly stop the simulation with terminateDissemination()
    EventId m_event_cemDisseminationStart;


  };
}

#endif // CEBASICSERVICE_H
