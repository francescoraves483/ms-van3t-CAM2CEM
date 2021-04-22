#ifndef SIMPLECAMSENDERCEM_H
#define SIMPLECAMSENDERCEM_H

#include "ns3/gps-tc.h"
#include "ns3/traci-client.h"
#include "ns3/application.h"
#include "ns3/asn_utils.h"

#include "ns3/denBasicService.h"
#include "ns3/caBasicService.h"
#include "ns3/ceBasicService.h"


namespace ns3 {

class simpleCAMSenderCEM : public Application
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  simpleCAMSenderCEM ();

  virtual ~simpleCAMSenderCEM ();

  void receiveCAM(CAM_t *cam, Address address);
  void receiveDENM(denData denm, Address from);
  void receiveCEM (CEM_t *cem, Address from);

  void StopApplicationNow ();

protected:
  virtual void DoDispose (void);

private:

  DENBasicService m_denService; //!< DEN Basic Service object
  CABasicService m_caService; //!< CA Basic Service object
  CEBasicService m_ceService; //!< CE Basic Service object
  Ptr<btp> m_btp; //! BTP object
  Ptr<GeoNet> m_geoNet; //! GeoNetworking Object

  Ptr<Socket> m_socket;

  virtual void StartApplication (void);
  virtual void StopApplication (void);

  Ptr<TraciClient> m_traci_client; //!< TraCI SUMO client
  Ptr<GPSRawTraceClient> m_gps_raw_tc_client; //!< Raw GNSS Data Trace client

  std::string m_id; //!< vehicle id
  bool m_real_time; //!< To decide wheter to use realtime scheduler

  EventId m_sendCamEvent; //!< Event to send the CAM

  /* Counters */
  int m_cam_sent;

  bool m_app_stopped;

  int m_rx_cem;
  int m_rx_cam;
};

} // namespace ns3

#endif /* SIMPLECAMSENDERCEM_H */

