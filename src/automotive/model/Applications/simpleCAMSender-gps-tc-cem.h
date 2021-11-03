#ifndef SIMPLECAMSENDERCEM_H
#define SIMPLECAMSENDERCEM_H

#include "ns3/gps-tc.h"
#include "ns3/traci-client.h"
#include "ns3/application.h"
#include "ns3/asn_utils.h"

#include "ns3/denBasicService.h"
#include "ns3/caBasicService.h"
#include "ns3/ceBasicService.h"
#include "ns3/PRRSupervisor.h"


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

  void setCSVOfstream(std::ofstream *ofptr) {m_csv_stream_ptr = ofptr;}
  void setRawTraceClient(std::shared_ptr<GPSRawTraceClient> &gps_raw_tc_client) {m_gps_raw_tc_client=gps_raw_tc_client;}

  void StopApplicationNow ();
  void stopDissemination();

protected:
  virtual void DoDispose (void);

private:
  DENBasicService m_denService; //!< DEN Basic Service object
  CABasicService m_caService; //!< CA Basic Service object
  CEBasicService m_ceService; //!< CE Basic Service object
  Ptr<btp> m_btp; //! BTP object
  Ptr<GeoNet> m_geoNet; //! GeoNetworking Object

  std::string m_model; //!< Communication Model (possible values: 80211p and cv2x)

  Ptr<Socket> m_socket;

  virtual void StartApplication (void);
  virtual void StopApplication (void);

  Ptr<TraciClient> m_traci_client; //!< TraCI SUMO client
  std::shared_ptr<GPSRawTraceClient> m_gps_raw_tc_client; //!< Raw GNSS Data Trace client

  std::string m_id; //!< vehicle id
  bool m_real_time; //!< To decide wheter to use realtime scheduler

  EventId m_sendCamEvent; //!< Event to send the CAM

  /* Counters */
  int m_cam_sent;
  int m_cem_sent;

  bool m_app_stopped;

  int m_rx_cem;
  int m_rx_cam;

  uint64_t m_cam_bytes_tx, m_cam_bytes_rx;
  uint64_t m_cem_bytes_tx, m_cem_bytes_rx;

  double m_dissemination_delay_seconds;
  double m_terminate_at;
  EventId m_event_terminate_at;

  bool m_terminate_at_triggered;

  // CSV file ofstream (the file should have been already opened before!)
  std::ofstream *m_csv_stream_ptr;

  Ipv4Address m_ipAddress; //!< C-V2X self IP address (set by 'v2v-cv2x.cc')
  bool m_send_cem;
  
  Ptr<PRRSupervisor> m_PRR_supervisor = nullptr;

};

} // namespace ns3

#endif /* SIMPLECAMSENDERCEM_H */

