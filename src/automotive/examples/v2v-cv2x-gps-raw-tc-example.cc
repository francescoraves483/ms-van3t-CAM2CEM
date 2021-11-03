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
*/


#include "ns3/automotive-module.h"
#include "ns3/gps-tc-module.h"
#include "ns3/gps-raw-tc-module.h"
#include "ns3/cv2x_lte-v2x-helper.h"
#include "ns3/config-store.h"
#include "ns3/internet-module.h"
#include "ns3/cv2x-module.h"
#include "ns3/mobility-module.h"
#include <ns3/node-list.h>

#include <ns3/flow-monitor.h>
#include <ns3/flow-monitor-helper.h>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("v2v-cv2x-gps-raw-tc-example");

int
main (int argc, char *argv[])
{
  /*
   * In this example instead of relying on SUMO, the vehicles' mobility will be managed by the "gps-tc" module.
   * "gps-tc" is a module that is able to load GPS traces in a specific CSV format, with few prerequisites, and use
   * them to simulate the presence of N vehicles from pre-recorded real GPS traces.
   * For the time being, the CSV format should be equal to the one available in "/src/gps-tc/examples/GPS-Traces-Sample" and
   * the positioning updates should occur, if possible at least at around 5-10 Hz (1 Hz would not be enough for an effective
   * dynamic frequency management for the CAMs).
   *
   * In this example the generated vehicles will broadcast CAMs and CEMs, using 802.11p.
   *
   * If the correct prerequisites for ns-3 PyViz are installed, the user can also see the moving nodes in a GUI by running this
   * example with the "--vis" option.
   */

  /*** 0.a App Options ***/
  std::string sumo_folder = "src/automotive/examples/sumo_files_v2i_map/";
  std::string mob_trace = "cars.rou.xml";
  std::string sumo_config ="src/automotive/examples/sumo_files_v2i_map/map.sumo.cfg";

  std::string raw_trace_file_path = "src/gps-raw-tc/trace-example/GPS-Raw-Traces-Sample/";
  std::string gps_raw_trace = "multi_constellation_CEM_v8.txt";

  std::string csv_prefix = "gps-raw-tc-log";

  bool verbose = false;
  bool realtime = false;

  bool sumo_gui = true;
  double sumo_updates = 0.01;

  double simTime = 100;
  double terminateAt = -1;

  int numberOfNodes;
  uint32_t nodeCounter = 0;

  double dissemination_delay = 0.0;
  
  std::string csv_name_cumulative;
  bool send_cem = true;
  double m_baseline_prr = 150.0;

  /*** 0.b LENA + V2X Options ***/
  double ueTxPower = 23.0;                // Transmission power in dBm
  double probResourceKeep = 0.4;          // Probability to select the previous resource again [0.0-0.8]
  uint32_t mcs = 20;                      // Modulation and Coding Scheme
  bool harqEnabled = false;               // Retransmission enabled (harq not available yet)
  bool adjacencyPscchPssch = true;        // Subchannelization scheme
  bool partialSensing = false;            // Partial sensing enabled (actual only partialSensing is false supported)
  uint16_t sizeSubchannel = 10;           // Number of RBs per subchannel
  uint16_t numSubchannel = 3;             // Number of subchannels per subframe
  uint16_t startRbSubchannel = 0;         // Index of first RB corresponding to subchannelization
  uint16_t pRsvp = 20;                    // Resource reservation interval
  uint16_t t1 = 4;                        // T1 value of selection window
  uint16_t t2 = 100;                      // T2 value of selection window
  uint16_t slBandwidth;                   // Sidelink bandwidth

  CommandLine cmd;

  xmlDocPtr rou_xml_file;

  /* Cmd Line option for application */
  cmd.AddValue ("realtime", "Use the realtime scheduler or not", realtime);
  cmd.AddValue ("sumo-gui", "Use SUMO gui or not", sumo_gui);
  cmd.AddValue ("sumo-updates", "SUMO granularity", sumo_updates);
  cmd.AddValue ("sumo-folder","Position of sumo config files",sumo_folder);
  cmd.AddValue ("mob-trace", "Name of the mobility trace file", mob_trace);
  cmd.AddValue ("sumo-config", "Location and name of SUMO configuration file", sumo_config);
  cmd.AddValue ("csv-prefix", "Prefix for the CSV file name for logging", csv_prefix);
  cmd.AddValue ("dissemination-delay", "Delay, in seconds, after which the CAM and CEM dissemination can start", dissemination_delay);
  cmd.AddValue ("dissemination-terminate-at", "Instant in time at which the dissemination shall be terminated (-1 = disabled, i.e. terminate when the simulation terminates)", terminateAt);
  cmd.AddValue ("send-cem", "If true, CEMs will be disseminated", send_cem);

  cmd.AddValue ("raw-trace-folder","Position of Raw GNSS data trace files",raw_trace_file_path);
  cmd.AddValue ("gps-raw-trace", "Name of the Raw GNSS Data trace file", gps_raw_trace);
  
  cmd.AddValue ("baseline", "Baseline for PRR calculation", m_baseline_prr);
  cmd.AddValue ("csv-log-cumulative", "Name of the CSV log file for the cumulative (average) PRR and latency data", csv_name_cumulative);

  /* Cmd Line option for v2x */
  cmd.AddValue ("tx-power", "UEs transmission power [dBm]", ueTxPower);
  cmd.AddValue ("adjacencyPscchPssch", "Scheme for subchannelization", adjacencyPscchPssch);
  cmd.AddValue ("sizeSubchannel", "Number of RBs per Subchannel", sizeSubchannel);
  cmd.AddValue ("numSubchannel", "Number of Subchannels", numSubchannel);
  cmd.AddValue ("startRbSubchannel", "Index of first subchannel index", startRbSubchannel);
  cmd.AddValue ("T1", "T1 Value of Selection Window", t1);
  cmd.AddValue ("T2", "T2 Value of Selection Window", t2);
  cmd.AddValue ("mcs", "Modulation and Coding Scheme", mcs);
  cmd.AddValue ("pRsvp", "Resource Reservation Interval", pRsvp);
  cmd.AddValue ("probResourceKeep", "Probability for selecting previous resource again", probResourceKeep);

  cmd.AddValue("sim-time", "Total duration of the simulation [s]", simTime);

  cmd.Parse (argc, argv);

  if (verbose)
    {
      LogComponentEnable ("v2v-80211p-gps-tc", LOG_LEVEL_INFO);
      LogComponentEnable ("CABasicService", LOG_LEVEL_INFO);
      LogComponentEnable ("DENBasicService", LOG_LEVEL_INFO);
    }

  NS_LOG_INFO("Configuring C-V2X channel...");
  /*** 0.c V2X Configurations ***/
  /* Set the UEs power in dBm */
  Config::SetDefault ("ns3::cv2x_LteUePhy::TxPower", DoubleValue (ueTxPower));
  Config::SetDefault ("ns3::cv2x_LteUePhy::RsrpUeMeasThreshold", DoubleValue (-10.0));
  /* Enable V2X communication on PHY layer */
  Config::SetDefault ("ns3::cv2x_LteUePhy::EnableV2x", BooleanValue (true));

  /* Set power */
  Config::SetDefault ("ns3::cv2x_LteUePowerControl::Pcmax", DoubleValue (ueTxPower));
  Config::SetDefault ("ns3::cv2x_LteUePowerControl::PsschTxPower", DoubleValue (ueTxPower));
  Config::SetDefault ("ns3::cv2x_LteUePowerControl::PscchTxPower", DoubleValue (ueTxPower));

  if (adjacencyPscchPssch)
  {
      slBandwidth = sizeSubchannel * numSubchannel;
  }
  else
  {
      slBandwidth = (sizeSubchannel+2) * numSubchannel;
  }

  /* Configure for UE selected */
  Config::SetDefault ("ns3::cv2x_LteUeMac::UlBandwidth", UintegerValue (slBandwidth));
  Config::SetDefault ("ns3::cv2x_LteUeMac::EnableV2xHarq", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::cv2x_LteUeMac::EnableAdjacencyPscchPssch", BooleanValue (adjacencyPscchPssch));
  Config::SetDefault ("ns3::cv2x_LteUeMac::EnablePartialSensing", BooleanValue (partialSensing));
  Config::SetDefault ("ns3::cv2x_LteUeMac::SlGrantMcs", UintegerValue (mcs));
  Config::SetDefault ("ns3::cv2x_LteUeMac::SlSubchannelSize", UintegerValue (sizeSubchannel));
  Config::SetDefault ("ns3::cv2x_LteUeMac::SlSubchannelNum", UintegerValue (numSubchannel));
  Config::SetDefault ("ns3::cv2x_LteUeMac::SlStartRbSubchannel", UintegerValue (startRbSubchannel));
  Config::SetDefault ("ns3::cv2x_LteUeMac::SlPrsvp", UintegerValue (pRsvp));
  Config::SetDefault ("ns3::cv2x_LteUeMac::SlProbResourceKeep", DoubleValue (probResourceKeep));
  Config::SetDefault ("ns3::cv2x_LteUeMac::SelectionWindowT1", UintegerValue (t1));
  Config::SetDefault ("ns3::cv2x_LteUeMac::SelectionWindowT2", UintegerValue (t2));

  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults();

  /* Use the realtime scheduler of ns3 */
  if(realtime)
      GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));

  NS_LOG_INFO("Reading the .rou file...");
  std::string path = sumo_folder + mob_trace;
  /* Load the .rou.xml document */
  xmlInitParser();
  rou_xml_file = xmlParseFile(path.c_str ());
  if (rou_xml_file == NULL)
    {
      NS_FATAL_ERROR("Error: unable to parse the specified XML file: "<<path);
    }
  numberOfNodes = XML_rou_count_vehicles(rou_xml_file);

  xmlFreeDoc(rou_xml_file);
  xmlCleanupParser();

  if(numberOfNodes==-1)
    {
      NS_FATAL_ERROR("Fatal error: cannot gather the number of vehicles from the specified XML file: "<<path<<". Please check if it is a correct SUMO file.");
    }
  NS_LOG_INFO("The .rou file has been read: " << numberOfNodes << " vehicles will be present in the simulation.");

  /*** 1. Create LTE objects   ***/
  Ptr<cv2x_PointToPointEpcHelper>  epcHelper = CreateObject<cv2x_PointToPointEpcHelper> ();
  Ptr<cv2x_LteHelper> lteHelper = CreateObject<cv2x_LteHelper> ();
  lteHelper->SetEpcHelper (epcHelper);

  // Disable eNBs for out-of-coverage modelling
  lteHelper->DisableNewEnbPhy();

  /* V2X */
  Ptr<cv2x_LteV2xHelper> lteV2xHelper = CreateObject<cv2x_LteV2xHelper> ();
  lteV2xHelper->SetLteHelper (lteHelper);

  /* Configure eNBs' antenna parameters before deploying them. */
  lteHelper->SetEnbAntennaModelType ("ns3::cv2x_NistParabolic3dAntennaModel");
  lteHelper->SetAttribute ("UseSameUlDlPropagationCondition", BooleanValue(true));
  Config::SetDefault ("ns3::cv2x_LteEnbNetDevice::UlEarfcn", StringValue ("54990")); // EARFCN 54990 -> 5855-5890-5925 MHz
  lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::cv2x_CniUrbanmicrocellPropagationLossModel"));
  NS_LOG_INFO("Antenna parameters set. Current EARFCN: 54990, current frequency: 5.89 GHz");

  /*** 2. Create Internet and ipv4 helpers ***/
  InternetStackHelper internet;
  Ipv4StaticRoutingHelper ipv4RoutingHelper;

  // std::map<std::string,GPSTraceClient*> GPSTCMap; // [old line of code for using GPS-tc instead of SUMO]
  std::map<std::string,std::shared_ptr<GPSRawTraceClient>> GPSRawTCMap;

  // GPSTraceClientHelper GPSTCHelper; // [old line of code for using GPS-tc instead of SUMO]
  GPSRawTraceClientHelper GPSRawTCHelper;

  // GPSTCHelper.setVerbose(verbose); // [old line of code for using GPS-tc instead of SUMO]

  // std::string path = trace_file_path + gps_trace; // [old line of code for using GPS-tc instead of SUMO]
  // GPSTCMap = GPSTCHelper.createTraceClientsFromCSV(path); // [old line of code for using GPS-tc instead of SUMO]

  GPSRawTCHelper.setHeaderPresence (false);
  GPSRawTCHelper.setOneTraceAllVehicles (true);

  // Set an many vehicles for the Raw GPS Data Traces as the vehicles available in the GPS Trace Client traces
  // GPSRawTCHelper.setSingleTraceModeVehicles (GPSTCMap.size ()); // [old line of code for using GPS-tc instead of SUMO]
  GPSRawTCHelper.setSingleTraceModeVehicles (numberOfNodes);

  std::string raw_path = raw_trace_file_path + gps_raw_trace;
  GPSRawTCMap = GPSRawTCHelper.createRawTraceClientsFromCSV (raw_path);

  // int numberOfNodes=GPSTCMap.size (); // [old line of code for using GPS-tc instead of SUMO]

  NS_LOG_INFO("The .rou file has been read: " << numberOfNodes << " vehicles will be present in the simulation.");

  /* Set the simulation time (in seconds) */
  NS_LOG_INFO("Simulation will last " << simTime << " seconds");
  ns3::Time simulationTime (ns3::Seconds(simTime));

  /*** 1. Create containers for OBUs ***/
  NodeContainer obuNodes;
  NodeContainer enbNodes;
  enbNodes.Create(1);
  obuNodes.Create(numberOfNodes);

  /*** 5. Setup Mobility and position node pool ***/
  MobilityHelper mobility;
  mobility.Install (obuNodes);
  mobility.Install(enbNodes);

  /* Set the eNB to a fixed position */
  Ptr<MobilityModel> mobilityeNBn = enbNodes.Get (0)->GetObject<MobilityModel> ();
  mobilityeNBn->SetPosition (Vector (0, 0, 20.0)); // set eNB to fixed position - it is still disabled

  /*** 5. Install LTE Devices to the nodes + assign IP to UEs + manage buildings ***/
  lteHelper->InstallEnbDevice (enbNodes); // If you don't do it, the simulation crashes

  /* Required to use NIST 3GPP model */
  BuildingsHelper::Install (obuNodes);
  BuildingsHelper::Install (enbNodes);
  // BuildingsHelper::MakeMobilityModelConsistent (); Removed because DEPRECATED from 3.31
  for (NodeList::Iterator nit = NodeList::Begin (); nit != NodeList::End (); ++nit)
    {
      Ptr<MobilityModel> mm = (*nit)->GetObject<MobilityModel> ();
      if (mm != 0)
        {
          Ptr<MobilityBuildingInfo> bmm = mm->GetObject<MobilityBuildingInfo> ();
          NS_ABORT_MSG_UNLESS (0 != bmm, "node " << (*nit)->GetId () << " has a MobilityModel that does not have a MobilityBuildingInfo");
          bmm->MakeConsistent (mm);
        }
    }

  lteHelper->SetAttribute("UseSidelink", BooleanValue (true));
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (obuNodes);

  /* Install the IP stack on the UEs */
  internet.Install (obuNodes);
  Ipv4InterfaceContainer ueIpIface;

  /* Assign IP address to UEs */
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevs));
  for (uint32_t u = 0; u < obuNodes.GetN (); ++u)
    {
      Ptr<Node> ueNode = obuNodes.Get (u);
      /* Set the default gateway for the UE */
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

      NS_LOG_INFO("Node "<< ueNode->GetId () << " has been assigned an IP address: " << ueNode->GetObject<Ipv4> ()->GetAddress(1,0).GetLocal());
    }

  NS_LOG_INFO("Configuring sidelink...");

  /* Create sidelink groups */
  std::vector<NetDeviceContainer> txGroups;
  txGroups = lteV2xHelper->AssociateForV2xBroadcast(ueLteDevs, numberOfNodes);

  /* Compute average number of receivers associated per transmitter and vice versa */
  std::map<uint32_t, uint32_t> txPerUeMap;
  std::map<uint32_t, uint32_t> groupsPerUe;
  std::vector<NetDeviceContainer>::iterator gIt;
  for(gIt=txGroups.begin(); gIt != txGroups.end(); gIt++)
      {
          uint32_t numDevs = gIt->GetN();

          uint32_t nId;

          for(uint32_t i=1; i< numDevs; i++)
              {
                  nId = gIt->Get(i)->GetNode()->GetId();
                  txPerUeMap[nId]++;
              }
      }

  std::map<uint32_t, uint32_t>::iterator mIt;
  for(mIt=txPerUeMap.begin(); mIt != txPerUeMap.end(); mIt++)
      {
          groupsPerUe [mIt->second]++;
      }

  std::vector<uint32_t> groupL2Addresses;
  uint32_t groupL2Address = 0x00;
  std::vector<Ipv4Address> ipAddresses;
  Ipv4AddressGenerator::Init(Ipv4Address ("225.0.0.0"), Ipv4Mask ("255.0.0.0"));
  Ipv4Address clientRespondersAddress = Ipv4AddressGenerator::NextAddress (Ipv4Mask ("255.0.0.0"));
  NetDeviceContainer activeTxUes;


  for(gIt=txGroups.begin(); gIt != txGroups.end(); gIt++)
      {
          /* Create Sidelink bearers */
          NetDeviceContainer txUe ((*gIt).Get(0));
          activeTxUes.Add(txUe);
          NetDeviceContainer rxUes = lteV2xHelper->RemoveNetDevice ((*gIt), txUe.Get (0));
          Ptr<cv2x_LteSlTft> tft = Create<cv2x_LteSlTft> (cv2x_LteSlTft::TRANSMIT, clientRespondersAddress, groupL2Address);
          lteV2xHelper->ActivateSidelinkBearer (Seconds(0.0), txUe, tft);
          tft = Create<cv2x_LteSlTft> (cv2x_LteSlTft::RECEIVE, clientRespondersAddress, groupL2Address);
          lteV2xHelper->ActivateSidelinkBearer (Seconds(0.0), rxUes, tft);

          /* store and increment addresses */
          groupL2Addresses.push_back (groupL2Address);
          ipAddresses.push_back (clientRespondersAddress);
          groupL2Address++;
          clientRespondersAddress = Ipv4AddressGenerator::NextAddress (Ipv4Mask ("255.0.0.0"));
      }

  /* Creating sidelink configuration */
  Ptr<cv2x_LteUeRrcSl> ueSidelinkConfiguration = CreateObject<cv2x_LteUeRrcSl>();
  ueSidelinkConfiguration->SetSlEnabled(true);
  ueSidelinkConfiguration->SetV2xEnabled(true);

  cv2x_LteRrcSap::SlV2xPreconfiguration preconfiguration;
  preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommPreconfigGeneral.carrierFreq = 54890;
  preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommPreconfigGeneral.slBandwidth = slBandwidth;

  preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.nbPools = 1;
  preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.nbPools = 1;

  cv2x_SlV2xPreconfigPoolFactory pFactory;
  pFactory.SetHaveUeSelectedResourceConfig (true);
  pFactory.SetSlSubframe (std::bitset<20> (0xFFFFF));
  pFactory.SetAdjacencyPscchPssch (adjacencyPscchPssch);
  pFactory.SetSizeSubchannel (sizeSubchannel);
  pFactory.SetNumSubchannel (numSubchannel);
  pFactory.SetStartRbSubchannel (startRbSubchannel);
  pFactory.SetStartRbPscchPool (0);
  pFactory.SetDataTxP0 (-4);
  pFactory.SetDataTxAlpha (0.9);

  preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0] = pFactory.CreatePool ();
  preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0] = pFactory.CreatePool ();
  ueSidelinkConfiguration->SetSlV2xPreconfiguration (preconfiguration);

  lteHelper->InstallSidelinkV2xConfiguration (ueLteDevs, ueSidelinkConfiguration);

  /*** 6. Setup Traci and start SUMO ***/
  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();
  sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumo_config));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));    // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (sumo_updates)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", BooleanValue (sumo_gui));
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (1.0));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (false));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));
  sumoClient->SetAttribute ("SumoAdditionalCmdOptions", StringValue ("--verbose=true --collision.action warn --collision.check-junctions"));
  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (1.0)));

  /*** 7. Setup interface and application for dynamic nodes ***/
  simpleCAMSenderCEMHelper SimpleCAMSenderCEMHelper;
  SimpleCAMSenderCEMHelper.SetAttribute ("TraCIClient", PointerValue(sumoClient));
  SimpleCAMSenderCEMHelper.SetAttribute ("RealTime", BooleanValue(realtime));
  SimpleCAMSenderCEMHelper.SetAttribute ("SendCEM", BooleanValue(send_cem));

  // Create vector with the GPS Trace Client map values
  // [old lines of code for using GPS-tc instead of SUMO]
//  std::vector<GPSTraceClient*> v_gps_tc;
//  GPS_TC_MAP_ITERATOR(GPSTCMap,GPSTCit) {
//    v_gps_tc.push_back (GPS_TC_IT_OBJECT(GPSTCit));
//  }

  // Create vector with the GPS Raw Trace Client map values
  std::vector<std::shared_ptr<GPSRawTraceClient>> v_raw_gps_tc;
  GPS_RAW_TC_MAP_ITERATOR(GPSRawTCMap,GPSRawTCit) {
    v_raw_gps_tc.push_back (GPS_RAW_TC_IT_OBJECT(GPSRawTCit));
  }

  // Open a new CSV file for logging
  std::ofstream csv_filestream;
  std::string csv_filename = csv_prefix + std::to_string(numberOfNodes) + ".csv";

  csv_filestream.open(csv_filename.c_str ());

  csv_filestream << "vehicle_ID,CAM_rx,CAM_tx,CAM_bytes_tx,CAM_bytes_rx,CEM_rx,CEM_tx,CEM_bytes_tx,CEM_bytes_rx" << std::endl;
  
  PRRSupervisor prrSup(m_baseline_prr);
  prrSup.setTraCIClient(sumoClient);

  int i=0;
  /* callback function for node creation */
  std::function<Ptr<Node> ()> setupNewWifiNode = [&] () -> Ptr<Node>
    {

      if (nodeCounter >= obuNodes.GetN())
        NS_FATAL_ERROR("Node Pool empty!: " << nodeCounter << " nodes created.");

      Ptr<Node> includedNode = obuNodes.Get(nodeCounter);

      /* Install Application */
      //SimpleCAMSenderCEMHelper.SetAttribute ("GPSRawClient", PointerValue(v_raw_gps_tc[nodeCounter]));
      // SimpleCAMSenderCEMHelper.SetAttribute ("DisseminationDelay", DoubleValue(dissemination_delay));

      if(terminateAt > 0)
      {
          SimpleCAMSenderCEMHelper.SetAttribute ("TerminateAt", DoubleValue(terminateAt));
      }
      SimpleCAMSenderCEMHelper.SetAttribute ("IpAddr", Ipv4AddressValue(ipAddresses[i]));
      i++;
      SimpleCAMSenderCEMHelper.SetAttribute ("Model", StringValue("cv2x"));
      SimpleCAMSenderCEMHelper.SetAttribute ("PRRSupervisor", PointerValue (&prrSup));
      ApplicationContainer setupAppSimpleSender = SimpleCAMSenderCEMHelper.Install (includedNode);

      setupAppSimpleSender.Get (0)->GetObject<simpleCAMSenderCEM>()->setCSVOfstream(&csv_filestream);
      setupAppSimpleSender.Get (0)->GetObject<simpleCAMSenderCEM>()->setRawTraceClient(v_raw_gps_tc[nodeCounter]);

      setupAppSimpleSender.Start (Seconds(dissemination_delay) - Simulator::Now ());
      setupAppSimpleSender.Stop (simulationTime - Simulator::Now () - Seconds (0.01));

      nodeCounter++; // increment counter for next node

      return includedNode;
    };

  /* callback function for node shutdown */
  std::function<void (Ptr<Node>)> shutdownWifiNode = [] (Ptr<Node> exNode)
    {
      /* stop all applications */
    if(exNode->GetNApplications () > 0)
    {
        Ptr<simpleCAMSenderCEM> AppSimpleSender_ = exNode->GetApplication(0)->GetObject<simpleCAMSenderCEM>();

        if(AppSimpleSender_)
          AppSimpleSender_->StopApplicationNow();

         /* set position outside communication range */
        Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
        mob->SetPosition(Vector(-1000.0+(rand()%25),320.0+(rand()%25),250.0));// rand() for visualization purposes
    }

      /* NOTE: further actions could be required for a safe shut down! */
    };

  /* Start TraCI client with given function pointers */
  sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

//  int nodeCounterRaw = 0;
//  std::function<Ptr<Node>()> gpsRawStartFcn = [&] () -> Ptr<Node>
//  {
//      Ptr<Node> includedNode = obuNodes.Get(nodeCounterRaw);
//      nodeCounterRaw++; // increment counter for next node
//      return includedNode;
//  };

  std::function<void (Ptr<Node>)> gpsRawEndFcn = [] (Ptr<Node> exNode)
  {
    if(exNode->GetNApplications () > 0)
    {
        Ptr<simpleCAMSenderCEM> AppSimpleSender_ = exNode->GetApplication(0)->GetObject<simpleCAMSenderCEM>();

        if(AppSimpleSender_)
        {
          AppSimpleSender_->stopDissemination();
        }
//          AppSimpleSender_->StopApplicationNow();

        /* set position outside communication range */
//       Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
//       mob->SetPosition(Vector(-1000.0+(rand()%25),320.0+(rand()%25),250.0)); // rand() for visualization purposes
    }
  };

  // Start "playing" the GPS Trace for each vehicle (i.e. make the vehicle start their movements)
  // "Seconds(0)" is specified to "playTrace" to reproduce all the traces since the beginning
  // of the simulation. A different amount of time for all the vehicles or a different amount of time
  // for each vehicle can also be specified to delay the reproduction of the traces
  // [old lines of code for using GPS-tc instead of SUMO]
//  GPS_TC_MAP_ITERATOR(GPSTCMap,GPSTCit) {
//      GPS_TC_IT_OBJECT(GPSTCit)->GPSTraceClientSetup(setupNewWifiNode,shutdownWifiNode);
//      GPS_TC_IT_OBJECT(GPSTCit)->playTrace(Seconds(0));
//  }

  int nodeCounterRaw = 0;
  GPS_RAW_TC_MAP_ITERATOR (GPSRawTCMap,GPSRawTCit) {
      GPS_TC_IT_OBJECT(GPSRawTCit)->GPSRawTraceClientSetup(obuNodes.Get(nodeCounterRaw),gpsRawEndFcn);
      nodeCounterRaw++;
  }

  /*** 8. Start Simulation ***/
  Simulator::Stop (simulationTime);

  Simulator::Run ();

  Simulator::Destroy ();

  csv_filestream.close();
  
  if(csv_name_cumulative!="")
  {
      std::ofstream csv_cum_ofstream;
      std::string full_csv_name = csv_name_cumulative + ".csv";

      if(access(full_csv_name.c_str(),F_OK)!=-1)
      {
          // The file already exists
          csv_cum_ofstream.open(full_csv_name,std::ofstream::out | std::ofstream::app);
      }
      else
      {
          // The file does not exist yet
          csv_cum_ofstream.open(full_csv_name);
          csv_cum_ofstream << "current_txpower_dBm,avg_PRR,avg_latency_ms" << std::endl;
      }

      csv_cum_ofstream << ueTxPower << "," << prrSup.getAveragePRR () << "," << prrSup.getAverageLatency () << std::endl;

      std::cout << "Average PRR: " << prrSup.getAveragePRR () << std::endl;
      std::cout << "Average latency (ms): " << prrSup.getAverageLatency () << std::endl;
  }

  return 0;
}
