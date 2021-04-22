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
#include "ns3/internet-module.h"
#include "ns3/wave-module.h"
#include "ns3/mobility-module.h"
#include "ns3/packet-socket-helper.h"

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("v2v-80211p-gps-raw-tc-example");

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

  // Admitted data rates for 802.11p
  std::vector<float> rate_admitted_values{3,4.5,6,9,12,18,24,27};
  std::string datarate_config;

  /*** 0.a App Options ***/
  std::string sumo_folder = "src/automotive/examples/sumo_files_v2v_map/";
  std::string mob_trace = "cars.rou.xml";
  std::string sumo_config ="src/automotive/examples/sumo_files_v2v_map/map.sumo.cfg";

  std::string raw_trace_file_path = "src/gps-raw-tc/trace-example/GPS-Raw-Traces-Sample/";
  std::string gps_raw_trace = "multi_constellation_CEM_v8.txt";

  bool verbose = false;
  bool realtime = false;
  int txPower=26;
  float datarate=12;

  bool sumo_gui = true;
  double sumo_updates = 0.01;

  double simTime = 100;

  int numberOfNodes;
  uint32_t nodeCounter = 0;

  CommandLine cmd;

  xmlDocPtr rou_xml_file;

  /* Cmd Line option for application */
  cmd.AddValue ("realtime", "Use the realtime scheduler or not", realtime);
  cmd.AddValue ("sumo-gui", "Use SUMO gui or not", sumo_gui);
  cmd.AddValue ("sumo-updates", "SUMO granularity", sumo_updates);
  cmd.AddValue ("sumo-folder","Position of sumo config files",sumo_folder);
  cmd.AddValue ("mob-trace", "Name of the mobility trace file", mob_trace);
  cmd.AddValue ("sumo-config", "Location and name of SUMO configuration file", sumo_config);

  cmd.AddValue ("raw-trace-folder","Position of Raw GNSS data trace files",raw_trace_file_path);
  cmd.AddValue ("gps-raw-trace", "Name of the Raw GNSS Data trace file", gps_raw_trace);

  /* Cmd Line option for 802.11p */
  cmd.AddValue ("tx-power", "OBUs transmission power [dBm]", txPower);
  cmd.AddValue ("datarate", "802.11p channel data rate [Mbit/s]", datarate);

  cmd.AddValue("sim-time", "Total duration of the simulation [s]", simTime);

  cmd.Parse (argc, argv);

  if(std::find(rate_admitted_values.begin(), rate_admitted_values.end(), datarate) == rate_admitted_values.end())
    {
      NS_FATAL_ERROR("Fatal error: invalid 802.11p data rate" << datarate << "Mbit/s. Valid rates are: 3, 4.5, 6, 9, 12, 18, 24, 27 Mbit/s.");
    }
  else
    {
      if(datarate==4.5)
        {
          datarate_config = "OfdmRate4_5MbpsBW10MHz";
        }
      else
        {
          datarate_config = "OfdmRate" + std::to_string((int)datarate) + "MbpsBW10MHz";
        }
    }

  if (verbose)
    {
      LogComponentEnable ("v2v-80211p-gps-tc", LOG_LEVEL_INFO);
      LogComponentEnable ("CABasicService", LOG_LEVEL_INFO);
      LogComponentEnable ("DENBasicService", LOG_LEVEL_INFO);
    }

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

  // std::map<std::string,GPSTraceClient*> GPSTCMap; // [old line of code for using GPS-tc instead of SUMO]
  std::map<std::string,GPSRawTraceClient*> GPSRawTCMap;

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
  obuNodes.Create(numberOfNodes);

  /*** 2. Create and setup channel   ***/
  YansWifiPhyHelper wifiPhy;
  wifiPhy.Set ("TxPowerStart", DoubleValue (txPower));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txPower));
  NS_LOG_INFO("Setting up the 802.11p channel @ " << datarate << " Mbit/s, 10 MHz, and tx power " << (int)txPower << " dBm.");

  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);
  /* To be removed when BTP is implemented */
  //Config::SetDefault ("ns3::ArpCache::DeadTimeout", TimeValue (Seconds (1)));

  /*** 3. Create and setup MAC ***/
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue (datarate_config), "ControlMode", StringValue (datarate_config));
  NetDeviceContainer netDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, obuNodes);
  wifiPhy.EnablePcapAll ("CEMTraces"); // Uncomment this to create .pcap files with the exchanged CAM/CEM/... messages

  /*** 4. Create Internet and ipv4 helpers ***/
  PacketSocketHelper packetSocket;
  packetSocket.Install (obuNodes);

  /*** 5. Setup Mobility and position node pool ***/
  MobilityHelper mobility;
  mobility.Install (obuNodes);

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
  sumoClient->SetAttribute ("SumoAdditionalCmdOptions", StringValue ("--verbose true"));
  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (1.0)));

  /*** 7. Setup interface and application for dynamic nodes ***/
  simpleCAMSenderCEMHelper SimpleCAMSenderCEMHelper;
  SimpleCAMSenderCEMHelper.SetAttribute ("TraCIClient", PointerValue(sumoClient));
  SimpleCAMSenderCEMHelper.SetAttribute ("RealTime", BooleanValue(realtime));

  // Create vector with the GPS Trace Client map values
  // [old lines of code for using GPS-tc instead of SUMO]
//  std::vector<GPSTraceClient*> v_gps_tc;
//  GPS_TC_MAP_ITERATOR(GPSTCMap,GPSTCit) {
//    v_gps_tc.push_back (GPS_TC_IT_OBJECT(GPSTCit));
//  }

  // Create vector with the GPS Raw Trace Client map values
  std::vector<GPSRawTraceClient*> v_raw_gps_tc;
  GPS_RAW_TC_MAP_ITERATOR(GPSRawTCMap,GPSRawTCit) {
    v_raw_gps_tc.push_back (GPS_RAW_TC_IT_OBJECT(GPSRawTCit));
  }

  /* callback function for node creation */
  std::function<Ptr<Node> ()> setupNewWifiNode = [&] () -> Ptr<Node>
    {

      if (nodeCounter >= obuNodes.GetN())
        NS_FATAL_ERROR("Node Pool empty!: " << nodeCounter << " nodes created.");

      Ptr<Node> includedNode = obuNodes.Get(nodeCounter);

      /* Install Application */
      SimpleCAMSenderCEMHelper.SetAttribute ("GPSRawClient", PointerValue(v_raw_gps_tc[nodeCounter]));
      ApplicationContainer setupAppSimpleSender = SimpleCAMSenderCEMHelper.Install (includedNode);

      setupAppSimpleSender.Start (Seconds (0.0));
      setupAppSimpleSender.Stop (simulationTime - Simulator::Now () - Seconds (0.1));

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

  int nodeCounterRaw = 0;
  std::function<Ptr<Node>()> gpsRawStartFcn = [&] () -> Ptr<Node>
  {
      Ptr<Node> includedNode = obuNodes.Get(nodeCounterRaw);
      nodeCounterRaw++; // increment counter for next node
      return includedNode;
  };

  std::function<void (Ptr<Node>)> gpsRawEndFcn = [] (Ptr<Node> exNode)
  {
    /* stop all applications */
    if(exNode->GetNApplications () > 0)
    {
        Ptr<simpleCAMSenderCEM> AppSimpleSender_ = exNode->GetApplication(0)->GetObject<simpleCAMSenderCEM>();

        if(AppSimpleSender_)
          AppSimpleSender_->StopApplicationNow();

        /* set position outside communication range */
       Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
       mob->SetPosition(Vector(-1000.0+(rand()%25),320.0+(rand()%25),250.0)); // rand() for visualization purposes
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

  GPS_RAW_TC_MAP_ITERATOR (GPSRawTCMap,GPSRawTCit) {
      GPS_TC_IT_OBJECT(GPSRawTCit)->GPSRawTraceClientSetup(gpsRawStartFcn,gpsRawEndFcn);
  }

  /*** 8. Start Simulation ***/
  Simulator::Stop (simulationTime);

  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
