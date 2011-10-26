/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 University of Washington
 *
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
 *
 */

/* This example demonstrates the use of Ns2TransmobilityHelper class to work with mobility.
 * Using Bonnmotion we generated the trace files which are loaded with the helper.
 * The scenario is a 5x5 ManhattanGrid network with 24 nodes, mean speed 10m/s, minimum speed 5m/s
 * and grid distance of 500m
 *
 * Detailed example description.
 *
 *  - intended usage: this should be used in order to load ns2 movement trace files into ns3.
 *  - behavior:
 *      - Ns2TransmobilityHelperTrace object is created, whith the specified trace file. At this moment, only
 *      specify the file, and no movements are scheduled.
 *      - A log file is created, using the log file name argument.
 *      - A node container is created with the correct node number specified in the command line.
 *      - Use Install method of Ns2TransmobilityHelperTrace to set mobility to nodes. At this moment, file is
 *      read line by line, and the movement is scheduled in the simulator.
 *      - A callback is configured, so each time a node changes its course a log message is printed.
 *  - expected output: example prints out messages generated by each read line from the ns2 movement trace file.
 *                     For each line, it shows if the line is correct, or of it has errors and in this case it will
 *                     be ignored.
 *
 * Usage of ns2-mobility-trace:
 *
 *          ./waf --run "examples/mobility/ns2-mobility-trace \
 *                --traceFile=/home/ntinos/bonnmotion-1.5/bin/grid2.ns_movements
 *                 --nodeNum=24  --duration=100.0 --logFile=ns2-mobility-trace.log"
 *
 *          NOTE: ns2-traces-file could be an absolute or relative path. You could use the file default.ns_movements
 *                included in the same directory that the present file.
 *          NOTE 2: Number of nodes present in the trace file must match with the command line argument.
 *                  Note that you must know it before to be able to load it.
 *          NOTE 3: Duration must be a positive number. Note that you must know it before to be able to load it.
 */


// There are a number of command-line options available to control
// the default behavior.  The list of available command-line options
// can be listed with the following command:
// ./waf --run "wifi-simple-adhoc-grid --help"
//
// Note that all ns-3 attributes (not just the ones exposed in the below
// script) can be changed at command line; see the ns-3 documentation.
//
// For instance, for this configuration, the physical layer will
// stop successfully receiving packets when distance increases beyond
// the default of 500m.
// 
// The source node and sink node can be changed like this:
// 
// ./waf --run "wifi-simple-adhoc --sourceNode=20 --sinkNode=10"
//
// This script can also be helpful to put the Wifi layer into verbose
// logging mode; this command will turn on all wifi logging:
// 
// ./waf --run "wifi-simple-adhoc-grid --verbose=1"
//
// By default, trace file writing is off-- to enable it, try:
// ./waf --run "wifi-simple-adhoc-grid --tracing=1"
//
// When you are done tracing, you will notice many pcap trace files 
// in your directory.  If you have tcpdump installed, you can try this:
//
// tcpdump -r wifi-simple-adhoc-grid-0-0.pcap -nn -tt
//

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/clwpr-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"

#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>


/********** Useful macros **********/

///
/// \brief Gets the delay between a given time and the current time.
///
/// If given time is previous to the current one, then this macro returns
/// a number close to 0. This is used for scheduling events at a certain moment.
///
#define DELAY(time) (((time) < (Simulator::Now ())) ? Seconds (0.000001) : \
                     (time - Simulator::Now () + Seconds (0.000001)))

NS_LOG_COMPONENT_DEFINE ("WifiSimpleAdhocGridClwpr");

using namespace ns3;

void ReceivePacket (Ptr<Socket> socket)
{
  NS_LOG_INFO ("Received one packet!");
}

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize, 
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
      socket->Send (Create<Packet> (pktSize));
      Simulator::Schedule (pktInterval, &GenerateTraffic, 
                           socket, pktSize,pktCount-1, pktInterval);
   NS_LOG_INFO ( "Packet #" << pktCount << " queued." );
    }
  else
    {
      socket->Close ();
    }
}

static void PacketCnF (const Ipv4Header &header, Ptr< const Packet> p){

  NS_LOG_INFO ("A packet from: "<< header.GetIdentification() <<" to: " << header.GetDestination() << " has been queued (CnF)");
  std::cout <<" CnF packet \n";
}

static void PacketDequeue (const Ipv4Header &header, Ptr< const Packet> p){

  NS_LOG_INFO ("A packet from: "<< header.GetIdentification() <<" to: " << header.GetDestination() << " has been queued (CnF)");
  std::cout << " Route Found \n" ;
}

//static void PacketDrop (const Ipv4Header &header, Ptr<const Packet> p, ns3::Ipv4L3Protocol::DropReason reason, Ptr<Ipv4> ipv4, uint32_t _if){
//
 // NS_LOG_INFO ("A packet with ID "<< header.GetIdentification() << " has been dropped (Ipv4L3) at node " << ipv4->GetAddress(1,0));
 // NS_LOG_INFO (" The reason is :" << reason);

//  std::cout << " Drop Packet because "<< reason <<"\n";
//}

static void MacTrace (Ptr<const Packet> p){
  NS_LOG_UNCOND ("A packet has been dropped by MAC");

}

static void WifiRemStationTrace (Mac48Address){
  NS_LOG_UNCOND ("The trans. of a data packet has been faild by MAC layer");

}

// Prints actual position and velocity when a course change event occurs
//static void
//CourseChange (std::ostream *os, std::string foo, Ptr<const MobilityModel> mobility)
//{
//  Vector pos = mobility->GetPosition (); // Get position
//  Vector vel = mobility->GetVelocity (); // Get velocity
//
//  // Prints position and velocities
//  *os << Simulator::Now () << " POS: x=" << pos.x << ", y=" << pos.y
//      << ", z=" << pos.z << "; VEL:" << vel.x << ", y=" << vel.y
//      << ", z=" << vel.z << std::endl;
//}


int main (int argc, char *argv[])
{
  std::string traceFile;
  std::string logFile;
  double duration;
  double start = 17.0;
  int movingNodes;

  std::string phyMode ("DsssRate2Mbps");
  double distance = 500;  // m
  uint32_t packetSize = 512; // bytes
  uint32_t numPackets = 40;
  uint32_t numNodes = 101;  // by default, 5x5
  uint32_t sinkNode = 100;
  uint32_t sourceNode = 0;
  double interval = 1.0; // seconds
  bool verbose = false;
  bool tracing = false;
  bool map = false;
  bool debug = false;
  bool predict = false;
  bool cache = false;
  double cacheTime = 10.0;
  int range = 500;
  double hello = 1.5;
  double txrth = 0;
  bool vanet = false;
  double AngFactor = 1;
  double DistFactor = 1;
  double UtilFactor = 1;
  double MACFactor = 1;
  double CnFFactor = 1;
  double RoadFactor = 1;
  double SNRFactor = -1;
  bool normalize=false;
  bool enchance = false;

  // Enable logging from the ns2 helper
//  LogComponentEnable ("Ns2MobilityHelper",LOG_LEVEL_DEBUG);

  CommandLine cmd;

  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("distance", "distance (m)", distance);
  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
  cmd.AddValue ("interval", "interval (seconds) between packets", interval);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("tracing", "turn on ascii and pcap tracing", tracing);
  cmd.AddValue ("numNodes", "number of nodes", numNodes);
  cmd.AddValue ("sinkNode", "Receiver node number", sinkNode);
  cmd.AddValue ("sourceNode", "Sender node number", sourceNode);
  cmd.AddValue ("traceFile", "Ns2 movement trace file", traceFile);
  cmd.AddValue ("movingNodes", "Number of moving nodes", movingNodes);
  cmd.AddValue ("duration", "Duration of Simulation", duration);
  cmd.AddValue ("start", "Start of the Application Sending", start);
  cmd.AddValue ("logFile", "Log file", logFile);
  cmd.AddValue ("map", "Enable the MAP integration", map);
  cmd.AddValue ("emap", "Enable the MAP integration", enchance);
  cmd.AddValue ("norm", "Enable the MAP integration", normalize);
  cmd.AddValue ("predict", "Enable the Position prediction", predict);
  cmd.AddValue ("dbg", "turn on Debug tracing", debug);
  cmd.AddValue ("TxR", "Select TxRange -- 250 or 500m", range);
  cmd.AddValue ("hello", "Hello Interval time in seconds", hello);
  cmd.AddValue ("cT", "Caching time in seconds", cacheTime);
  cmd.AddValue ("CnF", "Enabling Carry'n'Forward mechanism", cache);

  cmd.AddValue ("AngFactor", "Enabling Carry'n'Forward mechanism", AngFactor);
  cmd.AddValue ("DistFactor", "Enabling Carry'n'Forward mechanism", DistFactor);
  cmd.AddValue ("UtilFactor", "Enabling Carry'n'Forward mechanism", UtilFactor);
  cmd.AddValue ("MACFactor", "Enabling Carry'n'Forward mechanism", MACFactor);
  cmd.AddValue ("CnFFactor", "Enabling Carry'n'Forward mechanism", CnFFactor);
  cmd.AddValue ("RoadFactor", "Enabling Carry'n'Forward mechanism", RoadFactor);
  cmd.AddValue ("SNRFactor", "Enabling Carry'n'Forward mechanism", SNRFactor);

  cmd.AddValue ("TxRTh", "Select Tx Threshold for neighbor selection", txrth);
  cmd.AddValue ("vanet", " Set 802.11p ", vanet);
  cmd.Parse (argc, argv);

  if (debug)  LogComponentEnable ("ClwprRoutingProtocol",LOG_LEVEL_DEBUG);
  if (vanet) {
      phyMode = "OfdmRate3MbpsBW10MHz";
  }
  // Check command line arguments
  if (traceFile.empty () || movingNodes <= 0 || duration <= 0)
    {
      std::cout << "Usage of " << argv[0] << " :\n\n"
      "./waf --run \"ns2-mobility-trace"
      " --traceFile=/home/ntinos/bonnmotion-1.5/bin/grid2.ns_movements"
      " --nodeNum=24 --duration=100.0 --logFile=main-ns2-mob.log \" \n\n"
      "NOTE: ns2-traces-file could be an absolute or relative path. You could use the file default.ns_movements\n"
      "      included in the same directory that the present file.\n\n"
      "NOTE 2: Number of nodes present in the trace file must match with the command line argument and must\n"
      "        be a positive number. Note that you must know it before to be able to load it.\n\n"
      "NOTE 3: Duration must be a positive number. Note that you must know it before to be able to load it.\n\n";

      return 0;
    }

  // Convert to time object
  Time interPacketInterval = Seconds (interval);

  // disable fragmentation for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
  // turn off RTS/CTS for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("500"));
  // Fix non-unicast data rate to be the same as that of unicast
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", 
                      StringValue (phyMode));

//  NodeContainer m_c, s_c;
//  m_c.Create (movingNodes);
//  s_c.Create (numNodes - movingNodes);
  NodeContainer c;
  c.Create (numNodes);

  // The below set of helpers will help us to put together the wifi NICs we want
  WifiHelper wifi;
  if (verbose)
    {
      wifi.EnableLogComponents ();  // Turn on all Wifi logging
    }

  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  // set it to zero; otherwise, gain will be added
//  wifiPhy.Set ("RxGain", DoubleValue (-10) );
  // ns-3 supports RadioTap and Prism tracing extensions for 802.11b
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO); 

  YansWifiChannelHelper wifiChannel ;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::TwoRayGroundPropagationLossModel",
		  	  	  	  	  	  	  "SystemLoss", DoubleValue(1),
		  	  	  	  	  	  	  "HeightAboveZ", DoubleValue(1.5));

  wifiPhy.Set ("TxPowerStart", DoubleValue(16.02));
  wifiPhy.Set ("TxPowerEnd", DoubleValue(16.02));
  wifiPhy.Set ("TxPowerLevels", UintegerValue(1));
  wifiPhy.Set ("TxGain", DoubleValue(1));
  wifiPhy.Set ("RxGain", DoubleValue(1));
  if (range == 500){
  wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue(-83));
  wifiPhy.Set ("CcaMode1Threshold", DoubleValue(-86));
  }
  else if (range == 250){
  wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue(-77));
  wifiPhy.Set ("CcaMode1Threshold", DoubleValue(-80));
  }
  wifiPhy.SetChannel (wifiChannel.Create ());

  // Add a non-QoS upper mac, and disable rate control
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  if (vanet){
//       wifiMac = QosWifiMacHelper::Default ();
       wifi.SetStandard (WIFI_PHY_STANDARD_80211p_CCH);
  }
  else {
//      wifiMac = NqosWifiMacHelper::Default ();
      wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
  }
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue(phyMode),
                                   "ControlMode",StringValue(phyMode));
  // Set it to adhoc mode
  wifiMac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, c);


  // Create Ns2MobilityHelper with the specified trace log file as parameter
  Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);

  // open log file for output
  std::ofstream os;
  os.open (logFile.c_str ());

  ns2.Install (); // configure movements for each node, while reading trace file
  // Configure callback for logging
//  Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
//                   MakeBoundCallback (&CourseChange, &os));

  MobilityHelper mobility;
//  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
//    "MinX", DoubleValue (0.0),
//    "MinY", DoubleValue (0.0),
//    "DeltaX", DoubleValue (distance),
//    "DeltaY", DoubleValue (distance),
//    "GridWidth", UintegerValue (5),
//    "LayoutType", StringValue ("RowFirst"));
  Ptr<ListPositionAllocator> positionAlloc =  CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (2000.0, 2000.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (c.Get(sinkNode));
  mobility.Install (c.Get(sourceNode));

  // Enable CLWPR
  ClwprHelper clwpr;
  Ipv4StaticRoutingHelper staticRouting;

  clwpr.Set("MapFlag", BooleanValue(map));
  clwpr.Set("PredictFlag", BooleanValue (predict));
  clwpr.Set("HelloInterval", TimeValue(Seconds(hello)));
  clwpr.Set("CacheFlag", BooleanValue(cache));
  clwpr.Set("MaxQueueTime", TimeValue(Seconds(cacheTime)));
  clwpr.Set("TxThreshold", DoubleValue(range - txrth));

  clwpr.Set("NormFlag", BooleanValue(normalize));
  clwpr.Set("EMapFlag", BooleanValue(enchance));

  clwpr.Set("DistFact", DoubleValue(DistFactor));
  clwpr.Set("AngFact", DoubleValue(AngFactor));
  clwpr.Set("UtilFact", DoubleValue(UtilFactor));
  clwpr.Set("MACFact", DoubleValue(MACFactor));
  clwpr.Set("CnFFact", DoubleValue(CnFFactor));
  clwpr.Set("SNRFact", DoubleValue(SNRFactor));
  clwpr.Set("RoadFact", DoubleValue(RoadFactor));

  //  LogComponentEnableAll(LOG_PREFIX_NODE);

  Ipv4ListRoutingHelper list;
//  list.Add (staticRouting, 0);
  list.Add (clwpr, 10);

  InternetStackHelper internet;
  internet.SetRoutingHelper (list);
  internet.Install (c);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);


  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (c.Get (sinkNode), tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  recvSink->Bind (local);
  recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));

  Ptr<Socket> source = Socket::CreateSocket (c.Get (sourceNode), tid);
  InetSocketAddress remote = InetSocketAddress (i.GetAddress (sinkNode, 0), 80);
  source->Connect (remote);

  if (tracing == true)
    {
      AsciiTraceHelper ascii;
      wifiPhy.EnableAsciiAll (ascii.CreateFileStream ("wifi-simple-adhoc-clwpr-moving.tr"));
      wifiPhy.EnablePcap ("./test/wifi-simple-adhoc-grid", devices);
      // Trace routing tables
      Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("wifi-simple-adhoc-clwpr-moving.routes", std::ios::out);
      clwpr.PrintRoutingTableAllEvery (Seconds (2), routingStream);

      // To do-- enable an IP-level trace that shows forwarding events only
    }
  
  Simulator::Schedule (DELAY(Seconds (start)), &GenerateTraffic,
                       source, packetSize, numPackets, interPacketInterval);

  // Output what we are doing
  NS_LOG_UNCOND ("Testing from node " << sourceNode <<
		         " to " << sinkNode <<
		         " , Angle Factor " << AngFactor <<
		         " , Distance Factor " << DistFactor <<
		         " , Utilization Factor " << UtilFactor <<
		         " , MAC Factor " << MACFactor <<
		         " , CnF Factor " << CnFFactor <<
		         " , SNR Factor " << SNRFactor <<
		         " , Road Factor " << RoadFactor );


  Config::ConnectWithoutContext("/NodeList/*/$ns3::clwpr::RoutingProtocol/CarryNForward", MakeCallback (&PacketCnF));
  Config::ConnectWithoutContext("/NodeList/*/$ns3::clwpr::RoutingProtocol/Dequeue", MakeCallback (&PacketDequeue));
//  Config::ConnectWithoutContext("/NodeList/*/$ns3::Ipv4L3Protocol/Drop", MakeCallback (&PacketDrop));
  Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/WifiRemoteStationManager/MacTxFinalDataFailed", MakeCallback (&WifiRemStationTrace));
  Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRxDrop", MakeCallback (&MacTrace));
  Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop", MakeCallback (&MacTrace));
  // 8. Install FlowMonitor on all nodes
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();
  monitor->SetAttribute("DelayBinWidth", DoubleValue(0.001));
  monitor->SetAttribute("JitterBinWidth", DoubleValue(0.001));
  monitor->SetAttribute("PacketSizeBinWidth", DoubleValue(1000));


  Simulator::Stop (Seconds (duration));
  Simulator::Run ();
  // 10. Print per flow statistics
  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
  std::string dest;
  std::string src;

  if (numNodes == 202) {
      dest = std::string("10.1.1.201");
      src = std::string("10.1.1.202");
  }
  else if (numNodes == 102){
      dest = std::string( "10.1.1.101");
      src = std::string("10.1.1.102");
  }

  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {
	  Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
      if (t.sourceAddress== src.c_str() && t.destinationAddress == dest.c_str())
//      if (t.destinationAddress == "10.1.1.201")
        {
          std::cout << "Flow " << i->first  << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
//          std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
//          std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
          if (i->second.rxPackets != 0){
//        	  std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / (duration - start) / 1024 / 1024  << " Mbps\n";
        	  std::cout << "  PDR = " << ((double)i->second.rxBytes / (double)i->second.txBytes)*100.0  << " \n";
        	  std::cout << "  Mean HOP count = " << (i->second.timesForwarded / i->second.rxPackets +1) << "\n";
        	  std::cout << "  Mean Delay = " << (i->second.delaySum.GetSeconds() / i->second.rxPackets) << "\n";
                  uint32_t totalDrop =0;
        	  for (uint32_t j=0; j < i->second.packetsDropped.size() ; j++){
       	           totalDrop += i->second.packetsDropped[j];
     	      }
                std::cout << "  Drop Packets = " << totalDrop << "\n" ;
          }
          else {
              std::cout << "  PDR = 0 \n";
          }

        }
    }
//  monitor->SerializeToXmlFile("clwpr-moving-flow.flowmon", true, true);
  // 11. Cleanup
  Simulator::Destroy ();
  os.close (); // close log file
  return 0;
}
