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

//
// This program configures a grid (default 5x5) of nodes on an 
// 802.11b physical layer, with
// 802.11b NICs in adhoc mode, and by default, sends one packet of 1000 
// (application) bytes to node 1.  
//
// The default layout is like this, on a 2-D grid.
//
// n20  n21  n22  n23  n24
// n15  n16  n17  n18  n19
// n10  n11  n12  n13  n14
// n5   n6   n7   n8   n9
// n0   n1   n2   n3   n4
//
// the layout is affected by the parameters given to GridPositionAllocator;
// by default, GridWidth is 5 and numNodes is 25..
//
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
// To see this effect, try running:
//
// ./waf --run "wifi-simple-adhoc --distance=500"
// ./waf --run "wifi-simple-adhoc --distance=1000"
// ./waf --run "wifi-simple-adhoc --distance=1500"
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
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/olsr-module.h"
#include "ns3/internet-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/physim-wifi-module.h"
#include "ns3/gnuplot.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

NS_LOG_COMPONENT_DEFINE ("WifiSimpleAdhocGridOlsr");

using namespace ns3;
void
printStats (FlowMonitor::FlowStats st)
{
        std::cout << "  Tx Bytes: " << st.txBytes << std::endl;
         std::cout << "  Rx Bytes: " << st.rxBytes << std::endl;
         std::cout << "  Tx Packets: " << st.txPackets << std::endl;
         std::cout << "  Rx Packets: " << st.rxPackets << std::endl;
         std::cout << "  Lost Packets: " << st.lostPackets << std::endl;
     if (st.rxPackets > 0)
     {
         std::cout << "  Mean{Delay}: " << (st.delaySum.GetSeconds() / st.rxPackets) << std::endl;
         std::cout << "  Mean{Jitter}: " << (st.jitterSum.GetSeconds() / (st.rxPackets-1)) << std::endl;
         std::cout << "  Mean{Hop Count}: " << st.timesForwarded / st.rxPackets + 1 << std::endl;
     }

     if (false)
     {
         std::cout << "Delay Histogram" << std::endl;
         for (uint32_t i=0; i<st.delayHistogram.GetNBins (); i++)
                 std::cout << " " << i << "(" << st.delayHistogram.GetBinStart (i) << "-"
                                 << st.delayHistogram.GetBinEnd (i) << "): " << st.delayHistogram.GetBinCount (i) << std::endl;

         std::cout << "Jitter Histogram" << std::endl;
         for (uint32_t i=0; i<st.jitterHistogram.GetNBins (); i++ )
                 std::cout << " " << i << "(" << st.jitterHistogram.GetBinStart (i) << "-"
                                 << st.jitterHistogram.GetBinEnd (i) << "): " << st.jitterHistogram.GetBinCount (i) << std::endl;

         std::cout << "PacketSize Histogram  "<< std::endl;
         for (uint32_t i=0; i<st.packetSizeHistogram.GetNBins (); i++ )
                         std::cout << " " << i << "(" << st.packetSizeHistogram.GetBinStart(i) << "-"
                                 << st.packetSizeHistogram.GetBinEnd (i) << "): " << st.packetSizeHistogram.GetBinCount (i) << std::endl;
     }

     for (uint32_t i=0; i<st.packetsDropped.size (); i++){
         std::cout << "  Packets dropped by reason " << i << ": " << st.packetsDropped [i] << std::endl;
         std::cout << "Bytes dropped by reason " << i << ": " << st.bytesDropped[i] << std::endl;
     }
     std::cout << std::endl;
}
void ReceivePacket (Ptr<Socket> socket)
{
  NS_LOG_UNCOND ("Received one packet!");
}

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize, 
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
      socket->Send (Create<Packet> (pktSize));
      Simulator::Schedule (pktInterval, &GenerateTraffic, 
                           socket, pktSize,pktCount-1, pktInterval);
      std::cout << "Packet #" <<pktCount << " queued \n";
    }
  else
    {
      socket->Close ();
    }
}


int main (int argc, char *argv[])
{
  std::string phyMode ("OfdmRate6Mbps");
//  std::string phyMode ("DsssRate1Mbps");
  double distance = 500;  // m
  uint32_t packetSize = 1000; // bytes
  uint32_t numPackets = 40;
  uint32_t numNodes = 25;  // by default, 5x5
  uint32_t sinkNode = 24;
  uint32_t sourceNode = 0;
  double interval = 1.0; // seconds
  bool verbose = false;
  bool tracing = true;
  bool enableFlowMonitor = true;
  bool enablePlot = true;
  std::string Results("olsr.flowmon");
  std::string experiment;
  std::string format;
  std::string strategy;
  std::string runID;
  double duration = 50;
  double start = 20.0;

//  int rxGain = -5;

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
  cmd.AddValue("EnableMonitor", "Enable Flow Monitor", enableFlowMonitor);
  cmd.AddValue("EnableGNUPlot", "Enable Gnu Plot", enablePlot);
  cmd.AddValue("format", "Format to use for data output.",format);
  cmd.AddValue("experiment", "Identifier for experiment.",experiment);
  cmd.AddValue("strategy", "Identifier for strategy.",strategy);
  cmd.AddValue("run", "Identifier for run.",runID);
  cmd.AddValue ("duration", "Duration of Simulation", duration);
  cmd.AddValue ("start", "Start of the Application Sending", start);

  cmd.Parse (argc, argv);
  // Convert to time object
  Time interPacketInterval = Seconds (interval);

  // disable fragmentation for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
  // turn off RTS/CTS for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));
  // Fix non-unicast data rate to be the same as that of unicast
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", 
                      StringValue (phyMode));

  NodeContainer c;
  c.Create (numNodes);

  // The below set of helpers will help us to put together the wifi NICs we want
  WifiHelper wifi;
  if (verbose)
    {
      wifi.EnableLogComponents ();  // Turn on all Wifi logging
    }

//  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  PhySimWifiPhyHelper wifiPhy = PhySimWifiPhyHelper::Default();
  // set it to zero; otherwise, gain will be added
  wifiPhy.Set ("RxGain", DoubleValue (-10) );
  // ns-3 supports RadioTap and Prism tracing extensions for 802.11b
  wifiPhy.SetPcapDataLinkType (PhySimWifiPhyHelper::DLT_IEEE802_11_RADIO);


  PhySimWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss("ns3::PhySimVehicularChannelPropagationLoss",
		                          "ChannelProfile",EnumValue (V2V_URBAN_CANYON_ONCOMING));
//  YansWifiChannelHelper wifiChannel ;
//  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
//  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  wifiPhy.SetChannel (wifiChannel.Create ());

  // Add a non-QoS upper mac, and disable rate control
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
//  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
  wifi.SetStandard (WIFI_PHY_STANDARD_80211a);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue(phyMode),
                                   "ControlMode",StringValue(phyMode));
  // Set it to adhoc mode
  wifiMac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, c);

  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
    "MinX", DoubleValue (0.0),
    "MinY", DoubleValue (0.0),
    "DeltaX", DoubleValue (distance),
    "DeltaY", DoubleValue (distance),
    "GridWidth", UintegerValue (5),
    "LayoutType", StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (c);

  // Enable OLSR
  OlsrHelper olsr;
  Ipv4StaticRoutingHelper staticRouting;

//  LogComponentEnableAll(LOG_PREFIX_NODE);

  Ipv4ListRoutingHelper list;
//  list.Add (staticRouting, 0);
  list.Add (olsr, 10);

  InternetStackHelper internet;
  internet.SetRoutingHelper (list);
  internet.Install (c);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

//  Sink Node for the OnOffApp
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (c.Get (sinkNode), tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  recvSink->Bind (local);
  recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));
// Source Node for the OnOffApp
  Ptr<Socket> source = Socket::CreateSocket (c.Get (sourceNode), tid);
  InetSocketAddress remote = InetSocketAddress (i.GetAddress (sinkNode, 0), 80);
  source->Connect (remote);

  // Create the OnOff application to send UDP datagrams of size
  // 1000 bytes at a rate of 500 Kb/s

//  // Set up some default values for the simulation.  Use the
//  Config::SetDefault ("ns3::OnOffApplication::PacketSize", UintegerValue (1000));
//  Config::SetDefault ("ns3::OnOffApplication::DataRate", StringValue ("500kb/s"));
//
//  NS_LOG_INFO ("Create Applications.");
//  uint16_t port = 9;   // Discard port (RFC 863)
//  OnOffHelper onoff ("ns3::UdpSocketFactory",Address (InetSocketAddress (i.GetAddress (sourceNode), port)));
//  onoff.SetAttribute ("OnTime", RandomVariableValue (ConstantVariable (1)));
//  onoff.SetAttribute ("OffTime", RandomVariableValue (ConstantVariable (0)));
//  ApplicationContainer apps = onoff.Install (c.Get (sourceNode));
//  apps.Start (Seconds (21));
//  apps.Stop (Seconds (30));
//
//  // Create a packet sink to receive these packets
//  PacketSinkHelper sink ("ns3::UdpSocketFactory",
//    Address (InetSocketAddress (Ipv4Address::GetAny (), port)));
//  apps = sink.Install (c.Get (sinkNode));
//  apps.Start (Seconds (20));
//  apps.Stop (Seconds (50));

  if (tracing == true)
    {
      AsciiTraceHelper ascii;
      wifiPhy.EnableAsciiAll (ascii.CreateFileStream ("wifi-simple-adhoc-grid.tr"));
//      wifiPhy.EnablePcap ("wifi-simple-adhoc-grid", devices);
      // Trace routing tables
      Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("wifi-simple-adhoc-grid-olsr.routes", std::ios::out);
      olsr.PrintRoutingTableAllEvery (Seconds (2), routingStream);

      // To do-- enable an IP-level trace that shows forwarding events only
    }
//  // Flow Monitor
//  Ptr<FlowMonitor> flowmon;
//  FlowMonitorHelper flowmonHelper;
//
//  if (enableFlowMonitor)
//    {
//      flowmon = flowmonHelper.InstallAll ();
//      flowmon->SetAttribute("DelayBinWidth", DoubleValue(0.001));
//      flowmon->SetAttribute("JitterBinWidth", DoubleValue(0.001));
//      flowmon->SetAttribute("PacketSizeBinWidth", DoubleValue(20));
//
//    }
//  // Give OLSR time to converge-- 30 seconds perhaps
  Simulator::Schedule (Seconds (start), &GenerateTraffic,
                       source, packetSize, numPackets, interPacketInterval);

  // Output what we are doing
  NS_LOG_UNCOND ("Testing from node " << sourceNode << " to " << sinkNode << " with grid distance " << distance);

  // 8. Install FlowMonitor on all nodes
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();
  monitor->SetAttribute("DelayBinWidth", DoubleValue(0.001));
  monitor->SetAttribute("JitterBinWidth", DoubleValue(0.001));
  monitor->SetAttribute("PacketSizeBinWidth", DoubleValue(1000));



  Simulator::Stop (Seconds (duration));
  Simulator::Run ();

//  flowmon->CheckForLostPackets();
//  Ptr<Ipv4FlowClassifier> classifier =DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
//  if(Results == "")
//  {
//          std::string proto;
//          std::map< FlowId, FlowMonitor::FlowStats > stats = flowmon->GetFlowStats();
//          for (std::map< FlowId, FlowMonitor::FlowStats >::iterator flow=stats.begin(); flow!=stats.end(); flow++)
//          {
//                  Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow->first);
//                  switch(t.protocol)
//                  {
//                  case(6):
//                          proto = "TCP";
//                          break;
//                  case(17):
//                          proto = "UDP";
//                          break;
//                  default:
//                          exit(1);
//                  }
//                  std::cout << "FlowID: " << flow->first << " (" << proto << " "
//                                  << t.sourceAddress << "/" << t.sourcePort << " --> "
//                                  << t.destinationAddress << "/" << t.destinationPort << ")" <<std::endl;
//                  printStats(flow->second);
//          }
//  }
//  else{
//          flowmon->SerializeToXmlFile(Results, true, true);
//  }
//  if (enableFlowMonitor)
//    {
//      flowmon->SerializeToXmlFile ("olsr-static-grid.flowmon", true, true);


//  if (enablePlot){
//       Gnuplot gnuplot("DELAYSbyFLOW.png");
//       Gnuplot2dDataset dataset;
//       dataset.SetStyle(Gnuplot2dDataset::HISTEPS);
//       Ptr<Ipv4FlowClassifier> classifier =DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
//       for (std::map< FlowId, FlowMonitor::FlowStats >::iterator flow=flowmon->GetFlowStats().begin(); flow!=flowmon->GetFlowStats().end(); flow++)
//           {
//            Ipv4FlowClassifier::FiveTuple tupl = classifier->FindFlow(flow->first);
//            if(tupl.protocol == 17 && tupl.sourcePort == 698)
//                  continue;
//            dataset.Add((double)flow->first, (double)flow->second.delaySum.GetSeconds() / (double)flow->second.rxPackets);
//            }
//            gnuplot.AddDataset(dataset);
//            gnuplot.GenerateOutput(std::cout);
//  }
////  }



  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {
	  Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
      if (t.sourceAddress=="10.1.1.1" && t.destinationAddress == "10.1.1.25")
        {
          std::cout << "Flow " << i->first  << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
          std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
          std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
          if (i->second.rxPackets != 0){
//        	  std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / (duration - start) / 1024 / 1024  << " Mbps\n";
        	  std::cout << "  PDR = " << ((double)i->second.rxBytes / (double)i->second.txBytes)*100.0  << " \n";
        	  std::cout << "  Mean HOP count = " << (i->second.timesForwarded / i->second.rxPackets +1) << "\n";
        	  std::cout << "  Mean Delay = " << (i->second.delaySum.GetSeconds() / i->second.rxPackets) << "\n";
          }
        }
    }
  monitor->SerializeToXmlFile(Results, true, true);
  Simulator::Destroy ();


  return 0;
}

