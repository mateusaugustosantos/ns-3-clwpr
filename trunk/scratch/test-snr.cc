/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */

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

bool snrFlag = false;
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
//  NS_LOG_UNCOND ("Received one packet!");
}

void PrintSnr (Ptr<const Packet> packet, uint16_t channelFreqMhz, uint16_t channelNumber, uint32_t rate,
                bool isShortPreamble, double signalDbm, double noiseDbm)
{
//  NS_LOG_UNCOND (signalDbm);
  if (snrFlag == false){
  std::cout << "Signal = " << signalDbm << "\n";
  std::cout << "Noise = " << noiseDbm << "\n";
  std::cout << "SNR = " <<signalDbm - noiseDbm << "\n";
  snrFlag = true;
  }
}

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
      socket->Send (Create<Packet> (pktSize));
      Simulator::Schedule (pktInterval, &GenerateTraffic,
                           socket, pktSize,pktCount-1, pktInterval);
//      std::cout<< "Packet #" << pktCount << " queued."<<std::endl;
    }
  else
    {
      socket->Close ();
    }
}



int main (int argc, char *argv[])
{
  double duration = 10.0;
  double start = 3.0;
  int iteration = 1;
  std::string phyMode ("OfdmRate3MbpsBW10MHz");
//  std::string phyMode ("DsssRate2Mbps");
  double distance = 500;  // m
  uint32_t packetSize = 512; // bytes
  uint32_t numPackets = 10;
  uint32_t numNodes = 2;
  uint32_t sinkNode = 1;
  uint32_t sourceNode = 0;
  double interval = 1.0; // seconds
  bool verbose = false;
  bool tracing = false;

  LogComponentEnable ("ClwprRoutingProtocol",LOG_LEVEL_ALL);

  for (iteration = 1; iteration < 2; iteration ++){
      std::cout << "Distance =" << iteration * 5.5 << "\n";
      snrFlag = false;
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
      cmd.AddValue ("duration", "Duration of Simulation", duration);

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

      YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
    //  // set it to zero; otherwise, gain will be added
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
      wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue(-83));
      wifiPhy.Set ("CcaMode1Threshold", DoubleValue(-86));


      wifiPhy.SetChannel (wifiChannel.Create ());

      // Add a non-QoS upper mac, and disable rate control
//      NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
      QosWifiMacHelper wifiMac = QosWifiMacHelper::Default ();
//      wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
      wifi.SetStandard (WIFI_PHY_STANDARD_80211p_CCH);
      wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                    "DataMode",StringValue(phyMode),
                                       "ControlMode",StringValue(phyMode));
      // Set it to adhoc mode
      wifiMac.SetType ("ns3::AdhocWifiMac");
      NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, c);




      MobilityHelper mobility;
      Ptr<ListPositionAllocator> positionAlloc =  CreateObject<ListPositionAllocator> ();
      positionAlloc->Add (Vector (0.0, 0.0, 0.0));
      positionAlloc->Add (Vector (iteration*5.5, 0.0, 0.0));
      mobility.SetPositionAllocator (positionAlloc);
      mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
      mobility.InstallAll();

      ClwprHelper clwpr;
    //  Ipv4StaticRoutingHelper staticRouting;

    //  LogComponentEnableAll(LOG_PREFIX_NODE);

      Ipv4ListRoutingHelper list;
    //list.Add (staticRouting, 0);
    //  list.Add (olsr, 10);
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

      Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/Phy/$ns3::WifiPhy/PromiscSnifferRx",
                                       MakeCallback (&PrintSnr));

      if (tracing == true)
        {
          wifiPhy.EnablePcap ("./snr-test", devices);
        }

      Simulator::Schedule (DELAY(Seconds (start)), &GenerateTraffic,
                           source, packetSize, numPackets, interPacketInterval);

      Simulator::Stop (Seconds (duration));
      Simulator::Run ();

      Simulator::Destroy ();
  }
  return 0;
}

