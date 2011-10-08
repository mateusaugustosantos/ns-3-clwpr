
//Paradeigma 1

#include "ns3/core-module.h"
//#include "ns3/simulator-module.h"
//#include "ns3/node-module.h"
//#include "ns3/helper-module.h"
#include "ns3/applications-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/point-to-point-module.h"
#include "ns3/pointer.h"

using namespace ns3;

static void TxDrop(std::string path, Ptr<const Packet> packet)
{
 // print something, the packet for example
// packet->Print(std::cout);
 std::cout << "A packet has been dropped \n";

} 

NS_LOG_COMPONENT_DEFINE ("Paradeigma1");

int main (int argc,char *argv[])
{

LogComponentEnable ("UdpEchoClientApplication",LOG_LEVEL_INFO);
LogComponentEnable ("UdpEchoServerApplication",LOG_LEVEL_INFO);
//
//Arxikes times gia to mege8os paketou kai ton ru8mo metadoshs gia UDP
//
Config::SetDefault ("ns3::OnOffApplication::PacketSize", UintegerValue (1000));
Config::SetDefault ("ns3::OnOffApplication::DataRate", StringValue ("100Kb/s"));


//DefaultValue::Bind ("DropTailQueue::m_maxPackets", 10);

CommandLine cmd;
  bool enableFlowMonitor = true;
  cmd.AddValue("EnableMonitor", "Enable Flow Monitor", enableFlowMonitor);
  cmd.Parse (argc, argv);
//
//Dimiourgw tous komvous
//
NodeContainer n;
n.Create (6);
NodeContainer n0n3 = NodeContainer (n.Get(0), n.Get(3));
NodeContainer n1n2 = NodeContainer (n.Get(1), n.Get(2));
NodeContainer n2n3 = NodeContainer (n.Get(2), n.Get(3));
NodeContainer n4n3 = NodeContainer (n.Get(4), n.Get(3));
NodeContainer n4n5 = NodeContainer (n.Get(4), n.Get(5));
//
//Kollaw ola ta Prwtokolla Thlepikoinwniwn Stous Komvous
//
InternetStackHelper internet;
internet.Install (n);
//
//Dhmiourgw Ta Kanalia
//
PointToPointHelper p2p;

p2p.SetDeviceAttribute ("DataRate", StringValue ("2Mbps"));
p2p.SetChannelAttribute ("Delay", StringValue ("10ms"));
NetDeviceContainer d0d3 = p2p.Install (n0n3);

NetDeviceContainer d1d2 = p2p.Install (n1n2);

NetDeviceContainer d2d3 = p2p.Install (n2n3);

PointToPointHelper p2p2;

p2p2.SetDeviceAttribute ("DataRate", StringValue ("0.07Mbps"));
p2p2.SetChannelAttribute ("Delay", StringValue ("20ms"));
p2p2.SetQueue ("ns3::DropTailQueue",
                       "MaxPackets", StringValue ("10")); 
NetDeviceContainer d4d3 = p2p2.Install (n4n3);
//
//Katagrafh Xamenwn Paketwn
//
//uint32_t nDevices = d4d3.GetN ();
Ptr<NetDevice> p = d4d3.Get (1);
PointerValue val; 
p->GetAttribute ("TxQueue", val);
//Ptr<Queue> queue = val.Get ();
//Ptr<DropTailQueue> droptail = DynamicCast<DropTailQueue> (queue); 

//std::cout << p << std::endl;

p2p.SetDeviceAttribute ("DataRate", StringValue ("0.5Mbps"));
p2p.SetChannelAttribute ("Delay", StringValue ("100ms"));
NetDeviceContainer d4d5 = p2p.Install (n4n5);
//
//Diamoirazoume IP dieu8unseis
//
Ipv4AddressHelper ipv4;
ipv4.SetBase ("10.1.1.0", "255.255.255.0");
Ipv4InterfaceContainer i0i3 = ipv4.Assign (d0d3);

ipv4.SetBase ("10.1.2.0", "255.255.255.0");
Ipv4InterfaceContainer i1i2 = ipv4.Assign (d1d2);

ipv4.SetBase ("10.1.3.0", "255.255.255.0");
Ipv4InterfaceContainer i2i3 = ipv4.Assign (d2d3);

ipv4.SetBase ("10.1.4.0", "255.255.255.0");
Ipv4InterfaceContainer i4i3 = ipv4.Assign (d4d3);

ipv4.SetBase ("10.1.5.0", "255.255.255.0");
Ipv4InterfaceContainer i4i5 = ipv4.Assign (d4d5);
//
//Dimiourgoume Routing Tables anamesa stous komvous
Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
//
//Dimiourgw Mia Genitria Sta8eris kinisis CBR Kai Enhmerwnw Apostolea Gia To Poios 
//einai o Dektis kai se poia Porta akouei
//
uint16_t port = 9;   // Discard port (RFC 863)-Porta Aporipshs Server-  
                     // Den Epistrefontai Ack
//
OnOffHelper onoff ("ns3::UdpSocketFactory", 
  Address (InetSocketAddress (i4i5.GetAddress (1), port)));
onoff.SetAttribute ("OnTime", RandomVariableValue (ConstantVariable (1)));
onoff.SetAttribute ("OffTime", RandomVariableValue (ConstantVariable (0)));
ApplicationContainer apps = onoff.Install (n.Get (1));
apps.Start (Seconds (0.1));
apps.Stop (Seconds (124));
//
//Dimiourgw Ena Dekti Paketwn Kai ton topo8etw ston komvo 5
//
PacketSinkHelper sink ("ns3::UdpSocketFactory",
    Address (InetSocketAddress (Ipv4Address::GetAny (), port)));
apps = sink.Install (n.Get (5));
apps.Start (Seconds (0.1));
apps.Stop (Seconds (124));
//
//Dhmiourgw FTP kinhsh Mesw TCP.
//
uint16_t servPort = 21;

OnOffHelper onoff2 ("ns3::TcpSocketFactory", Address (InetSocketAddress (i4i5.GetAddress (1), servPort)));
onoff2.SetAttribute ("OnTime", RandomVariableValue (ConstantVariable (1)));
onoff2.SetAttribute ("OffTime", RandomVariableValue (ConstantVariable (0)));
ApplicationContainer apps2 = onoff2.Install (n.Get (0));
apps2.Start (Seconds (1));
apps2.Stop (Seconds (124));
//
//Dhmiourgw Ena Dekti Paketwn Gia TCP kinhsh kai ton topo8etw ston komvo 5
//
PacketSinkHelper sink2 ("ns3::TcpSocketFactory",
                Address (InetSocketAddress (Ipv4Address::GetAny (), servPort)));
apps2 = sink2.Install (n4n5.Get (1));
apps2.Start (Seconds (1));
apps2.Stop (Seconds (124.5));

//////////////////////////////////////////////////////////////////////////////////////////////// 
Config::Connect ("/NodeList/3/DeviceList/*/$ns3::PointToPointNetDevice/TxQueue/Drop", MakeCallback( &TxDrop ));
//TxDrop("/NodeList/*/DeviceList/*/$ns3::PointToPointNetDevice/TxQueue/Drop",);


//AsciiTraceHelper ascii;
//p2p.EnableAsciiAll (ascii.CreateFileStream ("Paradeigma1.tr"));
//p2p.EnablePcapAll ("ParadeigmaEna");

// Flow Monitor
Ptr<FlowMonitor> flowmon;
if (enableFlowMonitor)
  {
    FlowMonitorHelper flowmonHelper;
    flowmon = flowmonHelper.InstallAll ();
  }

//Ptr<Queue> queue0 = CreateObject<DropTailQueue> (); 
//uint32_t DropQ0 = queue0->GetTotalDroppedPackets();
//std::cout << "Total DROPPED PACKETS "<< " - Queue Node0 = " << DropQ0 
//<< std::endl; 
  Simulator::Stop (Seconds(125));
  NS_LOG_INFO ("Run Simulation.");

  Simulator::Run ();

  if (enableFlowMonitor)
    {
      flowmon->SerializeToXmlFile ("paradeigma1.flowmon", false, false);
    }

  Simulator::Destroy ();
  NS_LOG_INFO ("Done.");

  return 0;
}
























