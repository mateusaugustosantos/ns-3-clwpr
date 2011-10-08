/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010 Jens Mittag, Stylianos Papanastasiou
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
 * Authors:
 *      Jens Mittag <jens.mittag@kit.edu>
 *      Stylianos Papanastasiou <stylianos@gmail.com>
 */

#include "ns3/node.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-preamble.h"
#include "ns3/wifi-phy.h"
#include "ns3/packet.h"
#include "ns3/config.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"
#include "ns3/object-factory.h"
#include "ns3/physim-wifi-phy.h"
#include "ns3/physim-wifi-channel.h"
#include "ns3/mobility-model.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/physim-signal-detector-opt.h"
#include "physim-wifi-signal-detection-test.h"
#include <itpp/itcomm.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("PhySimWifiSignalDetectionTest");

PhySimWifiSignalDetectionTest::PhySimWifiSignalDetectionTest ()
  : TestCase ("PhySim WiFi signal detection test case")
{
}

PhySimWifiSignalDetectionTest::~PhySimWifiSignalDetectionTest ()
{
}

void
PhySimWifiSignalDetectionTest::DoRun (void)
{

  SetErrorStatus (false);
  PhySimWifiPhy::ClearCache ();
  PhySimWifiPhy::ResetRNG ();

  // Use random scrambler (default)
  Config::SetDefault ("ns3::PhySimScrambler::UseFixedScrambler", BooleanValue (false) );
  // Take care of transformation from IT++ to IEEE notation
  Config::SetDefault ("ns3::PhySimOFDMSymbolCreator::IEEECompliantMode", BooleanValue (true) );
  Config::SetDefault ("ns3::PhySimSignalDetector::IEEECompliantMode", BooleanValue (true) );

  // Create a channel object
  Ptr<PhySimWifiChannel> channel = CreateObject<PhySimWifiUniformChannel> ();
  // Create a PHY object
  Config::SetDefault ("ns3::PhySimWifiPhy::TxPowerEnd", DoubleValue (0.0) );
  Config::SetDefault ("ns3::PhySimWifiPhy::TxPowerStart", DoubleValue (0.0) );
  Config::SetDefault ("ns3::PhySimWifiPhy::TxPowerLevels", UintegerValue (1) );
  Config::SetDefault ("ns3::PhySimWifiPhy::TxGain", DoubleValue (0.0) );
  Config::SetDefault ("ns3::PhySimWifiPhy::RxGain", DoubleValue (0.0) );
  Config::SetDefault ("ns3::PhySimWifiPhy::TxCenterFrequencyTolerance", UintegerValue (0) );
  Config::SetDefault ("ns3::PhySimWifiPhy::Frequency", DoubleValue (2.4e9) );
  Config::SetDefault ("ns3::PhySimWifiPhy::SymbolTime", TimeValue (MicroSeconds (4)) );
  Config::SetDefault ("ns3::PhySimWifiPhy::NormalizeOFDMSymbols", BooleanValue (false) );
  Ptr<PhySimWifiPhy> phy = CreateObject<PhySimWifiPhy> ();

  // Attach PHY object to channel
  phy->SetChannel (channel);

  // Connect trace source for Tx events
  phy->TraceConnectWithoutContext ("Tx", MakeCallback (&PhySimWifiSignalDetectionTest::PhyTxCallback, this));

  // Also create a net device object and a mobility object
  Ptr<Node> node = CreateObject<Node> ();
  Ptr<WifiNetDevice> device = CreateObject<WifiNetDevice> ();
  Ptr<MobilityModel> mobility = CreateObject<ConstantPositionMobilityModel> ();
  mobility->SetPosition (Vector (1.0, 0.0, 0.0));
  node->AggregateObject (mobility);
  phy->SetMobility (node);
  phy->SetDevice (device);

  // Create a packet object (which means we take the bit sequence that represents the packet payload
  // used in the IEEE 802.11 (2007) standard in Annex G
  itpp::bvec bitSequence = itpp::randb (800);
  uint32_t bytes = bitSequence.size () / 8;
  uint8_t *payload = new uint8_t[100];
  for (uint32_t i = 0; i < bytes; i++)
    {
      itpp::bvec extract = bitSequence ((i * 8), (i * 8) + 8 - 1);
      payload[i] = itpp::bin2dec ( extract, false );
    }
  Ptr<Packet> packet = Create<Packet> ( (const uint8_t*) payload, 100);
  WifiMode mode = WifiPhy::GetOfdmRate36Mbps (); // mode does not matter actually


  // Send a packet over the PHY
  phy->SendPacket (packet, mode, WIFI_PREAMBLE_LONG, 1);

  // add 50 samples of random Gaussian noise
  itpp::cvec noise = itpp::randn_c (50);
  itpp::cvec samples = m_txSamples;
  samples.ins (0,noise);

  // Create a signal detector
  Ptr<PhySimSignalDetector>  detector = CreateObject<PhySimSignalDetector> ();
  RunSingle (detector, packet, samples);
  // Create an optimised signal detector
  detector = CreateObject<PhySimSignalDetectorOpt> ();
  RunSingle (detector, packet, samples);

  return ;
}

bool
PhySimWifiSignalDetectionTest::RunSingle (Ptr<PhySimSignalDetector> detector, Ptr<Packet> packet, itpp::cvec &samples)
{
  const int realShortSymbolsStart = 50;       // short symbols start at index position 50 in the samples
  const int tol = 10;         // +-10 symbols offset is very tolerable
  int32_t beginShortSymbols = 0;
  detector->ScanPreamble (packet, 0, samples, beginShortSymbols);
  bool success;

  // Check if we get anywhere near the 50 mark
  NS_TEST_ASSERT_MSG_EQ_TOL (beginShortSymbols, realShortSymbolsStart, tol, "beginShortSymbols mis-estimates significantly the true short training sequence start ("
                                  << beginShortSymbols << " vs " << realShortSymbolsStart << ")");
  success = !((beginShortSymbols) > (realShortSymbolsStart) + (tol) || (beginShortSymbols) < (realShortSymbolsStart) - (tol));                                                     \
  if (!success)
    {
      NS_LOG_DEBUG ("FAIL: The estimate and true location of the start of the short training symbols differ significantly");
    }
  else
    {
      NS_LOG_DEBUG ("PASS: The estimate and true location of the start of the short training symbols are approximately the same");
    }

  if (success)
    {
      // If the short training sequence is detected
      int32_t beginLongSymbols = detector->ScanForLongSeq (packet, 0, samples (beginShortSymbols, samples.size () - 1));
      int32_t endShortSymbols = beginLongSymbols + beginShortSymbols - 32;
      // Check if endofShortSymbols+1=209, i.e. the real long training sequence should start at sample 210
      NS_TEST_ASSERT_MSG_EQ (endShortSymbols + 1, 210, "beginLongSymbols mis-estimates the true long training sequence start ("
                                          << endShortSymbols + 1 << " vs 210 )");
      success = (endShortSymbols + 1 == 210);
      if (!success)
        {
          NS_LOG_DEBUG ("FAIL: The estimate and true location of the start of the long training symbols differ significantly");
        }
      else
        {
          NS_LOG_DEBUG ("PASS: The estimate and true location of the start of the long training symbols are approximately the same");
        }
    }
  return true;
}

void
PhySimWifiSignalDetectionTest::PhyTxCallback (Ptr<const Packet> packet, Ptr<const PhySimWifiPhyTag> tag)
{
  m_txSamples = tag->GetTxedSamples ();
}

