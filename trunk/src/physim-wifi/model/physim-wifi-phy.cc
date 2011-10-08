/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006 INRIA, 2009-2010 Jens Mittag, Stylianos Papanastasiou
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
 * Based on yans-wifi-phy.h by:
 *      Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 * Major modifications by:
 *      Jens Mittag <jens.mittag@kit.edu>
 *      Stylianos Papanastasiou <stylianos@gmail.com>
 */

#include "physim-wifi-phy.h"
#include "physim-wifi-channel.h"
#include "physim-ofdm-symbolcreator.h"
#include "physim-wifi-phy-state-helper.h"
#include "ns3/wifi-mode.h"
#include "ns3/wifi-preamble.h"
#include "ns3/error-rate-model.h"
#include "ns3/simulator.h"
#include "ns3/packet.h"
#include "ns3/random-variable.h"
#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/double.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"
#include "ns3/integer.h"
#include "ns3/string.h"
#include "ns3/object-factory.h"
#include "ns3/enum.h"
#include "ns3/pointer.h"
#include "ns3/net-device.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/clwpr-routing-protocol.h"

NS_LOG_COMPONENT_DEFINE ("PhySimWifiPhy");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (PhySimWifiPhy);

const double PhySimWifiPhy::m_normFactor = sqrt (64.0 * 64.0 / 52.0);
itpp::cvec PhySimWifiPhy::m_cachedPreamble;
bool PhySimWifiPhy::m_rngReset;

TypeId
PhySimWifiPhy::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::PhySimWifiPhy")
    .SetParent<WifiPhy> ()
    .AddConstructor<PhySimWifiPhy> ()
    .AddAttribute ("CcaModelThreshold",
                   "The energy of a received signal should be higher than this threshold (dbm) to allow "
                   "the PHY layer to declare CCA BUSY state. According to the IEEE 802.11 standard, this is "
                   "the minimum receiver sensitivity at lowest modulation plus 20 dB, which makes it -62 dBm"
                   "for 20 MHz channels, -65 dBm for 10 MHz channels and -68 dBm for 5 MHz channels",
                   DoubleValue (-62.0),
                   MakeDoubleAccessor (&PhySimWifiPhy::m_ccaMode1ThresholdDbm),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("EnergyDetectionThreshold",
                   "The energy detection that is used to decide whether the incoming frame is considered for reception "
                   "at all. If the energy is too low, it will be part of the thermal noise anyways.",
                   DoubleValue (-104.0),
                   MakeDoubleAccessor (&PhySimWifiPhy::m_edThresholdDbm),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("TxGain",
                   "Transmission gain (dB).",
                   DoubleValue (0.0),
                   MakeDoubleAccessor (&PhySimWifiPhy::m_txGainDb),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("RxGain",
                   "Reception gain (dB).",
                   DoubleValue (0.0),
                   MakeDoubleAccessor (&PhySimWifiPhy::m_rxGainDb),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("TxPowerLevels",
                   "Number of transmission power levels available between "
                   "TxPowerBase and TxPowerEnd included.",
                   UintegerValue (1),
                   MakeUintegerAccessor (&PhySimWifiPhy::m_nTxPower),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("TxPowerStart",
                   "Minimum available transmission level (dbm).",
                   DoubleValue (16.0206),
                   MakeDoubleAccessor (&PhySimWifiPhy::m_txPowerBaseDbm),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("TxPowerEnd",
                   "Maximum available transmission level (dbm).",
                   DoubleValue (16.0206),
                   MakeDoubleAccessor (&PhySimWifiPhy::m_txPowerEndDbm),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("NormalizeOFDMSymbols",
                   "Flag indicating whether we will normalize the OFDM symbols to unit power",
                   BooleanValue (true),
                   MakeBooleanAccessor (&PhySimWifiPhy::m_normalizeOfdmBlocks),
                   MakeBooleanChecker ())
    .AddAttribute ("CalculateHeaderSinr",
                   "Flag indicating whether to calculate the Header SINR or not (will be stored inside of PhySimWifiPhyTag for evaluation purposes, but increases processing time)",
                   BooleanValue (false),
                   MakeBooleanAccessor (&PhySimWifiPhy::m_calculateHeaderSinr),
                   MakeBooleanChecker ())
    .AddAttribute ("CalculatePayloadSinr",
                   "Flag indicating whether to calculate the Payload SINR or not (will be stored inside of PhySimWifiPhyTag for evaluation purposes, but increases processing time)",
                   BooleanValue (false),
                   MakeBooleanAccessor (&PhySimWifiPhy::m_calculatePayloadSinr),
                   MakeBooleanChecker ())
    .AddAttribute ("CalculateOverallSinr",
                   "Flag indicating whether to calculate the overall and average SINR of the frame or not (will be stored inside of PhySimWifiPhyTag for evaluation purposes, but increases processing time)",
                   BooleanValue (false),
                   MakeBooleanAccessor (&PhySimWifiPhy::m_calculateOverallSinr),
                   MakeBooleanChecker ())
    .AddAttribute ("Standard",
                   "The standard chosen configures a set of transmission modes and some PHY-specific constants.",
                   EnumValue (WIFI_PHY_STANDARD_80211a),
                   MakeEnumAccessor (&PhySimWifiPhy::ConfigureStandard),
                   MakeEnumChecker (WIFI_PHY_STANDARD_80211a, "802.11a - 20 MHz",
                                    WIFI_PHY_STANDARD_80211_10Mhz, "802.11a - 10 MHz",
                                    WIFI_PHY_STANDARD_80211_5Mhz, "802.11a - 5 MHz",
                                    WIFI_PHY_STANDARD_80211p_CCH, "802.11p - CCH",
                                    WIFI_PHY_STANDARD_80211p_SCH, "802.11p - SCH"))
    .AddAttribute ("State", "The state of the PHY layer",
                   PointerValue (),
                   MakePointerAccessor (&PhySimWifiPhy::m_state),
                   MakePointerChecker<PhySimWifiPhyStateHelper> ())
    .AddAttribute ("SymbolTime",
                   "Duration of an OFDM symbol (e.g. 4 microseconds for 20 MHz channels, 8 microseconds for 10 MHz channels)",
                   TimeValue (MicroSeconds (4)),
                   MakeTimeAccessor (&PhySimWifiPhy::SetSymbolTime),
                   MakeTimeChecker ())
    .AddTraceSource ("PreambleOk",
                     "A preamble has been detected successfully.",
                     MakeTraceSourceAccessor (&PhySimWifiPhy::m_preambleOkTrace))
    .AddTraceSource ("PreambleError",
                     "A preamble has not been detected successfully.",
                     MakeTraceSourceAccessor (&PhySimWifiPhy::m_preambleErrorTrace))
    .AddTraceSource ("RxOk",
                     "A packet has been received successfully.",
                     MakeTraceSourceAccessor (&PhySimWifiPhy::m_rxOkTrace))
    .AddTraceSource ("RxError",
                     "A packet has not been received successfully.",
                     MakeTraceSourceAccessor (&PhySimWifiPhy::m_rxErrorTrace))
    .AddTraceSource ("HeaderOk",
                     "A signal header has been decoded successfully.",
                     MakeTraceSourceAccessor (&PhySimWifiPhy::m_headerOkTrace))
    .AddTraceSource ("HeaderError",
                     "A signal header has not been decoded successfully.",
                     MakeTraceSourceAccessor (&PhySimWifiPhy::m_headerErrorTrace))
    .AddTraceSource ("Tx", "Packet transmission is starting.",
                     MakeTraceSourceAccessor (&PhySimWifiPhy::m_txTrace))
    .AddTraceSource ("CcaBusyStart", "Start of a CCA busy phase",
                     MakeTraceSourceAccessor (&PhySimWifiPhy::m_ccaBusyStartTrace))
    .AddAttribute ("TxCenterFrequencyTolerance",
                   "The transmitted center frequency tolerance (in ppm) according to 17.3.9.4 of the 802.11 standard."
                   "(defined as 20ppm for 20MHz and 10Mhz channels, 10ppm for 5Mhz channels)",
                   UintegerValue (20),
                   MakeUintegerAccessor (&PhySimWifiPhy::m_txCenterFreqTolerance),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("FixedPhaseOffset",
                   "Defines a fixed phase offset for oscillator effects instead of using a random integer value within the range specified by the "
                   "attribute 'TxCenterFrequencyTolerance'. If this value is set to 0 (default), it is ignored.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&PhySimWifiPhy::m_fixedPhaseOffset),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("Frequency",
                   "The carrier frequency at which the communication is performed (in Hz)",
                   DoubleValue (5.150e9),
                   MakeDoubleAccessor (&PhySimWifiPhy::m_frequency),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("ChannelEstimator",
                   "Defines the channel estimator implementation which shall be used for this Phy",
                   StringValue ("ns3::PhySimChannelFrequencyOffsetEstimator"),
                   MakeStringAccessor (&PhySimWifiPhy::SetChannelEstimator),
                   MakeStringChecker ())
    .AddAttribute ("SignalDetector",
                   "Defines the signal detector implementation which shall be used for this Phy",
                   StringValue ("ns3::PhySimSignalDetector"),
                   MakeStringAccessor (&PhySimWifiPhy::SetSignalDetector),
                   MakeStringChecker ())
  ;
  return tid;
}

PhySimWifiPhy::PhySimWifiPhy ()
  : m_endRxEvent (),
    m_random (0.0, 1.0),
    m_estimator (0)
{
  NS_LOG_FUNCTION (this);
  m_state = CreateObject<PhySimWifiPhyStateHelper> ();

  m_shortSymbol
    = sqrt (13.0 / 6.0) * itpp::cvec ("0 0 (1,1) 0 0 0 (-1,-1) 0 0 0 (1,1) 0 0 0 (-1,-1) 0 0 0 (-1,-1) 0 0 0 (1,1) 0 0 0 0 0 0 0 (-1,-1) 0 0 0 (-1,-1) 0 0 0 (1,1) 0 0 0 (1,1) 0 0 0 (1,1) 0 0 0 (1,1) 0 0");
  m_longSymbol
    = "1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 0 1 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 -1 1 1 -1 -1 1 -1 1 -1 1 1 1 1";

  /*
   * Define the binary values of the 4-bit rate field in an array.
   * Element
   *    0 --> BPSK_1/2,
   *    1 --> BPSK_2/3,
   *    2 --> QPSK_1/2,
   *    3 --> QPSK_3/4,
   *    4 --> QAM16_1/2
   *    5 --> QAM16_3/4,
   *    6 --> QAM64_2/3,
   *    7 --> QAM64_3/4
   */

  m_ofdmSymbolCreator = CreateObject<PhySimOFDMSymbolCreator> ();
  m_convEncoder = CreateObject<PhySimConvolutionalEncoder> ();
  m_interleaver = CreateObject<PhySimBlockInterleaver> ();
  m_scrambler = CreateObject<PhySimScrambler> ();
  m_interference = CreateObject<PhySimInterferenceHelper> ();

  m_estimator = m_ofdmSymbolCreator->GetChannelEstimator ();
}

PhySimWifiPhy::~PhySimWifiPhy ()
{
  NS_LOG_FUNCTION (this);
}

void
PhySimWifiPhy::SetChannelEstimator (std::string type)
{
  m_ofdmSymbolCreator->SetChannelEstimator (type);
  m_estimator = m_ofdmSymbolCreator->GetChannelEstimator ();
}

void
PhySimWifiPhy::SetSignalDetector (std::string type)
{
  ObjectFactory factory;
  factory.SetTypeId (type);
  m_signalDetector = factory.Create<PhySimSignalDetector> ();
}

void
PhySimWifiPhy::SetSymbolTime (Time duration)
{
  NS_LOG_FUNCTION (this << duration);
  m_symbolTime = duration;
  m_sampleTime = 0.05e-6 * (duration.GetMicroSeconds () / 4);
  m_interference->SetSymbolTime (duration);
}

void
PhySimWifiPhy::SetChannel (Ptr<PhySimWifiChannel> channel)
{
  m_channel = channel;
  m_channel->Add (this);
}

void
PhySimWifiPhy::SetDevice (Ptr<Object> device)
{
  m_device = device;
  m_state->SetNetDevice (device->GetObject<NetDevice> ());
}

void
PhySimWifiPhy::SetMobility (Ptr<Object> mobility)
{
  m_mobility = mobility;
}

Ptr<Object>
PhySimWifiPhy::GetDevice (void) const
{
  return m_device;
}

Ptr<Object>
PhySimWifiPhy::GetMobility (void)
{
  return m_mobility;
}

double
PhySimWifiPhy::GetTxPowerStart (void) const
{
  return m_txPowerBaseDbm;
}

double
PhySimWifiPhy::GetTxPowerEnd (void) const
{
  return m_txPowerEndDbm;
}

uint32_t
PhySimWifiPhy::GetNTxPower (void) const
{
  return m_nTxPower;
}

void
PhySimWifiPhy::SetReceiveOkCallback (WifiPhy::RxOkCallback callback)
{
  m_state->SetReceiveOkCallback (callback);
}

void
PhySimWifiPhy::SetReceiveErrorCallback (WifiPhy::RxErrorCallback callback)
{
  m_state->SetReceiveErrorCallback (callback);
}

void
PhySimWifiPhy::RegisterListener (WifiPhyListener *listener)
{
  m_state->RegisterListener (listener);
}

bool
PhySimWifiPhy::IsStateCcaBusy (void)
{
  return m_state->IsStateCcaBusy ();
}

bool
PhySimWifiPhy::IsStateIdle (void)
{
  return m_state->IsStateIdle ();
}

bool
PhySimWifiPhy::IsStateBusy (void)
{
  return m_state->IsStateBusy ();
}

bool
PhySimWifiPhy::IsStateRx (void)
{
  return m_state->IsStateSync ();
}

bool
PhySimWifiPhy::IsStateTx (void)
{
  return m_state->IsStateTx ();
}

bool
PhySimWifiPhy::IsStateSwitching (void)
{
  // TODO: Implement channel switching support once multi-channel operation is supported
  return false;
}

Time
PhySimWifiPhy::GetStateDuration (void)
{
  return m_state->GetStateDuration ();
}

Time
PhySimWifiPhy::GetDelayUntilIdle (void)
{
  return m_state->GetDelayUntilIdle ();
}

Time
PhySimWifiPhy::GetLastRxStartTime (void) const
{
  return m_state->GetLastRxStartTime ();
}

Time
PhySimWifiPhy::CalculateTxDuration (uint32_t size, WifiMode payloadMode, enum WifiPreamble preamble) const
{
  return m_interference->CalculateTxDuration (size, payloadMode, preamble);
}

uint32_t
PhySimWifiPhy::GetNModes (void) const
{
  return m_deviceRateSet.size ();
}

WifiMode
PhySimWifiPhy::GetMode (uint32_t mode) const
{
  return m_deviceRateSet[mode];
}

double
PhySimWifiPhy::CalculateSnr (WifiMode txMode, double ber) const
{
  // TODO: maybe fix that and return average SINR of the frame (if possible)
  // return m_interference->GetErrorRateModel ()->CalculateSnr (txMode, ber);
  return 0.0;
}

void
PhySimWifiPhy::SetChannelNumber (uint16_t id)
{
  // TODO: Implement channel switching support once multi-channel operation is supported
}

uint16_t
PhySimWifiPhy::GetChannelNumber () const
{
  // TODO: Implement channel switching support once multi-channel operation is supported
  return 0;
}

Ptr<WifiChannel>
PhySimWifiPhy::GetChannel (void) const
{
  return m_channel;
}

void
PhySimWifiPhy::ConfigureStandard (enum WifiPhyStandard standard)
{
  NS_LOG_FUNCTION (this << standard);
  m_deviceRateSet.clear ();
  switch (standard)
    {
    case WIFI_PHY_STANDARD_80211a:
      Configure80211a ();
      break;
    case WIFI_PHY_STANDARD_80211_10Mhz:
      Configure80211a_10Mhz ();
      break;
    case WIFI_PHY_STANDARD_80211_5Mhz:
      Configure80211a_5Mhz ();
      break;
    case WIFI_PHY_STANDARD_80211p_CCH:
      Configure80211p_CCH ();
      break;
    case WIFI_PHY_STANDARD_80211p_SCH:
      Configure80211p_SCH ();
      break;
    default:
      NS_ASSERT (false);
      break;
    }
}

void
PhySimWifiPhy::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  m_channel = 0;
  m_deviceRateSet.clear ();
  m_device = 0;
}

void
PhySimWifiPhy::Configure80211a (void)
{
  NS_LOG_FUNCTION (this);
  m_frequency = 5.2e9;
  SetSymbolTime ( MicroSeconds (4) );
  m_txCenterFreqTolerance = 20;
  m_ccaMode1ThresholdDbm = -62.0;
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate6Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate9Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate12Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate18Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate24Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate36Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate48Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate54Mbps ());
}

void
PhySimWifiPhy::Configure80211a_10Mhz (void)
{
  NS_LOG_FUNCTION (this);
  m_frequency = 5.2e9;
  SetSymbolTime ( MicroSeconds (8) );
  m_txCenterFreqTolerance = 20;
  m_ccaMode1ThresholdDbm = -65.0;
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate3MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate4_5MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate6MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate9MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate12MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate18MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate24MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate27MbpsBW10MHz ());
}

void
PhySimWifiPhy::Configure80211a_5Mhz (void)
{
  NS_LOG_FUNCTION (this);
  m_frequency = 5.2e9;
  SetSymbolTime ( MicroSeconds (16) );
  m_txCenterFreqTolerance = 10;
  m_ccaMode1ThresholdDbm = -68.0;
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate1_5MbpsBW5MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate2_25MbpsBW5MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate3MbpsBW5MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate4_5MbpsBW5MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate6MbpsBW5MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate9MbpsBW5MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate12MbpsBW5MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate13_5MbpsBW5MHz ());
}

void
PhySimWifiPhy::Configure80211p_CCH (void)
{
  NS_LOG_FUNCTION (this);
  m_frequency = 5.9e9;
  SetSymbolTime ( MicroSeconds (8) );
  m_txCenterFreqTolerance = 20;
  m_ccaMode1ThresholdDbm = -65.0;
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate3MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate4_5MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate6MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate9MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate12MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate18MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate24MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate27MbpsBW10MHz ());
}

void
PhySimWifiPhy::Configure80211p_SCH (void)
{
  NS_LOG_FUNCTION (this);
  m_frequency = 5.9e9;
  SetSymbolTime ( MicroSeconds (8) );
  m_txCenterFreqTolerance = 20;
  m_ccaMode1ThresholdDbm = -65.0;
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate3MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate4_5MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate6MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate9MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate12MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate18MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate24MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate27MbpsBW10MHz ());
}

void
PhySimWifiPhy::ResetRNG ()
{
  m_rngReset = false;
}

void
PhySimWifiPhy::ClearCache ()
{
  m_cachedPreamble.clear ();
  m_cachedPreamble.set_size (0);
}

void
PhySimWifiPhy::StartReceivePacket (Ptr<Packet> packet,
                                   Ptr<PhySimWifiPhyTag> tag)
{
  NS_LOG_FUNCTION (this << packet << tag );

  // Apply rxAntennaGain
  itpp::cvec rxedSamples = tag->GetRxedSamples ();
  if (m_rxGainDb != 0)
    {
      rxedSamples *= sqrt (PhySimHelper::DbToRatio (m_rxGainDb));
    }

  double rxPower = PhySimHelper::RatioToDb (PhySimHelper::GetOFDMSymbolSignalStrength (rxedSamples));
  NS_LOG_DEBUG ("PhySimWifiPhy:StartReceivePacket() " << Simulator::Now ().GetSeconds () << " sec - current state = " << m_state->GetState ());
  NS_LOG_DEBUG ("PhySimWifiPhy:StartReceivePacket() " << Simulator::Now ().GetSeconds () << " sec - rxedSamples.size() = " << rxedSamples.size ());
  NS_LOG_DEBUG ("PhySimWifiPhy:StartReceivePacket() " << Simulator::Now ().GetSeconds () << " sec - first OFDM symbol has signal strength of " << rxPower << " dBm");
  NS_LOG_DEBUG ("PhySimWifiPhy:StartReceivePacket() " << Simulator::Now ().GetSeconds () << " sec - m_edThresholdDbm = " << m_edThresholdDbm << " dBm");
  if (rxPower < m_edThresholdDbm)
    {
      return;
    }
#ifdef  NS3_LOG_ENABLE
  uint32_t blocks = (uint32_t)(std::floor (rxedSamples.size () / 80.0));
#endif
  Time rxDuration = CalculateTxDuration ( packet->GetSize (), tag->GetTxWifiMode (), tag->GetWifiPreamble () );

  Time endRx = Simulator::Now () + rxDuration;
  NS_LOG_DEBUG ( "PhySimWifiPhy:StartReceivePacket() " << Simulator::Now ().GetSeconds () <<
                 " sec - received frame with " << blocks << " OFDM symbols --> rxDuration = " <<
                 rxDuration.GetMicroSeconds () << " microseconds endRx = " << endRx.GetSeconds () << " sec" );

  // Create a packet event and add the packet to the interference helper list
  Ptr<PhySimInterferenceHelper::Event> event = m_interference->Add (packet, tag);

  switch (m_state->GetState ())
    {

    case PhySimWifiPhy::SYNCING:
    case PhySimWifiPhy::TX:

      // Only check for CCA busy condition in SYNC and TX state.
      CheckForNextCcaBusyStart ();
      NotifyRxDrop (packet);
      break;

    case PhySimWifiPhy::RX:

      NS_LOG_DEBUG ("PhySimWifiPhy:StartReceivePacket() " << Simulator::Now ().GetSeconds () <<
                    " sec - drop packet because already in " << m_state->GetState ());

      NotifyRxDrop (packet);
      break;

    case PhySimWifiPhy::CCA_BUSY:
    case PhySimWifiPhy::IDLE:

      Time preambleDuration = MicroSeconds ( 4.0 * m_symbolTime.GetMicroSeconds () );
      NS_LOG_LOGIC ("PhySimWifiPhy:StartReceivePacket() " << Simulator::Now ().GetSeconds () <<
                    " sec - will schedule a new EndSync event " << preambleDuration.GetMicroSeconds () << " microseconds ahead in the future");

      EventId endPreambleEvent = Simulator::Schedule (preambleDuration, &PhySimWifiPhy::EndPreamble, this,
                                                      packet,
                                                      tag,
                                                      event);
      m_endPreambleEvents.insert ( std::pair<Ptr<PhySimInterferenceHelper::Event>, EventId> (event, endPreambleEvent));

      CheckForNextCcaBusyStart ();

      break;
    }
}

void
PhySimWifiPhy::SendPacket (Ptr<const Packet> packet, WifiMode mode, enum WifiPreamble preamble, uint8_t txPowerLevel)
{
  NS_LOG_FUNCTION (this << packet << mode << preamble << (uint32_t) txPowerLevel);
  NS_LOG_DEBUG ("PhySimWifiPhy:SendPacket() " << Simulator::Now ());
  NS_LOG_DEBUG ("PhySimWifiPhy:SendPacket() current state = " << m_state->GetState ());

  // If the requested WiFi mode is not OFDM based, discard the request
  if (mode.GetModulationClass () != WIFI_MOD_CLASS_OFDM)
    {
      return;
    }

  /* Transmission can happen if:
   *  - we are syncing on a packet. It is the responsability of the
   *    MAC layer to avoid doing this but the PHY does nothing to
   *    prevent it.
   *  - we are idle
   */
  NS_ASSERT (!m_state->IsStateTx ());

  // Take the seed number from ns3::SeedManager and give it to the global
  // IT++ random number generator
  if (!m_rngReset)
    {
      itpp::RNG_reset (SeedManager::GetSeed ());
      m_rngReset = true;
    }

  // If no data bits are given, then we create a random bit sequence
  // otherwise, we take the bits that are available via packet->PeekData ()
  uint32_t length = packet->GetSize ();
  uint32_t sSizeBefore = packet->GetSerializedSize();
  uint8_t *payload = (uint8_t*) malloc (packet->GetSize ());
  memset (payload, 0, packet->GetSize ());
  packet->CopyData (payload, packet->GetSize ());
  uint32_t sSizeAfter = packet->GetSerializedSize();
  itpp::bvec bits;

  if (sSizeBefore == sSizeAfter)
    { // if there is custom payload provided, we'll use it
      for (uint32_t i = 0; i < length; i++)
        {
          uint8_t byte = payload[i];
          bits.ins (bits.size (), PhySimHelper::DecToBin (byte, 8));
        }
    }
  else
    { // else we create a random bit sequence
      bits = itpp::randb (length*8);
    }
  free (payload);

  NS_LOG_DEBUG ("PhySimWifiPhy:SendPacket() - packet->GetSize() = " << length << " bytes & bits.length() = " << bits.length ());
  NS_ASSERT (length >= 1 && length <= 4095); // MSPDU must adhere to these (Table 17-15 in Std.)

  // use cached preamble if existing
  if (m_cachedPreamble.size () < 320)
    {
      NS_LOG_DEBUG ("PhySimWifiPhy:SendPacket() - constructing preamble");
      m_cachedPreamble = ConstructPreamble ();
    }

  itpp::cvec c_header = ConstructSignalHeader (bits.length (), mode);
  itpp::cvec c_data = ConstructData (bits, mode);

  // compose final result
  itpp::cvec endresult = itpp::cvec (m_cachedPreamble);
  // add overlap of header with preamble
  endresult (endresult.size () - 1) += c_header (0);
  // add rest of header
  endresult.ins (endresult.size (), c_header (1, c_header.size () - 1));
  // add overlap of header with first DATA
  endresult (endresult.size () - 1) += c_data (0);
  // add rest of data
  endresult.ins (endresult.size (), c_data (1, c_data.size () - 1));

  // If we want to normalize the power of each OFDM symbol block
  if (m_normalizeOfdmBlocks)
    {
      endresult *= m_normFactor;
    }

  // Apply Transmission Power amplification and Antenna gain
  double txPowerDbm = PhySimHelper::GetPowerDbm (txPowerLevel, m_nTxPower, m_txPowerBaseDbm, m_txPowerEndDbm);
  endresult *= sqrt ( PhySimHelper::DbToRatio (txPowerDbm + m_txGainDb) );

  NS_LOG_DEBUG ("PhySimWifiPhy:SendPacket() txPowerDbm = " << txPowerDbm);
  NS_LOG_DEBUG ("PhySimWifiPhy:SendPacket() txAntGain = " << m_txGainDb);
  NS_LOG_DEBUG ("PhySimWifiPhy:SendPacket() applied a transmit power amplification factor of " << sqrt ( PhySimHelper::DbToRatio (txPowerDbm + m_txGainDb) ));

  // Cancel all running end(Preamble|Header|RX) events
  CancelAllRunningEndPreambleEvents ();
  CancelAllRunningEndHeaderEvents ();
  if (m_endRxEvent.IsRunning ())
    {
      m_endRxEvent.Cancel ();
    }
  NotifyTxBegin (packet);

  uint32_t dataRate500KbpsUnits = mode.GetDataRate () / 500000;
  bool isShortPreamble = (WIFI_PREAMBLE_SHORT == preamble);
  // TODO: Update call below once multi-channel operation is supported
  NotifyPromiscSniffTx (packet, (uint16_t) m_frequency, 0, dataRate500KbpsUnits, isShortPreamble);
  Time txDuration = CalculateTxDuration (packet->GetSize (), mode, preamble);

  // Apply oscillator effect of transmitter/receiver --this accounts for both
  int32_t phaseOffset;
  if (m_fixedPhaseOffset > 0)
    {
      phaseOffset = m_fixedPhaseOffset;
    }
  else
    {
      phaseOffset = (m_txCenterFreqTolerance != 0
                     ? itpp::randi (-2 * m_txCenterFreqTolerance, 2 * m_txCenterFreqTolerance)
                     : 0);
    }

  // Frequency offset tolerance is 10ppm for 5 MHz channel in 802.11 or 10 MHz vehicular; it is 20 for
  // 10 MHz and 20 MHz 802.11 transceivers (non-p). The offset is exp(j*2pi*Df*Ts*i), where Df=errorInPpm*Fc
  // In 802.11p, look up 17.3.9.4
  NS_LOG_DEBUG ("PhySimWifiPhy:SendPacket() - phaseOffset = " << phaseOffset << " (m_txCenterFreqTolerance = " << m_txCenterFreqTolerance << ")");
  double Df = phaseOffset * (1e-6) * m_frequency;
  double phaseFactor = 2.0 * M_PI * Df * m_sampleTime;
  NS_LOG_DEBUG ("PhySimWifiPhy:SendPacket() - phaseFactor = " << phaseFactor);

  itpp::cvec oscillatedOutput (endresult.size ());
  if (phaseFactor != 0)
    {
      for (int32_t i = 0; i < oscillatedOutput.size (); ++i)
        {
          oscillatedOutput (i) = endresult (i) * std::complex<double> (cos (phaseFactor * (i + 1)), sin (phaseFactor * (i + 1)));
        }
    }
  else
    {
      oscillatedOutput = endresult;
    }
  // Create PhySimWifiPhyTag, fill its attributes and send packet over the channel
  Ptr<PhySimWifiPhyTag> phyTag = Create<PhySimWifiPhyTag> ();
  phyTag->SetTxParameters (preamble, mode, txDuration, bits, endresult, m_frequency, m_sampleTime, m_device->GetObject<NetDevice> ());
  phyTag->SetTxPower (txPowerDbm);

  // Set the default RX samples to the one after oscillation effects have
  // been applied.
  phyTag->SetRxSamples (oscillatedOutput);

  m_state->SwitchToTx (txDuration, packet, phyTag);
  m_txTrace (packet, phyTag);

  NS_LOG_LOGIC ("PhySimWifiPhy:SendPacket() transmit packet " << packet->GetUid () << " with size of " << (8 * packet->GetSize ()) << " bits");
  NS_LOG_INFO ("PhySimWifiPhy:SendPacket() transmit duration: " << txDuration.GetMicroSeconds () << " microseconds");
  NS_LOG_DEBUG ("PhySimWifiPhy:SendPacket() samples consist of " << phyTag->GetTxedSamples ().size () << " samples");

  m_channel->Send (this, packet, phyTag);
}

itpp::cvec
PhySimWifiPhy::ConstructPreamble ()
{
  // OFDM the short symbol
  itpp::cvec ofdmShortSymbol = m_ofdmSymbolCreator->Modulate (m_shortSymbol);

  // Create the ShortSymbolSequence (This is twice the ofdmShortSymbol and the next 33 elements)
  itpp::cvec ofdmShortSymbolSequence (ofdmShortSymbol);
  ofdmShortSymbolSequence.ins (ofdmShortSymbolSequence.size (), ofdmShortSymbol);
  ofdmShortSymbolSequence.ins (ofdmShortSymbolSequence.size (), ofdmShortSymbol.mid (0, 33));
  PhySimHelper::WindowFunction (ofdmShortSymbolSequence); // apply window function

  // The following constructs the long training symbol sequence
  itpp::cvec ofdmFirstLongSymbol = m_ofdmSymbolCreator->Modulate (m_longSymbol, 32);
  itpp::cvec ofdmLongSymbolSequence (ofdmFirstLongSymbol);
  ofdmLongSymbolSequence.ins (ofdmLongSymbolSequence.size (), ofdmFirstLongSymbol.mid (32, 64));

  // The last element is a repetition of the first sample of the non-cp part of the OFDM symbol
  ofdmLongSymbolSequence.ins (ofdmLongSymbolSequence.size (), ofdmFirstLongSymbol (32));
  PhySimHelper::WindowFunction (ofdmLongSymbolSequence);

  // Construct the final preamble
  itpp::cvec preambleSequence (321);
  preambleSequence.replace_mid (0, ofdmShortSymbolSequence);
  preambleSequence (160) += ofdmLongSymbolSequence (0); // the overlap
  preambleSequence.replace_mid (161, ofdmLongSymbolSequence (1, 160));

  return preambleSequence;
}

itpp::cvec
PhySimWifiPhy::ConstructSignalHeader (uint32_t length, const WifiMode mode)
{
  NS_ASSERT ( length % 8 == 0 && length <= 4095 * 8 && length >= 8); // MSPDU restrictions (Table 17-15 in Std)

  // Signal header is always modulated with BPSK and 1/2 coding rate
  m_ofdmSymbolCreator->SetModulationType (PhySimHelper::BPSK, 2);
  m_convEncoder->SetCodingRate (WIFI_CODE_RATE_1_2);

  // Construct header
  int hdr_len = length / 8; // turn to bytes
  itpp::bvec PLCP_Header (24); // place the header here
  PLCP_Header.zeros ();

  // Add the rate bits
  PLCP_Header.replace_mid (0, PhySimHelper::CreateRateField (mode)); // position 0..3
  PLCP_Header (4) = 0; // reserved bit
  itpp::bvec length_bits = PhySimHelper::DecToBin (hdr_len, 12);
  PLCP_Header.replace_mid (5, length_bits); // length bits, position 5..16
  PLCP_Header (17) = PhySimHelper::CalcEvenParity (PLCP_Header); // add parity bit

  // Do convolutional encoding
  itpp::bvec encoded = m_convEncoder->Encode (PLCP_Header);

  // Do interleaving
  m_interleaver->SetWifiMode (WifiPhy::GetOfdmRate6Mbps ());
  itpp::bvec interleavedHeader = m_interleaver->InterleaveBlock (encoded);
  itpp::cvec modulated = (m_ofdmSymbolCreator->Modulate (interleavedHeader, 0));

  // Also add another sample which will overlap with the first DATA symbol
  modulated.ins (modulated.size (), modulated (16));

  // Apply window function
  PhySimHelper::WindowFunction (modulated);

  return modulated;
}

itpp::cvec
PhySimWifiPhy::ConstructData (const itpp::bvec& bits, const WifiMode mode)
{
  itpp::bvec serviceField (16);
  serviceField.zeros (); // 16-bit all zero service field
  itpp::bvec tailField (6);
  tailField.zeros (); // 6-bit zero tail field

  // Calculate no. of symbols needed and creating padding
  // Note that in the calculations below we use bits.length rather than 8 * length (in bytes)
  uint32_t NDBPS = PhySimHelper::GetNDBPS (mode); // No. of data bits per symbol
  uint32_t NSYM = ceil (static_cast<double> (16 + bits.length () + 6) / static_cast<double> (NDBPS)); // 17-11
  uint32_t NDATA = NSYM * NDBPS; // 17-12 - length of data field
  uint32_t NPAD = NDATA - (16 + bits.length () + 6); // no. of pad bits needed

  // Create the data bits (SERVICE+PSDU+Tail+Pad)
  itpp::bvec dataBits (NDATA);
  dataBits.zeros ();
  dataBits.replace_mid (0, serviceField); // 0..15 service field
  dataBits.replace_mid (16, bits); // 16..16+bits.length()-1 the bits
  dataBits.replace_mid (16 + bits.length (), tailField);

  // 17.3.5.4 apply PLCP scrambler
  itpp::bvec scrambledDATA = m_scrambler->Scramble (dataBits);

  // 17.3.2.1 (f) After scrambling we have to reset the tail bits to 0
  for (int i = 0; i < 6; ++i)
    {
      scrambledDATA (scrambledDATA.size () - 1 - NPAD - i) = 0;
    }

  // 17.3.5.5 Do convolutional encoding
  m_convEncoder->SetCodingRate (mode.GetCodeRate ());
  itpp::bvec encodedScrambledDATA = m_convEncoder->Encode (scrambledDATA);

  // 17.3.5.6 Do interleaving and 17.3.5.7 - 17.3.5.9 OFDM
  // for each block -- return the result
  m_interleaver->SetWifiMode (mode);
  itpp::cvec modulated = InterleaveAndModulate (encodedScrambledDATA, mode);

  return modulated;
}

itpp::bvec
PhySimWifiPhy::DeconstructData (Ptr<Packet> packet, Ptr<PhySimWifiPhyTag> tag)
{
  NS_LOG_FUNCTION (this << packet << tag);

  // read reception wifi mode and PDU length in bytes
  WifiMode mode = tag->GetRxWifiMode ();
  uint32_t length = 8 * tag->GetLength ();

  uint32_t tailNservice = 22;   // service field (16 bits) plus tail bits (6 bits)
  uint32_t ndbps = PhySimHelper::GetNDBPS (mode);
  uint32_t padding = ndbps - (tailNservice + length) % (ndbps);
  uint32_t numSymbols = ceil ( (tailNservice + length + padding) / ndbps); // see Equation 17-11

  NS_LOG_DEBUG ("PhySimWifiPhy:DeconstructData() rxWifiMode: " << tag->GetRxWifiMode ().GetUniqueName ());
  NS_LOG_DEBUG ("PhySimWifiPhy:DeconstructData() numSybmols: " << numSymbols);
  NS_LOG_DEBUG ("PhySimWifiPhy:DeconstructData() numDataBitsPerSymbol: " << ndbps);
  NS_LOG_DEBUG ("PhySimWifiPhy:DeconstructData() length: " << length);

  // Apply offset estimate correction from training sequence
  // For now: take the data stream and decode it
  Time now = Simulator::Now ();
  Time start = now - NanoSeconds (m_symbolTime.GetNanoSeconds () * numSymbols);
  NS_LOG_DEBUG ("PhySimWifiPhy:DeconstructData() calling m_interference->GetCumulativeSamples(" << start << "," << now << ")");
  itpp::cvec samples = m_interference->GetCumulativeSamples (start, now);
  double estimate = tag->GetInitialEstimate ();
  itpp::cvec Data = m_estimator->ApplyEstimateFromTrainingSequence (samples, estimate, 1);

  // De-Interleave and demodulate (we are NOT in the signal header anymore)
  itpp::vec encodedScrambledData = DeinterleaveAndModulate (Data, mode);
  NS_LOG_DEBUG ("PhySimWifiPhy:DeconstructData() encodedScrambledData.size(): " << encodedScrambledData.size () << " bits");

  // Do Viterbi decoding
  m_convEncoder->SetCodingRate (mode.GetCodeRate ());
  NS_LOG_DEBUG ("PhySimWifiPhy:DeconstructData() called m_convEncoder->SetCodingRate(" << mode.GetCodeRate () << ")");
  itpp::bvec scrambledData = m_convEncoder->Decode (encodedScrambledData);

  // Descramble
  itpp::bvec initialState = scrambledData (0, 6); // read the scrambling sequence from decoded DATA
  itpp::bvec DATA = m_scrambler->DeScramble ( scrambledData ( 7, scrambledData.size () - 1 ), initialState );

  // Calculate no. of symbols needed
  uint32_t nDATA = numSymbols * ndbps; // Equation 17-12 - length of data field
  itpp::bvec finalData = DATA (16, nDATA - padding - 6 - 1);

  return finalData;
}

itpp::cvec
PhySimWifiPhy::InterleaveAndModulate (const itpp::bvec &encodedScrambledData, const WifiMode mode)
{
  uint32_t NCBPS = PhySimHelper::GetNCBPS (mode);
  m_interleaver->SetWifiMode (mode);
  m_ofdmSymbolCreator->SetModulationType (PhySimHelper::GetModulationType (mode), mode.GetConstellationSize ());

  int32_t startIndex = 0;
  int32_t endIndex = NCBPS - 1;
  int32_t blockIndex = 0;
  itpp::cvec finalModulatedData;
  itpp::cvec block;

  while ( endIndex < encodedScrambledData.size () )
    {
      blockIndex++; // Next block

      // Interleave and modulate the block. Then apply the window function
      block = InterleaveAndModulateBlock (encodedScrambledData (startIndex, endIndex), blockIndex);

      // Need to add the first element of the actual OFDM time sample (not the prefix)
      // at the end. This will be added as an overlap with the next time sample
      std::complex<double> (lastelement) = block (16);
      block.ins (block.size (), lastelement);

      // Windowing function for block
      PhySimHelper::WindowFunction (block);

      // Consider the block overlap
      if (blockIndex == 1)
        {       // First block just add...
          finalModulatedData.ins (finalModulatedData.size (), block);
        }
      else
        {
          // Overlap block with Data
          finalModulatedData (finalModulatedData.size () - 1) += block (0);
          finalModulatedData.ins (finalModulatedData.size (), block (1, block.size () - 1));
        }
      startIndex += NCBPS;
      endIndex += NCBPS;
    }

  return finalModulatedData;
}

itpp::cvec
PhySimWifiPhy::InterleaveAndModulateBlock (const itpp::bvec& input, const uint32_t index)
{
  NS_ASSERT ( index > 0 );
  itpp::bvec interleaved = m_interleaver->InterleaveBlock (input);
  itpp::cvec modulated = m_ofdmSymbolCreator->Modulate (interleaved, index);

  return modulated;
}

itpp::vec
PhySimWifiPhy::DeinterleaveAndModulate (const itpp::cvec &input, const WifiMode mode)
{
  NS_ASSERT ( input.size () % 80 == 0 );

  m_interleaver->SetWifiMode (mode);
  m_ofdmSymbolCreator->SetModulationType (PhySimHelper::GetModulationType (mode), mode.GetConstellationSize ());

  // There are 80 time samples per OFDM symbol
  const uint32_t sampleSize = 80;
  int32_t startIndex = 0;
  int32_t endIndex = sampleSize - 1;
  int32_t blockIndex = 0;
  itpp::vec finalDecodedData;
  itpp::vec block;

  while (endIndex < input.size ())
    {
      blockIndex++; // next block
      block = DeinterleaveAndModulateBlock ( input (startIndex, endIndex), blockIndex );
      finalDecodedData.ins ( finalDecodedData.size (), block );
      startIndex += sampleSize;
      endIndex += sampleSize;
    }

  return finalDecodedData;
}

itpp::vec
PhySimWifiPhy::DeinterleaveAndModulateBlock (const itpp::cvec& input, uint32_t symbolNo)
{
  itpp::vec demodulated = m_ofdmSymbolCreator->DeModulate (input, symbolNo);
  return m_interleaver->DeinterleaveBlock (demodulated);
}

void
PhySimWifiPhy::EndPreamble (Ptr<Packet> packet, Ptr<PhySimWifiPhyTag> tag, Ptr<PhySimInterferenceHelper::Event> event)
{
  NS_LOG_FUNCTION (this << packet << tag << event);

  Time now = Simulator::Now ();
  // deviation time which is added around the exact preamble when getting cumulative samples
  Time devTime = MicroSeconds (2);
  uint64_t sampleDuration = m_symbolTime.GetNanoSeconds () / 80.0;
  uint32_t devSamples = (devTime.GetNanoSeconds () / sampleDuration);
  // get cumulative samples
  Time pre = NanoSeconds (4 * m_symbolTime.GetNanoSeconds ());
  itpp::cvec samples = m_interference->GetCumulativeSamples (now - pre - devTime, now + devTime);
  int32_t beginShortSymbols = -1;

  double sinr = m_interference->CalculatePreambleSinr (event);
  NS_LOG_DEBUG ("PhySimWifiPhy:EndPreamble() preamble of packet " << packet->GetUid () << " has SINR of " << sinr);
  tag->SetPreambleSinr (sinr);

  // Continue with reception if the preamble was detected successfully
  // or if we perform BER calculations only
  if ( (sinr > 0.0) && m_signalDetector->ScanPreamble (packet, tag, samples, beginShortSymbols)
       && (m_state->IsStateCcaBusy () || m_state->IsStateIdle () || !m_state->IsStateSync ()))
    {
      NS_ASSERT (beginShortSymbols >= 0);
      int32_t beginLongSymbols = m_signalDetector->ScanForLongSeq (packet, tag, samples (beginShortSymbols, samples.size () - 1));
      int32_t endShortSymbols = beginLongSymbols + beginShortSymbols - 32;
      beginLongSymbols = endShortSymbols + 1;
      beginShortSymbols = beginLongSymbols - 160;

      // add correction because we use it later without the deviation around the preamble
      tag->SetLongSymbolStart (beginLongSymbols - devSamples);

      NS_LOG_DEBUG ("PhySimWifiPhy:EndPreamble() preamble detected successfully");
      NS_LOG_DEBUG ("PhySimWifiPhy:EndPreamble()   -> beginShortSymbols = " << beginShortSymbols);
      NS_LOG_DEBUG ("PhySimWifiPhy:EndPreamble()   -> endShortSymbols = " << endShortSymbols);
      NS_LOG_DEBUG ("PhySimWifiPhy:EndPreamble()   -> beginLongSymbols = " << beginLongSymbols);
      NS_LOG_DEBUG ("PhySimWifiPhy:EndPreamble()   -> devSamples = " << devSamples);

      // Check whether we have "valid" begin/end samples for the long/short symbols
      // if not, discard this packet. Note: this should never happen, since the thresholds
      // of the signal detector should avoid such a situation before getting here
      if (beginShortSymbols < 0 || (beginLongSymbols + 160 > samples.size ()))
        {
          double sinr = m_interference->CalculatePreambleSinr (event);
          NS_LOG_DEBUG ("PhySimWifiPhy:EndPreamble() discarding packet, due to invalid start/end times for the preamble symbols");
          tag->SetPreambleSinr (sinr);
          m_preambleErrorTrace (packet, tag);
          goto EndPreambleWrapUp;
        }

      // estimate coarse and fine channel offsets
      m_estimator->Reset ();
      double offset = m_estimator->GetInitialChannelEstimation ( samples (beginShortSymbols, beginLongSymbols + 160 - 1) );
      tag->SetInitialEstimate (offset);

      // schedule a new EndHeader event and switch to SYNCING state
      Time headerDuration = m_symbolTime;
      NS_LOG_LOGIC ("PhySimWifiPhy:EndPreamble()   -> scheduling new event for endHeader " << headerDuration.GetMicroSeconds () << " microseconds ahead in the future.");
      EventId endHeaderEvent = Simulator::Schedule (headerDuration, &PhySimWifiPhy::EndHeader, this,
                                                    packet,
                                                    tag,
                                                    event);
      m_endHeaderEvents.insert ( std::pair<Ptr<PhySimInterferenceHelper::Event>, EventId> (event, endHeaderEvent));

      m_state->SwitchToSync (headerDuration, packet, tag);
      m_preambleOkTrace (packet, tag);
    }
  else
    {
      m_preambleErrorTrace (packet, tag);

      // If we want to know the overall average SINR of this frame later
      if (m_calculateOverallSinr)
        {
          double sinr = m_interference->CalculateOverallSinr (event);
          tag->SetOverallSinr (sinr);
        }
    }

EndPreambleWrapUp:
  // Don't forget to delete this event from the list of endPreamble events
  NS_LOG_DEBUG ("PhySimWifiPhy:EndPreamble() erasing endPreamble event from list, liste.size() = " << m_endPreambleEvents.size ());
  m_endPreambleEvents.erase (event);
  NS_LOG_DEBUG ("PhySimWifiPhy:EndPreamble() now, liste.size() = " << m_endPreambleEvents.size ());
}

void
PhySimWifiPhy::EndHeader (Ptr<Packet> packet, Ptr<PhySimWifiPhyTag> tag, Ptr<PhySimInterferenceHelper::Event> event)
{
  NS_LOG_FUNCTION (this << packet << tag << event);

  Time now = Simulator::Now ();
  itpp::cvec samples = m_interference->GetCumulativeSamples (now - m_symbolTime, now);

  // next, apply offset estimation correction which was obtained from the training sequence
  double estimate = tag->GetInitialEstimate ();
  NS_LOG_DEBUG ("PhySimWifiPhy:EndHeader() applying estimate from training symbols: " << estimate);
  itpp::cvec Signal = m_estimator->ApplyEstimateFromTrainingSequence (samples, estimate, 0);

  // Save information about SINR of the header part
  if (m_calculateHeaderSinr)
    {
      double sinr = m_interference->CalculateHeaderSinr (event);
      NS_LOG_DEBUG ("PhySimWifiPhy:EndHeader() signal header of packet " << packet->GetUid () << " has SINR of " << sinr);
      tag->SetHeaderSinr (sinr);
    }

  // Continue with reception if the signal header was decoded successfully
  if ( ScanSignalField (tag, Signal) && (m_state->IsStateCcaBusy () || m_state->IsStateIdle () || m_state->IsStateSync ()) )
    {

      WifiMode mode = tag->GetRxWifiMode ();
      uint32_t length = 8 * tag->GetLength ();  // transform to bits

      NS_LOG_DEBUG ("PhySimWifiPhy:EndHeader() successfully decoded signal header:");
      NS_LOG_DEBUG ("PhySimWifiPhy:EndHeader()   ->  detected wifiMode: " << mode.GetUniqueName ());
      NS_LOG_DEBUG ("PhySimWifiPhy:EndHeader()   -> should be wifiMode: " << tag->GetTxWifiMode ().GetUniqueName ());
      NS_LOG_DEBUG ("PhySimWifiPhy:EndHeader()   ->  detected length: " << length << " bits");
      NS_LOG_DEBUG ("PhySimWifiPhy:EndHeader()   -> should be length: " << (8 * packet->GetSize ()) << " bits");

      uint32_t tailNservice = 22;     // service field (16 bits) plus tail bits (6 bits)
      uint32_t ndbps = PhySimHelper::GetNDBPS (mode);
      uint32_t padding = ndbps - (tailNservice + length) % (ndbps);

      // number of data symbols times symbol time
      uint64_t delay = m_symbolTime.GetMicroSeconds () *
        ceil ( (length + tailNservice + padding) / ndbps );

      Time payloadDuration = MicroSeconds (delay);
      m_endRxEvent = Simulator::Schedule (payloadDuration, &PhySimWifiPhy::EndRx, this,
                                          packet,
                                          tag,
                                          event);
      NS_LOG_LOGIC ("PhySimWifiPhy:EndHeader() signal header decoded successfully -> scheduled new event " <<
                    payloadDuration.GetMicroSeconds () << " microseconds ahead in the future.");

      m_state->SwitchFromSyncEndOk (packet, tag);
      m_headerOkTrace (packet, tag);
      m_state->SwitchToRx (payloadDuration);

      // If we go into RX state, we should cancel all running EndPreamble and EndHeader events, since
      // we have no frame capture support yet
      CancelAllRunningEndPreambleEvents ();
      CancelAllRunningEndHeaderEvents ();

      // If we are going into RX state, we can safely cancel all CCA busy events that are scheduled
      // Why? Because we will check the CCA condition again after EndRx()
      m_startCcaBusyEvent.Cancel ();
      m_endCcaBusyEvent.Cancel ();
    }
  else
    {
      NS_LOG_DEBUG ("PhySimWifiPhy:EndHeader() signal header NOT decoded successfully");
      m_state->SwitchFromSyncEndError (packet, tag);
      m_headerErrorTrace (packet, tag);

      // If we want to know the overall average SINR of this frame later
      if (m_calculateOverallSinr)
        {
          double sinr = m_interference->CalculateOverallSinr (event);
          tag->SetOverallSinr (sinr);
        }
    }
}

void
PhySimWifiPhy::EndRx (Ptr<Packet> packet, Ptr<PhySimWifiPhyTag> tag, Ptr<PhySimInterferenceHelper::Event> event)
{

  NS_LOG_FUNCTION (this << packet << tag << event);
  itpp::bvec finalData = DeconstructData (packet, tag);
  tag->SetRxDataBits (finalData);

#ifdef NS3_LOG_ENABLE
  uint32_t bytes = finalData.size () / 8;
  std::string rxPayload;
  for (uint32_t i = 0; i < bytes; i++)
    {
      itpp::bvec extract = finalData ((i * 8), (i * 8) + 8 - 1);
      const char c = itpp::bin2dec ( extract, false );
      rxPayload.append (&c);
    }
  NS_LOG_INFO ("PhySimWifiPhy:EndRx() received " << finalData.size () << " data bits: " << finalData);
  NS_LOG_INFO ("PhySimWifiPhy:EndRx() message:  " << rxPayload);
#endif

  // Save information about the average SINR of the data part
  if (m_calculatePayloadSinr)
    {
      double sinr = m_interference->CalculatePayloadSinr (event);
      NS_LOG_DEBUG ("PhySimWifiPhy:EndRx() payload of packet " << packet->GetUid () << " has SINR of " << sinr);
      tag->SetPayloadSinr (sinr);
    }
  // If we want to know the overall average SINR of this frame later
  if (m_calculateOverallSinr)
    {
      double sinr = m_interference->CalculateOverallSinr (event);
      tag->SetOverallSinr (sinr);
    }

  if ( finalData == tag->GetTxedDataBits () )
    {
      NS_LOG_LOGIC ("PhySimWifiPhy:EndRx() payload of packet " << packet->GetUid () << " decoded successfully");
      m_state->SwitchFromRxEndOk (packet, tag);
      uint32_t dataRate500KbpsUnits = tag->GetRxWifiMode ().GetDataRate () / 500000;
      bool isShortPreamble = (WIFI_PREAMBLE_SHORT == tag->GetWifiPreamble ());
      // TODO: Update call below to provide proper signal energy levels and noise levels
      // TODO: Update the call below once multi-channel operations is supported
      NotifyPromiscSniffRx (packet, (uint16_t) m_frequency, 0, dataRate500KbpsUnits, isShortPreamble, -1.0, -1.0);

      // **********  SNR TAG  *********** //
      SnrTag tag_(m_interference->CalculateOverallSinr (event));
      if (! packet->PeekPacketTag (tag_))
        {
          packet->AddPacketTag (tag_);
        }
      // *********  SNR TAG  *********** //

      m_rxOkTrace (packet, tag);
    }
  else
    {
      NS_LOG_DEBUG ("PhySimWifiPhy:EndRx() payload of packet " << packet->GetUid () << " not decoded successful");
      m_state->SwitchFromRxEndError (packet, tag);
      m_rxErrorTrace (packet, tag);
    }

  // Ok, we have finished a reception, so we have to make sure we track the
  // CCA busy status appropriately
  CheckForNextCcaBusyStart ();
}

bool
PhySimWifiPhy::ScanSignalField (Ptr<PhySimWifiPhyTag> tag, const itpp::cvec &input)
{
  NS_ASSERT (input.size () == 80);

  // Demodulate
  m_ofdmSymbolCreator->SetModulationType (PhySimHelper::BPSK, 2);
  itpp::vec demodulated = (m_ofdmSymbolCreator->DeModulate (input, true));

  // De-Interleave
  m_interleaver->SetWifiMode (WifiPhy::GetOfdmRate6Mbps ());
  itpp::vec deInterleavedHeader = m_interleaver->DeinterleaveBlock (demodulated);

  // Viterbi-Decode
  m_convEncoder->SetCodingRate (WIFI_CODE_RATE_1_2);
  itpp::bvec decoded = m_convEncoder->Decode (deInterleavedHeader);

  // if parity bit is o.k.
  if (PhySimHelper::CheckSignalField (decoded))
    {
      // read rate field, if unsupported rate, cancel here already
      WifiMode mode;
      if (!PhySimHelper::GetWifiMode (decoded (0, 3), mode))
        {
          return false;
        }
      tag->SetRxWifiMode (mode);

      if (!(mode == tag->GetRxWifiMode ()))
        {
          NS_LOG_DEBUG ("PhySimWifiPhy:ScanSignalField() returning false because detected wifiMode != transmitted wifiMode");
          return false;
        }

      // read length and apply plausibility check
      NS_LOG_DEBUG ("PhySimWifiPhy:ScanSignalField() lengthField = " << decoded (5, 16));
      uint32_t length = itpp::bin2dec ( decoded (5, 16), false );
      NS_LOG_DEBUG ("PhySimWifiPhy:ScanSignalField() --> length = " << length);
      tag->SetLength (length);
      if (length < 1 || length > 4095)
        {
          NS_LOG_DEBUG ("PhySimWifiPhy:ScanSignalField() returning false because length field failed plausibility check");
          return false; // make sure the length makes sense 17.2.3.1
        }
      if (!(length == tag->GetLength ()))
        {
          NS_LOG_DEBUG ("PhySimWifiPhy:ScanSignalField() returning false because detected frame length != correct frame length");
          return false;
        }

      return true;
    }
  NS_LOG_DEBUG ("PhySimWifiPhy:ScanSignalField() returning false because CheckSignalField() failed");
  return false;
}

void
PhySimWifiPhy::CancelAllRunningEndPreambleEvents ()
{
  if (!m_endPreambleEvents.empty ())
    {
      std::map<Ptr<PhySimInterferenceHelper::Event>, EventId>::iterator it;
      for (it = m_endPreambleEvents.begin (); it != m_endPreambleEvents.end (); it++)
        {
          if (it->second.IsRunning ())
            {
              it->second.Cancel ();
            }
        }
      m_endPreambleEvents.clear ();
      NS_LOG_DEBUG ("PhySimWifiPhy::CancelAllRunningEndPreambleEvents() - canceled all running endPreambleEvents. list.size() = " << m_endPreambleEvents.size ());
    }
}

void
PhySimWifiPhy::CancelAllRunningEndHeaderEvents ()
{
  if (!m_endHeaderEvents.empty ())
    {
      std::map<Ptr<PhySimInterferenceHelper::Event>, EventId>::iterator it;
      for (it = m_endHeaderEvents.begin (); it != m_endHeaderEvents.end (); it++)
        {
          if (it->second.IsRunning ())
            {
              it->second.Cancel ();
            }
        }
      m_endHeaderEvents.clear ();
      NS_LOG_DEBUG ("PhySimWifiPhy::CancelAllRunningEndHeaderEvents() - canceled all running endHeaderEvents. list.size() = " << m_endHeaderEvents.size ());
    }
}

void
PhySimWifiPhy::CheckForNextCcaBusyStart ()
{
  NS_LOG_FUNCTION_NOARGS ();

  if (m_endCcaBusyEvent.IsRunning ())
    {
      return;
    }

  Time delayUntilCcaStart = MicroSeconds (0);

  if (m_interference->IsEnergyReached (m_ccaMode1ThresholdDbm, delayUntilCcaStart))
    {
      NS_LOG_DEBUG ("PhySimWifiPhy:CheckForNextCcaBusyStart() - Yes! Energy raises above " << m_ccaMode1ThresholdDbm << " dBm");
      // If the next CCA busy phase is starting already right now, skip
      // the scheduling of a start event and go directly to StartCcaBusy
      if (delayUntilCcaStart.IsZero ())
        {
          NS_LOG_DEBUG ("PhySimWifiPhy:CheckForNextCcaBusyStart() - it starts immediately.");
          StartCcaBusy ();
        }
      else
        {
          NS_LOG_DEBUG ("PhySimWifiPhy:CheckForNextCcaBusyStart() - its starts in the future, after " << delayUntilCcaStart);
          NS_LOG_DEBUG ("PhySimWifiPhy:CheckForNextCcaBusyStart()   -> Simulator::Now() = " << Simulator::Now ());
          NS_LOG_DEBUG ("PhySimWifiPhy:CheckForNextCcaBusyStart()   -> m_endCcaBusyEvent.IsRunning() = " << m_endCcaBusyEvent.IsRunning ());
          NS_LOG_DEBUG ("PhySimWifiPhy:CheckForNextCcaBusyStart()   -> m_endCcaBusyEvent.IsExpired() = " << m_endCcaBusyEvent.IsExpired ());
          NS_LOG_DEBUG ("PhySimWifiPhy:CheckForNextCcaBusyStart()   -> m_endCcaBusyEvent.GetTs() = " << m_endCcaBusyEvent.GetTs ());
          NS_LOG_DEBUG ("PhySimWifiPhy:CheckForNextCcaBusyStart()   -> m_startCcaBusyEvent.IsRunning() = " << m_startCcaBusyEvent.IsRunning ());
          NS_LOG_DEBUG ("PhySimWifiPhy:CheckForNextCcaBusyStart()   -> m_startCcaBusyEvent.IsExpired() = " << m_startCcaBusyEvent.IsExpired ());
          NS_LOG_DEBUG ("PhySimWifiPhy:CheckForNextCcaBusyStart()   -> m_startCcaBusyEvent.GetTs() = " << m_startCcaBusyEvent.GetTs ());

          // Make sure no other events are running
          NS_ASSERT (m_endCcaBusyEvent.IsExpired ());
          // Think about this here again. Can this really happen?
          // Explanation: it can happen that a new CCA start phase is detected, and due to
          //              a new incoming frame, the CCA threshold is hit already at an earlier stage
          Time absoluteTime = delayUntilCcaStart + Simulator::Now ();
          if (m_startCcaBusyEvent.IsRunning () && (m_startCcaBusyEvent.GetTs () > static_cast<uint64_t> (absoluteTime.GetNanoSeconds ())) )
            {
              // Cancel the running event...
              m_startCcaBusyEvent.Cancel ();

              // ... and schedule a new event at the new starting time
              NS_LOG_DEBUG ("PhySimWifiPhy:CheckForNextCcaBusyStart() - canceling old event, scheduling new m_startCcaBusyEvent at time = " << absoluteTime);
              m_startCcaBusyEvent = Simulator::Schedule (delayUntilCcaStart, &PhySimWifiPhy::StartCcaBusy, this);
            }
          else if (m_startCcaBusyEvent.IsRunning () && (m_startCcaBusyEvent.GetTs () <= static_cast<uint64_t> (absoluteTime.GetNanoSeconds ())) )
            {
              NS_LOG_DEBUG ("PhySimWifiPhy:CheckForNextCcaBusyStart() - now new event scheduled, since there's already an event scheduled.");
            }
          else
            {
              NS_LOG_DEBUG ("PhySimWifiPhy:CheckForNextCcaBusyStart() - scheduling m_startCcaBusyEvent at time = " << absoluteTime);
              m_startCcaBusyEvent = Simulator::Schedule (delayUntilCcaStart, &PhySimWifiPhy::StartCcaBusy, this);
            }
        }
    }
  else
    {
      NS_LOG_DEBUG ("PhySimWifiPhy:CheckForNextCcaBusyStart() - CCA busy threshold will not be reached in the future.");
    }
}

/**
 * Handles the start of a new CCA busy phase, as defined in the IEEE 802.11 2007 standard in Section 17.3.10.5.
 * It will calculate the duration of the state, schedule an event for the end of the CCA busy phase and instruct
 * the physical layer state helper (PhySimWifiStateHelper) to switch to CCA busy.
 */
void
PhySimWifiPhy::StartCcaBusy ()
{
  NS_LOG_FUNCTION_NOARGS ();

  // Do nothing if we are already in CCA busy and simply wait until the end
  // of the phase has been reached
  if (m_endCcaBusyEvent.IsRunning ())
    {
      NS_LOG_LOGIC ("PhySimWifiPhy:StartCcaBusy() - doing nothing since we are already CCA busy and endCcaBusyEvent is already scheduled");
      return;
    }

  Time delayUntilCcaEnd = m_interference->GetEnergyDuration (m_ccaMode1ThresholdDbm);

  NS_LOG_FUNCTION ("PhySimWifiPhy:StartCcaBusy() - We will be busy for " << delayUntilCcaEnd);

  // Continue only if we will be busy in the future
  if (!delayUntilCcaEnd.IsZero ())
    {
      NS_LOG_FUNCTION ("PhySimWifiPhy:StartCcaBusy() - will call m_state->SwitchMaybeToCcaBusy (delayUntilCcaEnd);");
      m_state->SwitchMaybeToCcaBusy (delayUntilCcaEnd);
      m_ccaBusyStartTrace (m_device->GetObject<NetDevice> (), delayUntilCcaEnd);

      // Check if there is already an event for the end of CCA busy scheduled or not
      if (m_endCcaBusyEvent.IsRunning ())
        {
          m_endCcaBusyEvent.Cancel ();
        }

      // Schedule event at the end of the CCA busy phase
      NS_LOG_FUNCTION ("PhySimWifiPhy:StartCcaBusy() - scheduling endCcaBusy event after " << delayUntilCcaEnd);
      m_endCcaBusyEvent = Simulator::Schedule (delayUntilCcaEnd, &PhySimWifiPhy::EndCcaBusy, this);
    }
  else
    {
      NS_LOG_FUNCTION ("PhySimWifiPhy:StartCcaBusy() - skipping busy phase, since delayUntilIdle = 0");
    }

  // Make sure that this event is canceled
  m_startCcaBusyEvent.Cancel ();
}

/**
 * Handles the end of a previously started CCA busy phase. In here, we will check when we have to switch
 * to CCA busy again (maybe the next start of a CCA busy phase is in the future due to fading), which could be
 * immediately or later. We also should make sure that m_endCcaBusyEvent and m_startCcaBusyEvent are not running
 * at this point in time.
 */
void
PhySimWifiPhy::EndCcaBusy ()
{
  NS_LOG_FUNCTION (this << Simulator::Now ());

  NS_ASSERT (m_endCcaBusyEvent.IsExpired ());
  NS_ASSERT (m_startCcaBusyEvent.IsExpired ());

  // Proceed only if we are in a state where we have to take care
  if (m_state->GetState () == PhySimWifiPhy::CCA_BUSY
      || m_state->GetState () == PhySimWifiPhy::IDLE
      || m_state->GetState () == PhySimWifiPhy::SYNCING
      || m_state->GetState () == PhySimWifiPhy::TX )
    {
      CheckForNextCcaBusyStart ();
    }
}

std::ostream& operator<< (std::ostream& os, enum PhySimWifiPhy::State state)
{
  switch (state)
    {
    case PhySimWifiPhy::IDLE:
      return (os << "IDLE");
    case PhySimWifiPhy::CCA_BUSY:
      return (os << "CCA_BUSY");
    case PhySimWifiPhy::TX:
      return (os << "TX");
    case PhySimWifiPhy::SYNCING:
      return (os << "SYNCING");
    case PhySimWifiPhy::RX:
      return (os << "RX");
    default:
      NS_FATAL_ERROR ("Invalid PhySimWifiPhy state");
      return (os << "INVALID");
    }
}

} // namespace ns3
