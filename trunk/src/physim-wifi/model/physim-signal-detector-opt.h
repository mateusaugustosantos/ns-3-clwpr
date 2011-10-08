/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009-2010 Stylianos Papanastasiou, Jens Mittag
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
 * Stylianos Papanastasiou <stylianos@gmail.com>
 * Jens Mittag <jens.mittag@kit.edu>
 *
 */

#ifndef PHYSIM_SIGNALDETECTOROPT_H
#define PHYSIM_SIGNALDETECTOROPT_H

#include "ns3/object.h"
#include "physim-helper.h"
#include "physim-wifi-phy-tag.h"
#include "physim-signal-detector.h"
#include "ns3/traced-callback.h"
#include "ns3/callback.h"
#include "ns3/packet.h"
#include "ns3/ptr.h"
#include <itpp/itcomm.h>

namespace ns3 {
/*!
 * \brief 802.11a/p signal detector (optimised for speed).
 *
 *  A module implementing a two signal detection methods (A&B) as detailed in [Liu03].
 *  This version uses more memory but is faster than the other one.
 *
 * References:
 * [Liu03] Chia-Horng Liu, "On the design of OFDM signal detection algorithms for hardware implementation,"
 *              Global Telecommunications Conference, 2003. GLOBECOM '03. IEEE , vol.2, no.,
 *              pp. 596- 599 Vol.2, 1-5 Dec. 2003
 */
class PhySimSignalDetectorOpt : public PhySimSignalDetector
{
public:
  static TypeId GetTypeId (void);

  PhySimSignalDetectorOpt ();
  virtual ~PhySimSignalDetectorOpt ();

  bool ScanPreamble (Ptr<Packet> packet, Ptr<PhySimWifiPhyTag> tag, const itpp::cvec &input, int32_t &begin);
  int32_t ScanForLongSeq (Ptr<Packet> packet, Ptr<PhySimWifiPhyTag> tag, const itpp::cvec &input);
  double ComputeCorrelation (const itpp::cvec& input, const double normInputs, const int32_t window);
  double ComputeCorrelationUsingSampleSeq (const itpp::cvec& input, const double normInputs, const int32_t window);
};

} // namespace ns3

#endif /* PHYSIM_SIGNALDETECTOR_H */
