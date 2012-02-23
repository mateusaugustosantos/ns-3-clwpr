/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) Konstantinos Katsaros
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
 * Authors: Konstantinos Katsaros <K.Katsaros@surrey.ac.uk>
 */


#include "ns3/snr-tag.h"
#include "ns3/uinteger.h"


namespace ns3{

NS_OBJECT_ENSURE_REGISTERED (SnrTag);

TypeId
SnrTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SnrTag")
    .SetParent<Tag> ()
    .AddConstructor<SnrTag> ()
    .AddAttribute ("Snr", "The snr of the last packet received",
                   DoubleValue (0.0),
                   MakeDoubleAccessor (&SnrTag::Get),
                   MakeDoubleChecker<double> ())
  ;
  return tid;
}
TypeId
SnrTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t
SnrTag::GetSerializedSize (void) const
{
  return sizeof (double);
}
void
SnrTag::Serialize (TagBuffer i) const
{
  i.WriteDouble (m_snr);
}
void
SnrTag::Deserialize (TagBuffer i)
{
  m_snr = i.ReadDouble ();
}
void
SnrTag::Print (std::ostream &os) const
{
  os << "Snr=" << m_snr;
}
void
SnrTag::Set (double snr)
{
  m_snr = snr;
}
double
SnrTag::Get (void) const
{
  return m_snr;
}

} // namespace
