/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 Konstantinos Katsaros
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
 * Author: Konstantinos Katsaros <K.Katsaros@surrey.ac.uk>
 *
 */

#ifndef CLWPR_HEADER_H
#define CLWPR_HEADER_H

#include <stdint.h>
#include <vector>
#include "ns3/header.h"
#include "ns3/ipv4-address.h"
#include "ns3/nstime.h"
#include "ns3/vector.h"

namespace ns3 {
namespace clwpr {

double EmfToSeconds (uint8_t emf);
uint8_t SecondsToEmf (double seconds);

// 3.3.  Packet Format
//
//    The basic layout of any packet in clwpr is as follows (omitting IP and
//    UDP headers):
//
//        0                   1                   2                   3
//        0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |         Packet Length         |    Packet Sequence Number     |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |  Message Type |     Vtime     |         Message Size          | 
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+         
//       |  Time To Live |   Hop Count   |    Message Sequence Number    |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |                      Originator Address                       |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |                                                               |
//       :                            MESSAGE                            :
//       |                                                               |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       :                                                               :
//       :                                                               :
//                (etc.)
class PacketHeader : public Header
{
public:
  PacketHeader ();
  virtual ~PacketHeader ();

  void SetPacketLength (uint16_t length)
  {
    m_packetLength = length;
  }
  uint16_t GetPacketLength () const
  {
    return m_packetLength;
  }

  void SetPacketSequenceNumber (uint16_t seqnum)
  {
    m_packetSequenceNumber = seqnum;
  }
  uint16_t GetPacketSequenceNumber () const
  {
    return m_packetSequenceNumber;
  }

private:
  uint16_t m_packetLength;
  uint16_t m_packetSequenceNumber;

public:  
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual void Print (std::ostream &os) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
};


//        0                   1                   2                   3
//        0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |  Message Type |     Vtime     |         Message Size          |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |  Time To Live |   Hop Count   |    Message Sequence Number    |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |                      Originator Address                       |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

class MessageHeader : public Header
{
public:

  enum MessageType {
    HELLO_MESSAGE = 1,
    HNA_MESSAGE   = 4,
  };

  MessageHeader ();
  virtual ~MessageHeader ();

  void SetMessageType (MessageType messageType)
  {
    m_messageType = messageType;
  }
  MessageType GetMessageType () const
  {
    return m_messageType;
  }

  void SetVTime (Time time)
  {
    m_vTime = SecondsToEmf (time.GetSeconds ());
  }
  Time GetVTime () const
  {
    return Seconds (EmfToSeconds (m_vTime));
  }

  void SetOriginatorAddress (Ipv4Address originatorAddress)
  {
    m_originatorAddress = originatorAddress;
  }
  Ipv4Address GetOriginatorAddress () const
  {
    return m_originatorAddress;
  }

  void SetTimeToLive (uint8_t timeToLive)
  {
    m_timeToLive = timeToLive;
  }
  uint8_t GetTimeToLive () const
  {
    return m_timeToLive;
  }

  void SetHopCount (uint8_t hopCount)
  {
    m_hopCount = hopCount;
  }
  uint8_t GetHopCount () const
  {
    return m_hopCount;
  }

  void SetMessageSequenceNumber (uint16_t messageSequenceNumber)
  {
    m_messageSequenceNumber = messageSequenceNumber;
  }
  uint16_t GetMessageSequenceNumber () const
  {
    return m_messageSequenceNumber;
  }

   void SetMessageSize (uint16_t messageSize)
   {
     m_messageSize = messageSize;
   }
   uint16_t GetMessageSize () const
   {
     return m_messageSize;
   }
  
private:
  MessageType m_messageType;
  uint8_t m_vTime;
  Ipv4Address m_originatorAddress;
  uint8_t m_timeToLive;
  uint8_t m_hopCount;
  uint16_t m_messageSequenceNumber;
  uint16_t m_messageSize;

public:  
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual void Print (std::ostream &os) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);

// HELLO Message Format
//
//     0                   1                   2                   3
//     0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//    |                                                               |
//    |                                                               |
//    |              ORIGINATOR POSITION (X,Y)                        |
//    |                                                               |
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//    |                                                               |
//    |              ORIGINATOR VELOCITY (X,Y)                        |
//    |                                                               |
//    |                                                               |
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//    |                   ORIGINATOR HEADING                          |
//    |                                                               |
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//    | Orig.Road_ID  |    Htime      |          Utilization          |
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//    |        Utilization            |       MAC INFO                |
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ 
//    |         MAC INFO              |     Carry-n-Forward           |
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

  struct Hello
  {
    Vector position;
    double heading;
    Vector velocity;
    uint8_t roadId;
    uint8_t hTime;
    void SetHTime (Time time)
    {
      this->hTime = SecondsToEmf (time.GetSeconds ());
    }
    Time GetHTime () const
    {
      return Seconds (EmfToSeconds (this->hTime));
    }

    uint32_t utilization;
    uint32_t macInfo;
    uint16_t CnFinfo;
    
//    std::vector<LinkMessage> linkMessages;

    void Print (std::ostream &os) const;
    uint32_t GetSerializedSize (void) const;
    void Serialize (Buffer::Iterator start) const;
    uint32_t Deserialize (Buffer::Iterator start, uint32_t messageSize);
  };



  // 12.1.  HNA Message Format
  //
  //    The proposed format of an HNA-message is:
  //
  //        0                   1                   2                   3
  //        0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                         Network Address                       |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                             Netmask                           |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                         Network Address                       |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                             Netmask                           |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                              ...                              |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

  // Note: HNA stands for Host Network Association
  struct Hna
  {
    struct Association
    {
      Ipv4Address address;
      Ipv4Mask mask;
    };
    std::vector<Association> associations;

    void Print (std::ostream &os) const;
    uint32_t GetSerializedSize (void) const;
    void Serialize (Buffer::Iterator start) const;
    uint32_t Deserialize (Buffer::Iterator start, uint32_t messageSize);
  };

private:
  struct
  {
    Hello hello;
    Hna hna;
  } m_message; // union not allowed

public:



  Hello& GetHello ()
  {
    if (m_messageType == 0)
      {
        m_messageType = HELLO_MESSAGE;
      }
    else
      {
        NS_ASSERT (m_messageType == HELLO_MESSAGE);
      }
    return m_message.hello;
  }


  Hna& GetHna ()
  {
    if (m_messageType == 0)
      {
        m_messageType = HNA_MESSAGE;
      }
    else
      {
        NS_ASSERT (m_messageType == HNA_MESSAGE);
      }
    return m_message.hna;
  }


  const Hello& GetHello () const
  {
    NS_ASSERT (m_messageType == HELLO_MESSAGE);
    return m_message.hello;
  }


  const Hna& GetHna () const
  {
    NS_ASSERT (m_messageType == HNA_MESSAGE);
    return m_message.hna;
  }

  
};


static inline std::ostream& operator<< (std::ostream& os, const PacketHeader & packet)
{
  packet.Print (os);
  return os;
}

static inline std::ostream& operator<< (std::ostream& os, const MessageHeader & message)
{
  message.Print (os);
  return os;
}

typedef std::vector<MessageHeader> MessageList;

static inline std::ostream& operator<< (std::ostream& os, const MessageList & messages)
{
  os << "[";
  for (std::vector<MessageHeader>::const_iterator messageIter = messages.begin ();
       messageIter != messages.end (); messageIter++)
    {
      messageIter->Print (os);
      if (messageIter+1 != messages.end ())
        os << ", ";
    }
  os << "]";
  return os;
}


}} // namespace clwpr, ns3

#endif /* CLWPR_HEADER_H */

