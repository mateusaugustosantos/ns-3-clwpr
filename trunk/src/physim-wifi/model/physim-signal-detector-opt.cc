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


#include "physim-signal-detector-opt.h"
#include "ns3/log.h"
#include "ns3/assert.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/boolean.h"
#include "ns3/double.h"


NS_LOG_COMPONENT_DEFINE ("PhySimSignalDetectorOpt");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (PhySimSignalDetectorOpt);

TypeId
PhySimSignalDetectorOpt::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::PhySimSignalDetectorOpt")
    .SetParent<PhySimSignalDetector> ()
    .AddConstructor<PhySimSignalDetectorOpt> ();
  return tid;
}

PhySimSignalDetectorOpt::PhySimSignalDetectorOpt ()
{
}

PhySimSignalDetectorOpt::~PhySimSignalDetectorOpt ()
{
}

/*!
 * Scans the preamble and returns an offset in the input vector where the preamble seems to match (the last element is where one of
 * the short training sequences ends). It should not take more than 2-3 training sequence lengths to figure out this in most conditions.
 * We return where we think the short training sequence begins. If we cannot identify the beginning we return false
 * and set begin to 0.
 *
 * @param input input time samples (must be at least 48)
 * @param begin returns sample offset where we think training sequence starts...
 * @return false if no positive signal detected, the offset of the beginning of the training sequence we have "locked on" otherwise
 */
bool
PhySimSignalDetectorOpt::ScanPreamble (Ptr<Packet> packet, Ptr<PhySimWifiPhyTag> tag, const itpp::cvec &input, int32_t &begin)
{
  begin = 0;
  if (input.size () < 48)
    {
      return false;   // we need a larger sample to work on...
    }
  int32_t elementsToScan;
  int32_t scanWindowSize;  // No elements to scan up to
  int32_t norminputwindow;

  // which correlation method to use
  double (PhySimSignalDetectorOpt::*corrPtr)(const itpp::cvec & input, const double normInputs, const int32_t L);

  if (m_autoCorrelation)
    {
      scanWindowSize = 48; // 16 samples for short training sequence and 32 for average window
      corrPtr = &PhySimSignalDetectorOpt::ComputeCorrelation;
      norminputwindow = 32;
      NS_LOG_DEBUG ("PhySimSignalDetectorOpt:ScanPreamble() will use auto-correlation method");
    }
  else
    {
      scanWindowSize = 16; // 16 samples for short training sequence to compare to reference
      norminputwindow = scanWindowSize;
      corrPtr = &PhySimSignalDetectorOpt::ComputeCorrelationUsingSampleSeq;
      NS_LOG_DEBUG ("PhySimSignalDetectorOpt:ScanPreamble() will use correlation with known samples method");
    }
  elementsToScan = input.size () - scanWindowSize;
  NS_ASSERT ( input.size () >= scanWindowSize );
  int32_t lastElementToScan;
  float corr;
  uint32_t noPositiveMatches = 0;
  double normInputs = 0;
  double prevnormvalue = 0; // records previous |z|^2 value so we can subtract it in the next round
  bool firstround = true;

  // collect correlation values for the TraceSource 'm_shortSymbolsTrace'
  std::list<double> correlations;

  NS_LOG_DEBUG ("PhySimSignalDetectorOpt:ScanPreamble()  - elementsToScan: " << elementsToScan);

  // Do the actual correlation scan
  for (int32_t firstElemToScan = 0; firstElemToScan <= elementsToScan; ++firstElemToScan)
    {
      // Make sure we don't go overboard
      lastElementToScan = firstElemToScan + (scanWindowSize - 1);
      if (lastElementToScan >= input.size ())
        {
          break; // end of sequence to scan
        }
      if (firstround)
        {
          prevnormvalue = norm (input (0));  // record the first normInput value in the window
          for (int i = 1; i < norminputwindow; ++i)
            {
              normInputs += norm (input (i));      // for each other element in the first window
            }
          normInputs += prevnormvalue;    // now add the first value as well
          firstround = false;
        }
      else      // use the previous normInputs value
        {
          // special case check: if our norminputwindow is different then we need to scan back a bit
          // then we should not update the normInput value, because we are in the
          normInputs -= prevnormvalue;      // subtract the previous windows' first value
          normInputs += norm (input (lastElementToScan - (scanWindowSize
                                                          - norminputwindow))); // add the new value of the new window
          prevnormvalue = norm (input (firstElemToScan));   // record this windows' first value
        }

      corr = (this->*corrPtr)(input (firstElemToScan, lastElementToScan), normInputs, scanWindowSize);
      NS_LOG_DEBUG ("PhySimSignalDetectorOpt:ScanPreamble() firstElemToScan: " << firstElemToScan << " corr: " << corr);
      if (m_enableTrace)
        {
          correlations.push_back (corr);
        }
      if (corr >= m_corrThresh)
        {
          if (noPositiveMatches == 0)
            { // if we haven't one before
              begin = firstElemToScan; // record beginning
            }
          ++noPositiveMatches; // leave this in, in case we want to look at more matches in the future
          break;
        }
    } // end of scan
  NS_LOG_DEBUG ("PhySimSignalDetectorOpt:ScanPreamble() begin = " << begin);
  if (m_enableTrace)
    {
      m_shortSymbolsTrace (packet, tag, correlations);
    }
  return (noPositiveMatches > 0);   // true if we have seen a match, false otherwise
}

/*!
 * Given an input, scans for the long sequence therein.. Should be used to
 * achieve correct symbol timing. The returned offset should provide reasonable
 * confidence into where the long training sequence begins (not accounting for
 * the GI - so the OFDM frame containing the long sequence starts 32 samples
 * earlier than what the offset indicates).
 * Presently, there should be two peaks. One is for the first and the second for the other
 * long training symbol. We look out for both the greatest peak and the second greatest
 * and return whichever one came earlier.
 *
 * @param input time samples which include short and long training sequences
 * @return the offset where the long training sequence begins
 */
int32_t
PhySimSignalDetectorOpt::ScanForLongSeq (Ptr<Packet> packet, Ptr<PhySimWifiPhyTag> tag, const itpp::cvec &input)
{
  // The input should be at least 64 samples long for proper
  // detection. If its not, we simply give the middle of the input as return result
  // in order to prevent errors in the implementation logic
  if (input.size () < 64)
    {
      return (uint32_t)(input.size () / 2);
    }
  // Do the scan
  int32_t maxPosFirst = -1;
  int32_t maxPosSecond = -1;
  int32_t lastElementToScan;
  double corr;
  double maxSeenFirst = 0;
  double maxSeenSecond = 0;
  double normInputs = 0; // keep track of sum of |z|^2 of window inputs
  bool firstround = true;

  // collect correlation values for the TraceSource 'm_longSymbolsTrace'
  std::list<double> correlations;

  for (int32_t i = 0; i <= input.size () - 64; ++i)
    {
      // Make sure we don't go overboard
      lastElementToScan = i + 63;
      if (lastElementToScan >= input.size ())
        {
          break; // end of sequence to scan
        }
      if (firstround)
        {
          for (int j = 0; j < 64; ++j)
            {
              normInputs += norm (input (j)); // calc sum of |z|^2 for each element in the first window
            }
          firstround = false;
        }
      else      // use the previous normInputs value
        {
          normInputs -= norm (input (i - 1)); // subtract the previous value
          normInputs += norm (input (lastElementToScan)); // add the new value
        }

      corr = ComputeCorrelationUsingSampleSeq (input (i, lastElementToScan), normInputs, 64);
      if (m_enableTrace)
        {
          correlations.push_back (corr);
        }

      if (corr > maxSeenFirst)
        { // new global maximum
          maxSeenSecond = maxSeenFirst;
          maxPosSecond = maxPosFirst; // make the old maximum the second highest
          maxSeenFirst = corr;
          maxPosFirst = i; // record position and value of new maximum
        }
      else if (corr > maxSeenSecond)
        { // new second maximum
          maxSeenSecond = corr;
          maxPosSecond = i;
        }
    }
  if (m_enableTrace)
    {
      m_longSymbolsTrace (packet, tag, correlations);
    }

  // TODO: There is probably some checks to be done w.r.t. the distance between the two peaks
  // but for now this will do
  if (maxSeenFirst == -1)
    {
      return 0; // if we have not found anything
    }
  if (maxSeenSecond == -1)
    {
      return maxPosFirst; // if no second best recorded then go with first guess
    }
  return ((maxPosFirst > maxPosSecond) ? maxPosSecond : maxPosFirst);
}

/*!
 * \brief Returns correlation for a given input using method 1 (auto-correlation).
 *
 */
double PhySimSignalDetectorOpt::ComputeCorrelation (const itpp::cvec& input, const double normInputs, const int32_t window)
{
  double L = window - 16;
  std::complex<double> sum = 0; // The moving sum
  // Calculate Cn
  for (int32_t k = 0; k < L; ++k)
    {
      sum += ( input ( (input.size () - 1) - k) * conj (input ((input.size () - 1) - k - 16)) );
    }
  // Now divide the two to come up with a correlation
  double corr = std::abs (sum) / normInputs;
  return corr > 1 ? 1 : corr;
}

/*!
* \brief Returns correlation for a given input using method 2 (expected values).
* Note that for this method the input must be 16 or 64 samples long
* which means short and long training sequences respectively.
*/
double PhySimSignalDetectorOpt::ComputeCorrelationUsingSampleSeq (const itpp::cvec& input, const double normInputs, const int32_t window)
{
  NS_ASSERT ( input.size () == window );
  const itpp::cvec* refSymbols;
  if (window == 16)
    {
      if (m_ieeeCompliantMode)
        {
          refSymbols = &PhySimHelper::m_refConjShortSymbolCompleteIEEE;
        }
      else
        {
          refSymbols = &PhySimHelper::m_refConjShortSymbolComplete;
        }
    }
  else if (window == 64)
    {
      if (m_ieeeCompliantMode)
        {
          refSymbols = &PhySimHelper::m_refConjLongSymbolCompleteIEEE;
        }
      else
        {
          refSymbols = &PhySimHelper::m_refConjLongSymbolComplete;
        }
    }
  else
    {
      NS_LOG_DEBUG ("PhySimSignalDetectorOpt:ComputeCorrelationUsingSampleSeqLogic() error in correlation computation");
      return 0;
    }

  std::complex<double> sum = 0; // the moving sum

  double normFactor = sqrt (normInputs);

  for (int32_t k = 0; k < window; ++k)
    {
      sum += input (k) * (*refSymbols)(k);  // itpp::conj(refSymbols(k));

    }
  sum *= normFactor; // apply normalization
  return abs (sum) / normInputs;
}
} // namespace ns3
