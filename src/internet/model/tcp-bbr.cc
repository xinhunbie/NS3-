/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2013 ResiliNets, ITTC, University of Kansas 
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
 * Authors: Siddharth Gangadhar <siddharth@ittc.ku.edu>,
 *          Truc Anh N. Nguyen <annguyen@ittc.ku.edu>,
 *          Greeshma Umapathi
 *
 * James. P.G. Sterbenz <jpgs@ittc.ku.edu>, director
 * ResiliNets Research Group  http://wiki.ittc.ku.edu/resilinets
 * Information and Telecommunication Technology Center (ITTC)
 * and Department of Electrical Engineering and Computer Science
 * The University of Kansas Lawrence, KS USA.
 *
 * Work supported in part by NSF FIND (Future Internet Design) Program
 * under grant CNS-0626918 (Postmodern Internet Architecture),
 * NSF grant CNS-1050226 (Multilayer Network Resilience Analysis and Experimentation on GENI),
 * US Department of Defense (DoD), and ITTC at The University of Kansas.
 */

#include "tcp-bbr.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "rtt-estimator.h"
#include "tcp-socket-base.h"
#include <iostream>

NS_LOG_COMPONENT_DEFINE ("TcpBbr");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (TcpBbr);

TypeId
TcpBbr::GetTypeId (void)
{
  static TypeId tid = TypeId("ns3::TcpBbr")
    .SetParent<TcpNewReno>()
    .SetGroupName ("Internet")
    .AddConstructor<TcpBbr>()
    .AddAttribute("FilterType", "Use this to choose no filter or Tustin's approximation filter",
                  EnumValue(TcpBbr::TUSTIN), MakeEnumAccessor(&TcpBbr::m_fType),
                  MakeEnumChecker(TcpBbr::NONE, "None", TcpBbr::TUSTIN, "Tustin"))
    .AddAttribute("ProtocolType", "Use this to let the code run as Bbr or BbrPlus",
                  EnumValue(TcpBbr::Bbr),
                  MakeEnumAccessor(&TcpBbr::m_pType),
                  MakeEnumChecker(TcpBbr::Bbr, "Bbr",TcpBbr::BbrPLUS, "BbrPlus"))
    .AddTraceSource("EstimatedBW", "The estimated bandwidth",
                    MakeTraceSourceAccessor(&TcpBbr::m_currentBW),
                    "ns3::TracedValueCallback::Double")
  ;
  return tid;
}

TcpBbr::TcpBbr (void) :
  TcpNewReno (),
  m_currentBW (0), 
  m_lastSampleBW (0),
  m_lastBW (0),
  m_minRtt (Time (0)),
  m_ackedSegments (0),
  m_IsCount (false),
  m_alpha (2),
  m_beta (4),
  m_gamma (1),
  m_baseRtt (Time::Max ()),
  m_cntRtt (0),
  m_doingBbrNow (true),
  m_begSndNxt (0),
  m_minRttInterval (10),
  m_maxBwdRttCount (8),
  m_rttCounter(1),
  m_maxBwd(0),
  m_slowStart(true),
  m_slowStartPacketscount(3),
  m_drainFactor(0.8),
  m_probeFactor(1.2),
  m_currentRtt(Time(0))

{
  NS_LOG_FUNCTION (this);
}

TcpBbr::TcpBbr (const TcpBbr& sock) :
  TcpNewReno (sock),
  m_currentBW (sock.m_currentBW),
  m_lastSampleBW (sock.m_lastSampleBW),
  m_lastBW (sock.m_lastBW),
  m_minRtt (Time (0)),
  m_pType (sock.m_pType),
  m_fType (sock.m_fType),
  m_IsCount (sock.m_IsCount),
  m_alpha (sock.m_alpha),
    m_beta (sock.m_beta),
    m_gamma (sock.m_gamma),
    m_baseRtt (sock.m_baseRtt),
    m_cntRtt (sock.m_cntRtt),
    m_doingBbrNow (true),
    m_begSndNxt (0),
  m_minRttInterval(sock.m_minRttInterval),
  m_maxBwdRttCount(sock.m_maxBwdRttCount),
  m_rttCounter(sock.m_rttCounter),
  m_maxBwd(sock.m_maxBwd),
  m_slowStart(sock.m_slowStart),
  m_slowStartPacketscount(sock.m_slowStartPacketscount),
  m_drainFactor(sock.m_drainFactor),
  m_probeFactor(sock.m_probeFactor),
  m_currentRtt(sock.m_currentRtt)

{
  NS_LOG_FUNCTION (this);
  NS_LOG_LOGIC ("Invoked the copy constructor");
}

TcpBbr::~TcpBbr (void)
{
}


//map functionality to get min RTT and MaxThroughput 
//Saturday
void
TcpBbr::PktsAcked (Ptr<TcpSocketState> tcb, uint32_t packetsAcked,
                        const Time& rtt)
{
  NS_LOG_FUNCTION (this << tcb << packetsAcked << rtt);





  if (rtt.IsZero ())
    {
      NS_LOG_WARN ("RTT measured is zero!");
      return;
    }

  std::cerr<<"PktsAcked: current Timestamp: "<<rtt.GetSeconds()<<std::endl;

// min rtt window
  m_ackedSegments += packetsAcked;
 // m_minRtt = Time::Max();
    if(m_rttWindow.size()>200) //We need to set window size to be 8 Rtts instead of 8 packets received
  
    m_rttWindow.erase(m_rttWindow.begin());//erase oldest
    for(std::map<uint32_t,Time>::iterator it = m_rttWindow.begin();it!=m_rttWindow.end();++it )
    {                  std::cerr<<"PktsAcked: rtt window "<<it->second.GetSeconds()<<std::endl;
 
  m_minRtt = std::min(m_minRtt,it->second);}
  m_rttWindow[m_rttCounter]= rtt;


  
//max bdw window
  EstimateBW (rtt, tcb);
  m_maxBwd = 0;
  if(m_bdwWindow.size()>300) //We need to set window size to be 10 secs instead of 8 packets received
  
    m_bdwWindow.erase(m_bdwWindow.begin());//erase oldest
  for(std::map<uint32_t,double>::iterator it = m_bdwWindow.begin();it!=m_bdwWindow.end();++it )
  {  m_maxBwd = std::max(m_maxBwd,it->second);
                std::cerr<<"PktsAcked: bdw  "<<it->second<<std::endl;

   }
  
  m_bdwWindow[m_rttCounter] = m_currentBW;

 //   m_maxBwd =m_currentBW;
   
  if(m_timeWindow.size()>200) //We need to set window size to be 8 Rtts instead of 8 packets received
  
    m_timeWindow.erase(m_timeWindow.begin());//erase oldest
  m_timeWindow[m_rttCounter]= Simulator::Now();

  m_rttCounter++;
  m_currentRtt = rtt;



//  m_baseRtt = std::min (m_baseRtt, rtt);
 // m_cntRtt++;

/*
  // Update minRtt
  if (m_minRtt.IsZero ())
    {
      m_minRtt = rtt;
    }
  else
    {
      if (rtt < m_minRtt)
        {
          m_minRtt = rtt;
        }
    }

  NS_LOG_LOGIC ("MinRtt: " << m_minRtt.GetMilliSeconds () << "ms");

  if (m_pType == TcpBbr::Bbr)
    {
      EstimateBW (rtt, tcb);
    }
  else if (m_pType == TcpBbr::BbrPLUS)
    {
      if (!(rtt.IsZero () || m_IsCount))
        {
          m_IsCount = true;
          m_bwEstimateEvent.Cancel ();
          m_bwEstimateEvent = Simulator::Schedule (rtt, &TcpBbr::EstimateBW,
                                                   this, rtt, tcb);
        }
    }*/
}

//Sunday - Monday
//One day to wrap up and test out throughput
//already have max throughtput and minRtt every packet received
//
//slow start & drain phase
//probe max Bnadwidth & minRtt
//adjust sending rate on cwnd

//for today
//implement window 
//estimate bdw properly
//tune parameters
// need to log estimate bandwidth to see if it works properly
//in conclusion
//estimating bandwidth is not correct and thus we end up with situations where increasewindow is not called very usually


void
TcpBbr::IncreaseWindow (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
  NS_LOG_FUNCTION (this << tcb << segmentsAcked);

 /* if (!m_doingBbrNow)
    {
      // If Bbr is not on, we follow NewReno algorithm
      NS_LOG_LOGIC ("Bbr is not turned on, we follow NewReno algorithm.");
      TcpNewReno::IncreaseWindow (tcb, segmentsAcked);
      return;
    }*/
  //slow start
  tcb->m_ssThresh = 2147483646;
   std::cerr<<"IncreaseWindow: minRtt"<<m_minRtt.GetSeconds()<<std::endl;

          /*std::cerr<<"IncreaseWindow: m_currentBW: "<<m_currentBW<<std::endl;
   std::cerr<<"IncreaseWindow: drain1: "<<m_maxBwd<<std::endl;
          std::cerr<<"IncreaseWindow: drain2: "<<m_currentBW<<std::endl;
          std::cerr<<"IncreaseWindow: drain3: "<<fabs(m_maxBwd-m_currentBW)<<std::endl;*/
  if (true||m_slowStart)
    {
      std::cerr<<"IncreaseWindow: slowstart max bandwidth: "<<m_maxBwd<<std::endl;
      std::cerr<<"IncreaseWindow: slowstart current bandwidth: "<<m_currentBW<<std::endl;
      std::cerr<<"IncreaseWindow: slowstart min rtt: "<<m_minRtt.GetSeconds()<<std::endl;
      std::cerr<<"IncreaseWindow: slowstart current rtt: "<<m_currentRtt.GetSeconds()<<std::endl;

      segmentsAcked = TcpNewReno::SlowStart (tcb, segmentsAcked);
      if(m_slowStartPacketscount>=1)
        m_slowStartPacketscount--;
      if(m_slowStartPacketscount<1 && fabs(m_maxBwd-m_currentBW)<0.2*m_maxBwd)
        m_slowStart = false;

    }



    //drain phase, bandwidth stays the same but Rtt increases
    else if(fabs(m_maxBwd-m_currentBW)<0.1*m_maxBwd && m_currentRtt.GetSeconds()/m_minRtt.GetSeconds()>1.1)//if (tcb->m_lastAckedSeq >= m_begSndNxt)
    {
          std::cerr<<"IncreaseWindow: drain1: "<<m_maxBwd<<std::endl;
          std::cerr<<"IncreaseWindow: drain2: "<<m_currentBW<<std::endl;
          std::cerr<<"IncreaseWindow: drain3: "<<fabs(m_maxBwd-m_currentBW)<<std::endl;


   // m_cWnd = m_cWnd*m_drainFactor;
      tcb->m_cWnd = tcb->m_cWnd*m_drainFactor;
            std::cerr<<"IncreaseWindow: drain phase window "<<tcb->m_cWnd<<std::endl;

  }
  //probe phase
   else { // A Bbr cycle has finished, we do Bbr cwnd adjustment every RTT.

    tcb->m_cWnd = tcb->m_cWnd*m_probeFactor;
              std::cerr<<"IncreaseWindow: probe phase max bandwidth: "<<m_maxBwd<<std::endl;
            std::cerr<<"IncreaseWindow: probe phase current bandwidth: "<<m_currentBW<<std::endl;

                std::cerr<<"IncreaseWindow: probe phase window"<<tcb->m_cWnd <<std::endl;

      // NS_LOG_LOGIC ("A Bbr cycle has finished, we adjust cwnd once per RTT.");

      // // Save the current right edge for next Bbr cycle
      // m_begSndNxt = tcb->m_nextTxSequence;

      // /*
      //  * We perform Bbr calculations only if we got enough RTT samples to
      //  * insure that at least 1 of those samples wasn't from a delayed ACK.
      //  */
      // if (m_cntRtt <= 2)
      //   {  // We do not have enough RTT samples, so we should behave like Reno
      //     NS_LOG_LOGIC ("We do not have enough RTT samples to do Bbr, so we behave like NewReno.");
      //     TcpNewReno::IncreaseWindow (tcb, segmentsAcked);
      //   }
      // else
      //   {
      //     NS_LOG_LOGIC ("We have enough RTT samples to perform Bbr calculations");
      //     /*
      //      * We have enough RTT samples to perform Bbr algorithm.
      //      * Now we need to determine if cwnd should be increased or decreased
      //      * based on the calculated difference between the expected rate and actual sending
      //      * rate and the predefined thresholds (alpha, beta, and gamma).
      //      */
      //     uint32_t diff;
      //     uint32_t targetCwnd;
      //     uint32_t segCwnd = tcb->GetCwndInSegments ();

      //     /*
      //      * Calculate the cwnd we should have. baseRtt is the minimum RTT
      //      * per-connection, minRtt is the minimum RTT in this window
      //      *
      //      * little trick:
      //      * desidered throughput is currentCwnd * baseRtt
      //      * target cwnd is throughput / minRtt
      //      */
      //     double tmp = m_baseRtt.GetSeconds () / m_minRtt.GetSeconds ();
      //     targetCwnd = segCwnd * tmp;
      //     NS_LOG_DEBUG ("Calculated targetCwnd = " << targetCwnd);
      //     NS_ASSERT (segCwnd >= targetCwnd); // implies baseRtt <= minRtt

      //     /*
      //      * Calculate the difference between the expected cWnd and
      //      * the actual cWnd
      //      */
      //     diff = segCwnd - targetCwnd;
      //     NS_LOG_DEBUG ("Calculated diff = " << diff);

      //     if (diff > m_gamma && (tcb->m_cWnd < tcb->m_ssThresh))
      //       {
              
      //          * We are going too fast. We need to slow down and change from
      //          * slow-start to linear increase/decrease mode by setting cwnd
      //          * to target cwnd. We add 1 because of the integer truncation.
               
      //         NS_LOG_LOGIC ("We are going too fast. We need to slow down and "
      //                       "change to linear increase/decrease mode.");
      //         segCwnd = std::min (segCwnd, targetCwnd + 1);
      //         tcb->m_cWnd = segCwnd * tcb->m_segmentSize;
      //         tcb->m_ssThresh = GetSsThresh (tcb, 0);
      //         NS_LOG_DEBUG ("Updated cwnd = " << tcb->m_cWnd <<
      //                       " ssthresh=" << tcb->m_ssThresh);
      //       }
      //     else if (tcb->m_cWnd < tcb->m_ssThresh)
      //       {     // Slow start mode
      //         NS_LOG_LOGIC ("We are in slow start and diff < m_gamma, so we "
      //                       "follow NewReno slow start");
      //         segmentsAcked = TcpNewReno::SlowStart (tcb, segmentsAcked);
      //       }
      //     else
      //       {     // Linear increase/decrease mode
      //         NS_LOG_LOGIC ("We are in linear increase/decrease mode");
      //         if (diff > m_beta)
      //           {
      //             // We are going too fast, so we slow down
      //             NS_LOG_LOGIC ("We are going too fast, so we slow down by decrementing cwnd");
      //             segCwnd--;
      //             tcb->m_cWnd = segCwnd * tcb->m_segmentSize;
      //             tcb->m_ssThresh = GetSsThresh (tcb, 0);
      //             NS_LOG_DEBUG ("Updated cwnd = " << tcb->m_cWnd <<
      //                           " ssthresh=" << tcb->m_ssThresh);
      //           }
      //         else if (diff < m_alpha)
      //           {
      //             // We are going too slow (having too little data in the network),
      //             // so we speed up.
      //             NS_LOG_LOGIC ("We are going too slow, so we speed up by incrementing cwnd");
      //             segCwnd++;
      //             tcb->m_cWnd = segCwnd * tcb->m_segmentSize;
      //             NS_LOG_DEBUG ("Updated cwnd = " << tcb->m_cWnd <<
      //                           " ssthresh=" << tcb->m_ssThresh);
      //           }
      //         else
      //           {
      //             // We are going at the right speed
      //             NS_LOG_LOGIC ("We are sending at the right speed");
      //           }
      //       }
      //     tcb->m_ssThresh = std::max (tcb->m_ssThresh, 3 * tcb->m_cWnd / 4);
      //     NS_LOG_DEBUG ("Updated ssThresh = " << tcb->m_ssThresh);
    //    }
        
      // Reset cntRtt & minRtt every RTT
    //  m_cntRtt = 0;
    //  m_minRtt = Time::Max ();

    }
  /*else if (tcb->m_cWnd < tcb->m_ssThresh)
    {
      segmentsAcked = TcpNewReno::SlowStart (tcb, segmentsAcked);
    }*/
}

//estimate the goodput instead of throughput 
void
TcpBbr::EstimateBW (const Time &rtt, Ptr<TcpSocketState> tcb)
{
//  NS_LOG_FUNCTION (this);

  //NS_ASSERT (!rtt.IsZero ());

// m_currentBW = m_ackedSegments * tcb->m_segmentSize / rtt.GetSeconds ();
 // m_lastSampleBW = m_currentBW;
  //m_currentBW = (double)tcb->m_cWnd / rtt.GetSeconds();


//approximate accordding to last 20 packets received
  if(m_timeWindow.size()<=21)
      m_currentBW = (double)tcb->m_cWnd / rtt.GetSeconds();
    else {
      auto it = m_timeWindow.crbegin();
      auto ittwenty = it;
      for(int i =0;i<20;i++)
        ittwenty++;
    //  std::map<uint32_t,Time>::iterator itold = m_rttWindow.end()-20;

      m_currentBW= (double)20*tcb->m_segmentSize/(it->second.GetSeconds()-ittwenty->second.GetSeconds());
    }
    std::cerr<< "EstimateBW: raw bdw"<<m_currentBW<<" "<<std::endl;

  //std::cerr<< "cur segmentsize"<<tcb->m_segmentSize<<" "<<std::endl;
//  std::cerr<<"EstimateBW: rtt is "<<rtt.GetSeconds() <<std::endl;
 //   std::cerr<<"EstimateBW: cwnd is "<<tcb->m_cWnd <<std::endl;
     //std::cerr<<"m_ackedSegments is "<<m_ackedSegments <<std::endl;

  // if (m_pType == TcpBbr::BbrPLUS)
  //   {
  //     m_IsCount = false;
  //   }
  //if not in rtt
  m_ackedSegments = 0;
  //NS_LOG_LOGIC ("Estimated BW: " << m_currentBW);

  // Filter the BW sample

  double alpha = 0.9;

  m_currentBW = alpha*m_lastBW+(1-alpha)*m_currentBW;
  m_lastBW = m_currentBW;
  // if (m_fType == TcpBbr::NONE)
  //   {
  //   }
  // else if (m_fType == TcpBbr::TUSTIN)
  //   {
      double sample_bwe = m_currentBW;
      m_currentBW = (alpha * m_lastBW) + ((1 - alpha) * ((sample_bwe + m_lastSampleBW) / 2));
      m_lastSampleBW = sample_bwe;
      m_lastBW = m_currentBW;
  //  }
    std::cerr<< "EstimateBW: bdw"<<m_currentBW<<" "<<std::endl;

  //NS_LOG_LOGIC ("Estimated BW after filtering: " << m_currentBW);
}

uint32_t
TcpBbr::GetSsThresh (Ptr<const TcpSocketState> tcb,
                          uint32_t bytesInFlight)
{
  (void) bytesInFlight;
  NS_LOG_LOGIC ("CurrentBW: " << m_currentBW << " minRtt: " <<
                m_minRtt << " ssthresh: " <<
                m_currentBW * static_cast<double> (m_minRtt.GetSeconds ()));
  return 2147483646;
 // return std::max (2*tcb->m_segmentSize,
                   //uint32_t (m_currentBW * static_cast<double> (m_minRtt.GetSeconds ())));
}

Ptr<TcpCongestionOps>
TcpBbr::Fork ()
{
  return CreateObject<TcpBbr> (*this);
}

} // namespace ns3
