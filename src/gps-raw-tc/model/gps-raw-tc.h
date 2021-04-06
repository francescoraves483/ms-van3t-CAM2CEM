/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef GPS_RAW_TC_H
#define GPS_RAW_TC_H

// To use std::sort
#include <algorithm>

#include <functional>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <variant>

#include "ns3/core-module.h"
#include "ns3/node.h"
#include "ns3/gdp.h"

namespace ns3 {

  class GPSRawTraceClient : public Object
  {
      public:
          typedef enum {
            FULL_PRECISION_I_FRAME,
            DIFFERENTIAL_D_FRAME
          } e_frametype;

          typedef struct _iframe_data {
              GDP::satmap<double> pseudorange;
              GDP::satmap<double> pseudorange_uncertainty;
              GDP::satmap<int64_t> carrierphase;
              GDP::satmap<double> carrierphase_uncertainty;
              GDP::satmap<double> doppler;
              GDP::satmap<double> doppler_uncertainty;
              GDP::satmap<long int> signalstrength;
          } iframe_data_t;

          typedef struct _dframe_data {
              GDP::satmap<double> differential_pseudorange;
              GDP::satmap<double> differential_carrierphase;
              GDP::satmap<double> differential_doppler;
          } dframe_data_t;

          typedef struct _raw_positioning_data {
              e_frametype type;
              uint64_t cemTstamp;

              iframe_data_t iframe_data;
              dframe_data_t dframe_data;

              // To order the vector according to "cemTstamp"
              bool operator < (const _raw_positioning_data& str) const
              {
                  return (cemTstamp < str.cemTstamp);
              }
          } raw_positioning_data_t;

          GPSRawTraceClient(std::string vehID);
          virtual ~GPSRawTraceClient();
          void sortRawdata();
          void printDebugRawData();

          // Setter
          void setNewIFrame(uint64_t cemTstamp,iframe_data_t data);
          void setNewDFrame(uint64_t cemTstamp,dframe_data_t data);

          // Getter
          raw_positioning_data_t getLastFrameData();
          std::string getID();

          // Start "playing" the raw GPS trace
          void playTrace(Time const &delay);

          // Set the functions to create/destroy node
          void GPSRawTraceClientSetup(std::function<Ptr<Node>()> start_fcn,std::function<void(Ptr<Node>)> end_fcn);

          // Stop "playing" the trace
          void StopUpdates();

          // Function to set the callbacks to be called when new data for an "I" or "D" packet is available
          void setFrameCallback(std::function<void(raw_positioning_data_t)> frame_callback)
          {
            m_frame_callback=frame_callback;
          }

      private:
          std::vector<raw_positioning_data_t> vehiclesdata;
          long unsigned int m_lastvehicledataidx;
          std::string m_vehID;

          // Function pointers to the start/end trace callbacks
          std::function<Ptr<Node>()> m_startTrace;
          std::function<void(Ptr<Node>)> m_endTrace;

          void UpdateRawData();
          EventId m_event_updaterawdata;

          Ptr<Node> m_vehNode;

          void CreateNode(void);
          void UpdatePositions(void);

          // New data available callback
          std::function<void(raw_positioning_data_t)> m_frame_callback;
  };


  }

#endif /* GPS_TC_H */

