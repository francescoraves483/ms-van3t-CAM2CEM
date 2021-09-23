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
#include <memory>

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
              double pseudorange;
              double pseudorange_uncertainty;
              int64_t carrierphase;
              double carrierphase_uncertainty;
              double doppler;
              double doppler_uncertainty;
              long int signalstrength;
          } iframe_data_t;

          typedef struct _dframe_data {
              double differential_pseudorange;
              double differential_carrierphase;
              double differential_doppler;
          } dframe_data_t;

          typedef struct _raw_positioning_data {
              e_frametype type;
              double cemTstamp;
              double absTstamp;

              GDP::satmap<iframe_data_t> iframe_data;
              GDP::satmap<dframe_data_t> dframe_data;

              // To order the vector according to "cemTstamp"
              bool operator < (const _raw_positioning_data& str) const
              {
                  return (absTstamp < str.absTstamp);
              }
          } raw_positioning_data_t;

          GPSRawTraceClient(std::string vehID);
          virtual ~GPSRawTraceClient();
          void sortRawdata();
          void printDebugRawData();

          // Setter
          void setNewIFrame(double cemTstamp, double abstimestamp, GDP::satmap<iframe_data_t> data);
          void setNewDFrame(double cemTstamp, double abstimestamp, GDP::satmap<dframe_data_t> data);

          // Getter
          raw_positioning_data_t getLastFrameData();
          std::string getID();

          // Start "playing" the raw GPS trace
          void playTrace(Time const &delay);

          // Set the functions to create/destroy node
          void GPSRawTraceClientSetup(Ptr<Node> veh_node,std::function<void(Ptr<Node>)> end_fcn);

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

