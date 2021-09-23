/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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

 * Created by:
 *  Giuseppe Avino, Politecnico di Torino (giuseppe.avino@polito.it)
 *  Marco Malinverno, Politecnico di Torino (marco.malinverno1@gmail.com)
 *  Francesco Raviglione, Politecnico di Torino (francescorav.es483@gmail.com)
*/

#include <vector>
#include <sstream>
#include "gps-raw-tc-helper.h"

namespace {
    std::vector<std::string> getNextLineAndSplitIntoTokens(std::string row)
    {
        std::vector<std::string>   result;

        std::stringstream          lineStream(row);
        std::string                cell;

        while(std::getline(lineStream,cell, ','))
        {
            result.push_back(cell);
        }
        // This checks for a trailing comma with no data after it
        if (!lineStream && cell.empty())
        {
            // If there was a trailing comma then add an empty element
            result.push_back("");
        }

        return result;
    }
}

namespace ns3 {
    NS_LOG_COMPONENT_DEFINE("GPSRawTraceClientHelper");

    GPSRawTraceClientHelper::GPSRawTraceClientHelper()
    {
        m_verbose=false;
        m_header=false;
        m_singletrace=true;
        m_numVehicles = 0;
    }

    std::map<std::string,std::shared_ptr<GPSRawTraceClient>>
    GPSRawTraceClientHelper::createRawTraceClientsFromCSV(std::string filepath)
    {
      // Variables to read the file
      std::ifstream inFile;
      std::string line;
      std::vector<std::string> result;
      uint8_t shift_idx = 0;
      double currentGNSStimestamp;
      double currentAbsolutetimestamp;

      // Variables to scan each row of the file
      std::string currentVehId;
      std::map<std::string,std::shared_ptr<GPSRawTraceClient>> m_GPSRawTraceClient;

      if(m_numVehicles == 0 && m_singletrace == true)
      {
          NS_FATAL_ERROR("Error: attempted to use createRawTraceClientsFromCSV with a single trace and 0 total vehicles.");
      }

      inFile.open(filepath);
      if (!inFile)
      {
          NS_FATAL_ERROR("Unable to open the file: "<<filepath);
      }

      // Skip the header
      if (m_header) {
         std::getline(inFile, line);
      }

      if(m_singletrace == true)
      {
          for(int i=0;i<m_numVehicles;i++)
          {
              std::shared_ptr<GPSRawTraceClient> gpsrawclient = std::shared_ptr<GPSRawTraceClient>(new GPSRawTraceClient(std::to_string(i)));
              m_GPSRawTraceClient.insert(std::make_pair(std::to_string(i), gpsrawclient));
          }
      }

      while (std::getline(inFile, line))
      {
          std::istringstream iss(line);
          result = getNextLineAndSplitIntoTokens(line);

          currentGNSStimestamp = 0;
          currentAbsolutetimestamp = 0;

          // Get the vehicle id (if m_singletrace is not 'false')
          if(m_singletrace == false) {
              currentVehId = result[0];
              shift_idx = 1;
          }

          // Check if the element is present in the map m_GPSTraceClient (only when the log file contains data related to different vehicles)
          // Otherwise, if m_singletrace is true, the same Raw GPS Data trace will be used for all the vehicles (and m_numVehicles will be considered)
          if (m_singletrace == false && m_GPSRawTraceClient.find(currentVehId) == m_GPSRawTraceClient.end() ) {
              // Not found - needed to create the entry for such a vector
              std::shared_ptr<GPSRawTraceClient> gpsrawclient = std::shared_ptr<GPSRawTraceClient>(new GPSRawTraceClient(currentVehId));
              m_GPSRawTraceClient.insert(std::make_pair(currentVehId, gpsrawclient));
          }

          currentGNSStimestamp = std::stod(result[2 + shift_idx]);
          currentAbsolutetimestamp = std::stod(result[1 + shift_idx]);

          // Full precision interframe
          if(result[0 + shift_idx]=="I") {
              GDP::satmap<GPSRawTraceClient::iframe_data_t> iframedata;

              long numrows = stol(result[3 + shift_idx]);
              // Read the current "I" frame data
              for(int i=0;i<numrows;i++)
              {
                  std::getline(inFile, line);
                  std::istringstream iss(line);
                  result = getNextLineAndSplitIntoTokens(line);

                  if(result.size() < 9)
                  {
                      NS_FATAL_ERROR ("Error. Cannot read file " << filepath << ". A line with 9 fields was expected, but only " << result.size() << " were found.");
                  }

                  std::pair<int,int> map_idx = std::pair<int,int>(std::stoi(result[1]),std::stoi(result[0]));

                  iframedata[map_idx].pseudorange = std::stod(result[2]);
                  iframedata[map_idx].pseudorange_uncertainty = std::stod(result[3]);
                  iframedata[map_idx].carrierphase = std::stod(result[4]);
                  iframedata[map_idx].carrierphase_uncertainty = std::stod(result[5]);
                  iframedata[map_idx].doppler = std::stod(result[6]);
                  iframedata[map_idx].doppler_uncertainty = std::stod(result[7]);
                  iframedata[map_idx].signalstrength = std::stol (result[8]);
              }

              for(int i=0;m_singletrace == true && i<m_numVehicles;i++) {
                  m_GPSRawTraceClient[std::to_string(i)]->setNewIFrame(currentGNSStimestamp,currentAbsolutetimestamp,iframedata);
              }

              if(m_singletrace == false) {
                  m_GPSRawTraceClient[currentVehId]->setNewIFrame(currentGNSStimestamp,currentAbsolutetimestamp,iframedata);
              }
          // Differential intra-frame
          } else if(result[0]=="D") {
              // Read the current "D" frame data
              GDP::satmap<GPSRawTraceClient::dframe_data_t> dframedata;

              long numrows = stol(result[3 + shift_idx]);

              // Read the current "D" frame data
              for(int i=0;i<numrows;i++)
              {
                  std::getline(inFile, line);
                  std::istringstream iss(line);
                  result = getNextLineAndSplitIntoTokens(line);

                  if(result.size() < 5)
                  {
                      NS_FATAL_ERROR ("Error. Cannot read file " << filepath << ". A line with 5 fields was expected, but only " << result.size() << " were found.");
                  }

                  std::pair<int,int> map_idx = std::pair<int,int>(std::stoi(result[1]),std::stoi(result[0]));

                  dframedata[map_idx].differential_pseudorange = std::stod(result[2]);
                  dframedata[map_idx].differential_carrierphase = std::stod(result[3]);
                  dframedata[map_idx].differential_doppler = std::stod(result[4]);
              }

              for(int i=0;m_singletrace == true && i<m_numVehicles;i++) {
                  m_GPSRawTraceClient[std::to_string(i)]->setNewDFrame(currentGNSStimestamp,currentAbsolutetimestamp,dframedata);
              }

              if(m_singletrace == false) {
                  m_GPSRawTraceClient[currentVehId]->setNewDFrame(currentGNSStimestamp,currentAbsolutetimestamp,dframedata);
              }
          } else {
              NS_FATAL_ERROR("Error: wrong format in file: " + filepath);
          }
      }

//      // Sort for each object the vector "vehiclesdata" according to the timestamp
//      for(std::map<std::string,GPSRawTraceClient*>::iterator it=m_GPSRawTraceClient.begin(); it!=m_GPSRawTraceClient.end(); ++it) {
//          it->second->sortRawdata ();
//      }

      return m_GPSRawTraceClient;
    }
  }
