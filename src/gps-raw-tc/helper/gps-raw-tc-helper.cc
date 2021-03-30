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

namespace ns3 {
    NS_LOG_COMPONENT_DEFINE("GPSRawTraceClientHelper");

    GPSRawTraceClientHelper::GPSRawTraceClientHelper()
    {
        m_verbose=false;
        m_header=false;
        m_singletrace=true;
        m_increasingVehID=0;
    }

    std::map<std::string,GPSRawTraceClient*>
    GPSRawTraceClientHelper::createRawTraceClientsFromCSV(std::string filepath)
    {
      // Variables to read the file
      std::ifstream              inFile;
      std::string                line;
      std::vector<std::string>   result;

      // Variables to scan each row of the file
      std::string currentVehId;
      std::map<std::string,GPSRawTraceClient*> m_GPSRawTraceClient;

      inFile.open(filepath);
      if (!inFile) {
          NS_FATAL_ERROR("Unable to open the file: "<<filepath);
      }

      // Skip the header
      if (m_header) {
         std::getline(inFile, line);
      }

      while (std::getline(inFile, line))
      {

          std::istringstream iss(line);
          result = getNextLineAndSplitIntoTokens(line);

          // Get the vehicle id
          if(m_singletrace == false) {
              currentVehId = result[0];
          } else {
              currentVehId = m_increasingVehID++;
          }

          // Check if the element is present in the map m_GPSTraceClient
          if ( m_GPSTraceClient.find(currentVehId) == m_GPSTraceClient.end() ) {
            // Not found - needed to create the entry for such a vector
            GPSRawTraceClient* gpsrawclient = new GPSRawTraceClient(currentVehId);
            m_GPSRawTraceClient.insert(std::make_pair(currentVehId, gpsclient));
          }

          double lat=std::stod(result[2]);
          double lon=std::stod(result[3]);
          double tm_x,tm_y,tm_gamma,tm_kappa;

          // Project lat and lon to a Cartesian Plane using Transverse Mercator
          TransverseMercator_Forward (&tmerc,lon0,lat,lon,&tm_x,&tm_y,&tm_gamma,&tm_kappa);

          m_GPSTraceClient[currentVehId]->setLon0(lon0); // To allow later on to perform TransverseMercator_forward

          //std::cout<<"Converted ("<<lat<<","<<lon<<") into ("<<tm_x<<","<<tm_y<<") [reflon:"<<lon0<<"]"<<std::endl;

          // Save the info of this line of csv
          m_GPSTraceClient[currentVehId]->setTimestamp(result[1]);
          m_GPSTraceClient[currentVehId]->setLat(result[2]);
          m_GPSTraceClient[currentVehId]->setLon(result[3]);
          //
          m_GPSTraceClient[currentVehId]->setX(tm_x);
          m_GPSTraceClient[currentVehId]->setY(tm_y);

          m_GPSTraceClient[currentVehId]->setSpeedms(result[4]);
          m_GPSTraceClient[currentVehId]->setheading(result[5]); //define if rad or deg: 5:rad - 6:deg
          m_GPSTraceClient[currentVehId]->setAccelmsq(result[7]);

          // Find the minimum x and y values
          if(tm_x<min_tm_x)
          {
              min_tm_x=tm_x;
          }

          if(tm_y<min_tm_y)
          {
              min_tm_y=tm_y;
          }
      }

      // Sort for each object the vector "vehiclesdata" according to the timestamp
      // Shift also the x,y coordinate in order to have the origin (0,0) at the minimum y and minimum y point
      for(std::map<std::string,GPSTraceClient*>::iterator it=m_GPSTraceClient.begin(); it!=m_GPSTraceClient.end(); ++it) {
          it->second->sortVehiclesdata();
          it->second->shiftOrigin(min_tm_x,min_tm_y);
      }

      //*
      // PRINT ANY OBJECT AND ITS CORRESPONDING VECTOR *
      if(m_verbose==true) {
          for(std::map<std::string,GPSTraceClient*>::iterator it=m_GPSTraceClient.begin(); it!=m_GPSTraceClient.end(); ++it) {
              std::cout << "Key: " << it->first << std::endl;
              it->second->printVehiclesdata();
              std::cout << std::endl;
          }
      }
      //*/
      return m_GPSTraceClient;
    }
}

