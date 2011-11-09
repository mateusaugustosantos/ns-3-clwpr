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


/**
 *
 * The CLWPR MAP is indented as a navigation system to provide information about
 * the underlying road topology. The current implementation ony supports grid networks.
 * The GridMap class is developed as an initial and simplistic navigation system.
 * Due to the limitations set by the CLWPR protocol that only allows a 8-bit identification
 * of the road, which comprises of two 4-bit parts for x- and y-axis, respectively, the maximum
 * grid size is set to 15x15.
 *
 * \TODO
 * Import some kind of open API for navigation (google.maps, openstreetmap, etc)
 *
 */

#ifndef CLWPR_MAP_H_
#define CLWPR_MAP_H_

// MAX Road Network 15 X 15 set by CLWPR constraints
#define MAX_DIAMENSION 15
#define ROADERR 15

#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include "ns3/vector.h"
#include <utility>
#include "ns3/object.h"
namespace ns3 {


class GridMap : public Object
{
       private:
         double m_x; ///< x co-ordinate of vehicle [0-15]
         double m_y; ///< y co-ordinate of vehicle [0-15]
	 uint8_t m_roadID; ///< the ID of the road
	 int m_xd; ///< local x co-ordinate of road
	 int m_yd; ///< local y co-ordinate of road
	 int m_xMax, m_yMax; ///< max values for current map
	 int m_dist; ///< distance between blocks
	 std::pair<double, double> m_waypoints[MAX_DIAMENSION][MAX_DIAMENSION]; ///< the location of the junctions

	 /**
	  * This function initialises the waypoints (junctions)
	  *
	  */
	 void InitWayPoitns();

	 /**
          * This function returns the waypoint is closer to
          * a node on the x-axis
          *
          * @param p The position Vector of the node
          * @param r_index the Road index it is moving (y-axis)
          * @return the x-axis waypoint closer to the node position
          */
	 int FindWPX (Vector p, int r_index);

        /**
         * This function returns the waypoint is closer to
         * a node on the y-axis
         *
         * @param p The position Vector of the node
         * @param r_index the Road index it is moving (x-axis)
         * @return the y-axis waypoint closer to the node position
         */
	 int FindWPY (Vector p, int r_index);

     public:
	  static TypeId GetTypeId (void);

	 /**
	  * Default Constructor
	  */
	  GridMap();

	  /**
	   * Constructor for specific dimensions
	   * Currently only square blocks are formed. Could use a 4th parameter for
	   * different x,y block sizes
	   *
	   * @param x the number of x-axis roads
	   * @param y the number of y-axis roads
	   * @param d the distance between two intersections
	   */
	  GridMap(int x, int y, int d);

	  /**
	   * Default Destructor
	   */
	  ~GridMap();

          /**
           * Method to return the road id using the x,y parts
           *
           * @param r_x the x-axis part (4 MSB of ID)
           * @param r_y the y-axis part (4 LSB of ID)
           * @return the road ID
           */
	  uint8_t FindRoadID(int r_x, int r_y);

          /**
           * Method to return the x-axis part of the road ID
           *
           * @param r_id the road ID
           * @return the x-axis ID (4 MSB of r_id)
           */
	  int GetRoadXFromID (uint8_t r_id);

          /**
           * Method to return the y-axis part of the road ID
           *
           * @param r_id the road ID
           * @return the y-axis ID (4 LSB of r_id)
           */
	  int GetRoadYFromID (uint8_t r_id);

	  /**
           * Method to to set the local x-axis road co-ordinate
           *
           * @param v_x the vehicle's x co-ordinate
           */
	  void SetRoadXFromVehicle(double v_x);

          /**
           * Method to to set the local y-axis road co-ordinate
           *
           * @param v_y the vehicle's y co-ordinate
           */
	  void SetRoadYFromVehicle(double v_y);

	  /**
	   * Simple getter for the local axis road co-ordinate
	   *
	   * @return the local x-axis co-ordinate
	   */
	  int GetRoadXId();

          /**
           * Simple getter for the local axis road co-ordinate
           *
           * @return the local y-axis co-ordinate
           */
	  int GetRoadYId();

	  /**
	   * This method calculates the distance between two nodes following
	   * the road (waypoints). Also known as curvemetric distance.
	   *
	   * @param pos_a the position Vector of node A
	   * @param x_a the x-axis road co-ordinate of node A
	   * @param y_a the y-axis road co-ordinate of node A
           * @param pos_b the position Vector of node B
           * @param x_b the x-axis road co-ordinate of node B
           * @param y_b the y-axis road co-ordinate of node B
	   * @return the curvemetric distance
	   */
	  double GetCurvemetricDistance(Vector pos_a, int x_a, int y_a, Vector pos_b, int x_b, int y_b);

	  /**
	   * This method calculates the LOS and NLOS distance between two nodes that is mentioned to be
	   * used by the propagation loss model utilising visibility (buildings)
	   * See WINNER Propagation models --> https://www.ist-winner.org/WINNER2-Deliverables/D1.1.2v1.1.pdf
	   *
	   * @param pos_a the position Vector of node A
	   * @param x_a the x-axis road co-ordinate of node A
	   * @param y_a the y-axis road co-ordinate of node A
	   * @param pos_b the position Vector of node B
	   * @param x_b the x-axis road co-ordinate of node B
	   * @param y_b the y-axis road co-ordinate of node B
	   * @param &dist1 the LOS component
	   * @param &dist2 the NLOS component
	   */
	  void GetNLOSDistance(Vector pos_a, int x_a, int y_a, Vector pos_b, int x_b, int y_b, double &dist1, double &dist2);

	  /**
	   * Simple getter to return the local x-axis road id
	   * from the vehicle position
	   *
	   * @param v_x the x-axis co-ordinate of vehicles position
	   * @return the x-axis road id
	   */
	  int GetRoadXFromVehicle(double v_x);

          /**
           * Simple getter to return the local y-axis road id
           * from the vehicle position
           *
           * @param v_y the y-axis co-ordinate of vehicles position
           * @return the y-axis road id
           */
	  int GetRoadYFromVehicle(double v_y);
};
} // namespace ns3

#endif /* CLWPR_MAP_H_ */
