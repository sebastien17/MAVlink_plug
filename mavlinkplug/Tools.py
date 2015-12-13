#!/usr/bin/env python
# -*- coding: utf-8 -*-

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# This file is part of MAVlinkplug.

# MAVlinkplug is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# MAVlinkplug is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with MAVlinkplug.  If not, see <http://www.gnu.org/licenses/>.
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

import math


def distance_from_coordinates_degrees(deg_lat_long_tuple_1, deg_lat_long_tuple_2):

    earth_mean_radius = 6371000     # meters

    lat_long_tuple_1 = (math.radians(deg_lat_long_tuple_1[0]), math.radians(deg_lat_long_tuple_1[1]))   #deg 2 rad
    lat_long_tuple_2 = (math.radians(deg_lat_long_tuple_2[0]), math.radians(deg_lat_long_tuple_2[1]))   #deg 2 rad

    a = math.pow(math.sin((lat_long_tuple_1[0]-lat_long_tuple_2[0])/2), 2) + math.cos(lat_long_tuple_1[0])*math.cos(lat_long_tuple_2[0])*math.pow(math.sin((lat_long_tuple_1[1]-lat_long_tuple_2[1])/2), 2)
    c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))

    return c * earth_mean_radius



if(__name__ == '__main__'):
    pass