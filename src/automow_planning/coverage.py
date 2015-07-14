#!/usr/bin/env python

"""
/* Original work copyright team '"Moe" The Autonomous Lawnmower' - Auburn University
 *
 * This program is free software; you can redistribute it and/or modify it under the terms of
 * the GNU General Public License as published by the Free Software Foundation;
 * either version 3 of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with this program;
 * if not, see <http://www.gnu.org/licenses/>.
 *
 * This file is taken from the au_automow_common ROS stack
 **/

This module contains code to plan coverage paths
"""

from shapely.geometry import Point, Polygon, LineString
from shapely.geometry import MultiLineString, MultiPoint
from shapely.geometry import GeometryCollection
from shapely.geos import TopologicalError
import math
import numpy as np
from copy import deepcopy

from pprint import pprint

from logging import error

def generate_intersections(poly, width):
    "Subdivide a filed into coverage lines."
    starting_breakdown = poly.bounds[0:2]
    line = LineString([starting_breakdown, (starting_breakdown[0],
                                            starting_breakdown[1] +
                                            poly.bounds[3] - poly.bounds[1])])
    try:
        bounded_line = poly.intersection(line)
    except TopologicalError as e:
        error("Problem looking for intersection.", exc_info=1)
        return
    lines = [bounded_line]
    iterations = int(math.ceil((poly.bounds[2] - poly.bounds[0]) / width)) + 1
    for x in range(1, iterations):
        bounded_line = line.parallel_offset(x * width, 'right')
        if poly.intersects(bounded_line):
            try:
                bounded_line = poly.intersection(bounded_line)
            except TopologicalError as e:
                error("Problem looking for intersection.", exc_info=1)
                continue
            lines.append(bounded_line)
    return lines

def sort_to(point, list):
    "Sorts a set of points by distance to a point"
    l = deepcopy(list)
    l.sort(lambda x, y: cmp(x.distance(Point(*point)),
                            y.distance(Point(*point))))
    return l

def get_furthest(ps, origin):
    "Get a point along a line furthest away from a given point"
    orig_point = Point(*origin)
    return sorted(ps, lambda x, y: cmp(orig_point.distance(Point(*x)),
                                       orig_point.distance(Point(*y))))

def order_points(lines, initial_origin):
    "Return a list of points in a given coverage path order"
    origin = initial_origin
    results = []
    while True:
        if not len(lines):
            break
        lines = sort_to(origin, lines)
        f = lines.pop(0)
        if type(f) == GeometryCollection:
            continue
        if type(f) == MultiLineString:
            for ln in f:
                lines.append(ln)
            continue
        if type(f) == Point or type(f) == MultiPoint:
            continue
        xs, ys = f.xy
        ps = zip(xs, ys)
        (start, end) = get_furthest(ps, origin)
        results.append(origin)
        # results.append(start)
        results.append(start)
        # results.append(end)
        origin = end
    return results

def bottom_left(points):
    """returns the bottom left point from the list of points given"""
    pass

def decompose(polygon, origin=None, width=1.0):
    """
    Decompose the field into a list of points to cover the field.
    """
    p = generate_intersections(polygon, width)
    if origin == None:
        return order_points(p, polygon.bounds[0:2])
    else:
        return order_points(p, origin)

if __name__ == '__main__':
    from maptools import RotationTransform, rotation_tf_from_longest_edge
    from maptools import rotate_polygon_to, make_axis, plot_polygon
    from maptools import zoom_extents, rotate_from, plot_line, rotate_to
    
    ext = [(1, 1), (-3, 11), (4, 16), (10, 10)]
    polygon = Polygon(ext)
    polygon_points = np.array(polygon.exterior)
    
    rt = RotationTransform(66)
    
    tf_polygon = rotate_polygon_to(polygon, rt)
    
    origin = rotate_to(np.array([(1,1)]),rt).tolist()
    
    from time import time; start = time()
    result = decompose(tf_polygon, origin, width=0.5)
    print "Decomposition Time:", time()-start
    
    tf_result = rotate_from(np.array(result), rt)
    
    ll = LineString(tf_result)
    
    ax, _ = make_axis()
    plot_polygon(polygon, ax, color='blue')
    plot_polygon(tf_polygon, ax, color='blue')
    plot_line(ll, ax)
    ax.axvline(x=0, color='black')
    ax.axhline(y=0, color='black')
    zoom_extents(ax, [polygon,tf_polygon])
    import pylab; pylab.show()
