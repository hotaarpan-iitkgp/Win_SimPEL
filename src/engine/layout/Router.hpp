#pragma once

// Orthogonal wire routing around components on the schematic grid.
//
// findAStarPath and orthogonalPath are ports of findAStarPath / getOrthogonalPath
// from the web tool (src/schematic/routing.ts): 20 px grid, a 150-unit penalty per
// turn, a search box padded by 240 px, at most 3000 expansions, falling back to the
// L/Z-shaped heuristic route when the search fails. One addition: grid cells
// already used by a different net cost extra when entered along the same axis, so
// wires of different nets do not run on top of each other (they still cross).
//
// routeDesign rebuilds the wires of each net as a tree: the first wire runs
// between two pins, every further pin is connected by a wire from the nearest
// point of the tree (a junction on that wire) to the pin. Each wire stores its
// route in WireInstance::manualPath as the full polyline from its start (pin or
// junction) to its end pin; the schematic canvas and the SVG exporter draw it as
// long as its ends still match the endpoints. Connectivity is unchanged; for
// control nets the tree is rooted at the driving output, which is what the netlist
// generator expects. The netlist of the design is unchanged (tests/test_route.cpp).

#include "../Components.hpp"

#include <unordered_map>
#include <vector>

namespace CircuitSim {

struct RouteOptions {
    float grid = 20.0f;
    float turnPenalty = 150.0f;
    float margin = 240.0f;
    int maxIterations = 3000;
    float stub = 20.0f;           // straight run out of a pin before turning
    float otherNetPenalty = 400.0f;   // per cell run along another net's wire
    // routeDesign retries a failed search with this budget and margin (long wires
    // between distant blocks exceed routing.ts's 3000 expansions).
    int retryMaxIterations = 200000;
    float retryMargin = 600.0f;
};

struct RouteRect { float xMin, yMin, xMax, yMax; };

// Cells used by already-routed wires: the router penalises running along them.
class RouteOccupancy {
public:
    void addPath(const std::vector<Point2D>& path, int net, float grid);
    // Extra cost of stepping into (x, y) along axis horizontal/vertical for `net`.
    float penalty(float x, float y, bool horizontal, int net, float perCell) const;
private:
    struct Cell { int netH = -1, netV = -1; bool multiH = false, multiV = false; };
    std::unordered_map<long long, Cell> cells_;
    float grid_ = 20.0f;
};

// Grid path from `start` (leaving along dirStart; (0,0) = any) to `end` (arriving
// along dirEnd), avoiding the interiors of `obstacles`. Empty if not found.
std::vector<Point2D> findAStarPath(Point2D start, Point2D dirStart, Point2D end, Point2D dirEnd,
                                   const std::vector<RouteRect>& obstacles, const RouteOptions& opts = RouteOptions(),
                                   const RouteOccupancy* occupancy = nullptr, int net = -1);

// Full orthogonal route from pin p1 (outward direction dir1) to pin p2 (outward
// direction dir2), including both pins, simplified (no collinear points).
// *searchOk (if given) is false when the grid search failed and the heuristic
// route was returned instead.
std::vector<Point2D> orthogonalPath(Point2D p1, Point2D dir1, Point2D p2, Point2D dir2,
                                    const std::vector<RouteRect>& obstacles, const RouteOptions& opts = RouteOptions(),
                                    const RouteOccupancy* occupancy = nullptr, int net = -1, bool* searchOk = nullptr);

// Drops duplicate and collinear interior points.
std::vector<Point2D> simplifyPath(const std::vector<Point2D>& pts);

struct RouteStats {
    int nets = 0;              // nets rebuilt and routed
    int netsKept = 0;          // nets left as they were (see routeDesign)
    int wires = 0;
    int searchRetries = 0;     // searches that needed the larger budget
    int searchFailures = 0;    // wires that fell back to the heuristic route
};

// Rebuilds and routes the wires of every net whose endpoints are all component
// pins (junction-only or unresolvable endpoints keep their original wires).
// Control nets need exactly one driving output; others are kept as they are.
RouteStats routeDesign(CircuitDesign& design, const RouteOptions& opts = RouteOptions());

} // namespace CircuitSim
