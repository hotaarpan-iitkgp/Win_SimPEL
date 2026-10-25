#pragma once

// Automatic placement for circuits described by connectivity only (the Python API),
// so a saved design opens in the GUI as a readable schematic.
//
// Layered layout, left to right:
//   power stage   columns by distance from the sources along the power nets
//                 (the ground net is ignored, otherwise everything is adjacent);
//                 ground below the stage.
//   control       below the power stage; columns by longest signal path from the
//                 signal sources (feedback edges are broken by DFS), so signals
//                 flow left to right.
// Within a column, components are ordered by the barycentre of their neighbours
// in the previous column to reduce crossings. Positions are snapped to the
// schematic's 20 px grid and boxes never overlap. Placement changes geometry only:
// the netlist of the design is unchanged (tests/test_layout.cpp checks this over
// the whole design corpus).

#include "../Components.hpp"

namespace CircuitSim {

struct PlaceOptions {
    float grid = 20.0f;          // snap pitch (the GUI's grid)
    float columnGap = 100.0f;    // horizontal clearance between columns
    float rowGap = 80.0f;        // vertical clearance between blocks (labels)
    float sectionGap = 160.0f;   // between the power stage and the control section
};

// Places every component (rotation reset to 0), moves wire junction points to
// their target wire's midpoint and drops hand-drawn wire paths (they would no
// longer meet the pins).
void autoPlace(CircuitDesign& design, const PlaceOptions& opts = PlaceOptions());

// Axis-aligned extent of a component at rotation 0, from its pin layout, used for
// spacing: half width / half height around (x, y).
void placementExtent(const ComponentInstance& comp, float& halfW, float& halfH);

} // namespace CircuitSim
