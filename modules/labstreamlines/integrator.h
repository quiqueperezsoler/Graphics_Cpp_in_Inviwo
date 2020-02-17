/*********************************************************************
 *  Author  : Himangshu Saikia
 *  Init    : Wednesday, September 20, 2017 - 12:04:15
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#pragma once

#include <inviwo/core/common/inviwo.h>
#include <inviwo/core/datastructures/geometry/basicmesh.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <labstreamlines/labstreamlinesmoduledefine.h>
#include <labutils/scalarvectorfield.h>

namespace inviwo {

class IVW_MODULE_LABSTREAMLINES_API Integrator {

public:
    // Construction / Deconstruction
public:
    Integrator() {}
    virtual ~Integrator() = default;

    // Methods
public:
    // TODO: Implement the methods below (one integration step with either Euler or
    // Runge-Kutte of 4th order integration method)
    // Pass any other properties that influence the integration process
    // Examples would be the stepsize, inegreation direction, ...
    // static dvec2 RK4(const VectorField2& vectorField, const dvec2& position, ...);
    // static dvec2 Euler(const VectorField2& vectorField, const dvec2& position, ...);
};

}  // namespace inviwo
