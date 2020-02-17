/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2014-2019 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#ifndef IVW_SPOT_LIGHT_H
#define IVW_SPOT_LIGHT_H

#include <inviwo/core/common/inviwocoredefine.h>
#include <inviwo/core/common/inviwo.h>
#include <inviwo/core/datastructures/light/baselightsource.h>
#include <cmath>

namespace inviwo {

class SpotLight : public LightSource {
public:
    SpotLight() = default;
    virtual ~SpotLight() = default;
    virtual SpotLight* clone() const override { return new SpotLight(*this); }

    virtual float getArea() const override { return size_.x * size_.y; }
    /**
     * Get radiant flux (color) of light source.
     * @see setPower
     * @return Radiant flux in watt.
     */
    virtual vec3 getPower() const override {
        return getIntensity() * 2.f * static_cast<float>(M_PI) *
               (1.f - 5.f * (std::cos(glm::radians(coneRadiusAngle_)) +
                             std::cos(glm::radians(coneFallOffAngle_))));
    }

    LightSourceType getLightSourceType() const override { return LightSourceType::cone; }

    /**
     * Get world position of light source.
     *
     * @return World position of light source.
     */
    vec3 getPosition() const { return vec3(getCoordinateTransformer().getModelToWorldMatrix()[3]); }

    /**
     * Set world position of light source.
     *
     * @param position World position of light source.
     */
    void setPosition(const vec3& position) {
        worldMatrix_[3][0] = position[0];
        worldMatrix_[3][1] = position[1];
        worldMatrix_[3][2] = position[2];
    }

    /**
     * Get normalized general direction of light source.
     *
     * @return Normalized direction of light source.
     */
    const vec3& getDirection() const { return direction_; }

    /**
     * Set normalized direction of light source.
     *
     * @param direction Normalized direction of light source.
     */
    void setDirection(const vec3& direction) { direction_ = direction; }

    /**
     * Get cone radius angle of the light source.
     *
     * @return Cone radius angle of the light source.
     */
    const float& getConeRadiusAngle() const { return coneRadiusAngle_; }

    /**
     * Set the cone radius angle of the light source.
     *
     * @param angle Cone radius angle of the light source.
     */
    void setConeRadiusAngle(const float& angle) { coneRadiusAngle_ = angle; }

    /**
     * Get fall off angle of the light source.
     *
     * @return Fall off angle of the light source.
     */
    const float& getConeFallOffAngle() const { return coneFallOffAngle_; }

    /**
     * Set the fall off angle of the light source.
     *
     * @param angle Fall off angle of the light source.
     */
    void setConeFallOffAngle(const float& angle) { coneFallOffAngle_ = angle; }

    /**
     * Get value for checking full visibility (inside cone angle) against the light source.
     *
     * @return Value for checking full visibility (inside cone angle against the light source.
     */
    float getFullVisbilityValue() const { return std::cos(glm::radians(coneRadiusAngle_)); }

    /**
     * Get value for checking partial visibility (inside cone + falloff angle) against the light
     *source.
     *
     * @return Value for checking partial visibility (inside cone + falloff angle) against the light
     *source.
     */
    float getPartialVisbilityValue() const {
        return std::cos(glm::radians(coneRadiusAngle_ + coneFallOffAngle_));
    }

protected:
    vec3 position_;
    vec3 direction_;
    float coneRadiusAngle_;
    float coneFallOffAngle_;
};

}  // namespace inviwo

#endif  // IVW_SPOT_LIGHT_H