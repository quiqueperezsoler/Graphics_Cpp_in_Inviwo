/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2013-2019 Inviwo Foundation
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

#ifndef IVW_SIMPLEMESHRAM_H
#define IVW_SIMPLEMESHRAM_H

#include <inviwo/core/common/inviwocoredefine.h>
#include <inviwo/core/datastructures/geometry/mesh.h>

namespace inviwo {

class IVW_CORE_API SimpleMesh : public Mesh {
public:
    SimpleMesh(DrawType dt = DrawType::Points, ConnectivityType ct = ConnectivityType::None);
    SimpleMesh(const SimpleMesh& rhs) = default;
    SimpleMesh& operator=(const SimpleMesh& that) = default;
    virtual SimpleMesh* clone() const override;
    virtual ~SimpleMesh() = default;

    unsigned int addVertex(vec3 pos, vec3 texCoord, vec4 color);
    void addIndex(unsigned int idx);

    template <typename... Args>
    void addIndices(Args&&...);
    void setIndicesInfo(DrawType dt, ConnectivityType ct);
    const Buffer<vec3>* getVertexList() const;
    const Buffer<vec3>* getTexCoordList() const;
    const Buffer<vec4>* getColorList() const;
    const IndexBuffer* getIndexList() const;
};

namespace detail {

template <typename T>
void addIndices(SimpleMesh* mesh, T index) {
    mesh->addIndex(index);
}

template <typename T, typename... Args>
void addIndices(SimpleMesh* mesh, T index, Args... args) {
    mesh->addIndex(index);
    addIndices(mesh, args...);
}
}  // namespace detail

template <typename... Args>
void SimpleMesh::addIndices(Args&&... args) {
    detail::addIndices<Args...>(this, args...);
}

}  // namespace inviwo

#endif  // IVW_SIMPLEMESHRAM_H
