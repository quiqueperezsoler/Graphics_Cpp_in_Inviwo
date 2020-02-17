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

#ifndef IVW_PORTFACTORY_H
#define IVW_PORTFACTORY_H

#include <inviwo/core/common/inviwocoredefine.h>
#include <inviwo/core/common/inviwo.h>
#include <inviwo/core/ports/portfactoryobject.h>
#include <inviwo/core/util/factory.h>

namespace inviwo {

class IVW_CORE_API InportFactory : public Factory<Inport, const std::string&, const std::string&>,
                                   public StandardFactory<Inport, InportFactoryObject> {
public:
    InportFactory() = default;
    virtual ~InportFactory() = default;

    using StandardFactory<Inport, InportFactoryObject>::create;
    virtual bool hasKey(const std::string& key) const override;
    virtual std::unique_ptr<Inport> create(const std::string& className,
                                           const std::string& identifier) const override;
};

class IVW_CORE_API OutportFactory : public Factory<Outport, const std::string&, const std::string&>,
                                    public StandardFactory<Outport, OutportFactoryObject> {
public:
    OutportFactory() = default;
    virtual ~OutportFactory() = default;

    using StandardFactory<Outport, OutportFactoryObject>::create;
    virtual bool hasKey(const std::string& key) const override;
    virtual std::unique_ptr<Outport> create(const std::string& className,
                                            const std::string& identifier) const override;
};

}  // namespace inviwo

#endif  // IVW_PORTFACTORY_H
