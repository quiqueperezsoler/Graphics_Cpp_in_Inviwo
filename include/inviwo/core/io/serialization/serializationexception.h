/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2012-2019 Inviwo Foundation
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

#ifndef IVW_SERIALIZATIONEXCEPTION_H
#define IVW_SERIALIZATIONEXCEPTION_H

#include <inviwo/core/common/inviwocoredefine.h>
#include <inviwo/core/io/serialization/ticpp.h>
#include <inviwo/core/util/exception.h>
#include <inviwo/core/io/serialization/nodedebugger.h>

namespace inviwo {

class IVW_CORE_API SerializationException : public Exception {
public:
    struct SerializationExceptionData {
        SerializationExceptionData(std::string k = "", std::string t = "", std::string i = "",
                                   TxElement* n = nullptr)
            : key(k), type(t), id(i), nd(n) {}
        std::string key;
        std::string type;
        std::string id;
        NodeDebugger nd;
    };

    SerializationException(std::string message = "", ExceptionContext context = ExceptionContext(),
                           std::string key = "", std::string type = "", std::string id = "",
                           TxElement* n = nullptr);
    virtual ~SerializationException() noexcept = default;

    virtual const std::string& getKey() const noexcept;
    virtual const std::string& getType() const noexcept;
    virtual const std::string& getId() const noexcept;
    virtual const SerializationExceptionData& getData() const noexcept;

private:
    SerializationExceptionData data_;
};

namespace util {

IVW_CORE_API std::string formatSerializationError(const std::string& name, const std::string& src,
                                                  const std::string& dst, const std::string& err);
}  // namespace util

}  // namespace inviwo

#endif  // IVW_SERIALIZATIONEXCEPTION_H
