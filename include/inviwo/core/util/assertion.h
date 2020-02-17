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

#ifndef IVW_ASSERTION_H
#define IVW_ASSERTION_H

#include <inviwo/core/common/inviwocoredefine.h>
#include <ostream>
#include <sstream>
#include <string>

namespace inviwo {

IVW_CORE_API void assertion(const char* fileName, const char* functionName, long lineNumber,
                            std::string message);

namespace util {

IVW_CORE_API void debugBreak();

}  // namespace util

}  // namespace inviwo

#if defined(IVW_DEBUG) || defined(IVW_FORCE_ASSERTIONS)
#define IVW_ASSERT(condition, message)                                               \
    {                                                                                \
        if (!(bool(condition))) {                                                    \
            std::ostringstream stream__;                                             \
            stream__ << message;                                                     \
            ::inviwo::assertion(__FILE__, __FUNCTION__, __LINE__, (stream__.str())); \
        }                                                                            \
    }

// Deprecated
#define ivwAssert(condition, message) IVW_ASSERT(condition, message)

#else

#define IVW_ASSERT(condition, message)
#define ivwAssert(condition, message)

#endif

#endif  // IVW_ASSERTION_H
