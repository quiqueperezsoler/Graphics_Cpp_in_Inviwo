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

#ifndef IVW_SERIALIZER_H
#define IVW_SERIALIZER_H

#include <inviwo/core/io/serialization/serializebase.h>
#include <inviwo/core/util/exception.h>
#include <inviwo/core/util/stdextensions.h>
#include <inviwo/core/util/stringconversion.h>
#include <inviwo/core/io/serialization/serializationexception.h>

#include <flags/flags.h>

#include <type_traits>
#include <list>
#include <bitset>

namespace inviwo {

class Serializable;

class IVW_CORE_API Serializer : public SerializeBase {
public:
    /**
     * \brief Initializes serializer with a file name that will be used to set relative paths to
     *data.
     * The specified file name will not be used to write any content until writeFile() is called.
     *
     * @param fileName full path to xml file.
     * @param allowReference disables or enables reference management schemes.
     * @throws SerializationException
     */
    Serializer(const std::string& fileName, bool allowReference = true);

    virtual ~Serializer();

    /**
     * \brief Writes serialized data to the file specified by the currently set file name.
     *
     * @note File name needs to be set before calling this method.
     * @throws SerializationException
     */
    virtual void writeFile();
    /**
     * \brief Writes serialized data to stream.
     *
     * @param stream Stream to be written to.
     * @param format Format the output, i.e. insert line breaks and tabs.
     * @throws SerializationException
     */
    virtual void writeFile(std::ostream& stream, bool format = false);

    // std containers
    template <typename T>
    void serialize(const std::string& key, const std::vector<T>& sVector,
                   const std::string& itemKey = "item");

    template <typename T>
    void serialize(const std::string& key, const std::unordered_set<T>& sSet,
                   const std::string& itemKey = "item");

    template <typename T>
    void serialize(const std::string& key, const std::list<T>& container,
                   const std::string& itemKey = "item");

    template <typename K, typename V, typename C, typename A>
    void serialize(const std::string& key, const std::map<K, V, C, A>& map,
                   const std::string& itemKey = "item");

    template <typename K, typename V, typename H, typename C, typename A>
    void serialize(const std::string& key, const std::unordered_map<K, V, H, C, A>& map,
                   const std::string& itemKey = "item");

    // Specializations for chars
    void serialize(const std::string& key, const signed char& data,
                   const SerializationTarget& target = SerializationTarget::Node);
    void serialize(const std::string& key, const char& data,
                   const SerializationTarget& target = SerializationTarget::Node);
    void serialize(const std::string& key, const unsigned char& data,
                   const SerializationTarget& target = SerializationTarget::Node);

    // integers, reals, strings
    template <typename T, typename std::enable_if<std::is_integral<T>::value ||
                                                      util::is_floating_point<T>::value ||
                                                      util::is_string<T>::value,
                                                  int>::type = 0>
    void serialize(const std::string& key, const T& data,
                   const SerializationTarget& target = SerializationTarget::Node);

    // Enum types
    template <typename T, typename std::enable_if<std::is_enum<T>::value, int>::type = 0>
    void serialize(const std::string& key, const T& data,
                   const SerializationTarget& target = SerializationTarget::Node);

    // Flag types
    template <typename T>
    void serialize(const std::string& key, const flags::flags<T>& data,
                   const SerializationTarget& target = SerializationTarget::Node);

    // glm vector types
    template <typename Vec, typename std::enable_if<util::rank<Vec>::value == 1, int>::type = 0>
    void serialize(const std::string& key, const Vec& data);

    // glm matrix types
    template <typename Mat, typename std::enable_if<util::rank<Mat>::value == 2, int>::type = 0>
    void serialize(const std::string& key, const Mat& data);

    // bitsets
    template <unsigned N>
    void serialize(const std::string& key, const std::bitset<N>& bits);

    // serializable classes
    void serialize(const std::string& key, const Serializable& sObj);

    // pointers to something of the above.
    template <class T>
    void serialize(const std::string& key, const T* const& data);

    // unique_ptr to something of the above.
    template <class T, class D>
    void serialize(const std::string& key, const std::unique_ptr<T, D>& data);

protected:
    friend class NodeSwitch;
};

template <typename T>
void Serializer::serialize(const std::string& key, const std::vector<T>& vector,
                           const std::string& itemKey) {
    if (vector.empty()) return;

    auto node = std::make_unique<TxElement>(key);
    rootElement_->LinkEndChild(node.get());
    NodeSwitch nodeSwitch(*this, node.get());

    for (const auto& item : vector) {
        serialize(itemKey, item);
    }
}

template <typename T>
void Serializer::serialize(const std::string& key, const std::unordered_set<T>& set,
                           const std::string& itemKey) {
    if (set.empty()) return;

    auto node = std::make_unique<TxElement>(key);
    rootElement_->LinkEndChild(node.get());
    NodeSwitch nodeSwitch(*this, node.get());

    for (const auto& item : set) {
        serialize(itemKey, item);
    }
}

template <typename T>
void Serializer::serialize(const std::string& key, const std::list<T>& container,
                           const std::string& itemKey) {
    if (container.empty()) return;

    auto node = std::make_unique<TxElement>(key);
    rootElement_->LinkEndChild(node.get());

    NodeSwitch nodeSwitch(*this, node.get());
    for (const auto& item : container) {
        serialize(itemKey, item);
    }
}

template <typename K, typename V, typename C, typename A>
void Serializer::serialize(const std::string& key, const std::map<K, V, C, A>& map,
                           const std::string& itemKey) {
    if (!isPrimitiveType(typeid(K)))
        throw SerializationException("Error: map key has to be a primitive type", IVW_CONTEXT);

    if (map.empty()) return;

    auto node = std::make_unique<TxElement>(key);
    rootElement_->LinkEndChild(node.get());
    NodeSwitch nodeSwitch(*this, node.get());

    for (const auto& item : map) {
        serialize(itemKey, item.second);
        rootElement_->LastChild()->ToElement()->SetAttribute(SerializeConstants::KeyAttribute,
                                                             item.first);
    }
}

template <typename K, typename V, typename H, typename C, typename A>
void Serializer::serialize(const std::string& key, const std::unordered_map<K, V, H, C, A>& map,
                           const std::string& itemKey) {
    if (!isPrimitiveType(typeid(K)))
        throw SerializationException("Error: map key has to be a primitive type", IVW_CONTEXT);

    if (map.empty()) return;

    auto node = std::make_unique<TxElement>(key);
    rootElement_->LinkEndChild(node.get());
    NodeSwitch nodeSwitch(*this, node.get());

    for (const auto& item : map) {
        serialize(itemKey, item.second);
        rootElement_->LastChild()->ToElement()->SetAttribute(SerializeConstants::KeyAttribute,
                                                             item.first);
    }
}

template <class T, class D>
void Serializer::serialize(const std::string& key, const std::unique_ptr<T, D>& data) {
    serialize(key, data.get());
}

template <class T>
inline void Serializer::serialize(const std::string& key, const T* const& data) {
    if (!allowRef_) {
        serialize(key, *data);
    } else {
        if (refDataContainer_.find(data)) {
            TxElement* newNode = refDataContainer_.nodeCopy(data);
            newNode->SetValue(key);
            rootElement_->LinkEndChild(newNode);
            refDataContainer_.insert(data, newNode);
        } else {
            serialize(key, *data);
            refDataContainer_.insert(data, rootElement_->LastChild()->ToElement(), false);
        }
    }
}

// integers, reals, strings
template <typename T,
          typename std::enable_if<std::is_integral<T>::value || util::is_floating_point<T>::value ||
                                      util::is_string<T>::value,
                                  int>::type>
void Serializer::serialize(const std::string& key, const T& data,
                           const SerializationTarget& target) {
    if (target == SerializationTarget::Attribute) {
        rootElement_->SetAttribute(key, data);
    } else {
        auto node = std::make_unique<TxElement>(key);
        rootElement_->LinkEndChild(node.get());
        node->SetAttribute(SerializeConstants::ContentAttribute, data);
    }
}

// enum types
template <typename T, typename std::enable_if<std::is_enum<T>::value, int>::type>
void Serializer::serialize(const std::string& key, const T& data,
                           const SerializationTarget& target) {
    using ET = typename std::underlying_type<T>::type;
    const ET tmpdata{static_cast<const ET>(data)};
    serialize(key, tmpdata, target);
}

// Flag types
template <typename T>
void Serializer::serialize(const std::string& key, const flags::flags<T>& data,
                           const SerializationTarget& target) {
    serialize(key, data.underlying_value(), target);
}

// glm vector types
template <typename Vec, typename std::enable_if<util::rank<Vec>::value == 1, int>::type>
void Serializer::serialize(const std::string& key, const Vec& data) {
    auto node = std::make_unique<TxElement>(key);
    rootElement_->LinkEndChild(node.get());
    for (size_t i = 0; i < util::extent<Vec, 0>::value; ++i) {
        node->SetAttribute(SerializeConstants::VectorAttributes[i], data[i]);
    }
}

// glm matrix types
template <typename Mat, typename std::enable_if<util::rank<Mat>::value == 2, int>::type>
void Serializer::serialize(const std::string& key, const Mat& data) {
    auto node = std::make_unique<TxElement>(key);
    rootElement_->LinkEndChild(node.get());

    NodeSwitch nodeSwitch(*this, node.get());
    for (size_t i = 0; i < util::extent<Mat, 0>::value; ++i) {
        serialize("col" + toString(i), data[i]);
    }
}

template <unsigned N>
void Serializer::serialize(const std::string& key, const std::bitset<N>& bits) {
    serialize(key, bits.to_string());
}

}  // namespace inviwo
#endif
