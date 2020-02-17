/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2015-2019 Inviwo Foundation
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

#include <modules/cimg/cimgvolumereader.h>
#include <modules/cimg/cimgutils.h>
#include <inviwo/core/datastructures/volume/volumeramprecision.h>
#include <inviwo/core/datastructures/volume/volumedisk.h>
#include <inviwo/core/util/formatdispatching.h>
#include <inviwo/core/util/filesystem.h>
#include <inviwo/core/io/datareaderexception.h>

namespace inviwo {

CImgVolumeReader::CImgVolumeReader() : DataReaderType<Volume>() {
    addExtension(FileExtension("hdr", "Analyze 7.5"));
}

CImgVolumeReader* CImgVolumeReader::clone() const { return new CImgVolumeReader(*this); }

std::shared_ptr<Volume> CImgVolumeReader::readData(const std::string& filePath) {
    if (!filesystem::fileExists(filePath)) {
        throw DataReaderException("Error could not find input file: " + filePath, IVW_CONTEXT);
    }

    auto volumeDisk = std::make_shared<VolumeDisk>(filePath);
    volumeDisk->setLoader(new CImgVolumeRAMLoader(volumeDisk.get()));
    return std::make_shared<Volume>(volumeDisk);
}

void CImgVolumeReader::printMetaInfo(const MetaDataOwner& metaDataOwner, std::string key) const {
    if (auto metaData = metaDataOwner.getMetaData<StringMetaData>(key)) {
        std::string metaStr = metaData->get();
        replaceInString(metaStr, "\n", ", ");
        key[0] = static_cast<char>(toupper(key[0]));
        LogInfo(key << ": " << metaStr);
    }
}

CImgVolumeRAMLoader::CImgVolumeRAMLoader(VolumeDisk* volumeDisk) : volumeDisk_(volumeDisk) {}

CImgVolumeRAMLoader* CImgVolumeRAMLoader::clone() const { return new CImgVolumeRAMLoader(*this); }

std::shared_ptr<VolumeRepresentation> CImgVolumeRAMLoader::createRepresentation() const {
    void* data = nullptr;

    size3_t dimensions = volumeDisk_->getDimensions();
    DataFormatId formatId = DataFormatId::NotSpecialized;

    std::string fileName = volumeDisk_->getSourceFile();

    if (!filesystem::fileExists(fileName)) {
        std::string newPath = filesystem::addBasePath(fileName);

        if (filesystem::fileExists(newPath)) {
            fileName = newPath;
        } else {
            throw DataReaderException("Error could not find input file: " + fileName, IVW_CONTEXT);
        }
    }

    data = cimgutil::loadVolumeData(nullptr, fileName, dimensions, formatId);
    volumeDisk_->setDimensions(dimensions);

    return dispatching::dispatch<std::shared_ptr<VolumeRepresentation>, dispatching::filter::All>(
        volumeDisk_->getDataFormat()->getId(), *this, data);
}

void CImgVolumeRAMLoader::updateRepresentation(std::shared_ptr<VolumeRepresentation> dest) const {
    auto volumeDst = std::static_pointer_cast<VolumeRAM>(dest);

    size3_t dimensions = volumeDisk_->getDimensions();
    DataFormatId formatId = DataFormatId::NotSpecialized;

    std::string fileName = volumeDisk_->getSourceFile();

    if (!filesystem::fileExists(fileName)) {
        std::string newPath = filesystem::addBasePath(fileName);

        if (filesystem::fileExists(newPath)) {
            fileName = newPath;
        } else {
            throw DataReaderException("Error could not find input file: " + fileName, IVW_CONTEXT);
        }
    }

    cimgutil::loadVolumeData(volumeDst->getData(), fileName, dimensions, formatId);
    volumeDisk_->setDimensions(dimensions);
}

}  // namespace inviwo
