#include "image_processor.h"

// Include implementation
#include "image_processor_impl.cpp"

// ImageProcessor implementation
ImageProcessor::ImageProcessor(const ProcessorConfig& config) 
    : pImpl(std::make_unique<Impl>(config)) {}

ImageProcessor::~ImageProcessor() = default;

bool ImageProcessor::start() {
    return pImpl->start();
}

void ImageProcessor::stop() {
    pImpl->stop();
}

void ImageProcessor::setFrameCallback(FrameCallback callback) {
    pImpl->setFrameCallback(callback);
}

ImageProcessor::Statistics ImageProcessor::getStatistics() const {
    return pImpl->getStatistics();
}