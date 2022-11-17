#include "DynamicObjectsAvoidance/event_reader.h"

namespace DynamicObjectsAvoidance {

EventReader::EventReader(const std::filesystem::path &aedat4FilePath, const std::string &cameraName) {
	monoCameraRecordingPtr = std::make_unique<dv::io::MonoCameraRecording>(aedat4FilePath, cameraName);
	mCameraCapture         = false;
}

EventReader::EventReader(const std::string &cameraName) {
	cameraCapturePtr = std::make_unique<dv::io::CameraCapture>(cameraName);
	mCameraCapture   = true;
}

std::optional<cv::Size> EventReader::getFrameResolution() const {
	if (mCameraCapture) {
		return cameraCapturePtr->getFrameResolution();
	}
	else {
		return monoCameraRecordingPtr->getFrameResolution();
	}
}

std::optional<cv::Size> EventReader::getEventResolution() const {
	if (mCameraCapture) {
		return cameraCapturePtr->getEventResolution();
	}
	else {
		return monoCameraRecordingPtr->getEventResolution();
	}
}

std::optional<dv::EventStore> EventReader::getNextEventBatch() {
	if (mCameraCapture) {
		return cameraCapturePtr->getNextEventBatch();
	}
	else {
		return monoCameraRecordingPtr->getNextEventBatch();
	}
}

std::optional<dv::cvector<dv::IMU>> EventReader::getNextImuBatch() {
	if (mCameraCapture) {
		return cameraCapturePtr->getNextImuBatch();
	}
	else {
		return monoCameraRecordingPtr->getNextImuBatch();
	}
}

std::optional<dv::Frame> EventReader::getNextFrame() {
	if (mCameraCapture) {
		return cameraCapturePtr->getNextFrame();
	}
	else {
		return monoCameraRecordingPtr->getNextFrame();
	}
}

std::optional<dv::cvector<dv::Trigger>> EventReader::getNextTriggerBatch() {
	if (mCameraCapture) {
		return cameraCapturePtr->getNextTriggerBatch();
	}
	else {
		return monoCameraRecordingPtr->getNextTriggerBatch();
	}
}

bool EventReader::isEventStreamAvailable() const {
	if (mCameraCapture) {
		return true;
	}
	else {
		return monoCameraRecordingPtr->isEventStreamAvailable();
	}
}

bool EventReader::isFrameStreamAvailable() const {
	if (mCameraCapture) {
		return cameraCapturePtr->isFrameStreamAvailable();
	}
	else {
		return monoCameraRecordingPtr->isFrameStreamAvailable();
	}
}

bool EventReader::isImuStreamAvailable() const {
	if (mCameraCapture) {
		return true;
	}
	else {
		return monoCameraRecordingPtr->isImuStreamAvailable();
	}
}

bool EventReader::isTriggerStreamAvailable() const {
	if (mCameraCapture) {
		return true;
	}
	else {
		return monoCameraRecordingPtr->isTriggerStreamAvailable();
	}
}

std::optional<std::pair<int64_t, int64_t>> EventReader::getTimeRange() const {
	if (mCameraCapture) {
		return std::nullopt;
	}
	else {
		return monoCameraRecordingPtr->getTimeRange();
	}
}

bool EventReader::isConnected() const {
	if (mCameraCapture) {
		return cameraCapturePtr->isConnected();
	}
	else {
		return true;
	}
}
const std::unique_ptr<dv::io::CameraCapture> &EventReader::getCameraCapturePtr() const {
	return cameraCapturePtr;
}

const std::unique_ptr<dv::io::MonoCameraRecording> &EventReader::getMonoCameraRecordingPtr() const {
	return monoCameraRecordingPtr;
}

std::string EventReader::getCameraName() const {
	if (mCameraCapture) {
		return cameraCapturePtr->getCameraName();
	}
	else {
		return monoCameraRecordingPtr->getCameraName();
	}
}


}  // namespace DynamicObjectsAvoidance