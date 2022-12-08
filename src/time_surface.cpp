#include "DynamicObjectsAvoidance/time_surface.h"
#include "DynamicObjectsAvoidance/frame.h"

#include <boost/format.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <thread>
#include <chrono>

namespace DynamicObjectsAvoidance {
TimeSurface::TimeSurface(const std::string& cfg_path, int width, int height, int queueLen) {

    if (Config::SetParameterFile(cfg_path) == false) {
        LOG(ERROR) << "Cannot find config parameter file";
    }
    width_ = width;
    height_ = height;
    queueLen_ = queueLen;
    eqMat_ = std::vector<EventQueue>(width * height, EventQueue());
    eqT_ = std::vector<EventTimestampQueue>(width * height, EventTimestampQueue());
    decay_ms_ = Config::Get<int>("decay_ms");
    decay_sec_ = 1.0 / (decay_ms_ * 1e-3);
    time_surface_map_ = cv::Mat::zeros(height_, width_, CV_8U);
    time_surface_map_tmp_ = cv::Mat::zeros(height_, width_, CV_64F);
    time_surface_map_tmp2_ = cv::Mat::zeros(height_, width_, CV_64F);
    arr_T_ = new EventTimestampQueue[width * height];
    time_matrix_ = Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic>::Zero(height, width);
    time_matrix_double_ = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(height, width);
    t_diff_ = Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic>::Zero(height, width);
    t_diff_double_ = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(height, width);
}

TimeSurface::~TimeSurface() {
}


void TimeSurface::accumulate(const dv::EventStore &packet) {
    buffer.add(packet);
}

/**
 * Inserts the event store into the time surface.
 * @param store The event store to be added
 */
void TimeSurface::accept(const dv::EventStore &store) {
    for (uint64_t it = 1; it < store.size()-2; ++it) {
        time_matrix_(store.at(it).y(), store.at(it).x()) = store.at(it).timestamp();
    }
}

void TimeSurface::insertEvent(const dv::Event& e) {
    time_matrix_(e.y(), e.x()) = e.timestamp();
}

void TimeSurface::generateImage(const dv::EventStore& events, const int64_t& highest_time) {

    double dt(0.);
    double expVal(0.);
    for (auto &e : events) {
        dt = (highest_time - e.timestamp())*1e-6;
        expVal = -(dt-0.15)*(dt+0.15)  / 0.0225;
        time_surface_map_.ptr<double>(e.y())[e.x()] = expVal;
    }
    time_surface_map_ = 255.0 * time_surface_map_;
    time_surface_map_.convertTo(time_surface_map_, CV_8U);
    // median blur
    cv::medianBlur(time_surface_map_, time_surface_map_, 3);
    cv::imshow("timesurface_mine", time_surface_map_);
    cv::waitKey(1);
}

cv::Mat TimeSurface::generateTimeSurface(int64_t& highest_time) {
    cv::Mat image;
    for (const dv::concepts::AddressableEvent auto &event : buffer) {
        dv::runtime_assert(0 <= event.y() && event.y() <= height_, "event Y coordinate out of bounds");
        dv::runtime_assert(0 <= event.x() && event.x() <= width_, "event X coordinate out of bounds");

        auto &imgVal = image.at<uint8_t>(event.y(), event.x());
        // imgVal       = static_cast<uint8_t>(incrementLUT[imgVal]);
    }
    return image;
}
void TimeSurface::createTimeSurface(int64_t& highest_time) {
    // create exponential-decayed Time Surface map.
    double dt(0.0);
    double expVal(0.0);
    auto t1 = std::chrono::steady_clock::now();
    int64_t sum_diff(0);
    time_surface_map_tmp_ = cv::Mat::zeros(height_, width_, CV_64F);
    // std::vector<int64_t> sum_diff_vec;
    for(size_t x=0; x<width_; ++x) {
        for(size_t y=0; y<height_; ++y) {
            int64_t t = time_matrix_(y, x);
            if (t==0) continue;
            // For visualization of time matrix
            // double val = double(t - min) / double(range);
            // time_matrix_double_(y,x) = val;
            if (y>6&&y<height_-6&&x>6&&x<width_-6) {
                // u_int8_t cnt(0);
                // for (size_t patt_i = 0; patt_i < 8; ++patt_i) {
                //     int64_t diff = highest_time - time_matrix_(y + pattern_scale_[patt_i][0], x + pattern_scale_[patt_i][1]);
                //     if (diff > 250000) continue;
                //     sum_diff+=diff;
                //     ++cnt;
                // }
                // if (cnt == 0) continue;
                
                // sum_diff = int64_t(sum_diff / cnt);
                // if (sum_diff < 260) continue;
                // int16_t scale = int16_t(-150000000/(sum_diff-200)+100);//std::pow(scale_begin_end,0.5)*10.0;
                // dt = (highest_time - t)*1e-6;
                // if (sum_diff < 90000) {
                //     expVal = scale*dt*dt+1;
                // } else {
                //     expVal = -10000*dt*dt+1;
                // }

                dt = (highest_time - t)*1e-6;
                expVal = -10000*dt*dt+1;
                time_surface_map_tmp_.ptr<double>(y)[x] = 1;
            } else {
                dt = (highest_time - t)*1e-6;
                expVal = -10000*dt*dt+1;
                time_surface_map_tmp_.ptr<double>(y)[x] = 1;
            }
        }
    }
    time_surface_map_tmp_ = 255.0 * time_surface_map_tmp_;
    time_surface_map_tmp_.convertTo(time_surface_map_, CV_8UC1);
    
    // median blur
    cv::blur(time_surface_map_, time_surface_map_, cv::Size(3,3));
    cv::medianBlur(time_surface_map_, time_surface_map_, 3);
}




// internal use methods
	/**
	 * __INTERNAL_USE_ONLY__
	 * Decays the potential at coordinates x, y to the given time, respecting the
	 * decay function. Updates the time surface to the last decay.
	 * @param x The x coordinate of the value to be decayed
	 * @param y The y coordinate of the value to be decayed
	 * @param time The time to which the value should be decayed to.
	 */
	void TimeSurfaceAccumulator::decay(int16_t& x, int16_t& y, int64_t& time) {
		// normal handling for all the other functions
		int64_t lastDecayTime = decayTimeSurface_(y, x);
		// dv::runtime_assert(lastDecayTime <= time, "last decay time bigger than current time, time going backwards!");

		switch (decayFunction_) {
            case Decay::MYEXPONENTIAL: {
                potentialSurface_.at<float>(y, x)
					= static_cast<float>(
							  expf(-(static_cast<float>(time - lastDecayTime)) * static_cast<float>(decayParam_)));

				// decayTimeSurface_(y, x) = time;
                int64_t t = time_matrix_(y, x);
                if (t==0) return;
                double dt = (highestTime_ - t)*1e-6;
                double expVal(0);
                if (dt > 0.05) {
                    expVal = 0;
                } else {
                    expVal = 400*(dt-0.05)*(dt-0.05);
                }
                time_surface_map_.ptr<double>(y)[x] = expVal;

				break;
            }
			case Decay::LINEAR: {
                const float lastPotential = potentialSurface_.at<float>(y, x);
				potentialSurface_.at<float>(y, x)
					= (lastPotential >= neutralPotential_)
						  ? std::max(lastPotential
										 - static_cast<float>(static_cast<double>(time - lastDecayTime) * decayParam_),
							  neutralPotential_)
						  : std::min(lastPotential
										 + static_cast<float>(static_cast<double>(time - lastDecayTime) * decayParam_),
							  neutralPotential_);
				decayTimeSurface_(y, x) = time;
				break;
			}

			case Decay::EXPONENTIAL: {
                const float lastPotential = potentialSurface_.at<float>(y, x);
				potentialSurface_.at<float>(y, x)
					= ((lastPotential - neutralPotential_)
						  * static_cast<float>(
							  expf(-(static_cast<float>(time - lastDecayTime)) / static_cast<float>(decayParam_))))
					  + neutralPotential_;
				decayTimeSurface_(y, x) = time;
				break;
			}

			case Decay::STEP:
				// STEP decay is handled at frame generation time.
			case Decay::NONE:
			default: {
				break;
			}
		}
	}

    	void TimeSurfaceAccumulator::contribute(int16_t x, int16_t y, bool polarity) {
		const float lastPotential = potentialSurface_.at<float>(y, x);
		float contribution        = eventContribution_;
		if (!rectifyPolarity_ && !polarity) {
			contribution = -contribution;
		}

		float newPotential = std::min(std::max(lastPotential + contribution, minPotential_), maxPotential_);
		potentialSurface_.at<float>(y, x) = newPotential;
	}


    /**
	 * Accumulates all the events in the supplied packet and puts them onto the
	 * accumulation surface.
	 * @param packet The packet containing the events that should be
	 * accumulated.
	 */
	void TimeSurfaceAccumulator::accumulate(const dv::EventStore &packet) {
		if (potentialSurface_.empty()) {
			return;
		}

		if (packet.isEmpty()) {
			return;
		}

        if (decayFunction_ == Decay::MYEXPONENTIAL) {
    		highestTime_ = packet.getHighestTime();
            for (const dv::Event &e : packet) {
				// dv::runtime_assert(0 <= event.y() && event.y() <= shape_.height, "event Y coordinate out of bounds");
				// dv::runtime_assert(0 <= event.x() && event.x() <= shape_.width, "event X coordinate out of bounds");
                // int16_t y = event.y();
                // int16_t x = event.x();
                // int64_t t = event.timestamp();
				// decayTimeSurface_(y, x) = t;
                // decayTimeSurface_mat_.ptr<double>(event.y())[event.x()] = event.timestamp();
                time_matrix_(e.y(), e.x()) = e.timestamp();
			}
            return;
        }
		else if ((decayFunction_ == Decay::NONE) || (decayFunction_ == Decay::STEP)) {
			// for step and none, only contribute
			for (const dv::Event &event : packet) {
				dv::runtime_assert(0 <= event.y() && event.y() <= shape_.height, "event Y coordinate out of bounds");
				dv::runtime_assert(0 <= event.x() && event.x() <= shape_.width, "event X coordinate out of bounds");

				contribute(event.x(), event.y(), event.polarity());
			}
		}
		else if (decayFunction_ == Decay::EXPONENTIAL) {
			// for all others, decay before contributing
			for (const dv::Event &event : packet) {
                int16_t y = event.y();
                int16_t x = event.x();
                int64_t t = event.timestamp();
				dv::runtime_assert(0 <= y && y <= shape_.height, "event Y coordinate out of bounds");
				dv::runtime_assert(0 <= x && x <= shape_.width, "event X coordinate out of bounds");
				decay(x, y, t);
				contribute(event.x(), event.y(), event.polarity());
			}
		} else {
            LOG(INFO)<<"Wrong Decay Model";
            return;
        }

		if (resetTimestamp) {
			lowestTime_    = packet.getLowestTime();
			resetTimestamp = false;
		}
		highestTime_ = packet.getHighestTime();
	}

    /**
	 * Generates the accumulation frame (potential surface) at the time of the
	 * last consumed event.
	 * The function writes the output image into the given `frame` argument.
	 * The output frame will contain data with type CV_8U.
	 * @param frame the frame to copy the data to
	 */
	[[nodiscard]] dv::Frame TimeSurfaceAccumulator::generateFrame() {
		cv::Mat image;

		if (synchronousDecay_ && (decayFunction_ != Decay::NONE) && (decayFunction_ != Decay::STEP)) {
            double expVal(0);
			for (int16_t y = 0; y < shape_.height; y++) {
				for (int16_t x = 0; x < shape_.width; x++) {
					// decay(static_cast<int16_t&>(x), static_cast<int16_t&>(y), highestTime_);
					// decay(x, y, highestTime_);
                    // potentialSurface_.at<float>(y, x)
					// = static_cast<float>(
					// 		  expf(-(static_cast<float>(highestTime_ - decayTimeSurface_mat_.ptr<double>(y)[x]))
                    //            * static_cast<float>(decayParam_)));

                    int64_t t = time_matrix_(y, x);
                    if (t==0) continue;
                    double dt = (highestTime_ - t)*1e-6;
                    if (dt > 0.05) {
                        expVal = 0;
                    } else {
                        expVal = 400*(dt-0.05)*(dt-0.05);
                    }
                    time_surface_map_tmp_.ptr<double>(y)[x] = expVal;
				}
			}
		}

		// Normalize min-max
		// const double scaleFactor = 255.0 / static_cast<double>(maxPotential_ - minPotential_);
		// const double shiftFactor = -static_cast<double>(minPotential_) * scaleFactor;
		// potentialSurface_.convertTo(image, CV_8UC1, scaleFactor, shiftFactor);

        time_surface_map_tmp_ = 255.0 * time_surface_map_tmp_;
        time_surface_map_tmp_.convertTo(time_surface_map_, CV_8U);
        cv::medianBlur(time_surface_map_, image, 3);

		const auto frameTimestamp = lowestTime_;

		// in case of step decay function, clear potential surface
		if (decayFunction_ == Decay::STEP) {
			potentialSurface_.setTo(static_cast<double>(neutralPotential_));
			lowestTime_ = -1;
		}

		resetTimestamp = true;

		return {frameTimestamp, (highestTime_ - frameTimestamp), 0, 0, image, dv::FrameSource::ACCUMULATION};
	}


}  // namespace DynamicObjectsAvoidance