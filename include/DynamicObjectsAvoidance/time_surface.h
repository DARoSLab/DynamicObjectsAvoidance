#pragma once
#include "DynamicObjectsAvoidance/common_include.h"
#include <dv-processing/core/core.hpp>
#include <deque>
#include "DynamicObjectsAvoidance/config.h"
#include "dv-processing/core/frame.hpp"
#include<Eigen/Core>
#include <opencv2/core/eigen.hpp>

namespace DynamicObjectsAvoidance {
using EventQueue = std::deque<dv::Event>;
using EventTimestampQueue = std::deque<int64_t>;

class TimeSurface {
public:
    TimeSurface(const std::string& cfg_path, int width, int height, int queueLen);
    ~TimeSurface();
	void accumulate(const dv::EventStore &packet);
    void accept(const dv::EventStore &store);
    void insertEvent(const dv::Event& e);
    // bool insideImage(const size_t x, const size_t y);
    // void createTimeSurface(int64_t& highest_time, cv::Mat* depth_img);
    void createTimeSurface(int64_t& highest_time);
    bool getMostRecentEventBeforeT(const size_t x, const size_t y, const int64_t& t, dv::Event* ev);
    void generateImage(const dv::EventStore& e, const int64_t& highest_time);
    cv::Mat generateTimeSurface(int64_t& highest_time);
    // dv::EventStore ts_event_store_;

    inline EventTimestampQueue& getEventQueue(const size_t& x, const size_t& y) {
        return eqT_[x + width_ * y];
    }
    inline bool insideImage(const size_t x, const size_t y) {
        return !(x < 0 || x >= width_ || y < 0 || y >= height_);
    }
    inline double findDepthOnRGBDImage(int y, int x, cv::Mat* depth_img) {
        // 0.001 is the scale transfer depth value from intensity to meter
        double d = depth_img->at<ushort>(y, x) * 0.001;
        if ( d > 0.1 ) {
            return d;
        } else {
            // check the nearby points 
            int dx[4] = {-1,0,1,0};
            int dy[4] = {0,-1,0,1};
            for ( int i=0; i<4; i++ ) {
                // d = depth_.ptr<ushort>( y+dy[i] )[x+dx[i]];
                d = depth_img->at<double>(y+dy[i], x+dx[i]);
                if ( d > 0.1 ) {
                    return d;
                }
            }
        }
        return -1.0;
    }

    // inline int64_t& getEventQueue(const size_t& x, const size_t& y) {
    //     return time_matrix_[x + width_ * y];
    // }

	dv::EventStore buffer;
    std::vector<EventQueue> eqMat_;
    std::vector<EventTimestampQueue> eqT_;
    Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic> time_matrix_;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> time_matrix_double_;
    cv::Mat time_cv_;
    Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic> t_diff_;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> t_diff_double_;
    EventTimestampQueue* arr_T_;
    cv::Mat time_surface_map_, time_surface_map2_, time_surface_map_tmp_, time_surface_map_tmp2_;
    int width_;
    int height_;
    int queueLen_;
    int decay_ms_;
    double decay_sec_;
    // int pattern_scale_[8][2] = {{0,-2}, {-1,-1}, {-2,0}, {-1,1}, {0,2}, {1,1}, {2,0}, {1,-1}};
    // int pattern_scale_[8][2] = {{0,-3}, {-2,-2}, {-3,0}, {-2,2}, {0,3}, {2,2}, {3,0}, {2,-2}};
    int pattern_scale_[8][2] = {{0,-4}, {-3,-3}, {-4,0}, {-3,3}, {0,4}, {3,3}, {4,0}, {3,-3}};
    // int pattern_scale_[16][2] = {{0,-4}, {-3,-3}, {-4,0}, {-3,3}, {0,4}, {3,3}, {4,0}, {3,-3}, {-1,-4},{-4,-2},{-4,1},{-2,4},{1,4},{4,2},{4,-2},{1,-4}};
    // int pattern_scale_[8][2] = {{0,-6}, {-5,-5}, {-6,0}, {-5,5}, {0,6}, {5,5}, {6,0}, {5,-5}};
};



class TimeSurfaceAccumulator : public dv::AccumulatorBase {
public:
	/**
	 * Decay function to be used to decay the surface potential.
	 *
	 * * `NONE`: Do not decay at all. The potential can be reset manually
	 *    by calling the `clear` function
	 *
	 * * `LINEAR`: Perform a linear decay with  given slope. The linear decay goes
	 *    from currentpotential until the potential reaches the neutral potential
	 *
	 * * `EXPONENTIAL`: Exponential decay with time factor tau. The potential
	 *    eventually converges to zero.
	 *
	 * * `STEP`: Decay sharply to neutral potential after the given time.
	 *    Constant potential before.
	 */
	enum class Decay { NONE = 0, LINEAR = 1, EXPONENTIAL = 2, STEP = 3, MYEXPONENTIAL = 4};

private:
	// input
	bool rectifyPolarity_    = false;
	float eventContribution_ = .0;
	float maxPotential_      = .0;
	float neutralPotential_  = .0;
	float minPotential_      = .0;

	// decay
	Decay decayFunction_   = Decay::NONE;
	double decayParam_     = .0;
	bool synchronousDecay_ = false;

	// state
	dv::TimeSurface decayTimeSurface_;
	Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic> time_matrix_;
    cv::Mat decayTimeSurface_mat_;
	cv::Mat potentialSurface_;
	cv::Mat time_surface_map_;
	cv::Mat time_surface_map_tmp_;
	int64_t highestTime_ = 0;
	int64_t lowestTime_  = -1;
	bool resetTimestamp  = true;

	// internal use methods
	/**
	 * __INTERNAL_USE_ONLY__
	 * Decays the potential at coordinates x, y to the given time, respecting the
	 * decay function. Updates the time surface to the last decay.
	 * @param x The x coordinate of the value to be decayed
	 * @param y The y coordinate of the value to be decayed
	 * @param time The time to which the value should be decayed to.
	 */
	void decay(int16_t& x, int16_t& y, int64_t& time);

	/**
	 * __INTERNAL_USE_ONLY__
	 * Contributes the effect of a single event onto the potential surface.
	 * @param x The x coordinate of where to contribute to
	 * @param y The y coordinate of where to contribute to
	 * @param polarity The polarity of the contribution
	 */
	void contribute(int16_t x, int16_t y, bool polarity);
public:
	/**
	 * Silly default constructor. This generates an accumulator with zero size.
	 * An accumulator with zero size does not work. This constructor just exists
	 * to make it possible to default initialize an Accumulator to later redefine.
	 */
	TimeSurfaceAccumulator() : AccumulatorBase(cv::Size(0, 0)) {
	}

	/**
	 * Accumulator constructor
	 * Creates a new Accumulator with the given params. By selecting the params
	 * the right way, the Accumulator can be used for a multitude of applications.
	 * The class also provides static factory functions that adjust the parameters
	 * for common use cases.
	 *
	 * @param resolution The size of the resulting frame. This must be at least the
	 * dimensions of the eventstream supposed to be added to the accumulator,
	 * otherwise this will result in memory errors.
	 * @param decayFunction The decay function to be used in this accumulator.
	 * The decay function is one of `NONE`, `LINEAR`, `EXPONENTIAL`, `STEP`. The
	 * function behave like their mathematical definitions, with LINEAR AND STEP
	 * going back to the `neutralPotential` over time, EXPONENTIAL going back to 0.
	 * @param decayParam The parameter to tune the decay function. The parameter has
	 * a different meaning depending on the decay function chosen:
	 * `NONE`: The parameter is ignored
	 * `LINEAR`: The paramaeter describes the (negative) slope of the linear function
	 * `EXPONENTIAL`: The parameter describes tau, by which the time difference is divided.
	 * @param synchronousDecay if set to true, all pixel values get decayed to the same time
	 * as soon as the frame is generated. If set to false, pixel values remain at the state
	 * they had when the last contribution came in.
	 * @param eventContribution The contribution a single event has onto the potential
	 * surface. This value gets interpreted positively or negatively depending on the
	 * event polarity
	 * @param maxPotential The upper cut-off value at which the potential surface
	 * is clipped
	 * @param neutralPotential The potential the decay function converges to over time.
	 * @param minPotential The lower cut-off value at which the potential surface
	 * is clipped
	 * @param rectifyPolarity Describes if the polarity of the events should be kept
	 * or ignored. If set to true, all events behave like positive events.
	 */
	explicit TimeSurfaceAccumulator(const cv::Size &resolution, TimeSurfaceAccumulator::Decay decayFunction = Decay::EXPONENTIAL,
		double decayParam = 1.0e+6, bool synchronousDecay = false, float eventContribution = 0.15f,
		float maxPotential = 1.0f, float neutralPotential = 0.f, float minPotential = 0.f,
		bool rectifyPolarity = false) :
		AccumulatorBase(resolution),
		rectifyPolarity_(rectifyPolarity),
		eventContribution_(eventContribution),
		maxPotential_(maxPotential),
		neutralPotential_(neutralPotential),
		minPotential_(minPotential),
		decayFunction_(decayFunction),
		decayParam_(decayParam),
		synchronousDecay_(synchronousDecay),
		decayTimeSurface_(dv::TimeSurface(resolution)),
		potentialSurface_(cv::Mat(resolution, CV_32F, static_cast<double>(neutralPotential))),
		highestTime_(0) {
            decayTimeSurface_mat_= cv::Mat(resolution, CV_64FC1);
//     time_matrix_ = Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic>::Zero(height, width);
// typedef Eigen::Matrix<double, 4, 1> Vec4;
			time_matrix_ = Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic>::Zero(resolution.height, resolution.width);
    		time_surface_map_ = cv::Mat::zeros(resolution.height, resolution.width, CV_8U);
    		time_surface_map_tmp_ = cv::Mat::zeros(resolution.height, resolution.width, CV_64F);
	}

	/**
	 * Accumulates all the events in the supplied packet and puts them onto the
	 * accumulation surface.
	 * @param packet The packet containing the events that should be
	 * accumulated.
	 */
	void accumulate(const dv::EventStore &packet) override;

	/**
	 * Generates the accumulation frame (potential surface) at the time of the
	 * last consumed event.
	 * The function writes the output image into the given `frame` argument.
	 * The output frame will contain data with type CV_8U.
	 * @param frame the frame to copy the data to
	 */
	[[nodiscard]] dv::Frame generateFrame() override;

	/**
	 * Clears the potential surface by setting it to the neutral value.
	 * This function does not reset the time surface.
	 */
	void clear() {
		potentialSurface_ = cv::Mat(shape_, CV_32F, static_cast<double>(neutralPotential_));
		lowestTime_       = -1;
	}

	// setters
	/**
	 * If set to true, all events will incur a positive contribution to the
	 * potential surface
	 * @param rectifyPolarity The new value to set
	 */
	void setRectifyPolarity(bool rectifyPolarity) {
		TimeSurfaceAccumulator::rectifyPolarity_ = rectifyPolarity;
	}

	/**
	 * Contribution to the potential surface an event shall incur.
	 * This contribution is either counted positively (for positive events
	 * or when `rectifyPolatity` is set).
	 * @param eventContribution The contribution a single event shall incur
	 */
	void setEventContribution(float eventContribution) {
		TimeSurfaceAccumulator::eventContribution_ = eventContribution;
	}

	/**
	 * @param maxPotential the max potential at which the surface should be capped at
	 */
	void setMaxPotential(float maxPotential) {
		TimeSurfaceAccumulator::maxPotential_ = maxPotential;
	}

	/**
	 * Set a new neutral potential value. This will also reset the cached potential surface
	 * to the given new value.
	 * @param neutralPotential The neutral potential to which the decay function should go.
	 * Exponential decay always goes to 0. The parameter is ignored there.
	 */
	void setNeutralPotential(float neutralPotential) {
		TimeSurfaceAccumulator::neutralPotential_ = neutralPotential;
		potentialSurface_              = cv::Mat(shape_, CV_32F, static_cast<double>(neutralPotential_));
	}

	/**
	 * @param minPotential the min potential at which the surface should be capped at
	 */
	void setMinPotential(float minPotential) {
		TimeSurfaceAccumulator::minPotential_ = minPotential;
	}

	/**
	 * @param decayFunction The decay function the module should use to perform the decay
	 */
	void setDecayFunction(Decay decayFunction) {
		TimeSurfaceAccumulator::decayFunction_ = decayFunction;
	}

	/**
	 * The decay param. This is slope for linear decay, tau for exponential decay
	 * @param decayParam The param to be used
	 */
	void setDecayParam(double decayParam) {
		TimeSurfaceAccumulator::decayParam_ = 1./decayParam;
	}

	/**
	 * If set to true, all valued get decayed to the frame generation time at
	 * frame generation. If set to false, the values only get decayed on activity.
	 * @param synchronousDecay the new value for synchronoues decay
	 */
	void setSynchronousDecay(bool synchronousDecay) {
		TimeSurfaceAccumulator::synchronousDecay_ = synchronousDecay;
	}

	[[nodiscard]] bool isRectifyPolarity() const {
		return rectifyPolarity_;
	}

	[[nodiscard]] float getEventContribution() const {
		return eventContribution_;
	}

	[[nodiscard]] float getMaxPotential() const {
		return maxPotential_;
	}

	[[nodiscard]] float getNeutralPotential() const {
		return neutralPotential_;
	}

	[[nodiscard]] float getMinPotential() const {
		return minPotential_;
	}

	[[nodiscard]] Decay getDecayFunction() const {
		return decayFunction_;
	}

	[[nodiscard]] double getDecayParam() const {
		return decayParam_;
	}

	/**
	 * Accumulates the event store into the accumulator.
	 * @param store The event store to be accumulated.
	 * @return A reference to this TimeSurfaceAccumulator.
	 */
	TimeSurfaceAccumulator &operator<<(const dv::EventStore &store) {
		accumulate(store);
		return *this;
	}

	/**
	 * Retrieved a copy of the currently accumulated potential surface. Potential surface contains raw
	 * floating point values aggregated by the accumulator, the values are within the configured range of
	 * [minPotential; maxPotential]. This returns a deep copy of the potential surface.
	 * @return 	Potential surface image containing CV_32FC1 data.
	 */
	[[nodiscard]] cv::Mat getPotentialSurface() const {
		return potentialSurface_.clone();
	}
};
}  // namespace DynamicObjectsAvoidance

// #endif  // DynamicObjectsAvoidance_CONFIG_H
