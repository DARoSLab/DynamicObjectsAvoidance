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
    // accumulate(store);
    // dv::EventStore ts_event_store_;
    // ts_event_store_.add(store);
    // LOG(INFO)<<"accept 0";
    // LOG(INFO)<<"store is empty "<<ts_event_store_.isEmpty();
    // LOG(INFO)<<"size = "<<ts_event_store_.size();
    // LOG(INFO)<<"front = "<<ts_event_store_.front().x();
    // uint64_t i = 0;
    // for (const dv::concepts::AddressableEvent auto &e : store) {
    //     // i++;
    //     // std::cout<<i;
    //     // if (store.end()) LOG(INFO)<<"event info "<<event.x()<<" "<<event.y()<<" "<<event.timestamp();
    //     // insertEvent(event);
    //     int64_t t = e.timestamp();
    //     int16_t x = e.x();
    //     int16_t y = e.y();
    //     // time_matrix_(e.y(), e.x()) = e.timestamp();
    //     // i++;
    // }
    // for (auto it = store.begin(); it!=store.end()-1;) {
    //     if (i == 0) LOG(INFO)<<"enter, before. it = "<<&it;
    //     if (i == 1) LOG(INFO)<<"second iteration, before. it = "<<&it;
    //     time_matrix_((*it).y(), (*it).x()) = (*it).timestamp();
    //     if (i == 1) LOG(INFO)<<"second iteration, after. it = "<<&it;
    //     if (i == 0) LOG(INFO)<<"enter, after. it = "<<&it;
    //     i++;
    //     it++;
    //     if (i == 1) LOG(INFO)<<"first iteration. it = "<<&it;
    // }
    // LOG(INFO)<<"store.size = "<<store.size();
    for (uint64_t it = 0; it < store.size()-1; ++it) {
        // if (it % 100 == 0) {LOG(INFO)<<"store.size = "<<store.size()<<" it = "<<it;}
        // if (it < 5) LOG(INFO)<<"it before = "<<it<<" "<<store.at(it).timestamp();
        // if (it > store.size()-100) LOG(INFO)<<"it before = "<<it<<" "<<store.at(it).timestamp();
        // int64_t x = ts_event_store_.at(it).x();
        time_matrix_(store.at(it).y(), store.at(it).x()) = store.at(it).timestamp();
        // if (it < 5) LOG(INFO)<<"it after = "<<it<<" "<<store.at(it).timestamp();
        // if (it > store.size()-100) LOG(INFO)<<"it after = "<<it<<" "<<store.at(it).timestamp();
    }
    // LOG(INFO)<<"finish store time";
    // LOG(INFO)<<"accept 1";
    // for (auto &e : store) {
    //     EventTimestampQueue& tq = arr_T_[e.x() + width_ * e.y()];
    //     tq.push_back(e.timestamp()); // Most recent event is at back
    //     while(tq.size() > queueLen_)
    //         tq.pop_front();
    // }

}

void TimeSurface::insertEvent(const dv::Event& e) {
    // if(!insideImage(e.x(), e.y())) {
    //     return;
    // } else {
    //   EventTimestampQueue& tq = getEventQueue(e.x(), e.y());
    //   tq.push_back(e.timestamp()); // Most recent event is at back
    //   while(tq.size() > queueLen_)
    //     tq.pop_front(); // remove oldest events to fit queue length
    // }
    // if(!insideImage(e.x(), e.y())) {
    //     return;
    // } else {
        time_matrix_(e.y(), e.x()) = e.timestamp();
    // }
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

// void TimeSurface::insertEvent(const dv::Event e) {
//     if(!insideImage(e.x(), e.y())) {
//         return;
//     } else {
//       EventQueue& eq = getEventQueue(e.x(), e.y());
//       eq.push_back(e); // Most recent event is at back
//       while(eq.size() > queueLen_)
//         eq.pop_front(); // remove oldest events to fit queue length
//     }
// }


// bool TimeSurface::getMostRecentEventBeforeT(const size_t x,
//     const size_t y, const int64_t& t, dv::Event* ev) {
//     // Outside of image: false
//     if(!insideImage(x, y))
//         return false;

//     // No event at xy: false
//     EventQueue& eq = getEventQueue(x, y);
//     if(eq.empty())
//         return false;

//     // Loop through all events to find most recent event
//     // Assume events are ordered from latest to oldest
//     for(auto it = eq.rbegin(); it != eq.rend(); ++it)
//     {
//         const dv::Event& e = *it;
//         if(e.timestamp() < t)
//         {
//         *ev = *it;
//         return true;
//         }
//     }
//     return false;
// }

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
    
    // For visualization of time matrix
    // Eigen::MatrixXd::Index maxRow, maxCol;
	// Eigen::MatrixXd::Index minRow, minCol;
	// int64_t max = time_matrix_.maxCoeff(&maxRow,&maxCol);
    // int64_t min(max);
    // for(size_t y=0; y<height_; ++y) {
    //     for(size_t x=0; x<width_; ++x) {
    //         int64_t t = time_matrix_(y, x);
    //         if (t==0) continue;
    //         if (t < min) min = t;
    //     }
    // }
    // int64_t range = max - min;

    // auto time_matrix_copy = Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic>::Zero(height_, width_);
    // auto time_matrix_copy_d = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(height_, width_);
    // auto time_matrix_copy_d = time_matrix_.cast<double>();
    // auto min_m = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Ones(height_, width_)* double(min);
    // auto time_matrix_copy_d = time_matrix_.cast<double>() - min_m;
    // time_matrix_copy_d = time_matrix_copy_d *2;
    // cv::Mat time_cv;
    // cv::eigen2cv(time_matrix_copy_d, time_cv);
    // cv::imshow("time_matrix", time_cv);
    // for (size_t y = 0; y < height_)


    // LOG(INFO)<<"Time test 2";

    int64_t sum_diff(0);
    // std::vector<int64_t> sum_diff_vec;
    for(size_t x=0; x<width_; ++x) {
        for(size_t y=0; y<height_; ++y) {
            // if (y==100&&x==100) LOG(INFO)<<"Time test 2.0.0";
            int64_t t = time_matrix_(y, x);
            // if (y==100&&x==100) LOG(INFO)<<"Time test 2.0.1";
            if (t==0) continue;
            // For visualization of time matrix
            // double val = double(t - min) / double(range);
            // time_matrix_double_(y,x) = val;
            if (y>6&&y<height_-6&&x>6&&x<width_-6) {
                // sum_diff_vec.clear();
                // if (y==7&&x==7) LOG(INFO)<<"Time test 2.0";
                u_int8_t cnt(0);
                for (size_t patt_i = 0; patt_i < 8; ++patt_i) {
                    int64_t diff = highest_time - time_matrix_(y + pattern_scale_[patt_i][0], x + pattern_scale_[patt_i][1]);
                    // if (highest_time - time_matrix_(y + pattern_scale_[patt_i][0], x + pattern_scale_[patt_i][1]) > 250000) continue;
                    if (diff > 250000) continue;
                    // sum_diff_vec.push_back(highest_time - time_matrix_(y + pattern_scale_[patt_i][0], x + pattern_scale_[patt_i][1]));
                    sum_diff+=diff;
                    ++cnt;
                }
                // if (y==7&&x==7) LOG(INFO)<<"Time test 2.1";
                // if (sum_diff_vec.size() == 0) continue;
                if (cnt == 0) continue;
                // std::sort(sum_diff_vec.begin(), sum_diff_vec.end());
                // double sum = std::accumulate(std::begin(sum_diff_vec), std::end(sum_diff_vec), 0.0);  
                // double mean =  sum / sum_diff_vec.size();
                // double scale_begin_end(0.);
                // if (sum_diff_vec[0] > 0) {
                //     scale_begin_end = (sum_diff_vec[sum_diff_vec.size()-1] / sum_diff_vec[2]);
                // } else {
                //     scale_begin_end = 100.0;
                // }
                // if (y==7&&x==7) LOG(INFO)<<"Time test 2.2";
                // sum_diff = mean;
                sum_diff = int64_t(sum_diff / cnt);
                if (sum_diff < 260) continue;
                // double scale = -200000000/(sum_diff-2500)+2000;//std::pow(scale_begin_end,0.5)*10.0;
                // double scale = -150000000/(sum_diff-2500)+1000;//std::pow(scale_begin_end,0.5)*10.0;
                int16_t scale = int16_t(-150000000/(sum_diff-200)+100);//std::pow(scale_begin_end,0.5)*10.0;
                // double scale = (-150000000/(sum_diff-200)+100);//std::pow(scale_begin_end,0.5)*10.0;
                // if (scale_begin_end<6&&sum_diff>20000) scale *= 3;
                // if (scale_begin_end>10&&sum_diff<20000) scale *= .3;
                dt = (highest_time - t)*1e-6;
                // if (dt>0.02) scale = dt*50*(-10000); // for old pixels, we just let it decay faster
                if (sum_diff < 90000) {
                    expVal = scale*dt*dt+1;
                } else {
                    expVal = -10000*dt*dt+1;
                }
                // if (y==7&&x==7) LOG(INFO)<<"Time test 2.3";
                // if(x==98&&y==90)LOG(INFO)<<"90 98: sum_diff= "<<sum_diff<<"  scale= "<<scale<<"   dt= "<<dt<<"   expVal= "<<expVal
                // << "  int scale = "<<int16_t(-150000000/(sum_diff-200)+100);
                // if(x==310&&y==24)LOG(INFO)<<"24 310: sum_diff= "<<sum_diff<<"  scale= "<<scale<<"   dt= "<<dt<<"   expVal= "<<expVal<<"  scale_begin_end="<<scale_begin_end;
                // // if(x==98&&y==90)LOG(INFO)<<sum_diff_vec[0]<<" "<<sum_diff_vec[1]<<" "<<sum_diff_vec[2]<<" "<<sum_diff_vec[3]<<" "<<sum_diff_vec[4]<<" "
                // // <<sum_diff_vec[5]<<" "<<sum_diff_vec[6]<<" "<<sum_diff_vec[7]<<" "<<sum_diff_vec[8]<<" "<<sum_diff_vec[9]<<" "<<sum_diff_vec[10]<<" "<<sum_diff_vec[11]<<" "<<sum_diff_vec[12]<<" "
                // // <<sum_diff_vec[13]<<" "<<sum_diff_vec[14]<<" "<<sum_diff_vec[15];
                // if(x==98&&y==90)LOG(INFO)<<sum_diff_vec[0]<<" "<<sum_diff_vec[1]<<" "<<sum_diff_vec[2]<<" "<<sum_diff_vec[3]<<" "<<sum_diff_vec[4]<<" "
                // <<sum_diff_vec[5]<<" "<<sum_diff_vec[6]<<" "<<sum_diff_vec[7];
                // if(x==310&&y==24)LOG(INFO)<<sum_diff_vec[0]<<" "<<sum_diff_vec[1]<<" "<<sum_diff_vec[2]<<" "<<sum_diff_vec[3]<<" "<<sum_diff_vec[4]<<" "
                // <<sum_diff_vec[5]<<" "<<sum_diff_vec[6]<<" "<<sum_diff_vec[7];
                time_surface_map_tmp_.ptr<double>(y)[x] = expVal;
                // if (y==7&&x==7) LOG(INFO)<<"Time test 2.4";
            } else {
                dt = (highest_time - t)*1e-6;
                expVal = -10000*dt*dt+1;
                // expVal = -100*dt*dt*dt*dt+1;
                // expVal = -scale*dt*dt+1;
                // expVal = -25*(dt+0.2)*(dt-0.2);
                // expVal = std::exp(-dt * 60);
                time_surface_map_tmp_.ptr<double>(y)[x] = expVal;
            }
            // dt = (highest_time - t)*1e-6;
            // expVal = -10000*dt*dt+1;
            // time_surface_map_tmp2_.ptr<double>(y)[x] = expVal;
                // if (y==7&&x==7) LOG(INFO)<<"Time test 2.5";
        }
    }
    // LOG(INFO)<<"Time test 3";
    // LOG(INFO)<<"1.3";
    // LOG(INFO)<<"time_surface_map[90][98] = "<<time_surface_map_tmp_.ptr<double>(90)[98];
    time_surface_map_tmp_ = 255.0 * time_surface_map_tmp_;
    // LOG(INFO)<<"time_surface_map[90][98] = "<<time_surface_map_tmp_.ptr<double>(90)[98];
    time_surface_map_tmp_.convertTo(time_surface_map_, CV_8UC1);
    // LOG(INFO)<<"time_surface_map[90][98] = "<<int(time_surface_map_.ptr<uchar>(90)[98]);
    // LOG(INFO)<<"time_surface_map[24][310] = "<<int(time_surface_map_.ptr<uchar>(24)[310]);
    // time_surface_map_tmp2_ = 255.0 * time_surface_map_tmp2_;
    // time_surface_map_tmp2_.convertTo(time_surface_map2_, CV_8UC1);

    // LOG(INFO)<<"Time test 4";

    // For visualization of time matrix
    // cv::eigen2cv(time_matrix_double_, time_cv_);
    // time_cv_ = 255.0 * time_cv_;
    // time_cv_.convertTo(time_cv_, CV_8U);
    // cv::imshow("time", time_cv);
    // cv::waitKey(1);




    // LOG(INFO)<<"1.4";
    // auto t2 = std::chrono::steady_clock::now();
    // auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    // LOG(INFO) << "Previous timesurface cost time: " << time_used.count() << " seconds.";
            

    // auto t3 = std::chrono::steady_clock::now();
    // LOG(INFO)<<"1.5";
    // // t_diff_double_ = (-20*(highest_time - time_matrix_.array())*(highest_time - time_matrix_.array())).cast<double>();
    // // t_diff_double_ = ((t_diff_double_*1e-12).array()+1.0).matrix();
    // t_diff_double_ = (60*(time_matrix_.array()-highest_time)).exp().cast<double>();
    // t_diff_double_ = ((t_diff_double_*1e-12).array()).matrix();
    // LOG(INFO)<<"1.6";
    // cv::Mat mat(t_diff_double_.rows(), t_diff_double_.cols(), cv::DataType<double>::type);
    // cv::eigen2cv(t_diff_double_, mat);
    // LOG(INFO)<<"1.7";
    // mat = 255.0 * mat;
    // mat.convertTo(mat, CV_8U);
    // LOG(INFO)<<"1.8";
    // auto t4 = std::chrono::steady_clock::now();
    // auto time_used2 = std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3);
    // LOG(INFO) << "Current timesurface cost time: " << time_used2.count() << " seconds.";
    // // cv::imshow("mat", mat);
    // // cv::waitKey(1);





    // const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> tsDiff
	// 		= (highest_time - time_matrix_.array())*1e-6
    //         .max(std::numeric_limits<T>::min()).matrix().template cast<T>();

	// 	cv::Mat mat(tsDiff.rows(), tsDiff.cols(), cv::DataType<T>::type);
	// 	cv::eigen2cv(tsDiff, mat);
    
    // median blur
    cv::blur(time_surface_map_, time_surface_map_, cv::Size(3,3));
    cv::medianBlur(time_surface_map_, time_surface_map_, 3);
    LOG(INFO)<<"Time test 5";
    // cv::blur(time_surface_map2_, time_surface_map2_, cv::Size(3,3));
    // cv::medianBlur(time_surface_map2_, time_surface_map2_, 3);
    // LOG(INFO)<<"3 -- 98 90:"<<int(time_surface_map_.at<uchar>(90,98));
    // LOG(INFO)<<"4 -- 98 90:"<<int(time_surface_map2_.at<uchar>(90,98));
    // LOG(INFO)<<"time_surface_map[24][310] = "<<int(time_surface_map_.ptr<uchar>(24)[310]);
    // static uint64_t ts_cnt(0);
    // cv::Point2f pt0(100, 100);
    // cv::circle(time_surface_map_, pt0, 2, cv::Scalar(0, 250, 0), 1);
    // cv::circle(time_surface_map2_, pt0, 2, cv::Scalar(0, 250, 0), 1);
    // cv::putText(time_surface_map_, //target image
    // std::to_string(stdev), //text
    // cv::Point(20, 20), //top-left position
    // cv::FONT_HERSHEY_DUPLEX,
    // 0.6,
    // CV_RGB(255, 255, 255), //font color
    // 2);
    // cv::putText(time_surface_map2_, //target image
    // std::to_string(stdev), //text
    // cv::Point(20, 20), //top-left position
    // cv::FONT_HERSHEY_DUPLEX,
    // 0.6,
    // CV_RGB(255, 255, 255), //font color
    // 2);
    // cv::imwrite("/home/zh/data/img/event_test/"+std::to_string(ts_cnt)+".png", time_surface_map_);
    // cv::imwrite("/home/zh/data/img/event_test/"+std::to_string(ts_cnt)+"-2.png", time_surface_map2_);
    // ts_cnt++;
    // cv::imshow("timesurface_mine", time_surface_map_);
    // cv::waitKey(1);
    // static uint64_t i = 0;
    // if (i%10 == 0 && i > 50) {
    //     cv::imwrite("/home/zh/data/img/event/"+std::to_string(i)+".png", time_surface_map_);
    // }
    // i++;
    // LOG(INFO)<<"1.9";

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
        LOG(INFO)<<"TS 0.1";
		if (potentialSurface_.empty()) {
			return;
		}
        LOG(INFO)<<"TS 0.2";

		if (packet.isEmpty()) {
			return;
		}
        LOG(INFO)<<"TS 0.3";

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
            LOG(INFO)<<"TS 0.4";
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
        LOG(INFO)<<"TS 0.5";
		cv::Mat image;

		if (synchronousDecay_ && (decayFunction_ != Decay::NONE) && (decayFunction_ != Decay::STEP)) {
        LOG(INFO)<<"TS 0.6";
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
        LOG(INFO)<<"TS 0.7";
		}

		// Normalize min-max
		// const double scaleFactor = 255.0 / static_cast<double>(maxPotential_ - minPotential_);
		// const double shiftFactor = -static_cast<double>(minPotential_) * scaleFactor;
		// potentialSurface_.convertTo(image, CV_8UC1, scaleFactor, shiftFactor);

        time_surface_map_tmp_ = 255.0 * time_surface_map_tmp_;
        time_surface_map_tmp_.convertTo(time_surface_map_, CV_8U);
        cv::medianBlur(time_surface_map_, image, 3);
        LOG(INFO)<<"TS 0.8";

		const auto frameTimestamp = lowestTime_;

		// in case of step decay function, clear potential surface
		if (decayFunction_ == Decay::STEP) {
			potentialSurface_.setTo(static_cast<double>(neutralPotential_));
			lowestTime_ = -1;
		}

		resetTimestamp = true;
        LOG(INFO)<<"TS 0.9";

		return {frameTimestamp, (highestTime_ - frameTimestamp), 0, 0, image, dv::FrameSource::ACCUMULATION};
	}


}  // namespace DynamicObjectsAvoidance