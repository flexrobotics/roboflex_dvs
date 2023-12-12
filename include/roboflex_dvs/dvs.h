#ifndef ROBOFLEX_DVS__H
#define ROBOFLEX_DVS__H

#include <iostream>
#include <functional>
#include <list>
#include <cyusb.h>
#include <Eigen/Dense>
#include "roboflex_core/core.h"
#include "roboflex_core/core_nodes/frequency_generator.h"

namespace roboflex {
namespace dvs {

using std::ostream;

constexpr char ModuleName[] = "dvs";

/**
 * This is an srl message that carries the 'raw' data that is read
 * from the dvs device, with no parsing at all.
 */
class DVSRawData: public core::Message {
public:
    inline static const char MessageName[] = "DVSRawData";

    DVSRawData(core::Message& other): core::Message(other) {}
    DVSRawData(double t0, double t1, const uint8_t *byte_data, int num_bytes);

    double get_t0() const { return root_val("t0").AsDouble(); }
    double get_t1() const { return root_val("t1").AsDouble(); }
    const uint8_t* get_data() const { return root_val("data").AsBlob().data(); }
    const int get_length() const { return root_val("data").AsBlob().size(); }

    void print_on(ostream& os) const override;
};

/**
 * The datatype containing parsed dvs event data: two frames
 * (on events and off events) in two eigen matrices, where
 * each row corresponds to one event, the first column is x,
 * and the second column is y.
 */
class DVSEigenData: public core::Message {
public:
    typedef Eigen::Matrix<unsigned short, Eigen::Dynamic, 2> DVSFrame;

    inline static const char MessageName[] = "DVSEigenData";

    DVSEigenData(core::Message& other): core::Message(other) {}
    DVSEigenData(
        unsigned short *on_event_data, int num_on_events,
        unsigned short *off_event_data, int num_off_events,
        double t, double t0, double t1);

    double get_t() const { return root_val("t").AsDouble(); }
    double get_t0() const { return root_val("t0").AsDouble(); }
    double get_t1() const { return root_val("t1").AsDouble(); }

    const DVSFrame get_on_events() const {
        return serialization::deserialize_eigen_matrix<unsigned short, Eigen::Dynamic, 2>(root_val("on_events"));
    }

    const DVSFrame get_off_events() const {
        return serialization::deserialize_eigen_matrix<unsigned short, Eigen::Dynamic, 2>(root_val("off_events"));
    }

    void print_on(ostream& os) const override;
};

class DVSEigenImage: public core::Message {
public:
    typedef Eigen::Matrix<uint8_t, 320, 480, Eigen::RowMajor> DVSImage;

    inline static const char MessageName[] = "DVSEigenImage";

    DVSEigenImage(core::Message& other): core::Message(other) {}
    DVSEigenImage(const DVSImage& dvs_image);

    const DVSImage get_image() const {
        return serialization::deserialize_eigen_matrix<uint8_t, 320, 480>(root_val("image"));
    }

    void print_on(ostream& os) const override;
};


/**
 * This dvs sensor pushes the raw data it reads from the dvs
 * device, with no parsing or interpretation - that's left
 * to other nodes (the DVSEncoder node, to be precise).
 *
 * expects: nothing
 * signals: DVSRawData
 */
class DVSSensor: public core::RunnableNode {
public:
    DVSSensor(const std::string& name = "DVSSensor");
    virtual ~DVSSensor();
protected:
    void child_thread_fn() override;
    libusb_device_handle* dvs_handle_;
};


/**
 * Parses raw dvs data into "frames" (yeah, that means it's not
 * actually event-based).
 *
 * expects: DVSRawData
 * signals: DVSEigenData
 */
class DVSEncoder: public core::Node {
public:
    DVSEncoder(const std::string &name = "DVSEncoder");

    void receive(core::MessagePtr m) override;

protected:
    void got_event(bool on_off, int x, int y, unsigned int t);

    double t0;
    unsigned int prev_time_stamp;

    unsigned int current_on_event_index;
    unsigned int current_off_event_index;
    unsigned short current_on_events[640*480*2];
    unsigned short current_off_events[640*480*2];

	unsigned int longTs = 0; // * 10 usec
	unsigned int shortTs = 0; // * 1 msec
	unsigned int timeStamp = 0; // * usec
};

class DVSEigenToGrayScale: public nodes::FrequencyGenerator {
public:
    DVSEigenToGrayScale(
        float emit_frequency_hz = 24.0,
        const std::string &name = "DVSEigenToGrayScale");

    void receive(core::MessagePtr m) override;

protected:

    void on_trigger(double wall_clock_time) override;

    std::mutex image_mutex;
    //xt::xtensor<uint8_t, 2> accumulated_image;
    Eigen::Matrix<uint8_t, 320, 480, Eigen::RowMajor> accumulated_image;
};

} // namespace dvs
} // namespace roboflex

#endif // ROBOFLEX_DVS__H
