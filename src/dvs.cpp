#include <iostream>
#include "roboflex_dvs/dvs.h"
#include "roboflex_core/util/utils.h"

namespace roboflex {
namespace dvs {


// --- DVSRawData ---

DVSRawData::DVSRawData(double t0, double t1, const uint8_t *byte_data, int num_bytes):
    core::Message(ModuleName, MessageName)
{
    flexbuffers::Builder fbb = get_builder();
    WriteMapRoot(fbb, [&]() {
        fbb.Double("t0", t0);
        fbb.Double("t1", t1);
        fbb.Key("data");
        fbb.Blob(byte_data, num_bytes);
    });
}

void DVSRawData::print_on(ostream& os) const {
    os << "<DVSRawData"
       << " t0: " << get_t0()
       << " t1: " << get_t1()
       << " bytes: " << get_length()
       << " \"" << (int)get_data()[0] << " " << (int)get_data()[1] << " " << (int)get_data()[2] << " ...\" ";
    Message::print_on(os);
    os << ">";
}


// -- DVSEigenData --

DVSEigenData::DVSEigenData(
    unsigned short *on_event_data, int num_on_events,
    unsigned short *off_event_data, int num_off_events,
    double t, double t0, double t1):
        core::Message(ModuleName, MessageName)
{
    // NOTE! RowMajor might NOT be what you want!!!
    DVSFrame on_events = Eigen::Map<const Eigen::Matrix<unsigned short, Eigen::Dynamic, 2, Eigen::RowMajor>>(on_event_data, num_on_events, 2);
    DVSFrame off_events = Eigen::Map<const Eigen::Matrix<unsigned short, Eigen::Dynamic, 2, Eigen::RowMajor>>(off_event_data, num_off_events, 2);

    flexbuffers::Builder fbb = get_builder();
    WriteMapRoot(fbb, [&]() {
        fbb.Double("t", t);
        fbb.Double("t0", t0);
        fbb.Double("t1", t1);
        serialization::serialize_eigen_matrix(fbb, on_events, "on_events");
        serialization::serialize_eigen_matrix(fbb, off_events, "off_events");
    });
}

void DVSEigenData::print_on(ostream& os) const {
    os << "<DVSEigenData"
       << " times: (" << get_t0() << " - " << get_t1() << ")"
       << " t:" << get_t()
       << " on_events: (" << get_on_events().rows() << ", " << get_on_events().cols() << ")"
       << " off_events: (" << get_off_events().rows() << ", " << get_off_events().cols() << ") ";
    Message::print_on(os);
    os << ">";
}


// -- DVSEigenImage --

DVSEigenImage::DVSEigenImage(const DVSImage& dvs_image):
    core::Message(ModuleName, MessageName)
{
    flexbuffers::Builder fbb = get_builder();
    WriteMapRoot(fbb, [&]() {
        serialization::serialize_eigen_matrix(fbb, dvs_image, "image");
    });
}

void DVSEigenImage::print_on(ostream& os) const {
    os << "<DVSEigenImage"
       << " events: (" << get_image().rows() << ", " << get_image().cols() << ") ";
    Message::print_on(os);
    os << ">";
}


// --- DVSSensor ---

DVSSensor::DVSSensor(const std::string &name):
    core::RunnableNode(name),
    dvs_handle_(nullptr)
{
    // std::string script = "./build/third_party/dvs_semiconductor_code/dvsconf -l ./third_party/dvs_semiconductor_code/dvs_configurations/run_dvs_gen3.txt";
    // int dvs_initialized = system(script.c_str());
    // if (dvs_initialized != 0) {
    //     std::cout << "HEYHEYHEY" << std::endl
    //               << "Your DVS might not be initialized!" << std::endl
    //               << "I tried to run this, but it did not work:" << std::endl
    //               << script << std::endl;
    // }

    // Initialize CyUSB.
    int r = cyusb_open();
    if (r < 0) {
        throw std::runtime_error("Error opening library");
    } else if (r == 0) {
        throw std::runtime_error("No device found");
    } else if (r > 1) {
        throw std::runtime_error("More than 1 devices of interest found. Disconnect unwanted devices.");
    }

    // Detect the DVS.
    libusb_device_handle* h1 = cyusb_gethandle(0);
    if (cyusb_getvendor(h1) != 0x04b4) {
        cyusb_close();
        throw std::runtime_error("Cypress chipset not detected");
    }

    // Make sure there's no active kernel.
    r = libusb_kernel_driver_active(h1, 0);
    if (r != 0) {
        cyusb_close();
        throw std::runtime_error("Kernel driver active.");
    }

    // Claim the interface.
    r = libusb_claim_interface(h1, 0);
    if (r != 0) {
        cyusb_close();
        throw std::runtime_error("Error in claiming interface.");
    }

    // Ready to go!
    this->dvs_handle_ = h1;
}

DVSSensor::~DVSSensor()
{
    cyusb_close();
}

void DVSSensor::child_thread_fn()
{
    const int MAX_BUFFER_SIZE = 1024;
    const int BULK_TIMEOUT = 1000;

    uint8_t buffer[MAX_BUFFER_SIZE];
    int num_bytes_read;

    while (!this->stop_signal) {
        double t0 = core::get_current_time();

        // Read from the device.
        int r = libusb_bulk_transfer(
            this->dvs_handle_,
            0x81,
            buffer,
            MAX_BUFFER_SIZE,
            &num_bytes_read,
            BULK_TIMEOUT);

        // It broke. Just bail.
        if (r != 0) {
            cyusb_error(r);
            cyusb_close();
            std::string script = "./build/third_party/dvs_semiconductor_code/dvsconf -l ./third_party/dvs_semiconductor_code/dvs_configurations/run_dvs_gen3.txt";
            std::cout << "Did you do this? " << script << std::endl;
            throw std::runtime_error("Error in reading buffer: " + std::to_string(r));
        }

        if (num_bytes_read > 0) {

            // Take another time measurement.
            double t1 = core::get_current_time();

            // signal the data downstream.
            this->signal(std::make_shared<DVSRawData>(t0, t1, buffer, num_bytes_read));
        }
    }
}


// -- DVSEncoder --

DVSEncoder::DVSEncoder(const std::string& name):
    core::Node(name),
    t0(core::get_current_time()),
    prev_time_stamp(0)
{

}

void DVSEncoder::got_event(bool on_off, int x, int y, unsigned int t)
{
    if (prev_time_stamp == 0) {
        this->t0 = core::get_current_time();
        prev_time_stamp = t;
        current_on_event_index = 0;
        current_off_event_index = 0;
    }

    if (t != prev_time_stamp) {

        // We've come to a new timestamp, so send the previous frame

        if (current_on_event_index > 0 || current_off_event_index > 0) {
            double t1 = core::get_current_time();

            this->signal(std::make_shared<DVSEigenData>(
                current_on_events, current_on_event_index,
                current_off_events, current_off_event_index,
                prev_time_stamp, this->t0, t1));
        }

        this->t0 = core::get_current_time();

        prev_time_stamp = t;
        current_on_event_index = 0;
        current_off_event_index = 0;
    }

    if (on_off) {
        current_on_events[2*current_on_event_index] = x;
        current_on_events[2*current_on_event_index+1] = y;
        current_on_event_index += 1;
    } else {
        current_off_events[2*current_off_event_index] = x;
        current_off_events[2*current_off_event_index+1] = y;
        current_off_event_index += 1;
    }
}

void DVSEncoder::receive(core::MessagePtr m) 
{
    DVSRawData b(*m);

    int grpAddr = 0;
    int posY0 = 0;
    int posY = 0;
    int posX = 0;
    bool pol = false;

    //int packetID = 0;

    if (b.get_data() != nullptr && b.get_length() > 0) {

        unsigned char *buf = (unsigned char *)(b.get_data());
        int transferred = b.get_length();

		if (transferred % 4) transferred = (transferred / 4) * 4;

		for(int i=0; i<transferred; i+=4) {

			int header = buf[i] & 0x7C;
			//id = buf[i] & 0x03;

			if (buf[i] & 0x80) {	// Group Packet

				grpAddr = (buf[i+1] & 0xFC) >> 2;

				if (buf[i + 3]) {
					posY0 = grpAddr << 3;
					pol = (buf[i+1] & 0x01) ? true : false;
					for (int n=0; n<8; n++) {
						if ((buf[i+3] >> n) & 0x01) {
							posY = posY0 + n;
                            got_event(pol, posX, 479-posY, timeStamp);
						}
					}
				}

				if (buf[i + 2]) {
					grpAddr += (header >> 2);	// Offset
					posY0 = grpAddr << 3;
					pol = (buf[i+1] & 0x02) ? true : false;
					for (int n=0; n<8; n++) {
						if ((buf[i+2] >> n) & 0x01) {
							posY = posY0 + n;
                            got_event(pol, posX, 479-posY, timeStamp);
 						}
					}
				}

			} else {					// Normal Packet

				switch (header) {
					case (0x04) :	// 0000 01** | --ST TTTT | TTTT T-CC | CCCC CCCC	Column Address (10) + SubTimestamp (10)
						shortTs = ((buf[i+1] & 0x1F) << 5) | ((buf[i+2] & 0xF8) >> 3);
						timeStamp = longTs + shortTs;
						//posX = (((buf[i + 2] & 0x03) << 8) | (buf[i + 3] & 0xFF));		// Original
						posX = 319 - (((buf[i + 2] & 0x03) << 8) | (buf[i + 3] & 0xFF));		// Rotation
						break;

					case (0x08) :	// 0000 01** | --TT TTTT | TTTT TTTT | TTTT TTTT	Reference Timestamp (22)
						longTs = (((buf[i + 1] & 0x3F) << 16) | ((buf[i + 2] & 0xFF) << 8) | (buf[i + 3] & 0xFF)) * 1000;
						timeStamp = longTs + shortTs; // ?
                        break;

					case (0x40) :	// 0100 00** | --II IIII | IIII IIII | IIII IIII	Packet ID (22)
						// Packet ID is used to check packet loss
						//packetID = ((buf[i + 1] & 0x3F) << 26) | ((buf[i + 2] & 0xFF) << 18) | ((buf[i + 3] & 0xFF) << 10);
                        //std::cout << "packetid:" << packetID << std::endl;
						break;

					case (0x00) :	// 0000 0000 | 0000 0000 | 0000 0000 | 0000 0000	Padding
						//i = dataLen;	// ignore all the remaining packet data
						break;

					default :		// This should not happen
						break;
				}
			}
        }
    }
}


// -- DVSEigenToGrayScale --

DVSEigenToGrayScale::DVSEigenToGrayScale(
    float emit_frequency_hz,
    const std::string &name):
        nodes::FrequencyGenerator(emit_frequency_hz, name)
        //, accumulated_image(xt::ones<uint8_t>({320, 480}))
{
    accumulated_image.fill(128);
}

void DVSEigenToGrayScale::receive(core::MessagePtr m)
{
    DVSEigenData input(*m);

    const DVSEigenData::DVSFrame& on_events(input.get_on_events());
    const DVSEigenData::DVSFrame& off_events(input.get_off_events());

    const std::lock_guard<std::mutex> lock(image_mutex);

    for (int i=0; i<on_events.rows(); i++) {
        accumulated_image(on_events(i, 0), on_events(i, 1)) += 40;
    }

    for (int i=0; i<off_events.rows(); i++) {
        accumulated_image(off_events(i, 0), off_events(i, 1)) -= 40;
    }
}

void DVSEigenToGrayScale::on_trigger(double wall_clock_time)
{
    const std::lock_guard<std::mutex> lock(image_mutex);
    //this->signal(std::make_shared<core::TensorMessage<uint8_t, 2>>(accumulated_image, "DVSImage", "image"));
    this->signal(core::EigenMessage<uint8_t, 320, 480, Eigen::RowMajor>::Ptr(accumulated_image, "DVSImage", "image"));
    accumulated_image.fill(128);
}

} // namespace dvs
} // namespace roboflex
