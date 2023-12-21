#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <utility>

#include <tinyxml.h>
#include <kuka_eki_hw_interface/kuka_eki_hw_interface.h>

namespace device {
    EKI::EKI() : cartesian_position_(n_dof_, 0.0),
                 axis_position_(n_dof_, 0.0), joint_velocity_(n_dof_, 0.0),
                 joint_effort_(n_dof_, 0.0), deadline_(ios_),
                 eki_server_socket_(ios_, boost::asio::ip::udp::endpoint(
                         boost::asio::ip::udp::v4(), 0)) {

    }


    EKI::~EKI() = default;


    void EKI::eki_check_read_state_deadline() {
        // Check if deadline has already passed
        if (deadline_.expires_at() <= boost::asio::deadline_timer::traits_type::now()) {
            eki_server_socket_.cancel();
            deadline_.expires_at(boost::posix_time::pos_infin);
        }

        // Sleep until deadline exceeded
        deadline_.async_wait(boost::bind(&EKI::eki_check_read_state_deadline, this));
    }


    void EKI::eki_handle_receive(const boost::system::error_code &ec, size_t length,
                                 boost::system::error_code *out_ec, size_t *out_length) {
        *out_ec = ec;
        *out_length = length;
    }


    bool EKI::eki_read_state() {
        static boost::array<char, 2048> in_buffer;

        // Read socket buffer (with timeout)
        // Based off of Boost documentation example: doc/html/boost_asio/example/timeouts/blocking_udp_client.cpp
        deadline_.expires_from_now(boost::posix_time::seconds(eki_read_state_timeout_));  // set deadline
        boost::system::error_code ec = boost::asio::error::would_block;
        size_t len = 0;
        eki_server_socket_.async_receive(boost::asio::buffer(in_buffer),
                                         [capture0 = &ec, capture1 = &len](auto &&PH1, auto &&PH2) {
                                             return EKI::eki_handle_receive(std::forward<decltype(PH1)>(PH1),
                                                                            std::forward<decltype(PH2)>(PH2), capture0,
                                                                            capture1);
                                         });
        do
            ios_.run_one();
        while (ec == boost::asio::error::would_block);
        if (ec)
            return false;

        // Update joint positions from XML packet (if received)
        if (len == 0)
            return false;

        // Parse XML
        TiXmlDocument xml_in;
        in_buffer[len] = '\0';  // null-terminate data buffer for parsing (expects c-string)
        xml_in.Parse(in_buffer.data());
        TiXmlElement *robot_state = xml_in.FirstChildElement("RobotState");
        if (!robot_state)
            return false;
        TiXmlElement *xml_pos = robot_state->FirstChildElement("Pos");
        TiXmlElement *xml_vel = robot_state->FirstChildElement("Vel");
        TiXmlElement *xml_eff = robot_state->FirstChildElement("Eff");
        TiXmlElement *robot_command = robot_state->FirstChildElement("RobotCommand");
        if (!xml_pos || !xml_vel || !xml_eff || !robot_command)
            return false;

        // Extract axis positions
        double joint_pos;       // [deg]
        double cartesian_pos;   // [mm]
        double joint_vel;       // [%max]
        double joint_eff;       // [Nm]
        robot_state->Attribute("MovingStatus", &this->moving_status_);
        robot_state->Attribute("ProMove", &this->pro_move_);
        robot_state->Attribute("ActAdvance", &this->act_advance_);
        robot_state->Attribute("AdvanceVal", &this->advance_);
        for (auto i = 0; i < (long int) n_dof_; ++i) {
            xml_pos->Attribute(cartesian_coordinate_name_[i], &cartesian_pos);
            cartesian_position_[i] = cartesian_pos;  // convert deg to rad
            xml_pos->Attribute(axis_coordinate_name_[i], &joint_pos);
            axis_position_[i] = joint_pos;  // convert deg to rad
            xml_vel->Attribute(axis_coordinate_name_[i], &joint_vel);
            joint_velocity_[i] = joint_vel;
            xml_eff->Attribute(axis_coordinate_name_[i], &joint_eff);
            joint_effort_[i] = joint_eff;
        }

        // Extract number of command elements buffered on robot
        robot_command->Attribute("Size", &this->command_size_);

        return true;
    }


    bool EKI::eki_write_command(const ControlStruct &control_struct) {
        TiXmlDocument xml_out;
        auto *robot_command = new TiXmlElement("RobotCommand");
        auto *pos = new TiXmlElement("Pos");
        auto *empty_text = new TiXmlText("");
        robot_command->LinkEndChild(pos);
        pos->LinkEndChild(empty_text);   // force <Pos></Pos> format (vs <Pos />)
        robot_command->SetAttribute("ControlType", control_struct.control_type_);
        robot_command->SetAttribute("MovingType", control_struct.moving_type_);
        robot_command->SetAttribute("CoordinateType", control_struct.coordinate_type_);
        for (auto &&i: control_struct.kv_int_) {
            robot_command->SetAttribute(i.key_, i.value_);
        }
        for (auto &&i: control_struct.kv_double_) {
            robot_command->SetDoubleAttribute(i.key_, i.value_);
        }


        for (auto i = 0; i < (long int) n_dof_; ++i) {
            if (control_struct.target_cartesian_position_.size() == 6) {
                pos->SetAttribute(cartesian_coordinate_name_[i],
                                  std::to_string(control_struct.target_cartesian_position_[i]));
            }
            if (control_struct.target_axis_position_.size() == n_dof_) {
                pos->SetAttribute(axis_coordinate_name_[i],
                                  std::to_string(control_struct.target_axis_position_[i]));
            }
        }
        xml_out.LinkEndChild(robot_command);

        TiXmlPrinter xml_printer;
        xml_printer.SetStreamPrinting();  // no linebreaks
        xml_out.Accept(&xml_printer);

        eki_server_socket_.send_to(boost::asio::buffer(xml_printer.CStr(), xml_printer.Size()),
                                   eki_server_endpoint_);

        return true;
    }

    int EKI::DefaultMonitor(EKI* eki){
        std::ofstream fs{"read.log"};
        if (!fs.is_open())
            throw std::runtime_error("failed to open read.log");
        do {
            eki->Read();
            eki->LogStatus(fs);
            if (!eki->IsReady()) {
                if (eki->advance_ == 0) {
                    if (eki->act_advance_ == 0 && eki->moving_status_ == 1) {
                        eki->SetReady(1);
                    }
                }else{
                    if (eki->act_advance_ < eki->advance_) {
                        eki->SetReady(1);
                    }
                }
            }
        } while (eki->connected_);
        return 0;
    }


    void EKI::Init(const std::string &ip_address, const std::string &port) {
        // Set eki server
        eki_server_address_ = ip_address;
        eki_server_port_ = port;

#ifdef EKI_DEBUG_OUTPUT
        std::cout << "Loaded Kuka EKI hardware interface" << std::endl;
#endif
    }


    void EKI::Connect() {
        if (connected_) {
            throw std::runtime_error("EKI has been connected. Reconnecting is not allowed.");
        }
#ifdef EKI_DEBUG_OUTPUT
        std::cout << "Starting Kuka EKI hardware interface..." << std::endl;

        // Connect client
        std::cout << "... connecting to robot's EKI server..." << std::endl;
#endif
        boost::asio::ip::udp::resolver resolver(ios_);
        eki_server_endpoint_ = *resolver.resolve({boost::asio::ip::udp::v4(), eki_server_address_, eki_server_port_});
        boost::array<char, 1> ini_buf = {0};
        eki_server_socket_.send_to(boost::asio::buffer(ini_buf),
                                   eki_server_endpoint_);  // initiate contact to Connect server

        // Connect persistent actor to check for eki_read_state timeouts
        deadline_.expires_at(boost::posix_time::pos_infin);  // do nothing unit a Read is invoked (deadline_ = +inf)
        eki_check_read_state_deadline();

        // Initialize target_position_ from initial robot state (avoid bad (null) commands before controllers come up)
        if (!eki_read_state()) {
            std::string msg = "Failed to Read from robot EKI server within alloted time of "
                              + std::to_string(eki_read_state_timeout_) +
                              " seconds.  Make sure eki_hw_interface is running "
                              "on the robot controller and all configurations are correct.";
            throw std::runtime_error(msg);
        }
        connected_ = true;

        if (!Monitor) {
            throw std::runtime_error("Monitor() function is undefined.");
        }

        monitor_thread_ = std::thread(Monitor, this);
#ifdef EKI_DEBUG_OUTPUT
        std::cout << "... done. EKI hardware interface started!" << std::endl;
#endif
    }

    void EKI::Disconnect() {
        connected_ = false;
        monitor_thread_.join();
#ifdef EKI_DEBUG_OUTPUT
        std::cout << "Disconnected from robot's EKI server!" << std::endl;
#endif
    }

    void EKI::Read() {
        if (!eki_read_state()) {
            std::string msg = "Failed to Read from robot EKI server within alloted time of "
                              + std::to_string(eki_read_state_timeout_) +
                              " seconds.  Make sure eki_hw_interface is running "
                              "on the robot controller and all configurations are correct.";
            throw std::runtime_error(msg);
        }
    }

    std::vector<double> EKI::GetPosition(CoordinateType coordinate_type) {
        switch (coordinate_type) {
            case kCartesianCoordinateSystem:
                return cartesian_position_;
            case kAxisCoordinateSystem:
                return axis_position_;
            default:
                return cartesian_position_;
        }
    }

    void EKI::Write(const ControlStruct &control_struct) {
        static std::mutex io_mutex;
        {
            std::lock_guard<std::mutex> lk(io_mutex);
        }

        {
            std::scoped_lock lock(this->position_mutex_);

            {
                std::lock_guard<std::mutex> lk(io_mutex);
            }

            // only Write if max will not be exceeded
            eki_write_command(control_struct);
            if (control_struct.control_type_ != kConfig) {
                SetReady(0);
            } else{
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }


        // underflow/overflow checking
        // NOTE: this is commented as it results in a lot of logging output and the use of ROS_*
        //       logging macros breaks incurs (quite) some overhead. Uncomment and rebuild this
        //       if you'd like to use this anyway.
        //if (eki_cmd_buff_len_ >= advance_)
        //  ROS_WARN_STREAM("eki_hw_iface RobotCommand buffer overflow (current size " << eki_cmd_buff_len_
        //                  << " greater than or equal max allowed " << advance_ << ")");
        //else if (eki_cmd_buff_len_ == 0)
        //  ROS_WARN_STREAM("eki_hw_iface RobotCommand buffer empty");
    }

    void EKI::SetReady(int i) {
        static std::mutex io_mutex;
        {
            std::lock_guard<std::mutex> lk(io_mutex);
        }

        {
            std::scoped_lock lock(this->ready_mutex_);

            {
                std::lock_guard<std::mutex> lk(io_mutex);
            }
            this->ready_ = i;
        }
    }

    int EKI::IsReady() {
        static std::mutex io_mutex;
        {
            std::lock_guard<std::mutex> lk(io_mutex);
        }

        {
            std::scoped_lock lock(this->ready_mutex_);

            {
                std::lock_guard<std::mutex> lk(io_mutex);
            }
            return this->ready_;
        }
    }

    int EKI::PrintStatus() {
        std::cout << std::chrono::high_resolution_clock::to_time_t(std::chrono::high_resolution_clock::now()) << std::endl;
        std::cout << "Moving Status: ";
        std::cout << "ready_: " << this->ready_ << "\t";
        std::cout << "moving_status_: " << this->moving_status_ << "\t";
        std::cout << "pro_move_ :" << this->pro_move_ << "\t";
        std::cout << "advance_: " << this->advance_ << "\t";
        std::cout << "act_advance_: " << this->act_advance_ << "\t";
        std::cout << std::endl;

        std::cout << "Cartesian Position: {";
        for (auto it = cartesian_position_.begin(); it != cartesian_position_.end(); ++it) {
            if (std::next(it) != cartesian_position_.end()) {
                std::cout << *it << ", ";
            } else {
                std::cout << *it << "}" << std::endl;
            }
        }

        std::cout << "Axis Position: {";
        for (auto it = axis_position_.begin(); it != axis_position_.end(); ++it) {
            if (std::next(it) != axis_position_.end()) {
                std::cout << *it << ", ";
            } else {
                std::cout << *it << "}" << std::endl;
            }
        }
        std::cout << std::endl;
        return 0;
    }

    int EKI::LogStatus(std::ofstream &fs) {
        fs << std::chrono::high_resolution_clock::to_time_t(std::chrono::high_resolution_clock::now()) << std::endl;
        fs << "Moving Status: ";
        fs << "ready_: " << this->ready_ << "\t";
        fs << "moving_status_: " << this->moving_status_ << "\t";
        fs << "pro_move_ :" << this->pro_move_ << "\t";
        fs << "advance_: " << this->advance_ << "\t";
        fs << "act_advance_: " << this->act_advance_ << "\t";
        fs << std::endl;

        fs << "Cartesian Position: {";
        for (auto it = cartesian_position_.begin(); it != cartesian_position_.end(); ++it) {
            if (std::next(it) != cartesian_position_.end()) {
                fs << *it << ", ";
            } else {
                fs << *it << "}" << std::endl;
            }
        }

        fs << "Axis Position: {";
        for (auto it = axis_position_.begin(); it != axis_position_.end(); ++it) {
            if (std::next(it) != axis_position_.end()) {
                fs << *it << ", ";
            } else {
                fs << *it << "}" << std::endl;
            }
        }
        fs << std::endl;
        return 0;
    }

} // namespace eki
