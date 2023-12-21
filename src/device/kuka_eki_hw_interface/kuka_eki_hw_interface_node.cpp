// #include "kuka_eki_hw_interface/kuka_eki_hw_interface.h"
// #include <thread>
// // #include <fstream>

// int main(int argc, char const *argv[]) {

//     auto eki = new device::EKI;
//     eki->Init("10.1.0.101", "54600");
//     eki->ready_ = 1;
//     eki->Connect();

//     // set advance
//     {
//         device::ControlStruct cs;
//         cs.control_type_ = device::kConfig;
//         cs.kv_int_.emplace_back("Advance", 0);

//         eki->Write(cs);
//     }

//     {
//         device::ControlStruct cs;
//         cs.control_type_ = device::kConfig;
//         cs.kv_double_.emplace_back("OvPro", 15);
//         eki->Write(cs);
//     }

//     std::vector<std::vector<double>> points;
//     points.push_back({1500, 200, 1600, 0, 90, 0});

//     {

//         device::ControlStruct cs;
//         cs.control_type_ = device::kConfig;
//         cs.kv_double_.emplace_back("OvPro", 10);
//         eki->Write(cs);
//     }

//     for (auto &&i: points) {
//         while (!eki->IsReady()) {
//             std::this_thread::sleep_for(std::chrono::milliseconds(10));
//         }
//         device::ControlStruct control_struct;
//         control_struct.control_type_ = device::kMoving;
//         control_struct.moving_type_ = device::kPTP;
//         control_struct.coordinate_type_ = device::kCartesianCoordinateSystem;
//         control_struct.target_cartesian_position_ = i;

//         eki->Write(control_struct);
//         std::cerr << "Set Target: {";
//         for (auto it = control_struct.target_cartesian_position_.begin();
//              it != control_struct.target_cartesian_position_.end(); ++it) {
//             if (std::next(it) != control_struct.target_cartesian_position_.end()) {
//                 std::cerr << *it << ", ";
//             } else {
//                 std::cerr << *it << "}" << std::endl;
//             }
//         }

//         while (!eki->IsReady()) {
//             std::this_thread::sleep_for(std::chrono::milliseconds(10));
//         }
//     }

//     std::vector<std::vector<double>> rel;
//     rel.push_back({0, 0, 0, -90, 0, 0});
//     rel.push_back({0, 0, 0, 0, -15, 0});
//     {
//         for(auto &&i : rel) {
//             while (!eki->IsReady()) {
//                 std::this_thread::sleep_for(std::chrono::milliseconds(10));
//             }
//             eki::ControlStruct control_struct;
//             control_struct.control_type_ = eki::kMoving;
//             control_struct.moving_type_ = eki::kPTPRelTool;
//             control_struct.coordinate_type_ = eki::kCartesianCoordinateSystem;
//             control_struct.target_cartesian_position_ = i;

//             eki->Write(control_struct);
//             std::cerr << "Set Relative Target: {";
//             for (auto it = control_struct.target_cartesian_position_.begin();
//                  it != control_struct.target_cartesian_position_.end(); ++it) {
//                 if (std::next(it) != control_struct.target_cartesian_position_.end()) {
//                     std::cerr << *it << ", ";
//                 } else {
//                     std::cerr << *it << "}" << std::endl;
//                 }
//             }
//             while (!eki->IsReady()) {
//                 std::this_thread::sleep_for(std::chrono::milliseconds(10));
//             }
//         }
//     }

//     {
//         eki::ControlStruct cs;
//         cs.control_type_ = eki::kConfig;
//         cs.kv_double_.emplace_back("OvPro", 5);
//         eki->Write(cs);
//     }

// //    {
// //        eki::ControlStruct cs;
// //        cs.control_type_ = eki::kConfig;
// //        cs.kv_int_.emplace_back("MainAxis", 1);
// //        eki->Write(cs);
// //    }

// //    {
// //        eki::ControlStruct cs;
// //        cs.control_type_ = eki::kConfig;
// //        cs.kv_int_.emplace_back("MainAxis", 0);
// //        eki->Write(cs);
// //    }

//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     eki->Disconnect();

//     return 0;
// }
