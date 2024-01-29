#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>

#define SERIAL_PORT "/dev/ttyUSB1"
#define SERIAL_BAUDRATE 460800
#define SERIAL_BUF_SIZE 1024

#define DATA_HEADER_FIRST_BYTE 0x59
#define DATA_HEADER_SECOND_BYTE 0x53

#define DATA_LENGTH 99

#define ID_ACCELERATION "ACCELERATION"
#define ID_ANGULAR_VELOCITY "ANGULAR_VELOCITY"
#define ID_EULER_ANGLE "EULER_ANGLE"
#define ID_QUATERNION "QUATERNION"
#define ID_LOCATION "LOCATION"
#define ID_VELOCITY "VELOCITY"
#define ID_STATUS "STATUS"

#define DATA_ID_ACCELERATION 0x10
#define DATA_ID_ANGULAR_VELOCITY 0x20
#define DATA_ID_EULER_ANGLE 0x40
#define DATA_ID_QUATERNION 0x41
#define DATA_ID_LOCATION 0x68
#define DATA_ID_VELOCITY 0x70
#define DATA_ID_STATUS 0x80

#define DATA_LENGTH_ACCELERATION 12
#define DATA_LENGTH_ANGULAR_VELOCITY 12
#define DATA_LENGTH_EULER_ANGLE 12
#define DATA_LENGTH_QUATERNION 16
#define DATA_LENGTH_LOCATION 20
#define DATA_LENGTH_VELOCITY 12
#define DATA_LENGTH_STATUS 1

#define SCALE_ACCELERATION 1e-6
#define SCALE_ANGULAR_VELOCITY 1e-6
#define SCALE_EULER_ANGLE 1e-6
#define SCALE_QUATERNION 1e-6
#define SCALE_LOCATION 1e-10
#define SCALE_VELOCITY 1e-3


namespace Yesense_SDK {
    struct Data_Packet_ID {
        const std::string id; 
        const uint8_t data_id;
        const uint8_t data_length;
        const double data_scale; 
    };

    Data_Packet_ID DATA_PACKET_ID_ACCELERATION {ID_ACCELERATION, DATA_ID_ACCELERATION, DATA_LENGTH_ACCELERATION, SCALE_ACCELERATION}; // 14
    Data_Packet_ID DATA_PACKET_ID_ANGULAR_VELOCITY {ID_ANGULAR_VELOCITY, DATA_ID_ANGULAR_VELOCITY, DATA_LENGTH_ANGULAR_VELOCITY, SCALE_ANGULAR_VELOCITY}; // 28
    Data_Packet_ID DATA_PACKET_ID_EULER_ANGLE {ID_EULER_ANGLE, DATA_ID_EULER_ANGLE, DATA_LENGTH_EULER_ANGLE, SCALE_EULER_ANGLE}; // 42
    Data_Packet_ID DATA_PACKET_ID_QUATERNION {ID_QUATERNION, DATA_ID_QUATERNION, DATA_LENGTH_QUATERNION, SCALE_QUATERNION}; // 60
    Data_Packet_ID DATA_PACKET_ID_LOCATION {ID_LOCATION, DATA_ID_LOCATION, DATA_LENGTH_LOCATION, SCALE_LOCATION}; // 82
    Data_Packet_ID DATA_PACKET_ID_VELOCITY {ID_VELOCITY, DATA_ID_VELOCITY, DATA_LENGTH_VELOCITY, SCALE_VELOCITY}; // 96
    Data_Packet_ID DATA_PACKET_ID_STATUS {ID_STATUS, DATA_ID_STATUS, DATA_LENGTH_STATUS, 1}; // 99

    struct Data_Packet {
        Data_Packet_ID data_packet_id;
        std::vector<uint8_t> data_raw;
        std::vector<double> data_processed;

        void clear() {
            data_raw.clear();
            data_processed.clear();
        }

        int update(std::vector<uint8_t> datas, int index){
            if (datas[index] == data_packet_id.data_id && datas[index+1] == data_packet_id.data_length) {
                for (int i = 0; i < data_packet_id.data_length; i++) {
                    data_raw.push_back(datas[index+2+i]);
                }
                get_data();
                return index + 2 + data_packet_id.data_length;
            }
            return index;
        }
        
        double get_byte_data(int start, int bytes){
            int32_t result = 0;
            for(int i = 0;i < bytes;i++){
                result |= data_raw[start+i] << (8*i);
            }
            return (double)result * data_packet_id.data_scale;
        }

        uint8_t get_half_byte_data(int start){
            return data_raw[0] >> (4*start) & 0x0f;
        }
        
        std::vector<double> get_data() {
            switch(data_packet_id.data_id) {
                case DATA_ID_ACCELERATION:
                case DATA_ID_ANGULAR_VELOCITY:
                case DATA_ID_EULER_ANGLE:
                case DATA_ID_VELOCITY:
                    data_processed = std::vector<double> {
                        get_byte_data(0, 4),
                        get_byte_data(4, 4),
                        get_byte_data(8, 4)
                    };break;
                case DATA_ID_QUATERNION:
                    data_processed = std::vector<double> {
                        get_byte_data(0, 4),
                        get_byte_data(4, 4),
                        get_byte_data(8, 4),
                        get_byte_data(12, 4)
                    };break;
                case DATA_ID_LOCATION:
                    data_processed = std::vector<double> {
                        get_byte_data(0, 8),
                        get_byte_data(8, 8),
                        get_byte_data(16, 4) * 1e7
                    };break;
                case DATA_ID_STATUS:
                    data_processed = std::vector<double> {
                        (double)get_half_byte_data(0),
                        (double)get_half_byte_data(1),
                    };break;
                default:
                    ROS_WARN_STREAM("Unknown data id: " << data_packet_id.data_id);
                    data_processed = std::vector<double> {};
            }
            return data_processed;
        }

        void info() {
            ROS_WARN_STREAM("Data Packet ID: " << data_packet_id.id << " " << data_processed.size());
            for(auto &data : data_processed){
                ROS_INFO_STREAM(data_packet_id.id <<": " << data);
            }
        }
    };

    class Serial_Port_Data {
    private:
        std::vector<uint8_t> data_raw = std::vector<uint8_t>(3 + DATA_LENGTH);
        Data_Packet data_packets[7] = {
            Data_Packet {DATA_PACKET_ID_ACCELERATION, std::vector<uint8_t>(DATA_PACKET_ID_ACCELERATION.data_length)},
            Data_Packet {DATA_PACKET_ID_ANGULAR_VELOCITY, std::vector<uint8_t>(DATA_PACKET_ID_ANGULAR_VELOCITY.data_length)},
            Data_Packet {DATA_PACKET_ID_EULER_ANGLE, std::vector<uint8_t>(DATA_PACKET_ID_EULER_ANGLE.data_length)},
            Data_Packet {DATA_PACKET_ID_QUATERNION, std::vector<uint8_t>(DATA_PACKET_ID_QUATERNION.data_length)},
            Data_Packet {DATA_PACKET_ID_LOCATION, std::vector<uint8_t>(DATA_PACKET_ID_LOCATION.data_length)},
            Data_Packet {DATA_PACKET_ID_VELOCITY, std::vector<uint8_t>(DATA_PACKET_ID_VELOCITY.data_length)},
            Data_Packet {DATA_PACKET_ID_STATUS, std::vector<uint8_t>(DATA_PACKET_ID_STATUS.data_length)},
        };
        sensor_msgs::Imu data_imu;
    public:
        void reset() {
            data_raw.clear();   
            for(auto &data_packet : data_packets) data_packet.clear();
        }
        void update(uint8_t data) {
            data_raw.push_back(data);
        }
        bool update_finish() {
            return data_raw.size() == 3 + DATA_LENGTH;
        }
        void update_data_packet(ros::Publisher& pub) {
            int i = 3;
            for(auto &data_packet : data_packets) {
                i = data_packet.update(data_raw, i);
                data_packet.info();
            }
            publish(pub);
        }
        bool check_sum_valid(uint8_t& C1, uint8_t& C2) {
            uint8_t _C1 = 0, _C2 = 0;
            for(auto& data:data_raw) {
                _C1 += data;
                _C2 += _C1;
            }
            return _C1 == C1 && _C2 == C2;
        }
        void publish(ros::Publisher& pub) {
            data_imu.header.stamp = ros::Time::now();
            data_imu.header.frame_id = "imu_link";
            data_imu.orientation.w = data_packets[3].data_processed[0];
            data_imu.orientation.x = data_packets[3].data_processed[1];
            data_imu.orientation.y = data_packets[3].data_processed[2];
            data_imu.orientation.z = data_packets[3].data_processed[3];
            data_imu.angular_velocity.x = data_packets[1].data_processed[0]/180.0*M_PI;
            data_imu.angular_velocity.y = data_packets[1].data_processed[1]/180.0*M_PI;
            data_imu.angular_velocity.z = data_packets[1].data_processed[2]/180.0*M_PI;
            data_imu.linear_acceleration.x = data_packets[2].data_processed[0];
            data_imu.linear_acceleration.y = data_packets[2].data_processed[1];
            data_imu.linear_acceleration.z = data_packets[2].data_processed[2];
            pub.publish(data_imu);
        }
    }serial_port_data;

    struct Serial_Attribute {
        const std::string port = SERIAL_PORT;
        const int baud_rate = SERIAL_BAUDRATE;
        const int buf_size = SERIAL_BUF_SIZE;
    } serial_attribute;

    enum PARSE_STATE {
        PARSE_STATE_WAITING_FOR_HEADER,
        PARSE_STATE_GOT_HEADER_FIRST_BYTE,
        PARSE_STATE_GOT_HEADER_SECOND_BYTE,
        PARSE_STATE_GOT_RAW_DATA,
        PARSE_STATE_GOT_CHECKSUM,
    } parse_state;

    class Serial_Port { 
    private:
        serial::Serial ser;
        std::string read;
        std::vector<uint8_t> check = std::vector<uint8_t>(2);
    public:
        void init() {
            ROS_INFO("port:%s, rate:%d", serial_attribute.port.c_str(), serial_attribute.baud_rate);
            ser.setPort(serial_attribute.port);
            ser.setBaudrate(serial_attribute.baud_rate);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();
        }
        void open() {
            try {
                init();
            }
            catch (serial::IOException &e) {
                ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
                ros::Duration(5).sleep();
            }
            if (ser.isOpen()) {
                ROS_INFO_STREAM("Serial port " << ser.getPort() << " initialized and opened.");
            }
        }
        void run(ros::Publisher& pub) {
            try {
                if(ser.isOpen()) {
                    if (ser.available()) {
                        read = ser.read(ser.available());
                    }
                    for(unsigned int i = 0; i < read.size(); i++) {
                        switch(parse_state) {
                            case PARSE_STATE_WAITING_FOR_HEADER:
                                if (read.at(i) == char(DATA_HEADER_FIRST_BYTE)) { 
                                    parse_state = PARSE_STATE_GOT_HEADER_FIRST_BYTE;
                                } break;
                            case PARSE_STATE_GOT_HEADER_FIRST_BYTE:
                                if (read.at(i) == char(DATA_HEADER_SECOND_BYTE)) {
                                    serial_port_data.reset();
                                    parse_state = PARSE_STATE_GOT_HEADER_SECOND_BYTE;
                                } else {
                                    parse_state = PARSE_STATE_WAITING_FOR_HEADER;
                                } break;
                            case PARSE_STATE_GOT_HEADER_SECOND_BYTE:
                                serial_port_data.update((uint8_t)read.at(i));
                                if (serial_port_data.update_finish()) {
                                    parse_state = PARSE_STATE_GOT_RAW_DATA;
                                } break;
                            case PARSE_STATE_GOT_RAW_DATA:
                                if(check.size() < 2) {
                                    check.push_back((uint8_t)read.at(i));
                                }
                                if(check.size() == 2) {
                                    if (serial_port_data.check_sum_valid(check[0], check[1])) {
                                        serial_port_data.update_data_packet(pub);
                                        parse_state = PARSE_STATE_WAITING_FOR_HEADER;
                                    } else {
                                        parse_state = PARSE_STATE_WAITING_FOR_HEADER;
                                    }
                                    check.clear();
                                } break;
                            default:
                                ROS_ERROR_STREAM("Got error!" << parse_state);
                                parse_state = PARSE_STATE_WAITING_FOR_HEADER;
                                break;
                        }
                    }
                }
                else {
                    open();
                }
            }
            catch (serial::IOException &e)
            {
                ROS_ERROR_STREAM("Error reading from the serial port " << ser.getPort() << ". Closing connection.");
                ser.close();
            }
        }
    } serial_port;
};