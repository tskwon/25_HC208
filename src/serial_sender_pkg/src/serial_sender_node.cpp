#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <libserial/SerialPort.h>
#include <string>
#include <thread>
#include <atomic>
#include <vector>

class EnhancedSerialNode : public rclcpp::Node {
public:
    EnhancedSerialNode() : Node("enhanced_serial_node"), running_(true) {
        try {
            // 시리얼 포트 설정 (기존 코드와 동일)
            serialPort_.Open("/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_34331323635351104081-if00");
            serialPort_.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
            serialPort_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            serialPort_.SetParity(LibSerial::Parity::PARITY_NONE);
            serialPort_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
            serialPort_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
            
            serialPort_.SetVTime(1);
            serialPort_.SetVMin(0);

            if (serialPort_.IsOpen()) {
                RCLCPP_INFO(this->get_logger(), "✅ 시리얼 포트 열림!");
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ 시리얼 포트를 열 수 없습니다.");
            }
        } catch (const LibSerial::OpenFailed &e) {
            RCLCPP_FATAL(this->get_logger(), "❌ 예외 발생: %s", e.what());
        } catch (const std::exception &e) {
            RCLCPP_FATAL(this->get_logger(), "❌ 일반 예외 발생: %s", e.what());
        }
        
        // 기존 subscriber: 리프트 층 데이터 수신
        sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/lift/floor", 10,
            std::bind(&EnhancedSerialNode::topic_callback, this, std::placeholders::_1));
        
        // 새로운 publisher: 리프트 완료 신호 발행
        lift_complete_pub_ = this->create_publisher<std_msgs::msg::Int32>("/lift_complete", 10);
        
        // 시리얼 읽기 스레드 시작
        serial_read_thread_ = std::thread(&EnhancedSerialNode::serialReadLoop, this);
        
        RCLCPP_INFO(this->get_logger(), "🤖 향상된 시리얼 노드 시작됨");
        RCLCPP_INFO(this->get_logger(), "📡 구독: /lift/floor");
        RCLCPP_INFO(this->get_logger(), "📤 발행: /lift_complete");
    }
    
    ~EnhancedSerialNode() {
        running_ = false;
        if (serial_read_thread_.joinable()) {
            serial_read_thread_.join();
        }
        if (serialPort_.IsOpen()) {
            serialPort_.Close();
        }
    }

private:
    void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        int data = msg->data;
        
        // 공백이 포함된 명령어 유지
        std::vector<std::string> valid_floors = {
            " " ,         // 0:
            "30",       // 1: 1층 적재
            "32",       // 2: 2층 적재
            "34",       // 3: 3층 적재
            "31",       // 4: 1층 반납
            "33",       // 5: 2층 반납
            "35",       // 6: 3층 반납
            "40",       // 7: 1층 고객 상자 적재 
            "43",       // 8: 2층 고객 상자 반납
            "u 2550",   // 9: 리프트 위로 (공백 유지)
            "d 2550",    // 10: 리프트 아래로 (공백 유지)
            "7"
        };

        // 범위 체크 추가
        if (data < 0 || data > static_cast<int>(valid_floors.size())) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 잘못된 층 번호: %d", data);
            return;
        }

        const std::string& floor_command = valid_floors[data];
        
        // 빈 명령은 전송하지 않음
        if (floor_command.empty()) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 빈 명령은 전송하지 않음: %d", data);
            return;
        }

        if (serialPort_.IsOpen()) {
            try {
                // 방법 1: 한 번에 전체 명령 전송
                std::string full_command = floor_command + "\n";
                serialPort_.Write(full_command);
                
                // 방법 2: 또는 바이트 단위로 전송 (더 안전)
                // for (char c : floor_command) {
                //     serialPort_.WriteByte(c);
                // }
                // serialPort_.WriteByte('\n');
                
                RCLCPP_INFO(this->get_logger(), "🏢 리프트에 층 전송: '%s'", floor_command.c_str());
                
                // 특별한 경우 (u 2400, d 2400)에 대한 자동 완료 신호 발행
                if (floor_command == "u 2550" || floor_command == "d 2550") {
                    // 2초 후 자동으로 완료 신호 발행 (실제 하드웨어 응답 시뮬레이션)
                    std::thread([this, floor_command]() {
                        std::this_thread::sleep_for(std::chrono::seconds(2));
                        auto msg = std_msgs::msg::Int32();
                        msg.data = 1;
                        // lift_complete_pub_->publish(msg);
                        RCLCPP_INFO(this->get_logger(), "✅ 리프트 완료 신호 발행: %s 완료", floor_command.c_str());
                    }).detach();
                }
                
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "❌ 시리얼 쓰기 오류: %s", e.what());
                
                // 에러 발생 시에도 완료 신호 발행 (시스템이 멈추지 않도록)
                if (floor_command == "u 2400" || floor_command == "d 2400") {
                    std::thread([this]() {
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        auto msg = std_msgs::msg::Int32();
                        msg.data = 1;
                        lift_complete_pub_->publish(msg);
                        RCLCPP_INFO(this->get_logger(), "✅ 에러 후 리프트 완료 신호 발행");
                    }).detach();
                }
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ 시리얼 포트가 닫혀있음");
            
            // 포트가 닫혀있어도 완료 신호 발행 (시스템이 멈추지 않도록)
            if (floor_command == "u 2400" || floor_command == "d 2400") {
                std::thread([this]() {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    auto msg = std_msgs::msg::Int32();
                    msg.data = 1;
                    lift_complete_pub_->publish(msg);
                    RCLCPP_INFO(this->get_logger(), "✅ 포트 닫힘 상태에서 리프트 완료 신호 발행");
                }).detach();
            }
        }
    }

    void serialReadLoop() {
        std::string buffer;
        
        while (running_ && rclcpp::ok()) {
            if (serialPort_.IsOpen()) {
                try {
                    char byte;
                    try {
                        // libserial의 ReadByte는 void를 반환하므로 예외 처리로 타임아웃 감지
                        serialPort_.ReadByte(byte); // 100ms 타임아웃
                        
                        if (byte == '\n' || byte == '\r') {
                            if (!buffer.empty()) {
                                processReceivedData(buffer);
                                buffer.clear();
                            }
                        } else {
                            buffer += byte;
                        }
                    } catch (const LibSerial::ReadTimeout &e) {
                        // 타임아웃은 정상적인 동작이므로 무시
                        continue;
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "❌ 시리얼 읽기 오류: %s", e.what());
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
    
    void processReceivedData(const std::string& data) {
        RCLCPP_INFO(this->get_logger(), "📨 시리얼에서 수신: '%s'", data.c_str());
        
        try {
            int received_value = std::stoi(data);
            
            // 리프트에서 "1"을 받으면 완료 신호로 간주
            if (received_value == 1) {
                auto msg = std_msgs::msg::Int32();
                msg.data = 1;
                lift_complete_pub_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "✅ 리프트 완료 신호 발행: 1");
            } else {
                RCLCPP_INFO(this->get_logger(), "📊 리프트 응답: %d", received_value);
            }
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 수신 데이터 파싱 오류: %s", e.what());
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lift_complete_pub_;
    LibSerial::SerialPort serialPort_;
    std::thread serial_read_thread_;
    std::atomic<bool> running_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EnhancedSerialNode>());
    rclcpp::shutdown();
    return 0;
}