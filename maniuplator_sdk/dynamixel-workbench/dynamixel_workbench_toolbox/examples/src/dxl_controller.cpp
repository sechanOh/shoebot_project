#include <DynamixelWorkbench.h>
#include <cstdlib>
#include <dynamixel_item.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

bool run_none_camera_action = false;
bool readonly = true;
// bool readonly = false;

bool Initialize_DXL(const char *port_name, int baud_rate);
float Read_Present_Position(uint8_t dxl_id);
bool MinMax_Position_Setting();
void Set_PID_Param(int32_t PGain, int32_t IGain, int32_t DGain);
void Print_DXL_Setting();
void Print_DXL_Info(int i);
void Run_None_Camera_Action();

DynamixelWorkbench dxl_wb;
uint8_t *scanned_id = new uint8_t[0];
uint8_t dxl_cnt = 0;

using namespace std::chrono_literals;
using std::placeholders::_1;

class TeleopCommunication : public rclcpp::Node{
  public:
    TeleopCommunication()
    : Node("teleop_communication")
    {
      publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("present_joint_states", 10);
      subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "goal_joint_states", 10, std::bind(&TeleopCommunication::topic_callback, this, _1));

      timer_ = this->create_wall_timer(
      100ms, std::bind(&TeleopCommunication::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::Float64MultiArray();
      std::vector<double> tmp;
      for (int i=0; i<dxl_cnt; i++){
        tmp.push_back(static_cast<double>(Read_Present_Position(scanned_id[i])));
      }

      message.data = tmp;
      // RCLCPP_INFO(get_logger(), "%f, %f, %f", message.data[0], message.data[1], message.data[2]);
      publisher_->publish(message);
    }

    void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      for (int i=0; i<dxl_cnt; i++){
        dxl_wb.goalPosition(scanned_id[i], static_cast<float>(msg->data[i]));
      }
      // RCLCPP_INFO(get_logger(), "I heard: '%f, %f, %f'", msg->data[0], msg->data[1], msg->data[2]);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  if (argc == 2) {
    if (std::string(argv[1]).find("control") != std::string::npos) { // arg[1] includes control
      printf("Manipulator Control Mode!\n");
      readonly = false;
    }
  }

  // Liux defalut USB port
  const char *port_name = "/dev/ttyUSB0";
  // Open_Manipulator defalut baud_rate
  int baud_rate = 1000000;

  // Get Dynamixel id array and initialize DXL
  Initialize_DXL(port_name, baud_rate);

  // Check DXL setting
  Print_DXL_Setting();

  // if (run_none_camera_action){
  //   // Run manipulator to move shoes (hard coding)
  //   Run_None_Camera_Action(scanned_id);
  // }
  // if (readonly){
  //   printf("%d\n",dxl_cnt);
  //   while(true){
  //       sleep(1);
  //       for (int i=0;i<dxl_cnt;i++){
  //         printf("%d : %f\n", i, Read_Present_Position(scanned_id[i]));
  //       }
  //       printf("\n\n");
  //   }
  // }


  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopCommunication>());
  rclcpp::shutdown();



  // float tmp_list[5] = {-3.0, -0.125, 0.570, 1.075, -1.05};
  // for (int i=0;i<5;i++){
  //   tmp_list[i] = Read_Present_Position(scanned_id[i]);
  // }
  // printf("\n\n");

  // bool flag = 1;
  // while(true){
  //   sleep(1);
  //   if (flag == 1){
  //     tmp_list[1] += 0.1;
  //     flag = 0;
  //   }
  //   else{
  //     tmp_list[1] -= 0.1;
  //     flag = 1;
  //   }
  //   for (int j=0; j<5; j++){
  //     dxl_wb.goalPosition(scanned_id[j], tmp_list[j]);
  //   }
  // }
  
  // for (int i=0;i<5;i++){
  //     printf("\n\n");
  //     printf("rpm:                           %f\n", dxl_wb.getModelInfo(scanned_id[i])[0].rpm);
  //     printf("value_of_min_radian_position:  %ld\n", dxl_wb.getModelInfo(scanned_id[i])[0].value_of_min_radian_position);
  //     printf("value_of_zero_radian_position: %ld\n", dxl_wb.getModelInfo(scanned_id[i])[0].value_of_zero_radian_position);
  //     printf("value_of_max_radian_position:  %ld\n", dxl_wb.getModelInfo(scanned_id[i])[0].value_of_max_radian_position);
  //     printf("min_radian:                    %f\n", dxl_wb.getModelInfo(scanned_id[i])[0].min_radian);
  //     printf("max_radian:                    %f\n", dxl_wb.getModelInfo(scanned_id[i])[0].max_radian);
  // }

  // for (int i=0;i<1;i++){
  //     printf("%d\n", dxl_wb.getTheNumberOfControlItem(scanned_id[i]));
  //     const ControlItem *item = dxl_wb.getControlTable(scanned_id[i]);
  //     for (int j=0; j<dxl_wb.getTheNumberOfControlItem(scanned_id[i]); j++){
  //         printf("id:%d==========================\n", scanned_id[i]);
  //         printf("item_name:        %s\n", item[j].item_name);
  //         printf("address:          %d\n", item[j].address);
  //         printf("item_name_length: %d\n", item[j].item_name_length);
  //         printf("data_length:      %d\n", item[j].data_length);
  //     }
  // }

  delete[] scanned_id;
  return 0;
}

bool Initialize_DXL(const char *port_name, int baud_rate) {
  const char *log;
  bool result = false;
  uint8_t range = 100;

  // Initialize
  result = dxl_wb.init(port_name, baud_rate, &log);
  if (!result) {
    printf("%s\n", log);
    printf("Failed to init");
    return result;
  }

  // Find models
  result = dxl_wb.scan(scanned_id, &dxl_cnt, range, &log);
  if (!result) {
    printf("%s\n", log);
    printf("Failed to scan");
    return result;
  }

  // Initial setting
  // MinMax_Position_Setting(scanned_id);

  // Print model info
  for (int i = 0; i < dxl_cnt; i++) {
    // ping
    uint16_t model_number = 0;
    result = dxl_wb.ping(scanned_id[i], &model_number, &log);
    if (!result) {
      printf("%s\n", log);
      printf("Failed to ping");
      return result;
    }

    if (!readonly){
      // Set joint mode
      result = dxl_wb.jointMode(scanned_id[i], 0, 0, &log);
      if (!result) {
        printf("%s\n", log);
        printf("Failed to set jointMode");
        return result;
      }
    }
    printf("id : %d, model name : %s, model_number : %d\n", scanned_id[i],
           dxl_wb.getModelName(scanned_id[i]), model_number);
  }

  // Set PID
  Set_PID_Param(400, 800, 1200);

  return result;
}

float Read_Present_Position(uint8_t dxl_id) {
  const char *log;
  bool result = false;
  int32_t get_data = 0;

  result = dxl_wb.itemRead(dxl_id, "Present_Position", &get_data, &log);
  if (!result) {
    printf("%s\n", log);
    printf("Failed to get present position\n");
    return -1;
  }
  
  float rad = dxl_wb.convertValue2Radian(dxl_id, get_data);
  // printf("%.3f, ", rad);

  return rad;
}

bool MinMax_Position_Setting(){
  bool result = false;
  result = dxl_wb.writeRegister(scanned_id[0],
                                "Min_Position_Limit", 0);
  result = dxl_wb.writeRegister(scanned_id[0],
                                "Max_Position_Limit", 4095);
  result = dxl_wb.writeRegister(scanned_id[1],
                                "Min_Position_Limit", 0);
  result = dxl_wb.writeRegister(scanned_id[1],
                                "Max_Position_Limit", 4095);
  result = dxl_wb.writeRegister(scanned_id[2],
                                "Min_Position_Limit", 0);
  result = dxl_wb.writeRegister(scanned_id[2],
                                "Max_Position_Limit", 4095);
  result = dxl_wb.writeRegister(scanned_id[3],
                                "Min_Position_Limit", 0);
  result = dxl_wb.writeRegister(scanned_id[3],
                                "Max_Position_Limit", 4095);
  result = dxl_wb.writeRegister(scanned_id[4],
                                "Min_Position_Limit", 0);
  result = dxl_wb.writeRegister(scanned_id[4],
                                "Max_Position_Limit", 4095);
  return result;
}

void Set_PID_Param(int32_t PGain, int32_t IGain, int32_t DGain){
  bool result = true;
  int32_t F2Gain = 0;
  int32_t F1Gain = 0;
  for (int i=0; i<dxl_cnt-1; i++){
    dxl_wb.writeRegister(scanned_id[i],
                                "Position_D_Gain", DGain);
    dxl_wb.writeRegister(scanned_id[i],
                                "Position_I_Gain", IGain);
    dxl_wb.writeRegister(scanned_id[i],
                                "Position_P_Gain", PGain);
    dxl_wb.writeRegister(scanned_id[i],
                                "Feedforward_2nd_Gain", F2Gain);
    dxl_wb.writeRegister(scanned_id[i],
                                "Feedforward_1nd_Gain", F1Gain);
  }

  return;
}

void Print_DXL_Setting(){
  for (int i=0;i<dxl_cnt;i++){
      int32_t data = 0;
      printf("\nid:%d=======================\n", scanned_id[i]);
      dxl_wb.readRegister(scanned_id[i], "Position_D_Gain", &data);
      printf("Position_D_Gain: %d\n", data);
      dxl_wb.readRegister(scanned_id[i], "Position_I_Gain", &data);
      printf("Position_I_Gain: %d\n", data);
      dxl_wb.readRegister(scanned_id[i], "Position_P_Gain", &data);
      printf("Position_P_Gain: %d\n", data);
      dxl_wb.readRegister(scanned_id[i], "Feedforward_2nd_Gain", &data);
      printf("Feedforward_2nd_Gain: %d\n", data);
      dxl_wb.readRegister(scanned_id[i], "Feedforward_1nd_Gain", &data);
      printf("Feedforward_1nd_Gain: %d\n", data);
      dxl_wb.readRegister(scanned_id[i], "Max_Position_Limit", &data);
      printf("Max_Position_Limit: %d\n", data);
      dxl_wb.readRegister(scanned_id[i], "Min_Position_Limit", &data);
      printf("Min_Position_Limit: %d\n", data);
  }
}

void Print_DXL_Info(int i){
  int32_t data = 0;
  printf("\nid:%d=================================\n", scanned_id[i]);
  dxl_wb.readRegister(scanned_id[i], "Moving", &data);
  printf("Moving: %d\n", data);
  dxl_wb.readRegister(scanned_id[i], "Moving_Status", &data);
  printf("Moving_Status: %d\n", data);
  dxl_wb.readRegister(scanned_id[i], "Hardware_Error_Status", &data);
  printf("Hardware_Error_Status: %d\n", data);
  dxl_wb.readRegister(scanned_id[i], "Goal_PWM", &data);
  printf("Goal_PWM: %d\n", data);
  dxl_wb.readRegister(scanned_id[i], "Goal_Current", &data);
  printf("Goal_Current: %d\n", data);
  dxl_wb.readRegister(scanned_id[i], "Goal_Velocity", &data);
  printf("Goal_Velocity: %d\n", data);
  dxl_wb.readRegister(scanned_id[i], "Goal_Position", &data);
  printf("Goal_Position: %d\n", data);
  dxl_wb.readRegister(scanned_id[i], "Present_PWM", &data);
  printf("Present_PWM: %d\n", data);
  dxl_wb.readRegister(scanned_id[i], "Present_Current", &data);
  printf("Present_Current: %d\n", data);
  dxl_wb.readRegister(scanned_id[i], "Present_Velocity", &data);
  printf("Present_Velocity: %d\n", data);
  dxl_wb.readRegister(scanned_id[i], "Present_Position", &data);
  printf("Present_Position: %d\n", data);
  dxl_wb.readRegister(scanned_id[i], "Present_Input_Voltage", &data);
  printf("Present_Input_Voltage: %d\n", data);
  dxl_wb.readRegister(scanned_id[i], "Present_Temperature", &data);
  printf("Present_Temperature: %d\n", data);
}

void Run_None_Camera_Action(){
  const uint array_size = 15;
  float angle_list[array_size][5] = {{-3.0, -0.125, 0.570, 1.075, -1.05}, // Init pos
                                     {-2.000, -0.282, 0.356, 1.299, -1.05}, // Move-1
                                     {-1.000, -0.282, 0.356, 1.299, -1.05},  // Move-2
                                     {0.000, -0.235, 0.173, 1.844, -1.05},  // Move-3
                                     {0.000, 0.179, 0.942, 0.440, -1.05},  // Go down
                                     {0.000, 0.179, 0.942, 0.440, 0.6},   // Grab on
                                     {0.000, -1.287, 0.606, 1.256, 0.6},   // Go up
                                     {-0.811, -1.287, 0.606, 1.256, 0.6},  // Move left-1
                                     {-1.631, 0.017, -0.666, 1.583, 0.6},  // Move left-2
                                     {-1.631, 0.462, -0.943, 1.205, 0.6},  // Move forward
                                     {-1.631, 0.462, -0.943, 1.205, -1.05}, // Grab off
                                     {-1.644, -0.091, -0.411, 0.821, -1.05}, // Move backward-1
                                     {-1.644, -0.724, 0.239, 1.950, -1.05}, // Move backward-2
                                     {-2.603, -0.282, 0.356, 1.299, -1.05}, // Move-1
                                     {-3.0, -0.125, 0.570, 1.075, -1.05}}; // Init pos

  for (int i=0; i<array_size; i++){
    if (i==0) Set_PID_Param(50, 0, 5);
    if (i==5) Set_PID_Param(250, 10, 10);
    if (i==13) Set_PID_Param(50, 0, 5);
    for (int j=0; j<5; j++){
      dxl_wb.goalPosition(scanned_id[j], angle_list[i][j]);
    }
    sleep(1);
    while(true){
      int32_t data = 0;
      int32_t new_data = 0;
      for (int j=0; j<5; j++){
        printf("\n");
        Print_DXL_Info(j);
        dxl_wb.readRegister(scanned_id[j], "Moving", &data);
        new_data = new_data | data;
      }
      if (new_data == 0) break;
    }
  }
  return;
}

