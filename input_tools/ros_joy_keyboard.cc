#include <ros/init.h>
#include <ros/message_traits.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <sensor_msgs/Joy.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <vector>

#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_C 0x63
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_O 0x6f
#define KEYCODE_P 0x70
#define KEYCODE_H 0x68
#define KEYCODE_L 0x6c
#define KEYCODE_T 0x74
#define KEYCODE_G 0x67
#define KEYCODE_N 0x6e
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
#define KEYCODE_Q_CAP 0x51
#define KEYCODE_E_CAP 0x45
#define KEYCODE_O_CAP 0x4f
#define KEYCODE_P_CAP 0x50
#define KEYCODE_R_CAP 0x52
#define KEYCODE_L_CAP 0x4c
#define KEYCODE_T_CAP 0x54

enum Button {
  kButtonA = 0,
  kButtonB = 1,
  kButtonX = 2,
  kButtonY = 3,
  kButtonLB = 4,
  kButtonRB = 5,
  kButtonStart = 6,
  kButtonClear = 7,
  kButtonLeft = 12,
  kButtonRight = 13,
  kButtonUp = 14,
  kButtonDown = 15,
};

enum Axes {
  kAxesLeftH = 0,
  kAxesLeftV = 1,
  kAxesRightH = 3,
  kAxesRightV = 4,
  kAxesLT = 2,
  kAxesRT = 5,
};

class JoyStickKeyboard {
 public:
  explicit JoyStickKeyboard(const std::string& robot_name) {
    // // Use robot_name as topic namespace
    // joy_pub_ = node_.advertise<sensor_msgs::Joy>("/" + robot_name + "/joy", 1);
    joy_pub_ = node_.advertise<sensor_msgs::Joy>("/joy", 1);
    joy_data_.axes.resize(6);
    joy_data_.buttons.resize(16);

    tcgetattr(STDIN_FILENO, &old_);
    struct termios raw;
    memcpy(&raw, &old_, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Times out after 100ms.
    raw.c_cc[VTIME] = 1;
    raw.c_cc[VMIN] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);

    ResetAxesButtons();
    delta_sig_ = 0.01;
  }

  void ResetAxesButtons() {
    for (int axes = 0; axes < 6; ++axes) {
      joy_data_.axes[axes] = 0.0;
    }
    for (int button = 0; button < 16; ++button) {
      joy_data_.buttons[button] = 0;
    }
  }

  ~JoyStickKeyboard() { tcsetattr(STDIN_FILENO, TCSANOW, &old_); }

  void LoopReadKeyboard() {
    char c;
    bool dirty = false;

    while (ros::ok()) {
      // Gets the next event from the keyboard
      int rv;
      rv = read(STDIN_FILENO, &c, 1);
      if (rv <= 0) {
        if (errno == EAGAIN) continue;
        perror("read");
        return;
      }

      // If need re-initializes the joy_data?
      dirty = false;

      switch (c) {
        case KEYCODE_W:
          joy_data_.axes[kAxesLeftV] += delta_sig_;
          joy_data_.axes[kAxesLeftV] = joy_data_.axes[kAxesLeftV] < 1 ? joy_data_.axes[kAxesLeftV] : 1;
          dirty = true;
          break;

        case KEYCODE_S:
          joy_data_.axes[kAxesLeftV] -= delta_sig_;
          joy_data_.axes[kAxesLeftV] = joy_data_.axes[kAxesLeftV] > -1 ? joy_data_.axes[kAxesLeftV] : -1;
          dirty = true;
          break;

        case KEYCODE_A:
          joy_data_.axes[kAxesLeftH] += delta_sig_;
          joy_data_.axes[kAxesLeftH] = joy_data_.axes[kAxesLeftH] < 1 ? joy_data_.axes[kAxesLeftH] : 1;
          dirty = true;
          break;

        case KEYCODE_D:
          joy_data_.axes[kAxesLeftH] -= delta_sig_;
          joy_data_.axes[kAxesLeftV] = joy_data_.axes[kAxesLeftV] > -1 ? joy_data_.axes[kAxesLeftV] : -1;
          dirty = true;
          break;

        // Sets all buttons and axis to zero
        case KEYCODE_C:
          ResetAxesButtons();
          dirty = true;
          break;

        // Select different control mode
        case KEYCODE_1:
          joy_data_.buttons[kButtonA] = 1;
          joy_data_.buttons[kButtonB] = 0;
          joy_data_.buttons[kButtonX] = 0;
          joy_data_.buttons[kButtonY] = 0;
          dirty = true;
          break;
        case KEYCODE_2:
          joy_data_.buttons[kButtonA] = 0;
          joy_data_.buttons[kButtonB] = 1;
          joy_data_.buttons[kButtonX] = 0;
          joy_data_.buttons[kButtonY] = 0;
          dirty = true;
          break;
        case KEYCODE_3:
          joy_data_.buttons[kButtonA] = 0;
          joy_data_.buttons[kButtonB] = 0;
          joy_data_.buttons[kButtonX] = 1;
          joy_data_.buttons[kButtonY] = 0;
          dirty = true;
          break;
        case KEYCODE_4:
          joy_data_.buttons[kButtonA] = 0;
          joy_data_.buttons[kButtonB] = 0;
          joy_data_.buttons[kButtonX] = 0;
          joy_data_.buttons[kButtonY] = 1;
          dirty = true;
          break;
      }

      if (dirty) {
        joy_pub_.publish(joy_data_);
      }
    }
  }

 private:
  double delta_sig_;
  struct termios old_;
  ros::NodeHandle node_;
  sensor_msgs::Joy joy_data_;
  ros::Publisher joy_pub_;
};

int main(int argc, char** argv) {
  if (!isatty(STDIN_FILENO)) {
    std::cerr << "Not a terminal";
    return 1;
  }

  std::string robot_name = "oatmeal";
  if (argc == 2) {
    robot_name = argv[1];
  }

  ros::init(argc, argv, "teleop_keyboard");

  puts("Reading from keyboard, publish to topic '/joy'");
  puts("---------------------------");
  puts("Use 'WS' for move forward and backward");
  puts("Use 'AD' for yaw angle of base link");
  puts("Use 'C' for zero all input command");
  puts("Use '1234' for select control mode");

  JoyStickKeyboard tpk(robot_name);
  tpk.LoopReadKeyboard();

  return 0;
}
