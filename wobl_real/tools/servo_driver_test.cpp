// test_servo.cpp
#include <chrono>
#include <csignal>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <wobl_real/servo_driver.hpp>

static struct termios orig_termios;

// Restore terminal settings on exit
void restoreTerminal() { tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios); }

// Configure terminal for non-blocking, no-echo input
void setup_terminal() {
  tcgetattr(STDIN_FILENO, &orig_termios);
  atexit(restoreTerminal);

  struct termios newt = orig_termios;
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_cc[VMIN] = 0;
  newt.c_cc[VTIME] = 0;
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
}

// Check if 'q' or 'Q' was pressed
bool user_requested_release() {
  char c;
  if (read(STDIN_FILENO, &c, 1) == 1) {
    if (c == 'q' || c == 'Q')
      return true;
  }
  return false;
}

// Print diagnostics on one console line
void print_diagnostics(const ServoDriver::ServoState &st) {
  std::cout << "\rPos: " << st.position_rad << " rad | Vel: " << st.velocity_rps << " rps | Eff: " << st.effort_pct
            << "% | Volt: " << st.voltage_volts << " V | Temp: " << st.temperature_celcius
            << "°C | Cur: " << st.current_amps << "A   " << std::flush;
}

int main(int argc, char *argv[]) {

  if (argc != 2) {
    std::cout << "Usage: " << argv[0] << " <servo_id>\n";
    return 1;
  }

  const uint8_t ID = std::atoi(argv[1]);

  if (ID >= 254) {
    std::cout << "ID: " << ID << " are invalid\n";
    return 1;
  }
  setup_terminal();

  ServoDriver driver;
  constexpr double TORQUE_LIMIT = 80.0; // percent

  std::cout << "Initializing servo driver...\n";
  if (!driver.initialize()) {
    std::cerr << "Failed to initialize driver\n";
    return -1;
  }

  std::cout << "Pinging servo " << int(ID) << "... ";
  if (!driver.ping(ID)) {
    std::cerr << "no response\n";
    return -2;
  }
  std::cout << "OK\n\n";

  // Enable torque once at start
  driver.enable_torque(ID, true);

  //------------------------------------------------
  // 1) POSITION MODE TEST
  //------------------------------------------------
  std::cout << "=== POSITION MODE TEST ===\n";
  driver.set_mode(ID, ServoDriver::POSITION);

  std::vector<double> pos_sequence{0.0, 0.1, -0.1, 0.0}; // radians

  for (double target : pos_sequence) {
    break;
    std::cout << "Moving to position " << target << " rad\n";
    driver.write_position(ID, target, 0.2);
    // Wait until motion completes or error/quit
    while (true) {
      auto st = driver.read_state(ID);
      print_diagnostics(st);

      if (st.effort_pct > TORQUE_LIMIT) {
        std::cout << "\nTorque limit exceeded (" << st.effort_pct << "%). Disabling torque.\n";
        driver.enable_torque(ID, false);
        return 0;
      }

      if (!st.is_move) {
        std::cout << "\nReached target.\n\n";
        break;
      }

      if (user_requested_release()) {
        std::cout << "\nUser requested release. Disabling torque.\n";
        driver.enable_torque(ID, false);
        return 0;
      }

      usleep(50 * 1000); // 50 ms
    }
  }

  //------------------------------------------------
  // 2) VELOCITY MODE TEST
  //------------------------------------------------
  std::cout << "=== VELOCITY MODE TEST ===\n";
  driver.set_mode(ID, ServoDriver::VELOCITY);

  std::vector<double> vel_sequence{2.0, 1.0, -1.0, 1.0}; // rps

  for (double target_vel : vel_sequence) {
    std::cout << "Commanding velocity " << target_vel << " rps for 3 seconds\n";
    driver.write_velocity(ID, target_vel);

    auto start = std::chrono::steady_clock::now();
    while (true) {
      auto st = driver.read_state(ID);
      print_diagnostics(st);

      if (st.effort_pct > TORQUE_LIMIT) {
        std::cout << "\nTorque limit exceeded (" << st.effort_pct << "%). Disabling torque.\n";
        driver.enable_torque(ID, false);
        return 0;
      }

      if (user_requested_release()) {
        std::cout << "\nUser requested release. Disabling torque.\n";
        driver.enable_torque(ID, false);
        return 0;
      }

      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start);
      if (elapsed.count() >= 3) {
        std::cout << "\n3 seconds elapsed. Moving to next velocity.\n\n";
        break;
      }

      usleep(50 * 1000); // 50 ms
    }
  }

  driver.enable_torque(ID, false);
  std::cout << "Test complete. Exiting.\n";
  return 0;
}
