#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
  //pid.Init(1, 0.0, 1); //Car wobbles too much. It goes out of track and the ossiclations get amplified very quickly. Stayed in the center until the first corner only.
  //pid.Init(1, 0.0, 0.0); // WORSE. Same as above but wobbled more.
  //pid.Init(0.5, 0.0, 0.0); // Went further ahead and wobbled less.
  //pid.Init(0.1, 0.0, 0.0); // much better but couldn't the handle the corners
  //pid.Init(0.1, 1, 0.0);// car went round in a circle at the starting point.
  //pid.Init(0.1, 0.1, 0.0); // same as above but less wobble
  //pid.Init(0.1, 0.0, 1); // completed lap. drastic around corners. hitting curbs
  //pid.Init(0.1, 0.0, 2); // better, smoother and didn't hit the curbs
  //pid.Init(0.1, 0.0, 2.5); // corners were worse than I=2 but better than I=1
  //pid.Init(0.15, 0.0, 2.5); // best. increasing P made it return to center quicker in turns and hence didn't reach curb as close as before.
  //pid.Init(0.15, 0.01, 2.5); // wobbling started again
  //pid.Init(0.15, 0.001, 2.5); // best. much smoother cornering
  //pid.Init(0.15, 0.0001, 2.5); // best. smoother than last try
  //pid.Init(0.15, 0.00001, 2.5); // same as last excpet for the last corner which was wobbly
  //pid.Init(0.175, 0.00005, 2.5);// better the last try
  //pid.Init(0.175, 0.00005, 3); // Best. last corner smoothest until now
  //pid.Init(0.175, 0.00005, 5);// wobbly and took drastic turns
  pid.Init(0.155, 0.00005, 4); // Best. last corner smoothest
  
  // PID controller for throttle
  PID speed_pid;
  //speed_pid.Init(1, 0, 0); // goes fast but jumps out before the first turn
  //speed_pid.Init(0.1, 0, 0); // better but couldn't make it to first turn
  //speed_pid.Init(0.1, 0, 1); // reached first turn better
  //speed_pid.Init(0.1, 0, 3); // went slightly on the curb during the second turn and went out on the 4th turn. started applying brakes
  //speed_pid.Init(0.1, 0, 5); // 5th turn wobbling but completed lap. not too fast avg 35
  //speed_pid.Init(0.2, 0, 3);// increasing P made it worse
  //speed_pid.Init(0.05, 0, 3); // slowest
  //speed_pid.Init(0.15, 0, 3); // fast but went out of track
  //speed_pid.Init(0.15, 0.001, 5); // very fast but not stable
  //speed_pid.Init(0.11, 0.0001, 3); // wobbly
  //speed_pid.Init(0.11, 0.0, 3);// fast within track but very wobbly
  //speed_pid.Init(0.105, 0.0, 4);// fastest. avg 45. last turn wobbly
  //speed_pid.Init(0.1025, 0.0, 4.5);//wobbly. avg 40
  //speed_pid.Init(0.09, 0.0, 4.5);// best until now but on the last turn slight the car went slightly out
  //speed_pid.Init(0.075, 0.0, 4); // slow and last turn out
  speed_pid.Init(0.09, 0.0, 5); // best driving but a littler slower with avg speed of 32 and completed the entire track.
  

  h.onMessage([&pid, &speed_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          const double target_speed = 60.0;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          
          if(steer_value > 1){
            steer_value = 1;
          }
          
          else if(steer_value < -1){
            steer_value = -1;
          }
          
          // compute new throttle

          speed_pid.UpdateError(speed - target_speed);
          double throttle_value = speed_pid.TotalError();

          if(throttle_value > 1){
            throttle_value = 1.0;
          }else if(throttle_value < -1){
            throttle_value = -1.0;
          }
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = -steer_value;
          //msgJson["throttle"] = 0.3;
          msgJson["throttle"] = - throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}