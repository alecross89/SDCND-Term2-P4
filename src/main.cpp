#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include "PID.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != std::string::npos)
    {
        return "";
    }
    else if (b1 != std::string::npos && b2 != std::string::npos)
    {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

int main()
{
    uWS::Hub hub;

    PID pid;

    /*
      The P is the proportional part of the PID controller.  The part controls the steering angle.  The Proportional
      part multiplies the error (in our case 'cte') by the Proportional Gain (tau). The error is the deviation from 
      the vehicles position to the center of the road.  The drawback to only using P is that this produces a oscillatory motion
      about the center of the road causing it to steer off the road if the error or the proportionaly gain becomes too large. 
    */
    const auto p = 0.085;
    /*
    The I is the integral part of the PID controller.  The I term increases action in relation not only to the error but also 
    the time for which it has persisted.  So as the accumulated error increases and gets further from zero, the I term will
    increase over time. 
    */
    const auto i = 0.0008;
    /*
    The D is the derivative term of the PID controller.  This term doesn't consider the error, so a contoller consising
    of just the D term could not bring the system down to zero error.  The D term looks instead at the rate of change 
    of the error, and thus tries to bring this term down to zero.  It aims to dampen the ocillatory motion of the car.  If
    the car begins to deviate quickly from the center of the road, it will increase the vehicles control and try to bring it back
    to the center.  This term is useful for turns for when the rate at which the error is changing increases more then when 
    the car is just driving on a straight road.
    */
    const auto d = 0.95;
    /* 
    The choice of the P, I and D values was done manually.  Clearly an algorithm like Twiddle or SGD would allow 
    for a faster way to optimize PID controller.  I would like to try and implement one of these algorithms when
    I have more time.  Although I think satisfactory results were obtained.
    */


    pid.Init(p, i, d);

    hub.onMessage([&pid](uWS::WebSocket<uWS::SERVER> web_socket, char* data, size_t length, uWS::OpCode op_code) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(std::string(data).substr(0, length));
            if (s != "")
            {
                auto json_data = json::parse(s);
                std::string event = json_data[0].get<std::string>();
                if (event == "telemetry")
                {
                    // json_data[1] is the data JSON object
                    double cte = std::stod(json_data[1]["cte"].get<std::string>());
                    double speed = std::stod(json_data[1]["speed"].get<std::string>());
                    double angle = std::stod(json_data[1]["steering_angle"].get<std::string>());

                    // Calculate steering value here, remember the steering value is [-1, 1].
                    double steer_value = -pid.tau_p_ * pid.p_error_ - pid.tau_d_ * pid.d_error_ - pid.tau_i_ * pid.i_error_;

                    if(steer_value > 1.0) steer_value = 1.0;
                    if(steer_value < -1.0) steer_value = -1.0;


                    // Debug output
                    std::cout << "CTE: " << cte << " | Speed: " << speed << " | Angle: " << angle << " | Steering Value: " << steer_value << std::endl;

                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = 0.25;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";

                    web_socket.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                    pid.UpdateError(cte);
                    // pid = Twiddle(cte);
                }
            }
            else
            {
                // Manual driving
                std::string message = "42[\"manual\",{}]";
                web_socket.send(message.data(), message.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the program
    // doesn't compile :-(
    hub.onHttpRequest([](uWS::HttpResponse* response, uWS::HttpRequest request, char* data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (request.getUrl().valueLength == 1)
        {
            response->end(s.data(), s.length());
        }
        else
        {
            // i guess this should be done more gracefully?
            response->end(nullptr, 0);
        }
    });

    hub.onConnection(
        [&hub, &pid](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) { pid.UpdateError(0.0); std::cout << "Connected!!!" << std::endl; });

    hub.onDisconnection([&hub](uWS::WebSocket<uWS::SERVER> web_socket, int code, char* message, size_t length) {
        web_socket.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (hub.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    hub.run();
}