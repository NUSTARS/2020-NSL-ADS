#include <vector>

namespace nustars {
	static double proportionally_control(double expected, double no_drag, double yes_drag){
		double avg_drag = (no_drag + yes_drag) / 2;
		return (expected-avg_drag) / avg_drag;
	}

  static double get_conj(double x){
    return 1-x;
  }

  // returns [acceleration, velocity, altitude]
  // filter_data = [acceleration, velocity, altitude]
  // sensor_data = [acceleration, altitude, change_in_time]
  // filter_dif = [alt(n)-alt(n-1), vel(n)-vel(n-1), prev_change_in_time]
  static vector<double> filter(vector<double>& sensor_data, vector<double>& filter_data, vector<double>& fitler_dif){
    double sen_acc = sensor_data[0];
    double sen_alt = sensor_data[1];
    double dt = sensor_data[2];

    double fil_acc = filter_data[0];
    double fil_vel = filter_data[1];
    double fil_alt = filter_data[2];

    double dif_alt = fitler_dif[0];
    double dif_vel = fitler_dif[1];
    double prev_dt = filter_dif[2];

    // these need to be tuned lmao
    double wgt_acc = .7
    double wgt_vel = .5 // this one is different lmao
    double wgt_alt = .7

    double new_acc = wgt_acc*(fil_acc + dif_vel*prev_dt) + get_conj(wgt_acc)*sen_acc;
    double new_vel = fil_vel + dt*(wgt_vel*fil_acc) + prev_dt*get_conj(wgt_vel)*dif_alt;
    double new_alt = wgt_alt*(fil_alt + dt*fil_vel) + get_conj(wgt_alt)*sen_alt;

    vector<double> final_data = {new_acc, new_vel, new_alt};
  }
}
