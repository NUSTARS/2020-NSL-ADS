namespace nustars {
	static double proportionally_control(double expected, double no_drag, double yes_drag){
		double avg_drag = (no_drag + yes_drag) / 2;
		return (expected-avg_drag) / avg_drag;
	}
}