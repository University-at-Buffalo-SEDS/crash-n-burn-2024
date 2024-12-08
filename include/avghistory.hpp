// Maintains a history of recent averages of the samples
// passed in and provides access to an old average.
// T: Type of variables holding average
// S: Number of samples per average
// L: Length of history of averages
template <typename T, int S, int L>
class AvgHistory {
	T avg_history[L];
	uint8_t hist_len;
	uint8_t count;

public:
	AvgHistory() :
			avg_history{0},
			hist_len(0), count(0) {}

	// Calculates an average of the readings passed to it, with a delay.
	// Used to calculate an estimate of the local gravity and altitude above sea level.
	void add(T reading);

	// Returns whether enough samples have been fed in for the old_avg to be valid.
	bool full() { return hist_len >= L - 1; }

	// Returns an an older average of the samples fed in.
	T old_avg() { return avg_history[L - 1]; }
};

template <typename T, int S, int L>
void AvgHistory<T, S, L>::add(T reading)
{
	if (count > S) {
		for (size_t i = L - 1; i > 0; --i) {
			avg_history[i] = avg_history[i - 1];
		}
		count = 1;
		hist_len = std::min(L - 1, hist_len + 1);
	}
	// Compute a running average
	avg_history[0] *= T(count) / T(count + 1);
	avg_history[0] += reading / T(count + 1);
	count += 1;
}